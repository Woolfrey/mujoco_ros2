/**
 * @file   mujoco_ros.cpp
 * @author Jon Woolfrey
 * @email  jonathan.woolfrey@gmail.com
 * @date   April 2025
 * @version 1.2
 * @brief  A class for connecting a MuJoCo simulation with ROS2 communication.
 * 
 * @details This class launches a MuJoCo simulation and provides communication channels in ROS2 for controlling it.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://mujoco.org/ for more information about MuJoCo
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation
 */
 
#include <mujoco_ros2/mujoco_ros.hpp>
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

namespace
{
    constexpr double kPi = 3.14159265358979323846;                                                  // Local constant for deg<->rad conversion in the GUI panel (avoids <cmath> M_PI portability quirks)
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
MuJoCoROS::MuJoCoROS(const std::string &xmlLocation) : Node("mujoco_node")
{
    // Declare & get parameters for this node
    std::string jointStateTopicName   = this->declare_parameter<std::string>("joint_state_topic_name", "joint_state");
    std::string jointCommandTopicName = this->declare_parameter<std::string>("joint_command_topic_name", "joint_commands");
    std::string controlMode           = this->declare_parameter<std::string>("control_mode", "TORQUE");
    _simFrequency                     = this->declare_parameter<int>("simulation_frequency", 1000);
    _visFrequency                     = this->declare_parameter<int>("visualisation_frequency", 20);
    _commandTimeout                   = this->declare_parameter<double>("command_timeout", 0.5);
    _holdKp                           = this->declare_parameter<double>("hold_position_kp", 5.0);
    _holdKd                           = this->declare_parameter<double>("hold_position_kd", 1);
    
    if (_simFrequency <= 0) throw std::invalid_argument("[ERROR] [MuJoCo NODE] 'simulation_frequency' must be greater than zero.");
    if (_visFrequency <= 0) throw std::invalid_argument("[ERROR] [MuJoCo NODE] 'visualisation_frequency' must be greater than zero.");

    // Load the robot model     
    char errorMessage[1000] = "Could not load model.";                                              // We need this as an input argument.   
    _model = mj_loadXML(xmlLocation.c_str(), nullptr, errorMessage, 1000);                          // Try to load the model  
    if (not _model) throw std::runtime_error("[ERROR] [MuJoCo NODE] Problem loading model: " + std::string(errorMessage)); 
    _model->opt.timestep = 1.0/((double)_simFrequency);                                             // Match MuJoCo to node frequency
    
     // Resize arrays based on the number of joints in the model
    _jointState = mj_makeData(_model);                                                              // Initialize joint state
    if (not _jointState) throw std::runtime_error("[ERROR] [MuJoCo NODE] Failed to allocate MuJoCo data (mj_makeData returned null).");
    
    mj_forward(_model, _jointState);                                                                // Compute derived quantities (body/geom transforms) before the first control or render step
    
    _jointStateMessage.name.resize(_model->nq);
    _jointStateMessage.position.resize(_model->nq);
    _jointStateMessage.velocity.resize(_model->nq);
    _jointStateMessage.effort.resize(_model->nq);
    
    _commandTarget.assign(_model->nq, 0.0);                                                         // Only meaningful in VELOCITY/TORQUE modes -- see _positionTarget for POSITION mode and the shared hold/tracking target
    _lastCommandTime = this->get_clock()->now();                                                    // So the watchdog doesn't fire before the node has even finished starting

    _positionTarget.resize(_model->nq);
    for (int i = 0; i < _model->nq; ++i) _positionTarget[i] = _jointState->qpos[i];                  // Start by holding the model's initial pose, not zero

    // Detect what kind of feedback (if any) each actuator already does internally, so we know
    // whether to synthesize our own PD control or just pass values straight through.
    // NOTE: indexed by qpos index here, same simplifying assumption (one single-DOF joint per
    // actuator) used everywhere else in this file -- see the nq/nu/nv note flagged separately.
    _actuatorType.resize(_model->nq);
    for (int i = 0; i < _model->nq; ++i)
    {
        if (_model->actuator_biastype[i] == mjBIAS_AFFINE)
        {
            double kpTerm = _model->actuator_biasprm[i * mjNBIAS + 1];                              // Position feedback coefficient; nonzero => this actuator holds position internally
            
            _actuatorType[i] = (kpTerm != 0.0) ? ActuatorType::POSITION_SERVO : ActuatorType::VELOCITY_SERVO;
        }
        else
        {
            _actuatorType[i] = ActuatorType::MOTOR;                                                 // biastype NONE (a <motor>), or an unrecognised type -- treat ctrl as a direct force/torque input
        }
    }

    // Precompute each joint's slider bounds and display units (degrees for hinges, native units
    // for slides), so the GUI panel doesn't need to re-derive this every frame.
    _sliderInfo.resize(_model->nq);
    for (int i = 0; i < _model->nq; ++i)
    {
        bool isAngular = (_model->jnt_type[i] == mjJNT_HINGE);
        
        double lower, upper;
        if (_model->jnt_limited[i])
        {
            lower = _model->jnt_range[i*2 + 0];
            upper = _model->jnt_range[i*2 + 1];
        }
        else
        {
            // No limit defined in the model -- fall back to a generic, overridable-later default
            lower = isAngular ? -kPi : -1.0;
            upper = isAngular ?  kPi :  1.0;
        }
        
        _sliderInfo[i] = SliderRange{lower, upper, isAngular};
    }

    // Record joint names
    for (int i = 0; i < _model->nq; ++i) _jointStateMessage.name[i] = mj_id2name(_model, mjOBJ_JOINT, i);
       
    // Set the control mode  
         if (controlMode == "POSITION") _controlMode = POSITION;
    else if (controlMode == "VELOCITY") _controlMode = VELOCITY;
    else if (controlMode == "TORQUE"  ) _controlMode = TORQUE;
    else
    {   
        throw std::invalid_argument("[ERROR] [MuJoCo NODE] Unknown control mode. "
                                    "Argument was '" + controlMode + "', but expected 'POSITION', 'VELOCITY', or 'TORQUE'.");
    }
    
    // Warn (but don't block startup) about joints where the chosen control mode doesn't have a
    // clean interpretation for that joint's actuator type -- see position_tracking_ctrl().
    for (int i = 0; i < _model->nq; ++i)
    {
        bool mismatch = (_controlMode == TORQUE   and _actuatorType[i] != ActuatorType::MOTOR) or
                        (_controlMode == POSITION and _actuatorType[i] == ActuatorType::VELOCITY_SERVO);
        
        if (mismatch)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Joint '%s' uses an actuator type that doesn't naturally support %s mode; "
                        "its behaviour may not be what you expect.",
                        _jointStateMessage.name[i].c_str(), controlMode.c_str());
        }
    }

    // Create joint state publisher and joint command subscriber
    _jointCommandSubscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(jointCommandTopicName, 1,  std::bind(&MuJoCoROS::joint_command_callback, this, std::placeholders::_1));
    
    _jointStatePublisher = this->create_publisher<sensor_msgs::msg::JointState>(jointStateTopicName, 1);
    
    discover_wrench_sensors();                                                                      // Find any <force>/<torque> sensors in the model and create publishers for them
                      
             
    // Initialize Graphics Library FrameWork (GLFW)
    if (not glfwInit()) throw std::runtime_error("Failed to initialise Graphics Library Framework (GLFW).");

    _window = glfwCreateWindow(1200, 900, "MuJoCo Visualization", nullptr, nullptr);
    
    if (not _window) throw std::runtime_error("Failed to create Graphics Library Framework (GLFW) window.");
    
    // Make the OpenGL context current
    glfwMakeContextCurrent(_window);
    glfwSwapInterval(0);                                                                            // Don't block on vsync -- render_loop() paces itself at `_visFrequency` instead, so a stalled/minimized window can no longer hold up anything else

    // Initialize MuJoCo rendering context
    mjv_defaultCamera(&_camera);
    mjv_defaultOption(&_renderingOptions);
    mjv_defaultPerturb(&_perturbation);
    mjr_defaultContext(&_context);
    mjv_makeScene(_model, &_scene, 1000);
    
    // Declare & get parameters for camera, visualisation
    _camera.azimuth      = this->declare_parameter<double>("camera_azimuth", 135);    
    _camera.distance     = this->declare_parameter<double>("camera_distance", 2.5);
    _camera.elevation    = this->declare_parameter<double>("camera_elevation", -35);
    _camera.orthographic = this->declare_parameter<bool>("camera_orthographic", true);
    
    auto focalPoint = this->declare_parameter<std::vector<double>>("camera_focal_point", {0.0, 0.0, 0.5});
    
    for(int i = 0; i < 3; ++i) _camera.lookat[i] = focalPoint[i]; 

    // Create MuJoCo rendering context
    glfwMakeContextCurrent(_window);
    mjr_makeContext(_model, &_context, mjFONTSCALE_100);
    
    // Initialize Dear ImGui (joint sliders, camera-drag input is also read through its IO state)
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().FontGlobalScale = 2.0f;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(_window, true);                                                    // true = let ImGui install its own GLFW input callbacks
    ImGui_ImplOpenGL3_Init("#version 150");
    
    RCLCPP_INFO(this->get_logger(), "MuJoCo simulation initiated. "
                                    "Publishing the joint state to '%s' topic. "
                                    "Subscribing to joint %s commands via '%s' topic.",
                                    jointStateTopicName.c_str(), controlMode.c_str(), jointCommandTopicName.c_str());

}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Set the command target                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MuJoCoROS::set_target(const std::vector<double> &values)
{
    std::lock_guard<std::mutex> lock(_commandMutex);
    
    if (_sliderActive) return;                                                                      // A GUI slider currently has priority -- ignore the topic until it's released
    
    _commandTarget = values;
    
    if (_controlMode == POSITION) _positionTarget = values;                                         // POSITION mode: the commanded value IS the position target, directly
    
    _lastCommandTime = this->get_clock()->now();
    _watchdogTripped = false;                                                                       // A fresh topic command has arrived -- resume normal control
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                  Set the position target (GUI)                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MuJoCoROS::set_position_target(const std::vector<double> &values, bool dragging)
{
    std::lock_guard<std::mutex> lock(_commandMutex);
    
    _positionTarget  = values;
    _sliderActive    = dragging;                                                                    // While true, joint_command_callback() ignores the topic; the very next topic message after release resumes it
    _watchdogTripped = true;                                                                        // Forces the PD-hold/tracking path immediately in TORQUE mode, and freezes VELOCITY-mode integration, without waiting for the timeout
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                   Position-tracking control law                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
double MuJoCoROS::position_tracking_ctrl(int i) const
{
    if (_actuatorType[i] == ActuatorType::POSITION_SERVO) return _positionTarget[i];                // MuJoCo's own actuator gain already closes this loop -- just pass the setpoint through
    
    return _holdKp*(_positionTarget[i] - _jointState->qpos[i])                                      // Our own PD. Correct for MOTOR actuators; a documented rough fallback for VELOCITY_SERVO
         - _holdKd*_jointState->qvel[i]                                                             // (flagged at startup) since it has no real position channel to track against.
         + _jointState->qfrc_bias[i];
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Find & set up publishers for force/torque sensors                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MuJoCoROS::discover_wrench_sensors()
{
    for (int i = 0; i < _model->nsensor; ++i)
    {
        int type = _model->sensor_type[i];
        
        if (type != mjSENS_FORCE and type != mjSENS_TORQUE) continue;                              // Ignore everything else for now
        
        int siteId = _model->sensor_objid[i];
        std::string siteName = mj_id2name(_model, mjOBJ_SITE, siteId);
        
        auto &pair = _wrenchSensors[siteName];                                                     // Creates entry on first encounter
        
        if (type == mjSENS_FORCE) pair.forceAdr  = _model->sensor_adr[i];
        else                      pair.torqueAdr = _model->sensor_adr[i];
    }
    
    for (auto &[siteName, pair] : _wrenchSensors)
    {
        pair.publisher = this->create_publisher<geometry_msgs::msg::WrenchStamped>(siteName, 1);
        
        if (pair.forceAdr == -1 or pair.torqueAdr == -1)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Site '%s' has only a %s sensor -- the missing component will publish as zero.",
                        siteName.c_str(), pair.forceAdr == -1 ? "torque" : "force");
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Publishing %zu wrench sensor site(s).", _wrenchSensors.size());
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Update the simulation                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MuJoCoROS::update_simulation()
{
    bool watchdogTripped;
    std::vector<double> commandTarget;                                                              // Raw velocity/torque command from the topic (unused in POSITION mode)
    
    {
        std::lock_guard<std::mutex> lock(_commandMutex);
        
        double secondsSinceCommand = (this->get_clock()->now() - _lastCommandTime).seconds();
        
        if (secondsSinceCommand > _commandTimeout and not _watchdogTripped and not _sliderActive)
        {
            _watchdogTripped = true;                                                                // Just crossed the timeout -- latch this so we only capture the hold position once, right at the transition
            
            for (int i = 0; i < _model->nq; ++i) _positionTarget[i] = _jointState->qpos[i];         // Read-only access to _jointState from the physics thread itself; safe without _renderMutex
            
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "No joint command received in %.2f seconds. Holding position.", _commandTimeout);
        }
        
        commandTarget   = _commandTarget;
        watchdogTripped = _watchdogTripped;
    }
    
    {
        std::lock_guard<std::mutex> lock(_renderMutex);                                             // The render thread may be reading _jointState via mjv_updateScene
        
        switch (_controlMode)
        {
            case POSITION:
            {
                // _positionTarget already holds the commanded value directly (set in set_target()/set_position_target())
                for (int i = 0; i < _model->nq; ++i) _jointState->ctrl[i] = position_tracking_ctrl(i);
                
                break;
            }
            case VELOCITY:
            {
                for (int i = 0; i < _model->nq; ++i)
                {
                    if (_actuatorType[i] == ActuatorType::VELOCITY_SERVO)
                    {
                        // This actuator already IS a velocity servo -- pass the commanded velocity straight through.
                        // No position channel to hold with, so the safest "hold" here is simply zero velocity.
                        _jointState->ctrl[i] = watchdogTripped ? 0.0 : commandTarget[i];
                    }
                    else
                    {
                        if (not watchdogTripped) _positionTarget[i] += commandTarget[i] / (double)_simFrequency; // Integrate velocity to a running position target, once per physics step
                        // else: frozen -- _positionTarget stops advancing, holding the joint in place
                        
                        _jointState->ctrl[i] = position_tracking_ctrl(i);                            // MOTOR: our own PD. POSITION_SERVO: passthrough (MuJoCo tracks it)
                    }
                }
                
                break;
            }
            case TORQUE:
            {
                for (int i = 0; i < _model->nq; ++i)
                {
                    if (not watchdogTripped)
                    {
                        _jointState->ctrl[i] = commandTarget[i] + _jointState->qfrc_bias[i];         // Open-loop torque command + dynamics compensation
                    }
                    else
                    {
                        _jointState->ctrl[i] = position_tracking_ctrl(i);                            // PD hold about the position captured when the watchdog tripped, or set by a slider
                    }
                }
                
                break;
            }
            default:
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Unknown control mode. Unable to set joint commands.");
                break;
            }
        }

        mj_step(_model, _jointState);                                                               // Take a step in the simulation

        // Publish any force/torque sensor readings as WrenchStamped messages
        for (auto &[siteName, pair] : _wrenchSensors)
        {
            geometry_msgs::msg::WrenchStamped wrenchMessage;
            wrenchMessage.header.stamp    = this->get_clock()->now();
            wrenchMessage.header.frame_id = siteName;
            
            if (pair.forceAdr >= 0)
            {
                wrenchMessage.wrench.force.x = _jointState->sensordata[pair.forceAdr + 0];
                wrenchMessage.wrench.force.y = _jointState->sensordata[pair.forceAdr + 1];
                wrenchMessage.wrench.force.z = _jointState->sensordata[pair.forceAdr + 2];
            }
            // else: left zero-initialized (no force sensor at this site)
            
            if (pair.torqueAdr >= 0)
            {
                wrenchMessage.wrench.torque.x = _jointState->sensordata[pair.torqueAdr + 0];
                wrenchMessage.wrench.torque.y = _jointState->sensordata[pair.torqueAdr + 1];
                wrenchMessage.wrench.torque.z = _jointState->sensordata[pair.torqueAdr + 2];
            }
            // else: left zero-initialized (no torque sensor at this site)
            
            pair.publisher->publish(wrenchMessage);
        }

        // Add joint state data to ROS2 message
        for (int i = 0; i < _model->nq; ++i)
        {
            _jointStateMessage.position[i] = _jointState->qpos[i];
            _jointStateMessage.velocity[i] = _jointState->qvel[i];
            _jointStateMessage.effort[i]   = _jointState->actuator_force[i];
        }
    }

    _jointStateMessage.header.stamp = this->get_clock()->now();                                     // Add current time stamp (for rqt)
    
    _jointStatePublisher->publish(_jointStateMessage);                                              // Safe to call directly from the physics thread: rclcpp publishers are thread-safe, and no other thread touches _jointStateMessage
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Handle joint commands                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
MuJoCoROS::joint_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if ((int)msg->data.size() != _model->nq)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Received joint command with incorrect size.");
        return;
    }
    
    set_target(msg->data);                                                                          // Single chokepoint shared with any future GUI input (e.g. slider bars)
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Physics loop                                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MuJoCoROS::physics_loop()
{
    using Clock = std::chrono::steady_clock;
    
    auto period = std::chrono::duration<double>(1.0/(double)_simFrequency);
    auto next   = Clock::now();
    
    while (is_running() and rclcpp::ok())
    {
        update_simulation();
        
        next += std::chrono::duration_cast<Clock::duration>(period);
        
        std::this_thread::sleep_until(next);
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Render loop                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MuJoCoROS::render_loop()
{
    using Clock = std::chrono::steady_clock;
    
    auto period = std::chrono::duration<double>(1.0/(double)_visFrequency);
    auto next   = Clock::now();
    
    glfwMakeContextCurrent(_window);                                                                // Ensure OpenGL context is current on this (the calling) thread
    
    while (is_running() and rclcpp::ok() and not glfwWindowShouldClose(_window))
    {
        int width, height;
        glfwGetFramebufferSize(_window, &width, &height);
        
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        
        ImGuiIO &io = ImGui::GetIO();
        
        if (not io.WantCaptureMouse and height > 0)                                                 // Don't drag the camera while interacting with the GUI panel
        {
            if (io.MouseDown[ImGuiMouseButton_Left])                                                // Left-drag: rotate (shift = horizontal)
            {
                mjv_moveCamera(_model, io.KeyShift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V,
                               io.MouseDelta.x/(float)height, io.MouseDelta.y/(float)height, &_scene, &_camera);
            }
            else if (io.MouseDown[ImGuiMouseButton_Right])                                          // Right-drag: pan (shift = horizontal)
            {
                const float panSpeed = 5.0f;                                                             // >1 faster, <1 slower
                mjv_moveCamera(_model, io.KeyShift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V,
                               panSpeed*io.MouseDelta.x/(float)height, panSpeed*io.MouseDelta.y/(float)height, &_scene, &_camera);
            }
            
            if (io.MouseWheel != 0.0f) mjv_moveCamera(_model, mjMOUSE_ZOOM, 0, -0.05*io.MouseWheel, &_scene, &_camera); // Scroll: zoom
        }
        
        draw_gui_panel();
        
        ImGui::Render();
        
        {
            std::lock_guard<std::mutex> lock(_renderMutex);                                         // The physics thread may be mid mj_step
            
            mjv_updateScene(_model, _jointState, &_renderingOptions, NULL, &_camera, mjCAT_ALL, &_scene); // Update 3D rendering
        }

        mjrRect viewport = {0, 0, width, height};

        mjr_render(viewport, &_scene, &_context);                                                   // Render scene
        
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());                                     // Draw the GUI overlay on top of the 3D scene
        
        // Swap buffers and process events
        glfwSwapBuffers(_window);
        glfwPollEvents();
        
        next += std::chrono::duration_cast<Clock::duration>(period);
        
        std::this_thread::sleep_until(next);
    }
    
    _running.store(false, std::memory_order_release);                                               // Window closed (or shutdown requested elsewhere) -- signal the physics/ROS threads to stop
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        GUI slider panel                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MuJoCoROS::draw_gui_panel()
{
    float displayHeight = ImGui::GetIO().DisplaySize.y;
    
    if (not _showPanel)
    {
        // Collapsed: draw nothing but a small, always-visible tab to reopen the panel
        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.6f);
        ImGui::Begin("##gui_tab_closed", nullptr,
                      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                      ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings);
        
        if (ImGui::Button(">>")) _showPanel = true;
        
        ImGui::End();
        
        return;
    }
    
    std::vector<double> positionTargetSnapshot;
    {
        std::lock_guard<std::mutex> lock(_commandMutex);
        positionTargetSnapshot = _positionTarget;
    }
    
    std::vector<double> newTarget = positionTargetSnapshot;                                         // Non-dragged joints keep their current target; overwritten below only for the joint(s) actively being dragged
    bool anyActive = false;
    
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);                                        // Pinned to the left edge, full height -- see NoMove below
    ImGui::SetNextWindowSize(ImVec2(600, displayHeight), ImGuiCond_FirstUseEver);                   // Generous initial width so labels/sliders aren't clipped; user can still widen it
    ImGui::SetNextWindowSizeConstraints(ImVec2(280, displayHeight), ImVec2(900, displayHeight));    // Height locked (min == max); width free to resize within this range
    ImGui::SetNextWindowBgAlpha(0.85f);
    ImGui::Begin("MuJoCo Control", nullptr,
                  ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove);
    
    if (ImGui::Button("<< Hide")) _showPanel = false;
    
    ImGui::Separator();
    
    for (int i = 0; i < _model->nq; ++i)
    {
        const SliderRange &range = _sliderInfo[i];
        
        double toDisplay  = range.isAngular ? (180.0/kPi) : 1.0;                                    // Radians -> degrees for hinges; pass-through for slides
        double toInternal = range.isAngular ? (kPi/180.0) : 1.0;                                    // Degrees -> radians for hinges; pass-through for slides
        
        float value = static_cast<float>(positionTargetSnapshot[i] * toDisplay);
        float lower = static_cast<float>(range.lower * toDisplay);
        float upper = static_cast<float>(range.upper * toDisplay);
        
        std::string label = _jointStateMessage.name[i] + (range.isAngular ? " (deg)" : " (m)");
        
        ImGui::SliderFloat(label.c_str(), &value, lower, upper, "%.1f");
        
        if (ImGui::IsItemActive())
        {
            anyActive    = true;
            newTarget[i] = static_cast<double>(value) * toInternal;
        }
    }
    
    ImGui::End();
    
    if (anyActive)
    {
        set_position_target(newTarget, true);
        _wasDraggingLastFrame = true;
    }
    else if (_wasDraggingLastFrame)
    {
        set_position_target(newTarget, false);                                                      // Just released -- one final call so the topic can resume on its next message
        _wasDraggingLastFrame = false;
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           Destructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
MuJoCoROS::~MuJoCoROS()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    
    mj_deleteData(_jointState);
    mj_deleteModel(_model);
    mjv_freeScene(&_scene);
    mjr_freeContext(&_context);
    glfwDestroyWindow(_window);
    glfwTerminate();
}

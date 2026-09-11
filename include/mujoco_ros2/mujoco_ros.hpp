/**
 * @file   mujoco_ros.hpp
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

#ifndef MUJOCO_NODE_H
#define MUJOCO_NODE_H

#include <GLFW/glfw3.h>                                                                             // Graphics Library Framework; for visualisation
#include <atomic>                                                                                   // std::atomic<bool>
#include <geometry_msgs/msg/wrench_stamped.hpp>                                                     // For publishing force/torque sensor data
#include <iostream>                                                                                 // std::cerr, std::cout
#include <map>                                                                                      // std::map
#include <mujoco/mujoco.h>                                                                          // Dynamic simulation library
#include <mutex>                                                                                    // std::mutex, std::lock_guard
#include <string>                                                                                   // std::string
#include <rclcpp/rclcpp.hpp>                                                                        // ROS2 C++ libraries.
#include <sensor_msgs/msg/joint_state.hpp>                                                          // For publishing / subscribing to joint states.
#include <std_msgs/msg/float64_multi_array.hpp>
#include <thread>                                                                                   // std::thread
#include <vector>

enum ControlMode {POSITION, VELOCITY, TORQUE, UNKNOWN};                                             // This needs a global scope

/**
 * @brief What kind of feedback (if any) a MuJoCo actuator already does internally, detected once
 *        at model-load time from `actuator_biastype`/`actuator_biasprm`. This determines whether
 *        `ctrl` should be synthesized via our own PD loop (MOTOR) or simply passed through
 *        (POSITION_SERVO, VELOCITY_SERVO -- MuJoCo already closes the loop for these).
 */
enum class ActuatorType {MOTOR, POSITION_SERVO, VELOCITY_SERVO};

/**
 * @brief Precomputed slider bounds for one joint, in display units (degrees for hinge joints,
 *        native length units for slide joints). Computed once at load time from the model.
 */
struct SliderRange
{
    double lower;
    double upper;
    bool   isAngular;                                                                               ///< True for hinge joints (displayed/edited in degrees); false for slide joints (native units)
};

/**
 * @brief Pairs a MuJoCo <force> and <torque> sensor attached to the same site, so both can be
 *        published together as a single WrenchStamped message. Either address may be -1 if only
 *        one of the two sensors is present at that site.
 *
 * @note Only mjSENS_FORCE and mjSENS_TORQUE are currently detected -- other MuJoCo sensor types
 *       (touch, accelerometer, gyro, etc.) are ignored for now. Adding support for another type
 *       later is additive: a new sibling container + detection branch, without needing to touch
 *       this one.
 */
struct WrenchSensorPair
{
    int forceAdr  = -1;                                                                             ///< Index into sensordata; -1 if no force sensor at this site
    int torqueAdr = -1;                                                                             ///< Index into sensordata; -1 if no torque sensor at this site
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher;
};
        
/**
 * @brief This class launches both a MuJoCo simulation, and ROS2 node for communication.
 * 
 * @details Runs across three threads:
 *            - the physics thread (`physics_loop`), which steps the simulation and publishes joint state,
 *            - the render thread (`render_loop`), which MUST run on the same thread that constructed this
 *              object, since GLFW requires its OpenGL context to stay on the thread that created it, and
 *            - the ROS executor thread (`rclcpp::spin`, started by the caller), which services the joint
 *              command subscription.
 *          `_renderMutex` guards `_jointState` between the physics and render threads. `_commandMutex`
 *          guards the command/watchdog state between the ROS executor thread and the physics thread.
 */
class MuJoCoROS: public rclcpp::Node
{
    public:
            
        /**
         * @brief Contructor.
         * @param xmlLocation Where to find the XML file that defines the MuJoCo model.
         */
        MuJoCoROS(const std::string &xmlLocation);
        
       /**
        * @brief Deconstructor.
        */
        ~MuJoCoROS();
        
        /**
         * @brief Steps the simulation at `simulation_frequency` and publishes joint state. Intended to
         *        be run on its own thread; loops until `is_running()` is false.
         */
        void
        physics_loop();
        
        /**
         * @brief Runs the GLFW render loop at `visualisation_frequency`. MUST be called from the same
         *        thread that constructed this object. Blocks until the window is closed, or `is_running()`
         *        is false, at which point it also clears `is_running()` for the physics/ROS threads to see.
         */
        void
        render_loop();
        
        /**
         * @brief Sets the current joint command target and resets the command watchdog. This is the
         *        single entry point for anything that wants to drive the robot -- the ROS topic
         *        subscriber today, and a future GUI (e.g. slider bars) later -- so the watchdog and
         *        hold behaviour don't need to know or care which source a command came from.
         * @param values The commanded values, interpreted according to the current control mode
         *               (position setpoint, velocity, or torque).
         */
        void
        set_target(const std::vector<double> &values);
        
        /**
         * @brief Directly sets the position target -- for GUI sliders. Unlike `set_target()`, this
         *        always means "go to this position" regardless of control mode. While `dragging` is
         *        true, incoming topic commands are ignored (so the slider has uncontested control);
         *        pass `dragging = false` once released to let the topic resume on its next message.
         * @param values Desired position for each joint.
         * @param dragging True while the slider is actively being moved; false on release.
         */
        void
        set_position_target(const std::vector<double> &values, bool dragging);
        
        /**
         * @brief Whether the physics/render loops should keep running.
         */
        bool
        is_running() const { return _running.load(std::memory_order_acquire); }
        
    private:

        ControlMode _controlMode;                                                                   ///< POSITION, VELOCITY, or TORQUE
        
        mjModel *_model;                                                                            ///< Underlying model of the robot.
        mjData  *_jointState;                                                                       ///< Joint state data. Written by the physics thread; read by the render thread under `_renderMutex`.

        mjvCamera  _camera;                                                                         ///< Camera for viewing
        mjvOption  _renderingOptions;                                                               ///< As it says
        mjvPerturb _perturbation;                                                                   ///< Allows manual interaction
        mjvScene   _scene;                                                                          ///< The environment that the robot is rendered in

        mjrContext _context;                                                                        ///< OpenGL rendering resources (shaders, buffers) MuJoCo needs to draw the scene.

        GLFWwindow *_window;                                                                        ///< This displays the robot and environment.

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _jointStatePublisher;            ///< As it says on the label

        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr _jointCommandSubscriber;  ///< Subscriber for joint commands

        sensor_msgs::msg::JointState _jointStateMessage;                                            ///< For publishing joint state data over ROS2. Only touched by the physics thread.
        
        int _simFrequency = 1000;                                                                   ///< Rate the physics thread steps the simulation (Hz)
        
        int _visFrequency = 20;                                                                     ///< Rate the render thread redraws the scene (Hz)

        std::atomic<bool> _running{true};                                                           ///< Set false (e.g. on window close) to stop the physics/render loops

        std::mutex _renderMutex;                                                                    ///< Guards `_jointState`: physics thread locks it around `mj_step`; render thread locks it around `mjv_updateScene`.
        
        // --- Command / watchdog state, guarded by _commandMutex ---
        
        std::mutex _commandMutex;                                                                   ///< Guards everything below. Written by `set_target()`/`set_position_target()`; read once per step by the physics thread.
        
        std::vector<double> _commandTarget;                                                         ///< Last raw value from the topic: a velocity (VELOCITY mode) or torque (TORQUE mode) command. Not used in POSITION mode -- see `_positionTarget`.
        
        std::vector<double> _positionTarget;                                                        ///< The position every mode ultimately tracks: set directly in POSITION mode, integrated from `_commandTarget` in VELOCITY mode, and used as the hold point in TORQUE mode once the watchdog trips. Also what GUI sliders write to directly.
        
        rclcpp::Time _lastCommandTime;                                                              ///< When a topic command was last accepted (NOT updated while a slider has priority)
        
        double _commandTimeout = 0.5;                                                               ///< Seconds without a new topic command before the watchdog trips (`command_timeout` parameter)
        
        bool _watchdogTripped = false;                                                              ///< True once `_commandTimeout` has elapsed since the last topic command, OR a slider currently has priority
        
        bool _sliderActive = false;                                                                 ///< True while a GUI slider is actively being dragged; sets `joint_command_callback` to ignore incoming topic messages until release
        
        double _holdKp = 25.0;                                                                      ///< Proportional gain for tracking `_positionTarget` on MOTOR actuators (`hold_position_kp`)
        
        double _holdKd = 6.0;                                                                       ///< Derivative gain for tracking `_positionTarget` on MOTOR actuators (`hold_position_kd`)
        
        std::vector<ActuatorType> _actuatorType;                                                    ///< Per-joint actuator type, detected once at load time. NOTE: indexed like everything else here by qpos index (0..nq-1), which assumes one single-DOF joint per actuator -- the same simplifying assumption used throughout this file (see nq vs nu vs nv note).
        
        std::vector<SliderRange> _sliderInfo;                                                       ///< Per-joint slider bounds/units, computed once at load time from the model
        
        std::map<std::string, WrenchSensorPair> _wrenchSensors;                                     ///< Force/torque sensors found in the model, keyed by the site name they're attached to. Populated once at load time by discover_wrench_sensors().
        
        bool _showPanel = true;                                                                     ///< Whether the GUI slider panel is expanded (true) or collapsed to a small reopen tab (false)
        
        bool _wasDraggingLastFrame = false;                                                         ///< Detects the release edge (drag -> not dragging) so we fire exactly one "dragging = false" call to set_position_target()
    
        /**
         * @brief Advances the simulation by one step, applies the current command/watchdog logic,
         *        and publishes the resulting joint state and any wrench sensor readings. Called
         *        once per cycle from `physics_loop()`.
         */
        void
        update_simulation();
        
        /**
         * @brief Draws the ImGui joint-slider panel (or, when collapsed, the small tab used to
         *        reopen it) and forwards any active slider drag to `set_position_target()`. Called
         *        once per frame from `render_loop()`, on the same thread, after `ImGui::NewFrame()`.
         */
        void
        draw_gui_panel();
        
        /**
         * @brief Turns `_positionTarget[i]` into a `ctrl` value appropriate for that joint's actuator
         *        type: passthrough for a MuJoCo POSITION_SERVO actuator (it already closes the loop),
         *        or our own PD law otherwise. Caller must already hold `_renderMutex`.
         */
        double
        position_tracking_ctrl(int i) const;
        
        /**
         * @brief Scans every sensor defined in the model for `<force>`/`<torque>` types, groups them
         *        by the site they're attached to, and creates one WrenchStamped publisher per site
         *        (named after the site). Any other sensor type is currently ignored. Called once
         *        from the constructor, after the model has been loaded.
         */
        void
        discover_wrench_sensors();
        
        /**
         * @brief Callback function to handle incoming joint commands. Validates the message, then
         *        forwards it to `set_target()`.
         * @param msg The message containing joint commands.
         */
        void
        joint_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

#endif

####################################################################################################
#                                         PROJECT INFORMATION                                      #
####################################################################################################
cmake_minimum_required(VERSION 3.5)
project(mujoco_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

####################################################################################################
#                                          DEPENDENCIES                                            #
####################################################################################################
find_package(ament_cmake REQUIRED)                                                                  # So ROS2 can compile?
find_package(geometry_msgs REQUIRED)                                                                # geometry_msgs::msg::WrenchStamped
find_package(glfw3 REQUIRED)                                                                        # Graphics Library FrameWork
find_package(rclcpp REQUIRED)                                                                       # ROS2 C++ libraries
find_package(sensor_msgs REQUIRED)                                                                  # sensor_msgs::msg::JointState
find_package(std_msgs REQUIRED)                                                                     # std_msgs::msg::Float64MultiArray
find_package(Threads REQUIRED)                                                                      # std::thread (physics/render/ROS spin run on separate threads)

# MuJoCo
set(MUJOCO_PATH "/opt/mujoco/mujoco-3.2.0") # UPDATE THIS TO YOUR MUJOCO PATH
include_directories(include)                                                                        # Local header files
include_directories(${MUJOCO_PATH}/include/mujoco)                                                  # MuJoCo header files
include_directories(${MUJOCO_PATH}/include)                                                         # Additional MuJoCo header files
link_directories(${MUJOCO_PATH}/lib)                                                                # Location of MuJoCo libraries

# Dear ImGui: the GUI overlay (joint sliders, hide/show tab). It ships no prebuilt binaries, so we
# pull the source at configure time and compile it directly into our executable below. Pinned to a
# specific stable release tag -- bump deliberately, don't track a moving branch.
include(FetchContent)
FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG        v1.92.2
)
FetchContent_MakeAvailable(imgui)

set(IMGUI_SOURCES
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

####################################################################################################
#                                           EXECUTABLES                                            #
####################################################################################################
add_executable(mujoco_node src/nodes/mujoco_node.cpp
                           src/mujoco_ros.cpp
                           ${IMGUI_SOURCES})                                                         # Define new executable, list source files

target_include_directories(mujoco_node PRIVATE ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)    # So <imgui.h> and <backends/imgui_impl_*.h> resolve

target_link_libraries(mujoco_node mujoco glfw Threads::Threads)                                     # Link GLFW and threading libraries to the executable
                                     
ament_target_dependencies(mujoco_node rclcpp sensor_msgs std_msgs geometry_msgs)                    # List ROS2 dependencies for this executable


####################################################################################################
#                                           INSTALLATION                                           #
####################################################################################################

install(TARGETS mujoco_node
        DESTINATION lib/${PROJECT_NAME})                                                            # Install so ROS2 can find it

install(DIRECTORY launch
                  model
        DESTINATION share/${PROJECT_NAME})                                                          # Install so ROS2 can find them
        
ament_package()

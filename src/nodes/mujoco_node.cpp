/**
 * @file   mujoco_node.cpp
 * @author Jon Woolfrey
 * @email  jonathan.woolfrey@gmail.com
 * @date   April 2025
 * @version 1.2
 * @brief  Starts ROS2 and runs the MuJoCoNode.
 * 
 * @details This contains the main() function for the C++ executable.
 *          Its purpose is to start ROS2 and an instance of the MuJoCoNode class.
 *          The node itself runs across three threads: physics (this file starts it),
 *          ROS executor spin (this file starts it), and rendering (runs on this,
 *          the main, thread -- required since GLFW's OpenGL context must stay on
 *          the thread that created the window).
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://mujoco.org/ for more information about MuJoCo
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation
 */
#include <mujoco_ros2/mujoco_ros.hpp>
#include <iostream>
#include <thread>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                                                                       // Starts up ROS2
    
    if(argc < 2)
    {
        throw std::invalid_argument("[ERROR] Invalid number of arguments. Usage: mujoco_node path/to/scene.xml");                                  
    }
   
    std::string xmlPath = argv[1];
    
    try
    {
        auto mujocoNode = std::make_shared<MuJoCoROS>(xmlPath);                                     // Constructed on the main thread -- this is what creates the GLFW window
    
        std::thread physicsThread(&MuJoCoROS::physics_loop, mujocoNode);                            // Steps the simulation and publishes joint state
        std::thread rosThread([mujocoNode]() { rclcpp::spin(mujocoNode); });                        // Services the joint command subscription
        
        mujocoNode->render_loop();                                                                  // Blocks here, on the main thread, until the window closes or shutdown is requested
        
        rclcpp::shutdown();                                                                         // Unblocks rclcpp::spin() on rosThread
        
        physicsThread.join();
        rosThread.join();
        
        return 0; 
    }
    catch(const std::exception &exception)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), exception.what());
        
        rclcpp::shutdown();                                                                         // Stop ROS2
        
        return 1;                                                                                   // Flag error
    }
}

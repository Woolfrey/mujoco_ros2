#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    directory = get_package_share_directory('mujoco_ros2')                                          # Gets relative path of mujoco_ros2 package    
    
    xmlScenePath =  os.path.join(directory, 'model', 'scene.xml')
    
    if not os.path.exists(xmlScenePath):
        raise FileNotFoundError(f"Scene file does not exist: {xmlScenePath}.")


    mujoco = Node(
        package    = "mujoco_ros2",
        executable = "mujoco_node",
        output     = "screen",
        arguments  = [xmlScenePath],
        parameters = [   
                        {"joint_state_topic_name" : "joint_state"},
                        {"joint_command_topic_name" : "joint_commands"},
                        {"control_mode" : "VELOCITY"},
                        {"simulation_frequency" : 1000},
                        {"visualisation_frequency" : 20},
                        {"camera_focal_point": [0.0, 0.0, 0.25]},
                        {"camera_distance": 2.5},
                        {"camera_azimuth": 135.0},
                        {"camera_elevation": -20.0},
                        {"camera_orthographic": True}
                     ]
    )

    return LaunchDescription([mujoco])

#!/usr/bin/env python3
# Copyright 2021 OROCA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os
import xacro

def generate_launch_description():
    
    # urdf_path = os.path.join(
    #     get_package_share_directory('surgical_robot_control'),
    #     'urdf',
    #     'my_robot.urdf.xacro'
    #     )
    
    # urdf_path = LaunchConfiguration('xacro_path', default=None)
    

    return LaunchDescription([
        
        # DeclareLaunchArgument(
        #   'xacro_path',
        #   default_value=None,
        #   description='path to urdf.xacro file to publish'  
        # ),
        
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     parameters=[
        #     ],
        #     output='screen',
        # ),
        
        Node(
            package='surgical_robot_control',
            executable='surgical_robot_control_node',
            name='surgical_robot_control_node',
            output='screen',
        ),
    ])


# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription 
# from launch.actions import LogInfo
# from launch.substitutions import ThisLaunchFileDir
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node


# def generate_launch_description():
#   return LaunchDescription([
#     Node(
#       package='surgical_robot_control',
#       excutable='surgical_robot_control_node',
#       name='surgical_robot_control_node',
#       output='screen',
#       # parameters=[
#       #   {'num_of_tools':'1'},
#       # ],
#     ),
#   ])
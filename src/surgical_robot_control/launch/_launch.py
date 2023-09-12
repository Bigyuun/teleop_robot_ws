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
from launch.substitutions import LaunchConfiguration, Command, ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os
import xacro
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    
    pkg_share = FindPackageShare('surgical_robot_control').find('surgical_robot_control')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    xacro_file = os.path.join(urdf_dir, 'surgical_tool.urdf.xacro')
    robot_desc = launch.substitutions.Command('xacro %s' % xacro_file)
    
    return LaunchDescription([
    
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
               {'robot_description':robot_desc}
            ],
            output='screen',
        ),
        
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='surgical_tool_joint_gui',
        ),
        
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen'),
        
        Node(
            package='surgical_robot_control',
            executable='surgical_robot_control_node',
            name='surgical_robot_control_node',
            output='screen',
        ),
    ])

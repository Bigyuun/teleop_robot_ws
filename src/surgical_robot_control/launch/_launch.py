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
    
    urdf_file = os.path.join(
        get_package_share_directory('surgical_robot_control'),
        'urdf',
        # 'test.urdf'
        'surgical_tool.urdf'
    )
    with open(urdf_file, 'r') as infp:
        robot_description_file = infp.read()
    
    pkg_share = FindPackageShare('surgical_robot_control').find('surgical_robot_control')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    
    xacro_file = os.path.join(urdf_dir, 'surgical_tool.urdf.xacro')
    robot_desc = launch.substitutions.Command('xacro %s' % xacro_file)
    
    xacro_file = os.path.join(urdf_dir, 'surgical_tool_left.urdf.xacro')
    surgical_tool_left_robot_desc = launch.substitutions.Command('xacro %s' % xacro_file)
    xacro_file = os.path.join(urdf_dir, 'surgical_tool_right.urdf.xacro')
    surgical_tool_right_robot_desc = launch.substitutions.Command('xacro %s' % xacro_file)
    
    rviz_packages_dir = get_package_share_directory('surgical_robot_control')

    return LaunchDescription([
      
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='left',
            name='robot_state_publisher',
            parameters=[
                {'use_sim_time': True},
                {'robot_description':surgical_tool_left_robot_desc}
            ],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='right',
            name='robot_state_publisher',
            parameters=[
                {'use_sim_time': True},
                {'robot_description':surgical_tool_right_robot_desc}
            ],
            output='screen',
        ),
        
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     namespace='left',
        #     name='surgical_tool_joint_gui',
        #     output='screen'
        # ),
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     namespace='right',
        #     name='surgical_tool_joint_gui',
        #     output='screen'
        # ),
        
        # Node(
        #     package='surgical_robot_control',
        #     executable='broadcaster',
        #     name='broadcaster',
        #     output='screen',
        # ),
        
        Node(
            package='surgical_robot_control',
            executable='surgical_robot_control_node',
            name='surgical_robot_control_node',
            output='screen',
        ),
        
        Node(
            package='surgical_robot_control',
            executable='joint_state_publisher',
            name='surgical_tool_joint_state_publisher',
            output='screen',
        ),
    ])

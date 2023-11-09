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

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  
  teleop_dir = get_package_share_directory('launch_pkg')
  launch_dir = os.path.join(teleop_dir, 'launch')
  
  launch_description = LaunchDescription()
  rviz_packages_dir = get_package_share_directory('surgical_robot_control')
  
  return LaunchDescription([

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        [get_package_share_directory('loadcell_pkg'), '/launch/_launch.py']),
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        [get_package_share_directory('surgical_robot_control'), '/launch/_launch.py'])
    ),
    
    IncludeLaunchDescription(      
      PythonLaunchDescriptionSource(
        [get_package_share_directory('tcp_pkg'), '/launch/demo.launch.py'])
    ),
    
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        [get_package_share_directory('teleop_twist_joy'), '/launch/teleop-launch.py']),
    ),
    
    Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      output='screen',
      arguments=['-d', os.path.join(rviz_packages_dir, 'rviz', 'rviz_env.rviz')]
    ),
  ])

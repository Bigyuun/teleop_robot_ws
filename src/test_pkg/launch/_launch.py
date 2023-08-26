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
#       package='test_pkg',
#       excutable='test_node',
#       name='test_node',
#       output='screen',
#       # parameters=[
#       #   {'num_of_tools':'1'},
#       # ],
#     ),
#   ])
  
  
  
  
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test_pkg',
            executable='test_node',
            name='test_node',
            output='screen',
        ),
    ])


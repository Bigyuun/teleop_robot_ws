// Copyright 2021 OROCA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// #ifndef BROADCASTER_2_HPP_
// #define BROADCASTER_2_HPP_

#include <cmath>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "custom_interfaces/msg/loadcell_state.hpp"
#include "custom_interfaces/msg/motor_state.hpp"
#include "hw_definition.hpp"

class ContinuumManipulator : public rclcpp::Node
{
public:
  ContinuumManipulator();

private:
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<geometry_msgs::msg::TransformStamped> tf_stamped_list_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr move_service_server_;

  geometry_msgs::msg::Twist surgical_tool_pose_left_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr surgical_tool_pose_left_subscriber_;

  geometry_msgs::msg::Twist surgical_tool_pose_right_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr surgical_tool_pose_right_subscriber_;

  double scale_of_unit_ = 1000;
  bool move_flag_;
};
// #endif  // BROADCASTER_HPP_
// [출처] 051 TF (오픈소스 소프트웨어 & 하드웨어: 로봇 기술 공유 카페 (오로카)) | 작성자 Routiful

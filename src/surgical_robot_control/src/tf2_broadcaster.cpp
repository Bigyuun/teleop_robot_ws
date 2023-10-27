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

#include <memory>

#include "tf2_broadcaster.hpp"
#include "hw_definition.hpp"

#define SINE_TEST 0

using namespace std::chrono_literals;

ContinuumManipulator::ContinuumManipulator()
: rclcpp::Node("continuum_manipulator"),
  move_flag_(true)
{
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = this->get_parameter("qos_depth", qos_depth);
  const auto QoS_RKL10V =
  rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  RCLCPP_INFO(this->get_logger(), "Move Coninuum Manipulator!");
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  auto set_move =
    [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void
    {
      move_flag_ = request->data;

      if (request->data) {
        response->message = "Move Coninuum Manipulator!";
      } else {
        response->message = "Stop Coninuum Manipulator!";
      }
    };
  move_service_server_ = create_service<std_srvs::srv::SetBool>("move", set_move);

  surgical_tool_pose_subscriber_ =
    this->create_subscription<geometry_msgs::msg::Twist>(
      "surgical_tool_pose",
      QoS_RKL10V,
      [this] (const geometry_msgs::msg::Twist::SharedPtr msg) -> void
      {
        RCLCPP_WARN_ONCE(this->get_logger(), "Subscribing the topic '/surgical_tool_pose.'");
        surgical_tool_pose_.linear = msg->linear;
        surgical_tool_pose_.angular = msg->angular;
      }
    );

  auto broadcast =
    [this]() -> void
    {
      static double rad = 0.0;

      tf_stamped_list_.clear();

      geometry_msgs::msg::TransformStamped tf_stamped;
      tf2::Quaternion quaternion;

      tf_stamped.header.stamp = this->now();
      tf_stamped.header.frame_id = "world";
      tf_stamped.child_frame_id = "seg_pan_1";
      tf_stamped.transform.translation.x = 0.1 + JOINT_INTERVAL*100/this->scale_of_unit_;
      tf_stamped.transform.translation.y = 0.0;
      tf_stamped.transform.translation.z = 0.0;
      #if SINE_TEST
      quaternion.setRPY(0, 0.3 * sin(rad), 0);
      #else
      quaternion.setRPY(
        0,
        0,
        surgical_tool_pose_.angular.z/NUM_OF_JOINT
      );
      #endif
      tf_stamped.transform.rotation.x = quaternion.x();
      tf_stamped.transform.rotation.y = quaternion.y();
      tf_stamped.transform.rotation.z = quaternion.z();
      tf_stamped.transform.rotation.w = quaternion.w();
      tf_stamped_list_.push_back(tf_stamped);


      tf_stamped.header.stamp = this->now();
      tf_stamped.header.frame_id = "seg_pan_1";
      tf_stamped.child_frame_id = "seg_tilt_1";
      tf_stamped.transform.translation.x = 0.5 * JOINT_INTERVAL*100/this->scale_of_unit_;
      tf_stamped.transform.translation.y = 0.0;
      tf_stamped.transform.translation.z = 0.0;
      #if SINE_TEST
      quaternion.setRPY(0.3 * sin(rad), 0, 0);
      #else
      quaternion.setRPY(
        0,
        surgical_tool_pose_.angular.y/NUM_OF_JOINT,
        0
      );
      #endif
      tf_stamped.transform.rotation.x = quaternion.x();
      tf_stamped.transform.rotation.y = quaternion.y();
      tf_stamped.transform.rotation.z = quaternion.z();
      tf_stamped.transform.rotation.w = quaternion.w();
      tf_stamped_list_.push_back(tf_stamped);


      tf_stamped.header.frame_id = "seg_tilt_1";
      tf_stamped.child_frame_id = "seg_pan_2";
      tf_stamped.transform.translation.x = 0.5 * JOINT_INTERVAL*100/this->scale_of_unit_;
      tf_stamped.transform.translation.y = 0.0;
      tf_stamped.transform.translation.z = 0.0;
      #if SINE_TEST
      quaternion.setRPY(0.3 * sin(rad), 0, 0);
      #else
      quaternion.setRPY(
        0,
        0,
        surgical_tool_pose_.angular.z/NUM_OF_JOINT
      );
      #endif
      tf_stamped.transform.rotation.x = quaternion.x();
      tf_stamped.transform.rotation.y = quaternion.y();
      tf_stamped.transform.rotation.z = quaternion.z();
      tf_stamped.transform.rotation.w = quaternion.w();
      tf_stamped_list_.push_back(tf_stamped);


      tf_stamped.header.frame_id = "seg_pan_2";
      tf_stamped.child_frame_id = "seg_tilt_2";
      tf_stamped.transform.translation.x = 0.5 * JOINT_INTERVAL*100/this->scale_of_unit_;
      tf_stamped.transform.translation.y = 0.0;
      tf_stamped.transform.translation.z = 0.0;

      #if SINE_TEST
      quaternion.setRPY(0.3 * sin(rad), 0, 0);
      #else
      quaternion.setRPY(
        0,
        surgical_tool_pose_.angular.y/NUM_OF_JOINT,
        0
      );
      #endif
      tf_stamped.transform.rotation.x = quaternion.x();
      tf_stamped.transform.rotation.y = quaternion.y();
      tf_stamped.transform.rotation.z = quaternion.z();
      tf_stamped.transform.rotation.w = quaternion.w();
      tf_stamped_list_.push_back(tf_stamped);


      tf_stamped.header.frame_id = "seg_tilt_2";
      tf_stamped.child_frame_id = "seg_pan_3";
      tf_stamped.transform.translation.x = 0.5 * JOINT_INTERVAL*100/this->scale_of_unit_;
      tf_stamped.transform.translation.y = 0.0;
      tf_stamped.transform.translation.z = 0.0;

      #if SINE_TEST
      quaternion.setRPY(0.3 * sin(rad), 0, 0);
      #else
      quaternion.setRPY(
        0,
        0,
        surgical_tool_pose_.angular.z/NUM_OF_JOINT
      );
      #endif
      tf_stamped.transform.rotation.x = quaternion.x();
      tf_stamped.transform.rotation.y = quaternion.y();
      tf_stamped.transform.rotation.z = quaternion.z();
      tf_stamped.transform.rotation.w = quaternion.w();
      tf_stamped_list_.push_back(tf_stamped);


      tf_stamped.header.frame_id = "seg_pan_3";
      tf_stamped.child_frame_id = "seg_tilt_3";
      tf_stamped.transform.translation.x = 0.5 * JOINT_INTERVAL*100/this->scale_of_unit_;
      tf_stamped.transform.translation.y = 0.0;
      tf_stamped.transform.translation.z = 0.0;

      #if SINE_TEST
      quaternion.setRPY(0.3 * sin(rad), 0, 0);
      #else
      quaternion.setRPY(
        0,
        surgical_tool_pose_.angular.y/NUM_OF_JOINT,
        0
      );
      #endif
      tf_stamped.transform.rotation.x = quaternion.x();
      tf_stamped.transform.rotation.y = quaternion.y();
      tf_stamped.transform.rotation.z = quaternion.z();
      tf_stamped.transform.rotation.w = quaternion.w();
      tf_stamped_list_.push_back(tf_stamped);


      tf_stamped.header.frame_id = "seg_tilt_3";
      tf_stamped.child_frame_id = "seg_pan_4";
      tf_stamped.transform.translation.x = 0.5 * JOINT_INTERVAL*100/this->scale_of_unit_;
      tf_stamped.transform.translation.y = 0.0;
      tf_stamped.transform.translation.z = 0.0;

      #if SINE_TEST
      quaternion.setRPY(0.3 * sin(rad), 0, 0);
      #else
      quaternion.setRPY(
        0,
        0,
        surgical_tool_pose_.angular.z/NUM_OF_JOINT
      );
      #endif
      tf_stamped.transform.rotation.x = quaternion.x();
      tf_stamped.transform.rotation.y = quaternion.y();
      tf_stamped.transform.rotation.z = quaternion.z();
      tf_stamped.transform.rotation.w = quaternion.w();
      tf_stamped_list_.push_back(tf_stamped);


      tf_stamped.header.frame_id = "seg_pan_4";
      tf_stamped.child_frame_id = "seg_tilt_4";
      tf_stamped.transform.translation.x = 0.5 * JOINT_INTERVAL*100/this->scale_of_unit_;
      tf_stamped.transform.translation.y = 0.0;
      tf_stamped.transform.translation.z = 0.0;
      #if SINE_TEST
      quaternion.setRPY(0.3 * sin(rad), 0, 0);
      #else
      quaternion.setRPY(
        0,
        surgical_tool_pose_.angular.y/NUM_OF_JOINT,
        0
      );
      #endif
      tf_stamped.transform.rotation.x = quaternion.x();
      tf_stamped.transform.rotation.y = quaternion.y();
      tf_stamped.transform.rotation.z = quaternion.z();
      tf_stamped.transform.rotation.w = quaternion.w();
      tf_stamped_list_.push_back(tf_stamped);

      // broadcast (publishing)
      tf_broadcaster_->sendTransform(tf_stamped_list_);

      if (move_flag_) {
        rad += 0.01;
      }
    };
  timer_ = this->create_wall_timer(10ms, broadcast);
}

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto node = std::make_shared<ContinuumManipulator>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
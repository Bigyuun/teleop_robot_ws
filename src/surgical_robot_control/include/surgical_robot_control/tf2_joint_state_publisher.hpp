#ifndef TF2_JOINT_STATE_PUBLISHER_HPP
#define TF2_JOINT_STATE_PUBLISHER_HPP

#pragma once

#include <chrono>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "hw_definition.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

class JointStatePublisher : public rclcpp::Node
{
public:
  explicit JointStatePublisher(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  void publishall();

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr surgical_tool_joint_state_publisher_;
  sensor_msgs::msg::JointState surgical_tool_joint_state_;

  // rclcpp::WallRate loop_rate_;
  rclcpp::TimerBase::SharedPtr timer_;


  geometry_msgs::msg::Twist surgical_tool_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr surgical_tool_pose_subscriber_;
};

#endif
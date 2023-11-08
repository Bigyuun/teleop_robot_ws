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
  void publish();

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  sensor_msgs::msg::JointState joint_state_;
  // rclcpp::WallRate loop_rate_;
  rclcpp::TimerBase::SharedPtr timer_;


  geometry_msgs::msg::Twist surgical_tool_pose_left_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr surgical_tool_pose_left_subscriber_;

  geometry_msgs::msg::Twist surgical_tool_pose_right_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr surgical_tool_pose_right_subscriber_;

};

#endif
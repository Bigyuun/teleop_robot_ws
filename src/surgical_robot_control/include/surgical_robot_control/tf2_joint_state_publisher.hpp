#ifndef TF2_JOINT_STATE_PUBLISHER_HPP_
#define TF2_JOINT_STATE_PUBLISHER_HPP_

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
  /**
   * @brief Construct a new Joint State Publisher object
   * @param node_options 
   */
  explicit JointStatePublisher(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  /**
   * @brief publish all topic data : joint_state
   * 
   */
  void publishall();

private:
  /**
   * @brief update the surgical tool joint_state
   */
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr surgical_tool_joint_state_publisher_;
  sensor_msgs::msg::JointState surgical_tool_joint_state_;

  // rclcpp::WallRate loop_rate_;
  /**
   * @brief update the joint state on timer frequency
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief data of surgical tool position
   */
  geometry_msgs::msg::Twist surgical_tool_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr surgical_tool_pose_subscriber_;
};

#endif  // TF2_JOINT_STATE_PUBLISHER_HPP_
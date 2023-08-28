#include <stdio.h>
#include <stdlib.h>
#include <future>
#include <string>
#include <thread>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <memory>
#include <vector>
#include <sys/signal.h>
#include <chrono>
#include <memory>
#include <functional>
#include <chrono>
#include <cmath>
#include <signal.h>

// Surgical Tool Class
#include "surgical_tool.hpp"
#include "hw_definition.hpp"

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "custom_interfaces/msg/motor_state.hpp"
#include "custom_interfaces/msg/motor_command.hpp"
#include "tcp_node.hpp"   // using #define NUM_OF_MOTRS

class KinematicsControlNode : public rclcpp::Node
{
public:
  using MotorState = custom_interfaces::msg::MotorState;
  using MotorCommand = custom_interfaces::msg::MotorCommand;

  explicit KinematicsControlNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~KinematicsControlNode();


  SurgicalTool ST_;
  // SurgicalTool STRight_;

  /**
   * @brief E, W, S, N direction check (+1) or (-1)
   * @note  In xbox's left axes, E:-, W:+, S:-, N:+
   */
  float mapping_joystick_to_bending_p();
  float mapping_joystick_to_bending_t();
  float mapping_joystick_to_forceps();
  // float mapping_joystick_to_bending_p(float axes, SurgicalTool ST);
  // float mapping_joystick_to_bending_t(float axes, SurgicalTool ST);
  // float mapping_joystick_to_forceps(float axes, SurgicalTool ST);

  /**
   * @author DY, JKim
   * @def    cal_kinematics
   * @brief  calculate target values(length of wire) from the surgical tool kinematics,
   * @param  actual position, actual_velocity and Controller(e.g. Xbox) input
   * @return target values
  */
  void cal_kinematics();

  float gear_encoder_ratio_conversion(float gear_ratio, int e_channel, int e_resolution);

private:
  void publishall();

  sensor_msgs::msg::Joy joystick_msg_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystic_subscriber_;
  MotorCommand kinematics_control_target_val_;
  rclcpp::Publisher<MotorCommand>::SharedPtr kinematics_control_publisher_;
  MotorState motor_state_;
  rclcpp::Subscription<MotorState>::SharedPtr motor_state_subscriber_;

  /**
   * @author DY
   * @brief kinematic info publisher
   */
  geometry_msgs::msg::Twist surgical_tool_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr surgical_tool_pose_publisher_;
};
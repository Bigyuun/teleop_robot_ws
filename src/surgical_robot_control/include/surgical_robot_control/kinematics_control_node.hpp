#ifndef KINEMATICS_CONTROL_NODE_HPP
#define KINEMATICS_CONTROL_NODE_HPP

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
#include <algorithm>

// Surgical Tool Class
#include "surgical_tool.hpp"
#include "hw_definition.hpp"

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "custom_interfaces/msg/motor_state.hpp"
#include "custom_interfaces/msg/motor_command.hpp"
#include "custom_interfaces/msg/loadcell_state.hpp"
// #include "tcp_node.hpp"   // using #define NUM_OF_MOTRS

typedef enum  {
  kStop = 0,
  kEnable = 1,
  kHoming = 5,
} OpMode;

class KinematicsControlNode : public rclcpp::Node
{
public:
  using MotorState = custom_interfaces::msg::MotorState;
  using MotorCommand = custom_interfaces::msg::MotorCommand;

  /**
   * @brief Construct a new Kinematics Control Node object
   * @param node_options ROS
   */
  explicit KinematicsControlNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Kinematics Control Node object
   */
  virtual ~KinematicsControlNode();

  /**
   * @brief sugical_tool.hpp
   */
  SurgicalTool ST_left_;
  SurgicalTool ST_right_;

  /**
   * @author DY
   * @brief E, W, S, N direction check (+1) or (-1)
   * @note  In xbox's left axes -> E:-, W:+, S:-, N:+
   *        In our definition   -> E:-, W:+, S:+, N:-
   *        mapping joystick data to angle of hardware limitation
   */
  double mapping_joystick_to_bending_p(SurgicalTool ST, float axes);
  double mapping_joystick_to_bending_t(SurgicalTool ST, float axes);
  double mapping_joystick_to_forceps(SurgicalTool ST, float axes);

  /**
   * @author DY, JKim
   * @def    cal_kinematics
   * @brief  calculate target values(length of wire) from the surgical tool kinematics,
   * @param  actual position, actual_velocity and Controller(e.g. Xbox) input
   * @return target values
  */
  void cal_kinematics();

  /**
   * @author DY
   * @brief multiply the ratio of all components
   * @param gear_ratio 
   * @param e_channel 
   * @param e_resolution 
   * @return float 
   */
  double gear_encoder_ratio_conversion(double gear_ratio, int e_channel, int e_resolution);

  /**
   * @brief Homing mode
   * @note Do this when starting the motor system
   */
  int8_t homing();
  void set_position_zero(int axis_num);
  void set_position_zero_all();

private:

  OpMode op_mode_ = kStop;

  /**
   * @author DY
   * @brief At firts, motor homing will be start once;
   */
  std::thread homingthread_;

  /**
   * @brief virtual_position
   */
  int virtual_pos[NUM_OF_MOTORS] = {0,};

  /**
   * @author DY
   * @brief ROS2 objects and arguments
   */
  void publishall();

  /**
   * @author DY
   * @brief joystick subscriber
   */
  sensor_msgs::msg::Joy joystick_msg_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystic_subscriber_;

  /**
   * @author DY
   * @brief target values publisher for operating motors
   */
  MotorCommand kinematics_control_target_val_;
  rclcpp::Publisher<MotorCommand>::SharedPtr kinematics_control_publisher_;

  /**
   * @author DY
   * @brief actual motor status subscriber
   */
  bool motorstate_op_flag_ = false;
  MotorState motor_state_;
  rclcpp::Subscription<MotorState>::SharedPtr motor_state_subscriber_;

  /**
   * @author DY
   * @brief kinematic info publisher
   */
  geometry_msgs::msg::Twist surgical_tool_pose_left_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr surgical_tool_pose_left_publisher_;
  geometry_msgs::msg::Twist surgical_tool_pose_right_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr surgical_tool_pose_right_publisher_;

  /**
   * @author DY
   * @brief Loadcell data subscriber
   */
  bool loadcell_op_flag_ = false;
  custom_interfaces::msg::LoadcellState loadcell_data_;
  rclcpp::Subscription<custom_interfaces::msg::LoadcellState>::SharedPtr loadcell_data_subscriber_;
};

#endif
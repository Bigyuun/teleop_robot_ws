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

// ROS2
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "custom_interfaces/msg/motor_state.hpp"
#include "custom_interfaces/msg/motor_command.hpp"
#include "tcp_node.hpp"   // using #define NUM_OF_MOTRS

class KinematicsControlNode final : public rclcpp::Node
{
public:
  using MotorState = custom_interfaces::msg::MotorState;
  using MotorCommand = custom_interfaces::msg::MotorCommand;
  explicit KinematicsControlNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~KinematicsControlNode();

  /**
   * @author DY, JKim
   * @def    cal_kinematics
   * @brief  calculate target values(velocity) from the surgical tool kinematics,
   * @param  actual position, actual_velocity and Controller(e.g. Xbox) input
   * @return target values
  */
  SurgicalTool ST;
  // SurgicalTool STRight_;

  // std::vector<int32_t> cal_kinematics(std::vector<int32_t> actual_pos[], std::vector<int32_t> actual_vel[]);
  MotorCommand cal_kinematics(MotorState state, sensor_msgs::msg::Joy controller_input);
  void gear_encoder_ratio_conversion();

private:
  void publishall();

  sensor_msgs::msg::Joy joystick_msg_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystic_subscriber_;
  MotorCommand kinematics_control_target_val_;
  rclcpp::Publisher<MotorCommand>::SharedPtr kinematics_control_publisher_;
  MotorState motor_state_;
  rclcpp::Subscription<MotorState>::SharedPtr motor_state_subscriber_;





};
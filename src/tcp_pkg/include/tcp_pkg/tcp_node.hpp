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
#include <signal.h>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

// DY
#define DEFAULT_IP "172.16.1.0"
#define DEFAULT_PORT 7777
#define DEFAULT_TCP_BUFFER_SIZE 512

#define NUM_OF_MOTORS 5

#define KEYBOARD_INPUT_MODE 0
#define SINEWAVE_TEST 1  // setting mode : 0-non sine wave / 1-sine wave

class TCPClientNode final : public rclcpp::Node  // keyword 'final' prevents further inheritance
{
public:
  explicit TCPClientNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~TCPClientNode(); // Keyword 'override' tell compiler that this inherited function must be implemented
  
private:
  /***************************
   * @author DY
   * @brief  tcp ip elements
  **************************/
  uint8_t Initialize();
  uint8_t TCPconfiguration();
  void SendThread();
  void RecvThread();
  
  std::string ip_ = DEFAULT_IP;
  std::string s_port_;
  uint32_t port_ = DEFAULT_PORT;

  int client_socket_;
  struct sockaddr_in server_addr_;
  char send_msg_[DEFAULT_TCP_BUFFER_SIZE] = {0,};
  char recv_msg_[DEFAULT_TCP_BUFFER_SIZE] = {0,};

  uint32_t buffer_size_;
  std::thread send_thread_;
  std::thread recv_thread_;

  int send_strlen_;
  int recv_strlen_;

  /**************************
   * @author DY
   * @brief  ROS2 elements
  **************************/
  sensor_msgs::msg::Joy joystick_msg_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystic_subscriber_;

  std_msgs::msg::Int32MultiArray tcp_read_msg_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr tcp_publisher_;
  std_msgs::msg::Int32MultiArray tcp_send_msg_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr tcp_subscriber_;
  
};




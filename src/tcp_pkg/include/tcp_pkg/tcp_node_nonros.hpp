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
#include <math.h>
#include <signal.h>

/**
 * @author DY
 * @brief Declare TCP network Setting
 */
#define DEFAULT_IP "172.16.1.0"
#define DEFAULT_PORT 7777
#define DEFAULT_TCP_BUFFER_SIZE 256

/**
 * @author DY
 * @brief Setting Motor
 */
#define NUM_OF_MOTORS 5
#define SINEWAVE_TEST 0  // setting mode : 0-non sine wave / 1-sine wave
#define TCP_SHOW 0       // 0 : none / 1-show send & recv msg of devices

class TCPClientNode
{
public:

  explicit TCPClientNode();
  virtual ~TCPClientNode();
  
private:
  /***************************
   * @author DY
   * @brief  tcp ip elements
  **************************/

  /**
   * @author DY
   * @brief Setting IP and Port number for TCP connection.
   * @return uint8_t success=0, fail=-1
   */
  uint8_t Initialize();

  /**
   * @author DY
   * @brief Try connection using declared IP and Port number.
   * @return uint8_t 
   */
  uint8_t TCPconfiguration();

  /**
   * @brief entering the thread loop
   * @note 1. receive data from device
   *       2. send data to device
   *       3. iteration 1 & 2
   */
  void CommThread();

  /**
   * @author DY
   * @brief sending data (target values of motors)
   * @protocol  4byte array
   *            * [#1 target val, #2 target val, #3, ... #N target val] 
   *            * Motor status = {actual_position, actual_velocity}
   * @param SINEWAVE_TEST 0 - using input data from another device
   *                      1 - not using input data from another device
   * @param send_val   type : int32_t(4 byte) array
   * @note  MasterMACS use only 4byte data array on TCP IP network
   */
  void sendmsg();

  /**
   * @author DY
   * @brief receive data
   * @protocol  4byte array
   *            * [#1 Motor status, #2 Motor Status, #3, ... #N Motor Status] 
   *            * Motor status = {actual_position, actual_velocity}
   * @note  MasterMACS use only 4byte data array on TCP IP network
   */
  void recvmsg();

  std::string ip_ = DEFAULT_IP;
  std::string s_port_;
  uint32_t port_ = DEFAULT_PORT;
  uint32_t buffer_size_;

  int client_socket_;
  struct sockaddr_in server_addr_;
  char send_msg_[DEFAULT_TCP_BUFFER_SIZE] = {0,};
  char recv_msg_[DEFAULT_TCP_BUFFER_SIZE] = {0,};
  int send_strlen_;
  int recv_strlen_;

  std::thread commthread_;

  /**************************
   * @author DY
   * @brief  ROS2 elements
  **************************/
  // void publishall();
  // sensor_msgs::msg::Joy joystick_msg_;
  // rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystic_subscriber_;

  // MotorState tcp_read_msg_;
  // rclcpp::Publisher<MotorState>::SharedPtr tcp_publisher_;
  // MotorCommand tcp_send_msg_;
  // rclcpp::Subscription<MotorCommand>::SharedPtr tcp_subscriber_;

  // // std_msgs::msg::Int32MultiArray tcp_read_msg_;
  // // rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr tcp_publisher_;
  // // std_msgs::msg::Int32MultiArray tcp_send_msg_;
  // // rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr tcp_subscriber_;
};




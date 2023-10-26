#include "tcp_node.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

TCPClientNode::TCPClientNode(const rclcpp::NodeOptions & node_options)
: Node("TCPClientNode", node_options)
{
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = this->get_parameter("qos_depth", qos_depth);

  const auto QoS_RKL10V =
  rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  tcp_read_msg_.actual_position.resize(NUM_OF_MOTORS); // pos & vel
  tcp_read_msg_.actual_velocity.resize(NUM_OF_MOTORS); // pos & vel
  tcp_read_msg_.actual_acceleration.resize(NUM_OF_MOTORS); // pos & vel
  tcp_read_msg_.actual_torque.resize(NUM_OF_MOTORS); // pos & vel
  tcp_publisher_ =
    this->create_publisher<MotorState>("motor_state", QoS_RKL10V);

  tcp_send_msg_.target_position.resize(NUM_OF_MOTORS);
  tcp_send_msg_.target_velocity_profile.resize(NUM_OF_MOTORS);
  tcp_subscriber_ =
    this->create_subscription<MotorCommand>(
      "kinematics_control_target_val",
      QoS_RKL10V,
      [this] (const MotorCommand::SharedPtr msg) -> void
      {
        tcp_send_msg_.stamp = msg->stamp;
        tcp_send_msg_.target_position = msg->target_position;
        tcp_send_msg_.target_velocity_profile = msg->target_velocity_profile;
      }
    );

  this->commthread_ = std::thread(&TCPClientNode::CommThread, this);
}


TCPClientNode::~TCPClientNode()
{
  char msg[] = "";
  send(this->client_socket_, msg, sizeof(msg), 0);
  close(this->client_socket_);

  commthread_.join();
}





uint8_t TCPClientNode::Initialize()
{
  return this->TCPconfiguration();
}

// ***********************
// TCP configuration
// ***********************
uint8_t TCPClientNode::TCPconfiguration() {

  this->buffer_size_ = DEFAULT_TCP_BUFFER_SIZE;
  this->client_socket_ = socket(PF_INET, SOCK_STREAM, 0);
  if (this->client_socket_ == -1) {printf("socket() error.");}

  memset(&this->server_addr_, 0, sizeof(this->server_addr_));
  this->server_addr_.sin_family = AF_INET;
  this->server_addr_.sin_addr.s_addr = inet_addr(this->ip_.c_str());
  this->server_addr_.sin_port = htons(this->port_);
  // this->server_addr_.sin_port = htons(atoi(argc[2]));
  RCLCPP_INFO(this->get_logger(), "Connecting to server... ");
  std::cout << this->server_addr_.sin_family << '/' << this->server_addr_.sin_addr.s_addr << '/' << this->server_addr_.sin_port << std::endl;

  int client_connect = connect(this->client_socket_, (struct sockaddr*)&this->server_addr_, sizeof(this->server_addr_));
  if (client_connect == -1) {
    std::cout << "connect() error." << std::endl;
    return -1;
  }
  else {
    std::cout << "connect() OK" << std::endl;
    return 0;
  }
}



void TCPClientNode::CommThread() {
  if (this->Initialize()) {
    std::cout << "[TCPClientNode] Init Error." << std::endl;
    return;
  }
  RCLCPP_INFO(this->get_logger(), "TCP communication Thread is onfigure");

  while(rclcpp::ok()) {
  // while(true) {
    this->recvmsg();
    this->sendmsg();
  }
}

// ************************
// Send
// ************************
void TCPClientNode::sendmsg()
{

#if SINEWAVE_TEST
  int32_t send_val[this->buffer_size_];
  static uint64_t counter = 0;
  for(int i=0; i<NUM_OF_MOTORS; i++){
      // send_val[i] = 10*sin(counter*0.002);
      memcpy(this->send_msg_ + i*sizeof(send_val[i]), &send_val[i], sizeof(send_val[i]));
  }
  this->send_strlen_ = send(this->client_socket_, this->send_msg_, this->buffer_size_, 0);
  // this->send_strlen_ = write(this->client_socket_, this->send_msg_, this->buffer_size_);
  #if TCP_SHOW
  for(int i=0; i<NUM_OF_MOTORS; i++) {
    std::cout << "[SEND] #" << i << " | val : " << send_val[i] << std::endl;
  }
  #endif
  counter++;
#else
  /**
   * @author DY
   * @brief struct에 접근하여 input으로 던져줄 것.
   *        ROS2 topic의 경우 배열로 던지고 받을 것.
  */
  int32_t send_val[this->buffer_size_];
  for(int i=0; i<NUM_OF_MOTORS; i++){
    send_val[i] = this->tcp_send_msg_.target_position[i];
    send_val[NUM_OF_MOTORS + i] = this->tcp_send_msg_.target_velocity_profile[i];
    memcpy(this->send_msg_ + i*sizeof(send_val[i]), &send_val[i], sizeof(send_val[i]));
    memcpy(this->send_msg_ + (i+NUM_OF_MOTORS)*sizeof(send_val[i+NUM_OF_MOTORS]), &send_val[i+NUM_OF_MOTORS], sizeof(send_val[i+NUM_OF_MOTORS]));
  }
  this->send_strlen_ = send(this->client_socket_, this->send_msg_, this->buffer_size_, 0);
  // this->send_strlen_ = write(this->client_socket_, this->send_msg_, this->buffer_size_);
#endif
}

// ************************
// Receive
// ************************
void TCPClientNode::recvmsg()
{
  int32_t recv_val[this->buffer_size_];
  this->recv_strlen_ = recv(this->client_socket_, this->recv_msg_, this->buffer_size_, 0);
  // this->recv_strlen_ = read(this->client_socket_, this->recv_msg_, this->buffer_size_);
  if (this->recv_strlen_ == -1) {
    std::cout << "read() error." << std::endl;
  }

  // EOF -> Reconnection to Server
  if (this->recv_strlen_ == 0) {  // disconnetion
    RCLCPP_WARN(this->get_logger(), "EOF from Server. Try to reconnect");
    char msg[] = "";
    send(this->client_socket_, msg, sizeof(msg), 0);
    close(this->client_socket_);

    if (this->Initialize()) {
      std::cout << "[TCPClientNode] Init Error." << std::endl;
      return;
    }
    RCLCPP_INFO(this->get_logger(), "TCP communication Thread is onfigure");
  }

  memcpy(recv_val, this->recv_msg_, sizeof(this->recv_msg_));

  /**
   * @author Bigyuun
   * @brief  if use Linux-Windows Format, it will may need endian conversion.
  */
#if TCP_SHOW
  for(int i=0; i<NUM_OF_MOTORS; i++){
      // std::cout << "[READ] #" << i << "| pos : " << htole16(recv_val[i*2]) << " / vel : " << htole16(recv_val[2*i + 1]) << std::endl;
      std::cout << "[READ] #" << i << "| pos : " << recv_val[i*2] << " / vel : " << recv_val[2*i + 1] << std::endl;
  }
#endif

  for(int i=0; i<NUM_OF_MOTORS; i++) {
    tcp_read_msg_.stamp = this->now();
    tcp_read_msg_.actual_position[i] = recv_val[2*i];
    tcp_read_msg_.actual_velocity[i] = recv_val[2*i+1];
  }
  publishall();
}

void TCPClientNode::publishall() {
  tcp_publisher_->publish(tcp_read_msg_);
}



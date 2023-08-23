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

  joystic_subscriber_ =
    this->create_subscription<sensor_msgs::msg::Joy>(
      "joy",
      QoS_RKL10V,
      [this] (const sensor_msgs::msg::Joy::SharedPtr msg) -> void
      {
        joystick_msg_.header = msg->header;
        joystick_msg_.axes = msg->axes;
        joystick_msg_.buttons = msg->buttons;

        // RCLCPP_INFO(
        //   this->get_logger(),
        //   "Header of the message : %ld, %ld",
        //   msg->header,
        //   msg->header);

        // std::cout << joystick_msg_.axes << std::endl;
        // std::cout << joystick_msg_.buttons << std::endl;
      }
    );

  tcp_read_msg_.actual_position.resize(NUM_OF_MOTORS); // pos & vel
  tcp_read_msg_.actual_velocity.resize(NUM_OF_MOTORS); // pos & vel
  tcp_read_msg_.actual_acceleration.resize(NUM_OF_MOTORS); // pos & vel
  tcp_read_msg_.actual_torque.resize(NUM_OF_MOTORS); // pos & vel
  tcp_publisher_ =
    this->create_publisher<MotorState>("motor_state", QoS_RKL10V);

  tcp_send_msg_.target_val.resize(NUM_OF_MOTORS);
  tcp_subscriber_ =
    this->create_subscription<MotorCommand>(
      "kinematics_control_target_val",
      QoS_RKL10V,
      [this] (const MotorCommand::SharedPtr msg) -> void
      {
        tcp_send_msg_.stamp = msg->stamp;
        tcp_send_msg_.target_val = msg->target_val;
      }
    );

  if (this->Initialize()) {
    std::cout << "[TCPClientNode] Init Error." << std::endl;
    return;
  }

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
  // ***********************
  // 1 - ip setting
  // ***********************
  while(true){
    std::cout << "[Command] Enter IP address. Enter is using dafault (default = " << DEFAULT_IP << ") : ";
    std::getline(std::cin, this->ip_);
    static uint8_t cnt = 0;

    // set default
    if(this->ip_ == "") {this->ip_ = DEFAULT_IP; std::cout << this->ip_ << std::endl; break;}

    // set manual
    for(int i=0; i<this->ip_.length(); i++) { if(this->ip_[i] == '.') cnt++; }
    if(cnt != 3) std::cout << "[ERROR] Check your ip address (3 .)" << std::endl;
    else {break;}
  }

  // ***********************
  // 2 - port setting
  // ***********************
  while(true){
    std::cout << "[Command] Enter Port. Enter is using dafault (default = " << DEFAULT_PORT << ") : ";
    std::getline(std::cin, this->s_port_);
    uint8_t tf = 0;

    // set default
    if(this->s_port_=="") {std::cout << this->s_port_ << std::endl; break;}

    // set manual
    for(int i=0; i<this->s_port_.length(); i++)
    {
      tf = isdigit(this->s_port_[i]);
      if(tf==0){
        std::cout << "[ERROR] Check your Port Number (int)" << std::endl;
        break;
      }
    }
    if(tf) {
      this->port_ = std::stoi(this->s_port_);
      break;
    }
  }
  std::cout << "[INFO] Set is done -> " << "IP = " << this->ip_ << " port = " << this->port_ << std::endl;

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
  int send_val[this->buffer_size_];
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
  long send_val[this->buffer_size_];
  for(int i=0; i<NUM_OF_MOTORS; i++){
    send_val[i] = this->tcp_send_msg_.data[i];
    memcpy(this->send_msg_ + i*sizeof(send_val[i]), &send_val[i], sizeof(send_val[i]));
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
  int recv_val[this->buffer_size_];

  // system("clear");  // clear every time
  this->send_strlen_ = recv(this->client_socket_, this->recv_msg_, this->buffer_size_, 0);
  // this->send_strlen_ = read(this->client_socket_, this->recv_msg_, this->buffer_size_);
  if (this->send_strlen_ == -1) {
    std::cout << "read() error." << std::endl;
  }

  if (this->send_strlen_ == 0) {  // disconnetion
    RCLCPP_WARN(this->get_logger(), "EOF from Server. Try to reconnect");
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
  // testint32_publisher_->publish(testint32_);
  // RCLCPP_INFO(this->get_logger(), "Published");
}



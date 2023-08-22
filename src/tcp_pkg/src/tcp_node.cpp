

#include "tcp_node.hpp"

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

        RCLCPP_INFO(
          this->get_logger(),
          "Header of the message : %ld, %ld",
          msg->header,
          msg->header);

        // std::cout << joystick_msg_.axes << std::endl;
        // std::cout << joystick_msg_.buttons << std::endl;
      }
    );

  tcp_publisher_ =
    this->create_publisher<std_msgs::msg::Int32MultiArray>("tcp_receiver", QoS_RKL10V);

  tcp_subscriber_ =
    this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "tcp_sender",
      QoS_RKL10V,
      [this] (const std_msgs::msg::Int32MultiArray::SharedPtr msg) -> void
      {
        tcp_send_msg_.layout = msg->layout;
        tcp_send_msg_.data = msg->data;
        RCLCPP_INFO(
          this->get_logger(),
          "TCP send msg(target val) : %ld",
          tcp_send_msg_.data);
      }
    );

  if (this->Initialize()) {
    std::cout << "[TCPClientNode] Init Error." << std::endl;
    return;
  }

  this->send_thread_ = std::thread(&TCPClientNode::SendThread, this);
  this->recv_thread_ = std::thread(&TCPClientNode::RecvThread, this);
  std::cout << "[TCPClientNode] Threads (Send, Recv) are created." << std::endl;
}


TCPClientNode::~TCPClientNode()
{
  close(this->client_socket_);
  this->send_thread_.join();
  this->recv_thread_.join();
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

void TCPClientNode::SendThread()
{
  std::cout << "TCP Send Thread Start" << std::endl;
  long send_val[this->buffer_size_];

#if SINEWAVE_TEST
  static uint64_t counter = 0;
  // spin
  while (true) {
    for(int i=0; i<NUM_OF_MOTORS; i++){
      send_val[i] = 10*sin(counter*0.01);
      memcpy(this->send_msg_ + i*sizeof(long), &send_val[i], sizeof(send_val[i]));
    }
    write(this->client_socket_, this->send_msg_, this->buffer_size_);
    counter++;
    usleep(1000000);
  }
#else
  // spin
  while (true) {
    /**
     * @author DY
     * @brief struct에 접근하여 input으로 던져줄 것.
     *        ROS2 topic의 경우 배열로 던지고 받을 것.
    */
    for(int i=0; i<NUM_OF_MOTORS; i++){
      send_val[i] = this->tcp_send_msg_.data[i];
      memcpy(this->send_msg_ + i*sizeof(long), &send_val[i], sizeof(send_val[i]));
    }
    write(this->client_socket_, this->send_msg_, this->buffer_size_);
    usleep(1000);
  }
#endif
}

void TCPClientNode::RecvThread()
{
  std::cout << "TCP Receive Thread Start" << std::endl;
  long recv_val[this->buffer_size_];
  static uint32_t counter = 0;

  // spin
  while(true) {
    /**
     * @brief spin
    */
    this->send_strlen_ = read(this->client_socket_, this->recv_msg_, this->buffer_size_);
    if (this->send_strlen_ == -1) {
      std::cout << "[Send Thread] read() error." << std::endl;
      continue;
    }

    if (this->send_strlen_ == 0) {  // disconnetion
      RCLCPP_WARN(this->get_logger(), "EOF from Server. Try to reconnect");
    }

    // Little-Endian
    memcpy(recv_val, this->recv_msg_, this->buffer_size_);

    for(int i=0; i<NUM_OF_MOTORS; i++) {
      tcp_read_msg_.data[2*i] = recv_val[2*i];
      tcp_read_msg_.data[2*i+1] = recv_val[2*i+1];
    }
    tcp_publisher_->publish(tcp_read_msg_);
    // system("cls");  // clear every time
    for(int n=0; n<NUM_OF_MOTORS; n++){
        std::cout << "#" << n << "| pos : " << recv_val[0+n*2] << " / vel : " << recv_val[1+n*2] << " - " << counter++ << std::endl;
    }
    usleep(100);
  }
}
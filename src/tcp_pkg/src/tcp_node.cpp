

#include "tcp_node.hpp"

using namespace std::chrono_literals;



TCPClientNode::TCPClientNode() : rclcpp::Node("TCPClientNode")
{
  auto qos = rclcpp::QoS(
    // The "KEEP_LAST" history setting tells DDS to store a fixed-size buffer of values before they
    // are sent, to aid with recovery in the event of dropped messages.
    // "depth" specifies the size of this buffer.
    // In this example, we are optimizing for performance and limited resource usage (preventing
    // page faults), instead of reliability. Thus, we set the size of the history buffer to 1.
    rclcpp::KeepLast(1)
  );
  // From http://www.opendds.org/qosusages.html: "A RELIABLE setting can potentially block while
  // trying to send." Therefore set the policy to best effort to avoid blocking during execution.
  qos.best_effort();

  // haptic_publisher_ = this->create_publisher<ecat_msgs::msg::HapticCmd>("HapticInput",qos);

  // // JKim - Subscription
  // slave_feedback_ = this->create_subscription<ecat_msgs::msg::DataReceived>("Slave_Feedback", qos,
  //                   std::bind(&HapticNode::HandleSlaveFeedbackCallbacks, this, std::placeholders::_1));

  // // CKim - Launch thread that will constantly read socket and publish data
  // future_ = exit_signal_.get_future();
  // comm_thread_ = std::thread(&HapticNode::commThread, this); 

  if (this->Initialize()) {
    std::cout << "[TCPClientNode] Init Error." << std::endl;
    return;
  }

  this->send_thread_ = std::thread(&TCPClientNode::SendThread, this);
  this->recv_thread_ = std::thread(&TCPClientNode::RecvThread, this);
  std::cout << "[TCPClientNode] Threads (Send, Recv) are created." << std::endl;

  this->send_thread_.join();
  this->recv_thread_.join();
  return;
}


TCPClientNode::~TCPClientNode()
{

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


  // ***********************
  // 3 - TCP configuration
  // ***********************
  this->buffer_size_ = DEFAULT_TCP_BUFFER_SIZE;
  this->client_socket_ = socket(PF_INET, SOCK_STREAM, 0);
  if (this->client_socket_ == -1) {printf("socket() error.");}

  memset(&this->server_addr_, 0, sizeof(this->server_addr_));
  this->server_addr_.sin_family = AF_INET;
  this->server_addr_.sin_addr.s_addr = inet_addr(this->ip_.c_str());
  this->server_addr_.sin_port = this->port_;
  RCLCPP_INFO(this->get_logger(), "Connecting to server... ");

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
    usleep(100);
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
      send_val[i] = this->ros2_input_[i];
      memcpy(this->send_msg_ + i*sizeof(long), &send_val[i], sizeof(send_val[i]));
    }
    write(this->client_socket_, this->send_msg_, this->buffer_size_);
    usleep(100);
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
      break;
    }

    // Little-Endian
    memcpy(recv_val, this->recv_msg_, this->buffer_size_);
    system("cls");  // clear every time
    for(int n=0; n<NUM_OF_MOTORS; n++){
        std::cout << "#" << n << "| pos : " << recv_val[0+n*2] << " / vel : " << recv_val[1+n*2] << " - " << counter++ << std::endl;
    }
    usleep(100);
  }
}



int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TCPClientNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}











// TCPClientNode:TCPClientNode() : rclcpp::Node("TCPClientNode")
// {

//   // DY - set IP and Port as Server
//   m_Port = "9800";
//   // m_Port = argv[1];
  
//   auto qos = rclcpp::QoS(
//     // The "KEEP_LAST" history setting tells DDS to store a fixed-size buffer of values before they
//     // are sent, to aid with recovery in the event of dropped messages.
//     // "depth" specifies the size of this buffer.
//     // In this example, we are optimizing for performance and limited resource usage (preventing
//     // page faults), instead of reliability. Thus, we set the size of the history buffer to 1.
//     rclcpp::KeepLast(1)
//   );
//   // From http://www.opendds.org/qosusages.html: "A RELIABLE setting can potentially block while
//   // trying to send." Therefore set the policy to best effort to avoid blocking during execution.
//   qos.best_effort();
//   // CKim - Initialize publisher
//   haptic_publisher_ = this->create_publisher<ecat_msgs::msg::HapticCmd>("HapticInput",qos);

//   // JKim - Subscription
//   slave_feedback_ = this->create_subscription<ecat_msgs::msg::DataReceived>("Slave_Feedback", qos,
//                     std::bind(&HapticNode::HandleSlaveFeedbackCallbacks, this, std::placeholders::_1));

//   // CKim - Launch thread that will constantly read socket and publish data
//   future_ = exit_signal_.get_future();
//   comm_thread_ = std::thread(&HapticNode::commThread, this);  
// }

// void HapticNode::HandleSlaveFeedbackCallbacks(const ecat_msgs::msg::DataReceived::SharedPtr msg)
//   {
// //      time_info_.GetTime();
//       for(int i=0; i < NUM_OF_SLAVES ; i++){
//         received_data_[i].actual_pos             =  msg->actual_pos[i];
//         received_data_[i].actual_vel             =  msg->actual_vel[i];
//         received_data_[i].status_word            =  msg->status_word[i];
//         received_data_[i].left_limit_switch_val  =  msg->left_limit_switch_val;
//         received_data_[i].right_limit_switch_val =  msg->right_limit_switch_val;
//         received_data_[i].p_emergency_switch_val =  msg->emergency_switch_val;
//         received_data_[i].com_status             =  msg->com_status;

//         // DY analog
//         received_data_[i].analog_input_1         =  msg->analog_input_1[i];
//         received_data_[i].analog_input_2         =  msg->analog_input_2[i];
//     }  
//   }




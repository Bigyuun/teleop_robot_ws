#include "tcp_node_nonros.hpp"

using namespace std::chrono_literals;

/**
 * @author DY
 * @brief Construct a new TCPClientNode::TCPClientNode object
 */
TCPClientNode::TCPClientNode()
{
  if (this->Initialize()) {
    std::cout << "[TCPClientNode] Init Error." << std::endl;
    return;
  }

  this->commthread_ = std::thread(&TCPClientNode::CommThread, this);
}

/**
 * @author DY
 * @brief Destroy the TCPClientNode::TCPClientNode object
 */
TCPClientNode::~TCPClientNode()
{
  char msg[] = "";
  send(this->client_socket_, msg, sizeof(msg), 0);
  close(this->client_socket_);

  commthread_.join();
}

/**
 * @author DY
 * @brief Setting IP and Port number for TCP connection.
 * @return uint8_t success=0, fail=-1
 */
uint8_t TCPClientNode::Initialize()
{
  // ***********************
  // 1 - ip setting
  // ***********************
  while(true){
    std::cout << "[TCPClientNode] Enter IP address. None("") is using dafault (default = " << DEFAULT_IP << ") : ";
    std::getline(std::cin, this->ip_);
    static uint8_t cnt = 0;

    // set default
    if(this->ip_ == "") {this->ip_ = DEFAULT_IP; std::cout << this->ip_ << std::endl; break;}

    // set manual
    for(int i=0; i<this->ip_.length(); i++) { if(this->ip_[i] == '.') cnt++; }
    if(cnt != 3) std::cout << "[TCPClientNode] [ERROR] Check your ip address (3 .)" << std::endl;
    else {break;}
  }

  // ***********************
  // 2 - port setting
  // ***********************
  while(true){
    std::cout << "[TCPClientNode] Enter Port. Enter is using dafault (default = " << DEFAULT_PORT << ") : ";
    std::getline(std::cin, this->s_port_);
    uint8_t tf = 0;

    // set default
    if(this->s_port_=="") {std::cout << this->s_port_ << std::endl; break;}

    // set manual
    for(int i=0; i<this->s_port_.length(); i++)
    {
      tf = isdigit(this->s_port_[i]);
      if(tf==0){
        std::cout << "[TCPClientNode] [ERROR] Check your Port Number (int)" << std::endl;
        break;
      }
    }
    if(tf) {
      this->port_ = std::stoi(this->s_port_);
      break;
    }
  }
  std::cout << "[TCPClientNode] [INFO] Set is done -> " << "IP = " << this->ip_ << " port = " << this->port_ << std::endl;

// ***********************
// TCP configuration
// ***********************
  return this->TCPconfiguration();
}

/**
 * @author DY
 * @brief Try connection using declared IP and Port number.
 * @return uint8_t 
 */
uint8_t TCPClientNode::TCPconfiguration() {

  this->buffer_size_ = DEFAULT_TCP_BUFFER_SIZE;
  this->client_socket_ = socket(PF_INET, SOCK_STREAM, 0);
  if (this->client_socket_ == -1) {printf("[TCPClientNode] socket() error.");}

  memset(&this->server_addr_, 0, sizeof(this->server_addr_));
  this->server_addr_.sin_family = AF_INET;
  this->server_addr_.sin_addr.s_addr = inet_addr(this->ip_.c_str());
  this->server_addr_.sin_port = htons(this->port_);
  // this->server_addr_.sin_port = htons(atoi(argc[2]));
  std::cout << "[TCPClientNode] Connecting to server... " << std::endl;;
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

/**
 * @brief entering the thread loop
 * @note 1. receive from device
 *       2. send to device
 *       3. iteration 1 & 2
 */
void TCPClientNode::CommThread() {
  std::cout << "[TCPClientNode] TCP communication Thread is onfigure" << std::endl;

  while(true) {
  // while(true) {
    this->recvmsg();
    this->sendmsg();
  }
}

/**
 * @author DY
 * @brief sending data (target values of motors)
 * @param SINEWAVE_TEST 0 - using input data from another device
 *                      1 - not using input data from another device
 * @param send_val   type : int32_t(4 byte) array
 * @note  MasterMACS use only 4byte data array on TCP IP network
 */
void TCPClientNode::sendmsg()
{

#if SINEWAVE_TEST
  int send_val[this->buffer_size_];
  static uint64_t counter = 0;
  for(int i=0; i<NUM_OF_MOTORS; i++){
      send_val[i] = 10*sin(counter*0.002);
      memcpy(this->send_msg_ + i*sizeof(send_val[i]), &send_val[i], sizeof(send_val[i]));
  }
  this->send_strlen_ = send(this->client_socket_, this->send_msg_, this->buffer_size_, 0);
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
    send_val[i] = 0;
    memcpy(this->send_msg_ + i*sizeof(send_val[i]), &send_val[i], sizeof(send_val[i]));
  }
  this->send_strlen_ = send(this->client_socket_, this->send_msg_, this->buffer_size_, 0);
#endif
}

/**
 * @author DY
 * @brief receive data
 * @note  MasterMACS use only 4byte data array on TCP IP network
 */
void TCPClientNode::recvmsg()
{
  int recv_val[this->buffer_size_];
  this->send_strlen_ = recv(this->client_socket_, this->recv_msg_, this->buffer_size_, 0);
  if (this->send_strlen_ == -1) {
    std::cout << "[TCPClientNode] read() error." << std::endl;
  }

  if (this->send_strlen_ == 0) {  // disconnetion
    std::cout << "[TCPClientNode] EOF from Server. Try to reconnect" << std::endl;
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
}



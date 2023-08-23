#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <signal.h>

#include "rclcpp/rclcpp.hpp"
#include "tcp_node.hpp"
#include "rcutils/cmdline_parser.h"

void print_help()
{
  printf("For ROS 2 topic subscriber, service server, action server rclcpp examples:\n");
  printf("calculator [-h]\n");
  printf("Options:\n");
  printf("\t-h Help           : Print this help function.\n");
}

void signal_callback_handler (int signum) {
  signal(signum, SIG_IGN);
  printf("Ctrl+C break.\n");  
  exit(1);
}

int main(int argc, char * argv[])
{
  if(rcutils_cli_option_exist(argv, argv + argc, "-h")){
    print_help();
    return 0;
  }

  rclcpp::init(argc, argv);
  signal(SIGINT, signal_callback_handler);
  auto node = std::make_shared<TCPClientNode>();
  std::cout << "ros spin() start" << std::endl;
  rclcpp::spin(node);
  rclcpp::shutdown();

  std::cout << "ROS node Shutdown" << std::endl;
  return 0;
}

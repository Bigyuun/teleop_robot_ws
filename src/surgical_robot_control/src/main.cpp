#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <signal.h>

#include "rclcpp/rclcpp.hpp"
#include "kinematics_control_node.hpp"
#include "rcutils/cmdline_parser.h"

void print_help()
{
  printf("For ROS 2 topic subscriber, service server, action server rclcpp examples:\n");
  printf("calculator [-h]\n");
  printf("Options:\n");
  printf("\t-h Help           : Print this help function.\n");
}

int main(int argc, char * argv[])
{
  if(rcutils_cli_option_exist(argv, argv + argc, "-h")){
    print_help();
    return 0;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<KinematicsControlNode>();
  std::cout << "ros spin() start" << std::endl;
  rclcpp::spin(node);
  rclcpp::shutdown();
  std::cout << "ROS node Shutdown" << std::endl;
  return 0;
}

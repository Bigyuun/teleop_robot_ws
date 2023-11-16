#include <chrono>
#include <string>

#include "tcp_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/motor_command.hpp"
#include "custom_interfaces/msg/motor_state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using MotorState = custom_interfaces::msg::MotorState;
using MotorCommand = custom_interfaces::msg::MotorCommand;

class DemoNode : public rclcpp::Node
{
public:
  DemoNode()
  : Node("demo_node"), count_(0)
  {
    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = this->get_parameter("qos_depth", qos_depth);

    const auto QoS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

    demo_node_publisher_ = this->create_publisher<MotorState>(
      "motor_state",
      QoS_RKL10V);
    timer_ = this->create_wall_timer(
      1ms, std::bind(&DemoNode::publish_msg, this));

    tcp_send_msg_.target_position.resize(NUM_OF_MOTORS);
    tcp_send_msg_.target_velocity_profile.resize(NUM_OF_MOTORS);
    demo_node_subscriber_ = this->create_subscription<MotorCommand>(
      "kinematics_control_target_val",
      QoS_RKL10V,
      [this] (const MotorCommand::SharedPtr msg) -> void
        {
          tcp_send_msg_.stamp = msg->stamp;
          tcp_send_msg_.target_position = msg->target_position;
          tcp_send_msg_.target_velocity_profile = msg->target_velocity_profile;
        }
    );
  }

private:

  void publish_msg()
  {
    auto msg = MotorState();

    msg.actual_position.resize(NUM_OF_MOTORS);
    msg.actual_velocity.resize(NUM_OF_MOTORS);
    msg.actual_acceleration.resize(NUM_OF_MOTORS);
    msg.actual_torque.resize(NUM_OF_MOTORS);
    
    msg.stamp = this->now();
    demo_node_publisher_->publish(msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<MotorState>::SharedPtr demo_node_publisher_;
  MotorCommand tcp_send_msg_;
  rclcpp::Subscription<MotorCommand>::SharedPtr demo_node_subscriber_;
  size_t count_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DemoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}


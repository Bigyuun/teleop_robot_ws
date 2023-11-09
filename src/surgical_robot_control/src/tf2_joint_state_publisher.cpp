// joint_publisher.cpp
#include "tf2_joint_state_publisher.hpp"

JointStatePublisher::JointStatePublisher(const rclcpp::NodeOptions & node_options)
: Node("JointStatePublisherNode", node_options)
{ 
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = this->get_parameter("qos_depth", qos_depth);

  const auto QoS_RKL10V =
  rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  surgical_tool_joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", qos_depth);
  
  surgical_tool_joint_state_.name = {
    "base-actuator",
    "tilt-1", "tilt-2", "tilt-3", "tilt-4", "tilt-5",
    "pan-1", "pan-2", "pan-3", "pan-4", "pan-5",
  };
  // Initialize joint positions to zero
  surgical_tool_joint_state_.position = std::vector<double>(1+2*NUM_OF_JOINT, 0.0);

  surgical_tool_pose_subscriber_ =
    this->create_subscription<geometry_msgs::msg::Twist>(
      "surgical_tool_pose",
      QoS_RKL10V,
      [this] (const geometry_msgs::msg::Twist::SharedPtr msg) -> void
      {
        RCLCPP_WARN_ONCE(this->get_logger(), "Subscribing the topic '/surgical_tool_pose.'");
        surgical_tool_pose_.linear = msg->linear;
        surgical_tool_pose_.angular = msg->angular;
      }
    );
  timer_ = this->create_wall_timer(10ms, std::bind(&JointStatePublisher::publishall, this));
}

void JointStatePublisher::publishall()
{
  // Set joint positions
  surgical_tool_joint_state_.header.stamp = this->now();
  surgical_tool_joint_state_.position[0] = 0;  // Increment joint positions
  for (int i = 1; i < 1+NUM_OF_JOINT; i++) {
      surgical_tool_joint_state_.position[i]                = surgical_tool_pose_.angular.y/NUM_OF_JOINT;  // Increment joint positions
      surgical_tool_joint_state_.position[NUM_OF_JOINT + i] = surgical_tool_pose_.angular.z/NUM_OF_JOINT;  // Increment joint positions
  }

  surgical_tool_joint_state_publisher_->publish(surgical_tool_joint_state_);

  // loop_rate_.sleep();
}

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto node = std::make_shared<JointStatePublisher>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
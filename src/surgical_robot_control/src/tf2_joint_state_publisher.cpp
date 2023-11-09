// joint_publisher.cpp
#include "tf2_joint_state_publisher.hpp"

JointStatePublisher::JointStatePublisher(const rclcpp::NodeOptions & node_options)
: Node("JointStatePublisherNode", node_options)
{ 
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = this->get_parameter("qos_depth", qos_depth);

  const auto QoS_RKL10V =
  rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  surgical_tool_left_publisher_ = create_publisher<sensor_msgs::msg::JointState>("left/joint_states", qos_depth);
  
  surgical_tool_left_joint_state_.name = {
    // "tilt-1", "tilt-2", "tilt-3", "tilt-4", "tilt-5",
    // "pan-1", "pan-2", "pan-3", "pan-4", "pan-5"
    "left/base-actuator",
    "left/tilt-1", "left/tilt-2", "left/tilt-3", "left/tilt-4", "left/tilt-5",
    "left/pan-1", "left/pan-2", "left/pan-3", "left/pan-4", "left/pan-5",
  };
  // Initialize joint positions to zero
  surgical_tool_left_joint_state_.position = std::vector<double>(1+2*NUM_OF_JOINT, 0.0);

  surgical_tool_right_publisher_ = create_publisher<sensor_msgs::msg::JointState>("right/joint_states", qos_depth);
  surgical_tool_right_joint_state_.name = {
    // "tilt-1", "tilt-2", "tilt-3", "tilt-4", "tilt-5",
    // "pan-1", "pan-2", "pan-3", "pan-4", "pan-5"
    "right/base-actuator",
    "right/tilt-1", "right/tilt-2", "right/tilt-3", "right/tilt-4", "right/tilt-5",
    "right/pan-1", "right/pan-2", "right/pan-3", "right/pan-4", "right/pan-5"
  };
  // Initialize joint positions to zero
  surgical_tool_right_joint_state_.position = std::vector<double>(1+2*NUM_OF_JOINT, 0.0);

  surgical_tool_pose_left_subscriber_ =
    this->create_subscription<geometry_msgs::msg::Twist>(
      "surgical_tool_left_pose",
      QoS_RKL10V,
      [this] (const geometry_msgs::msg::Twist::SharedPtr msg) -> void
      {
        RCLCPP_WARN_ONCE(this->get_logger(), "Subscribing the topic '/surgical_tool_pose_left.'");
        surgical_tool_pose_left_.linear = msg->linear;
        surgical_tool_pose_left_.angular = msg->angular;
      }
    );
  surgical_tool_pose_right_subscriber_ =
    this->create_subscription<geometry_msgs::msg::Twist>(
      "surgical_tool_right_pose",
      QoS_RKL10V,
      [this] (const geometry_msgs::msg::Twist::SharedPtr msg) -> void
      {
        RCLCPP_WARN_ONCE(this->get_logger(), "Subscribing the topic '/surgical_tool_pose_right.'");
        surgical_tool_pose_right_.linear = msg->linear;
        surgical_tool_pose_right_.angular = msg->angular;
      }
    );

  timer_ = this->create_wall_timer(10ms, std::bind(&JointStatePublisher::publishall, this));
}

void JointStatePublisher::publishall()
{
  // Set joint positions
  surgical_tool_left_joint_state_.header.stamp = this->now();
  surgical_tool_right_joint_state_.header.stamp = this->now();
  surgical_tool_left_joint_state_.position[0] = 0;  // Increment joint positions
  surgical_tool_right_joint_state_.position[0] = 0;  // Increment joint positions
  for (int i = 1; i < 1+NUM_OF_JOINT; i++) {
      surgical_tool_left_joint_state_.position[i] = surgical_tool_pose_left_.angular.y/NUM_OF_JOINT;  // Increment joint positions
      surgical_tool_right_joint_state_.position[i] = surgical_tool_pose_right_.angular.y/NUM_OF_JOINT;  // Increment joint positions
  }
  for (int i = 1+NUM_OF_JOINT; i < (1+2*NUM_OF_JOINT); i++) {
      surgical_tool_left_joint_state_.position[i] = surgical_tool_pose_left_.angular.z/NUM_OF_JOINT;  // Increment joint positions
      surgical_tool_right_joint_state_.position[i] = surgical_tool_pose_right_.angular.z/NUM_OF_JOINT;  // Increment joint positions
  }

  surgical_tool_left_publisher_->publish(surgical_tool_left_joint_state_);
  surgical_tool_right_publisher_->publish(surgical_tool_right_joint_state_);

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
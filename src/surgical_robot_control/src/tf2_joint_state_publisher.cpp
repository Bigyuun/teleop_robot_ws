// joint_publisher.cpp
#include "tf2_joint_state_publisher.hpp"

JointStatePublisher::JointStatePublisher(const rclcpp::NodeOptions & node_options)
: Node("JointStatePublisherNode", node_options)
{
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = this->get_parameter("qos_depth", qos_depth);

  const auto QoS_RKL10V =
  rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  publisher_ = create_publisher<sensor_msgs::msg::JointState>("left/joint_states", qos_depth);
  
  joint_state_.name = {
    "tilt-1", "tilt-2", "tilt-3", "tilt-4", "tilt-5",
    "pan-1", "pan-2", "pan-3", "pan-4", "pan-5"
    // "left/tilt-1", "left/tilt-2", "left/tilt-3", "left/tilt-4", "left/tilt-5",
    // "left/pan-1", "left/pan-2", "left/pan-3", "left/pan-4", "left/pan-5",
    // "right/tilt-1", "right/tilt-2", "right/tilt-3", "right/tilt-4", "right/tilt-5",
    // "right/pan-1", "right/pan-2", "right/pan-3", "right/pan-4", "right/pan-5"
  };
  // Initialize joint positions to zero
  joint_state_.position = std::vector<double>(20, 0.0);

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

  timer_ = this->create_wall_timer(10ms, std::bind(&JointStatePublisher::publish, this));
}

void JointStatePublisher::publish()
{
  // Set joint positions
  for (int i = 0; i < NUM_OF_JOINT; i++) {
      joint_state_.position[i] = surgical_tool_pose_left_.angular.y/NUM_OF_JOINT * M_PI/180;  // Increment joint positions
  }
  for (int i = 5; i < (5+NUM_OF_JOINT); i++) {
      joint_state_.position[i] = surgical_tool_pose_left_.angular.z/NUM_OF_JOINT * M_PI/180;  // Increment joint positions
  }

  // for (int i = 10; i < (10+NUM_OF_JOINT); i++) {
  //     joint_state_.position[i] = surgical_tool_pose_right_.angular.y/NUM_OF_JOINT * M_PI/180;  // Increment joint positions
  // }
  // for (int i = 15; i < (15+NUM_OF_JOINT); i++) {
  //     joint_state_.position[i] = surgical_tool_pose_right_.angular.z/NUM_OF_JOINT * M_PI/180;  // Increment joint positions
  // }

  publisher_->publish(joint_state_);

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
#include "kinematics_control_node.hpp"

using MotorState = custom_interfaces::msg::MotorState;
using MotorCommand = custom_interfaces::msg::MotorCommand;


KinematicsControlNode::KinematicsControlNode(const rclcpp::NodeOptions & node_options)
: Node("KinematicsControlNode", node_options)
{
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = this->get_parameter("qos_depth", qos_depth);

  const auto QoS_RKL10V =
  rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  this->joystic_subscriber_ =
    this->create_subscription<sensor_msgs::msg::Joy>(
      "joy",
      QoS_RKL10V,
      [this] (const sensor_msgs::msg::Joy::SharedPtr msg) -> void
      {
        joystick_msg_.header = msg->header;
        joystick_msg_.axes = msg->axes;
        joystick_msg_.buttons = msg->buttons;

        /**
         * @brief E, W, S, N direction check (+1) or (-1)
         * @note  In xbox's left axes, E:-, W:+, S:-, N:+
         */
        this->ST_.set_target(joystick_msg_.axes[0], (-1) * joystick_msg_.axes[1]);
      }
    );

  this->kinematics_control_target_val_.target_val.resize(NUM_OF_MOTORS);
  kinematics_control_publisher_ =
    this->create_publisher<MotorCommand>("kinematics_control_target_val", QoS_RKL10V);
  
  this->motor_state_.actual_position.resize(NUM_OF_MOTORS);
  this->motor_state_.actual_velocity.resize(NUM_OF_MOTORS);
  this->motor_state_.actual_acceleration.resize(NUM_OF_MOTORS);
  this->motor_state_.actual_torque.resize(NUM_OF_MOTORS);
  motor_state_subscriber_ =
    this->create_subscription<MotorState>(
      "motor_state",
      QoS_RKL10V,
      [this] (const MotorState::SharedPtr msg) -> void
      {
        this->motor_state_.stamp = msg->stamp;
        this->motor_state_.actual_position =  msg->actual_position;
        this->motor_state_.actual_velocity =  msg->actual_velocity;
        this->motor_state_.actual_acceleration =  msg->actual_acceleration;
        this->motor_state_.actual_torque =  msg->actual_torque;

        this->ST_.kinematics();
        // this->kinematics_control_target_val_ = this->gear_encoder_ratio_conversion();
        // this->kinematics_control_target_val_ = cal_kinematics(this->motor_state_, this->joystick_msg_);
        // kinematics_control_publisher_->publish(this->kinematics_control_target_val_);
      }
    );
  
  /**
   * @brief if use custom surgical tool, initialize.
   */
  // STLeft_.init_surgicaltool(1,1,1,1);
  // STRight_.init_surgicaltool(1,1,1,1);


}

KinematicsControlNode::~KinematicsControlNode() {

}

MotorCommand KinematicsControlNode::cal_kinematics(MotorState state, sensor_msgs::msg::Joy controller_input)
{
  MotorCommand result;
  result.target_val.resize(NUM_OF_MOTORS);

  /* code */
  /* input : actual pos & actual velocity & controller input */
  /* output : target value*/

  return result;
}

void KinematicsControlNode::gear_encoder_ratio_conversion() {

}

void KinematicsControlNode::publishall()
{

}





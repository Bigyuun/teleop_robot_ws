#include "kinematics_control_node.hpp"
#include "rclcpp/rclcpp.hpp"

using MotorState = custom_interfaces::msg::MotorState;
using MotorCommand = custom_interfaces::msg::MotorCommand;
using namespace std::chrono_literals;

KinematicsControlNode::KinematicsControlNode(const rclcpp::NodeOptions & node_options)
: Node("KinematicsControlNode", node_options)
{
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = this->get_parameter("qos_depth", qos_depth);

  const auto QoS_RKL10V =
  rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  RCLCPP_INFO(this->get_logger(), "joy node creating....");

  joystick_msg_.axes.resize(8);
  joystick_msg_.buttons.resize(12);
  joystic_subscriber_ =
    this->create_subscription<sensor_msgs::msg::Joy>(
      "joy",
      QoS_RKL10V,
      [this] (const sensor_msgs::msg::Joy::SharedPtr msg) -> void
      {
        joystick_msg_.header = msg->header;
        joystick_msg_.axes = msg->axes;
        joystick_msg_.buttons = msg->buttons;
        
        // RCLCPP_INFO(
        //   this->get_logger(),
        //   "Header of the message : %ld, %ld",
        //   msg->header);

        // std::cout << joystick_msg_.axes[0] << std::endl;
        // std::cout << joystick_msg_.axes[1] << std::endl;
        // std::cout << joystick_msg_.axes[2] << std::endl;
      }
    );
  RCLCPP_INFO(this->get_logger(), "joy node created!!");

  this->kinematics_control_target_val_.target_val.resize(NUM_OF_MOTORS);
  for (int i=0; i<NUM_OF_MOTORS; i++) {
    this->kinematics_control_target_val_.target_val[i] = 1;
  }
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

        this->cal_kinematics();
        this->kinematics_control_publisher_->publish(kinematics_control_target_val_);
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


/**
 * @brief E, W, S, N direction check (+1) or (-1)
 * @note  In xbox's left axes, E:-, W:+, S:-, N:+
 */
float KinematicsControlNode::mapping_joystick_to_bending_p() {
  float axes = this->joystick_msg_.axes[0];
  return (this->ST_.max_bending_deg_ * axes);
}
float KinematicsControlNode::mapping_joystick_to_bending_t() {
  float axes = this->joystick_msg_.axes[1];
  return ( -1 * this->ST_.max_bending_deg_ * axes);
}
float KinematicsControlNode::mapping_joystick_to_forceps() {
  float axes = this->joystick_msg_.axes[2];
  if ( axes >= 0) return ( this->ST_.max_forceps_deg_ * axes);
  else return 0;
}

void KinematicsControlNode::cal_kinematics() {
  /* code */
  /* input : actual pos & actual velocity & controller input */
  /* output : target value*/

  float pAngle = this->mapping_joystick_to_bending_p();
  float tAngle = this->mapping_joystick_to_bending_t();
  float gAngle  = this->mapping_joystick_to_forceps();
  std::cout << "=================================" << std::endl;
  std::cout << "pangle : " << pAngle << std::endl;
  std::cout << "tangle : " << tAngle << std::endl;
  std::cout << "gangle : " << gAngle << std::endl;

  this->ST_.get_bending_kinematic_result(pAngle, tAngle, gAngle);

  float f_val[NUM_OF_MOTORS];
  f_val[0] = this->ST_.wrLengthEast_;
  f_val[1] = this->ST_.wrLengthWest_;
  f_val[2] = this->ST_.wrLengthSouth_;
  f_val[3] = this->ST_.wrLengthNorth_;
  f_val[4] = this->ST_.wrLengthGrip;

  std::cout << "East  : " << f_val[0] << std::endl;
  std::cout << "West  : " << f_val[1] << std::endl;
  std::cout << "South : " << f_val[2] << std::endl;
  std::cout << "North : " << f_val[3] << std::endl;
  std::cout << "Grip  : " << f_val[4] << std::endl;

  // ratio conversion
  this->kinematics_control_target_val_.target_val[0] = f_val[0] * gear_encoder_ratio_conversion(GEAR_RATIO_44, ENCODER_CHANNEL, ENCODER_RESOLUTION);
  this->kinematics_control_target_val_.target_val[1] = f_val[1] * gear_encoder_ratio_conversion(GEAR_RATIO_44, ENCODER_CHANNEL, ENCODER_RESOLUTION);
  this->kinematics_control_target_val_.target_val[2] = f_val[2] * gear_encoder_ratio_conversion(GEAR_RATIO_44, ENCODER_CHANNEL, ENCODER_RESOLUTION);
  this->kinematics_control_target_val_.target_val[3] = f_val[3] * gear_encoder_ratio_conversion(GEAR_RATIO_44, ENCODER_CHANNEL, ENCODER_RESOLUTION);
  this->kinematics_control_target_val_.target_val[4] = f_val[4] * gear_encoder_ratio_conversion(GEAR_RATIO_3_9, ENCODER_CHANNEL, ENCODER_RESOLUTION);
  
  std::cout << "1 : " << this->kinematics_control_target_val_.target_val[0] << std::endl;
  std::cout << "2 : " << this->kinematics_control_target_val_.target_val[1] << std::endl;
  std::cout << "3 : " << this->kinematics_control_target_val_.target_val[2] << std::endl;
  std::cout << "4 : " << this->kinematics_control_target_val_.target_val[3] << std::endl;
  std::cout << "5 : " << this->kinematics_control_target_val_.target_val[4] << std::endl;
}

float KinematicsControlNode::gear_encoder_ratio_conversion(float gear_ratio, int e_channel, int e_resolution) {
  return gear_ratio * e_channel * e_resolution;
}

void KinematicsControlNode::publishall()
{

}





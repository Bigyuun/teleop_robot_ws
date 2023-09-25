#include "kinematics_control_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>  

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

  //===============================
  // joystick subscriber
  //===============================
  joystick_msg_.axes.resize(8);
  joystick_msg_.buttons.resize(12);
  joystick_msg_.axes[2] = 1;  // /joy has 1 as initial val
  joystick_msg_.axes[5] = 1;  // /joy has 1 as initial val
  joystic_subscriber_ =
    this->create_subscription<sensor_msgs::msg::Joy>(
      "joy",
      QoS_RKL10V,
      [this] (const sensor_msgs::msg::Joy::SharedPtr msg) -> void
      {
        RCLCPP_WARN_ONCE(this->get_logger(), "Subscribing the /joy.");
        joystick_msg_.header = msg->header;
        joystick_msg_.axes = msg->axes;
        joystick_msg_.buttons = msg->buttons;
      }
    );

  //===============================
  // target value publisher
  //===============================
  this->kinematics_control_target_val_.target_position.resize(NUM_OF_MOTORS);
  this->kinematics_control_target_val_.target_velocity_profile.resize(NUM_OF_MOTORS);
  for(int i=0; i<NUM_OF_MOTORS; i++) {
    this->kinematics_control_target_val_.target_velocity_profile[i] = PERCENT_100/2;
  }
  kinematics_control_publisher_ =
    this->create_publisher<MotorCommand>("kinematics_control_target_val", QoS_RKL10V);

  //===============================
  // surgical tool pose(degree) publisher
  //===============================
  surgical_tool_pose_publisher_ =
    this->create_publisher<geometry_msgs::msg::Twist>("surgical_tool_pose", QoS_RKL10V);

  //===============================
  // motor status subscriber
  //===============================
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
        RCLCPP_WARN_ONCE(this->get_logger(), "Subscribing the /motor_state.");
        this->motorstate_op_flag_ = true;
        this->motor_state_.stamp = msg->stamp;
        this->motor_state_.actual_position =  msg->actual_position;
        this->motor_state_.actual_velocity =  msg->actual_velocity;
        this->motor_state_.actual_acceleration =  msg->actual_acceleration;
        this->motor_state_.actual_torque =  msg->actual_torque;

        if(this->op_mode_ == kEnable) {
          this->cal_kinematics();
        }
        this->kinematics_control_publisher_->publish(this->kinematics_control_target_val_);
        this->surgical_tool_pose_publisher_->publish(this->surgical_tool_pose_);
      }
    );
  
  //===============================
  // loadcell data subscriber
  //===============================
  this->loadcell_data_.data.resize(10);
  this->loadcell_data_.threshold.resize(10);
  loadcell_data_subscriber_ =
    this->create_subscription<custom_interfaces::msg::LoadcellState>(
      "loadcell_data",
      QoS_RKL10V,
      [this] (const custom_interfaces::msg::LoadcellState::SharedPtr msg) -> void
      {
        this->loadcell_op_flag_ = true;
        this->loadcell_data_.threshold = msg->threshold;
        this->loadcell_data_.data = msg->data;
        RCLCPP_INFO_ONCE(this->get_logger(), "Subscribing the /loadcell_data.");
        RCLCPP_WARN_ONCE(this->get_logger(), "Subscribing the /loadcell_data.");
      }
    );
  /**
   * @brief if use custom surgical tool, initialize.
   */
  // STLeft_.init_surgicaltool(1,1,1,1);
  // STRight_.init_surgicaltool(1,1,1,1);


  /**
   * @brief homing
   */
  this->homingthread_ = std::thread(&KinematicsControlNode::homing, this);
}

KinematicsControlNode::~KinematicsControlNode() {

}

/**
 * @author DY
 * @brief E, W, S, N direction check (+1) or (-1)
 * @note  In xbox's left axes -> E:-, W:+, S:-, N:+
 *        In our definition   -> E:-, W:+, S:+, N:-
 *        mapping joystick data to angle of hardware limitation
 */
double KinematicsControlNode::mapping_joystick_to_bending_p() {
  double axes = this->joystick_msg_.axes[0];
  return (this->ST_.max_bending_deg_ * axes);
}
double KinematicsControlNode::mapping_joystick_to_bending_t() {
  double axes = this->joystick_msg_.axes[1];
  return ( -1 * this->ST_.max_bending_deg_ * axes);
}
double KinematicsControlNode::mapping_joystick_to_forceps() {
  double axes = this->joystick_msg_.axes[2];
  if ( axes >= 0) return ( this->ST_.max_forceps_deg_ * axes);
  else return 0;
}

void KinematicsControlNode::cal_kinematics() {
  /* code */
  /* input : actual pos & actual velocity & controller input */
  /* output : target value*/
  double pAngle = this->mapping_joystick_to_bending_p();
  double tAngle = this->mapping_joystick_to_bending_t();
  double gAngle  = this->mapping_joystick_to_forceps();

  this->surgical_tool_pose_.angular.x = pAngle;
  this->surgical_tool_pose_.angular.y = tAngle;
  this->ST_.get_bending_kinematic_result(pAngle, tAngle, gAngle);

  double f_val[DOF];
  f_val[0] = this->ST_.wrLengthEast_;
  f_val[1] = this->ST_.wrLengthWest_;
  f_val[2] = this->ST_.wrLengthSouth_;
  f_val[3] = this->ST_.wrLengthNorth_;
  f_val[4] = this->ST_.wrLengthGrip;
  // std::cout << "--------------------------" << std::endl;
  // std::cout << "East  : " << f_val[0] << " mm" << std::endl;
  // std::cout << "West  : " << f_val[1] << " mm" << std::endl;
  // std::cout << "South : " << f_val[2] << " mm" << std::endl;
  // std::cout << "North : " << f_val[3] << " mm" << std::endl;
  // std::cout << "Grip  : " << f_val[4] << " mm" << std::endl;

  // ratio conversion & Check Threshold of loadcell
  // In ROS2, there is no function of finding max(or min) value
  // for (int i=0; i<DOF; i++) {
  //   if (this->loadcell_data_.data[i] >= this->loadcell_data_.threshold[i]) {
  //     RCLCPP_WARN(
  //       this->get_logger(),
  //       "#%d Loadcell is %.2fg. Upper than %.2fg Threshold.",
  //       i, this->loadcell_data_.data[i], this->loadcell_data_.threshold[i]);
  //   }
  // }

  this->kinematics_control_target_val_.stamp = this->now();
  this->kinematics_control_target_val_.target_position[0] = DIRECTION_COUPLER * f_val[0] * gear_encoder_ratio_conversion(GEAR_RATIO_44, ENCODER_CHANNEL, ENCODER_RESOLUTION);
  this->kinematics_control_target_val_.target_position[1] = DIRECTION_COUPLER * f_val[1] * gear_encoder_ratio_conversion(GEAR_RATIO_44, ENCODER_CHANNEL, ENCODER_RESOLUTION);
  this->kinematics_control_target_val_.target_position[2] = DIRECTION_COUPLER * f_val[2] * gear_encoder_ratio_conversion(GEAR_RATIO_44, ENCODER_CHANNEL, ENCODER_RESOLUTION);
  this->kinematics_control_target_val_.target_position[3] = DIRECTION_COUPLER * f_val[3] * gear_encoder_ratio_conversion(GEAR_RATIO_44, ENCODER_CHANNEL, ENCODER_RESOLUTION);
  this->kinematics_control_target_val_.target_position[4] = DIRECTION_COUPLER * f_val[4] * gear_encoder_ratio_conversion(GEAR_RATIO_3_9, ENCODER_CHANNEL, ENCODER_RESOLUTION);

  this->kinematics_control_target_val_.target_position[0] = this->virtual_home_pos_[0]
                                                            + DIRECTION_COUPLER * f_val[0] * gear_encoder_ratio_conversion(GEAR_RATIO_44, ENCODER_CHANNEL, ENCODER_RESOLUTION);
  this->kinematics_control_target_val_.target_position[1] = this->virtual_home_pos_[1]
                                                            + DIRECTION_COUPLER * f_val[1] * gear_encoder_ratio_conversion(GEAR_RATIO_44, ENCODER_CHANNEL, ENCODER_RESOLUTION);
  this->kinematics_control_target_val_.target_position[2] = this->virtual_home_pos_[2]
                                                            + DIRECTION_COUPLER * f_val[2] * gear_encoder_ratio_conversion(GEAR_RATIO_44, ENCODER_CHANNEL, ENCODER_RESOLUTION);
  this->kinematics_control_target_val_.target_position[3] = this->virtual_home_pos_[3]
                                                            + DIRECTION_COUPLER * f_val[3] * gear_encoder_ratio_conversion(GEAR_RATIO_44, ENCODER_CHANNEL, ENCODER_RESOLUTION);
  this->kinematics_control_target_val_.target_position[4] = this->virtual_home_pos_[4]
                                                            + DIRECTION_COUPLER * f_val[4] * gear_encoder_ratio_conversion(GEAR_RATIO_3_9, ENCODER_CHANNEL, ENCODER_RESOLUTION);

#if MOTOR_CONTROL_SAME_DURATION
  /**
   * @brief find max value and make it max_velocity_profile 100 (%),
   *        other value have values proportional to 100 (%) each
   */
  static double prev_f_val[DOF];  // for delta length

  std::vector<double> abs_f_val(DOF-1, 0);  // 5th DOF is a forceps
  for (int i=0; i<DOF-1; i++) { abs_f_val[i] = std::abs(this->kinematics_control_target_val_.target_position[i] - this->motor_state_.actual_position[i]); }

  double max_val = *std::max_element(abs_f_val.begin(), abs_f_val.end()) + 0.00001; // 0.00001 is protection for 0/0 (0 divided by 0)
  int max_val_index = std::max_element(abs_f_val.begin(), abs_f_val.end()) - abs_f_val.begin();
  for (int i=0; i<(DOF-1); i++) { 
    this->kinematics_control_target_val_.target_velocity_profile[i] = (abs_f_val[i] / max_val) * PERCENT_100;
  }
  // last index means forceps. It doesn't need velocity profile
  this->kinematics_control_target_val_.target_velocity_profile[DOF-1] = PERCENT_100;
  
#else
  for (int i=0; i<DOF; i++) { 
    this->kinematics_control_target_val_.target_velocity_profile[i] = PERCENT_100;
  }
#endif
}

double KinematicsControlNode::gear_encoder_ratio_conversion(double gear_ratio, int e_channel, int e_resolution) {
  return gear_ratio * e_channel * e_resolution;
}

void KinematicsControlNode::set_position_zero() {
  for (int i=0; i<NUM_OF_MOTORS; i++) {
    this->virtual_home_pos_[i] = 0;
  }
}


int8_t KinematicsControlNode::homing() {
  this->op_mode_ = kHoming;
  // Calibration & Homing
  bool loop_out_flag[NUM_OF_MOTORS] = {false,};
  RCLCPP_WARN(this->get_logger(), "Start Homing");

  while(this->op_mode_ == kHoming) {
    // if loadcell is operating
    if (this->loadcell_op_flag_ == false || this->motorstate_op_flag_ == false) {
      std:: cout << "loadcell_op_flag_   : " << this->loadcell_op_flag_ << std::endl;
      std:: cout << "motorstate_op_flag_ : " << this->motorstate_op_flag_ << std::endl;
      rclcpp::sleep_for(1s);
      continue;
      // return -1;
    }

    else {
      /**
       * @brief Release the wire
       */
      RCLCPP_WARN_ONCE(this->get_logger(), "Releasing the wire...");
      while(this->op_mode_ == kHoming) {
        for (int i=0; i<NUM_OF_MOTORS; i++) {
          if(this->loadcell_data_.data[i] >= 20.0) {
            this->kinematics_control_target_val_.target_position[i] = this->motor_state_.actual_position[i] - 100;
            loop_out_flag[i] = false;
          }
          else {
            loop_out_flag[i] = true;
          }
        }

        // finish releasing the wire
        bool release_flag = false;
        for (int i=0; i<NUM_OF_MOTORS; i++) {
          if (loop_out_flag[i] == false) {
            release_flag = false;
            continue;
          }
          else {
            release_flag = true;
          }
        }
        if (release_flag == true) break;
      }

      /**
       * @brief Reel the wire
       */
      RCLCPP_WARN_ONCE(this->get_logger(), "Reeling the wire...");
      for (int i=0; i<NUM_OF_MOTORS; i++) {
        loop_out_flag[i] = false;
      }
      while(this->op_mode_ == kHoming) {
        for (int i=0;i <NUM_OF_MOTORS; i++) {
          if (this->loadcell_data_.data[i] <= 20.0) {
            this->kinematics_control_target_val_.target_position[i] = this->motor_state_.actual_position[i] + 10;
            loop_out_flag[i] = false;
          }
          else {
            loop_out_flag[i] = true;
          }
        }

        // finish reeling the wire
        bool reel_flag = false;
        for (int i=0; i<NUM_OF_MOTORS; i++) {
          if (loop_out_flag[i] == false) {
            reel_flag = false;
            continue;
          }
          else {
            reel_flag = true;
          }
        }
        if (reel_flag == true) break;
      }

      RCLCPP_WARN(this->get_logger(), "Finish Homing");
      for (int i=0; i<NUM_OF_MOTORS; i++ ) {
        this->virtual_home_pos_[i] = this->motor_state_.actual_position[i];
      }
      this->op_mode_ = kEnable;
      return 1;
    }
  }
}


void KinematicsControlNode::publishall()
{

}





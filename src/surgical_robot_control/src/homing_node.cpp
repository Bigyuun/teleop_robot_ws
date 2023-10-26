#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "custom_interfaces/msg/motor_state.hpp"
#include "custom_interfaces/msg/motor_command.hpp"
#include "custom_interfaces/msg/motor_command_csv.hpp"
#include "custom_interfaces/msg/loadcell_state.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>
#include <thread>

#include "hw_definition.hpp"

#define HOMING_THRESHOLD 20.0
#define SPEED_RELEASE  -1
#define SPEED_REEL    1


using namespace std::chrono_literals;

class HomingNode : public rclcpp::Node
{
public:
  using MotorState = custom_interfaces::msg::MotorState;
  using MotorCommandCSV = custom_interfaces::msg::MotorCommandCSV;
  using MotorCommand = custom_interfaces::msg::MotorCommand;
  HomingNode()
  : Node("homing_mode_publisher")
  {
    const auto QoS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    this->motor_command_.target_position.resize(NUM_OF_MOTORS);
    this->motor_command_.target_velocity_profile.resize(NUM_OF_MOTORS);
    motor_command_publisher_ = this->create_publisher<MotorCommand>("kinematics_control_target_val", QoS_RKL10V);
    RCLCPP_WARN(this->get_logger(), "homing_command publisher is created");

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
        }
      );

    //===============================
    // loadcell data subscriber
    //===============================
    this->loadcell_data_.data.resize(NUM_OF_MOTORS);
    this->loadcell_data_.threshold.resize(NUM_OF_MOTORS);
      loadcell_data_subscriber_ =
      this->create_subscription<custom_interfaces::msg::LoadcellState>(
        "loadcell_data",
        QoS_RKL10V,
        [this] (const custom_interfaces::msg::LoadcellState::SharedPtr msg) -> void
        {
          this->loadcell_op_flag_ = true;
          this->loadcell_data_.threshold = msg->threshold;
          this->loadcell_data_.data = msg->data;
          RCLCPP_WARN_ONCE(this->get_logger(), "Subscribing the /loadcell_data.");
        }
      );

    this->homingthread_ = std::thread(&HomingNode::homing, this);
  }
  
  ~HomingNode() {};

  void homing()
  { 
    while (true) 
    {
      if (this->motorstate_op_flag_ == false || this->loadcell_op_flag_ == false)
      {
        continue;
      } else {
        RCLCPP_WARN(this->get_logger(), "Start Homing Loop...");
        break;
      }
    }

    /**
     * @brief release the wires
     */
    RCLCPP_WARN(this->get_logger(), "Start releasing wires...");
    
    while (true)
    {
      for (int i=0; i<NUM_OF_MOTORS; i++)
      {
        if (loadcell_data_.data[i] <= HOMING_THRESHOLD) {
          this->motor_command_.target_position[i] = 0;
        } else {
          this->motor_command_.target_position[i] = SPEED_RELEASE;
        }
      }
      motor_command_publisher_->publish(motor_command_);
      
      // check
      uint16_t exist = std::count(std::begin(motor_command_.target_position), std::end(motor_command_.target_position), SPEED_RELEASE);
      // std::cout << "exist: " << exist << std::endl;
      if (exist > 0) {
        continue;
      } else {
        RCLCPP_WARN(this->get_logger(), "done");
        break;
      }
    }

    RCLCPP_WARN(this->get_logger(), "Wait 5s for stabilization of loadcell");
    for (int i=0; i<5; i++) {
      rclcpp::sleep_for(1s);
      std::cout << "remains " << 4-i << "s ..." << std::endl;
    }
    
    /**
     * @brief reel the wires
     */
    RCLCPP_WARN(this->get_logger(), "Start reeling wires...");
    while (true)
    {
      for (int i=0; i<NUM_OF_MOTORS; i++)
      {
        if (loadcell_data_.data[i] >= HOMING_THRESHOLD) {
          this->motor_command_.target_position[i] = 0;
        } else {
          this->motor_command_.target_position[i] = SPEED_REEL;
        }
      }
      this->motor_command_.target_position[4] = 0;  // Gripper.

      motor_command_publisher_->publish(motor_command_);

      // check
      uint16_t exist = std::count(std::begin(motor_command_.target_position), std::end(motor_command_.target_position), SPEED_REEL);
      // std::cout << "exist: " << exist << std::endl;
      if (exist > 0) {
        continue;
      } else {
        RCLCPP_WARN(this->get_logger(), "done");
        break;
      }
    }

    RCLCPP_WARN(this->get_logger(), "Finish Homing Loop");
    rclcpp::sleep_for(1s);
  }


private:
  std::thread homingthread_;

  // MotorCommandCSV motor_command_;
  // rclcpp::Publisher<MotorCommandCSV>::SharedPtr motor_command_publisher_;

  MotorCommand motor_command_;
  rclcpp::Publisher<MotorCommand>::SharedPtr motor_command_publisher_;

  bool motorstate_op_flag_;
  MotorState motor_state_;
  rclcpp::Subscription<MotorState>::SharedPtr motor_state_subscriber_;

  bool loadcell_op_flag_;
  custom_interfaces::msg::LoadcellState loadcell_data_;
  rclcpp::Subscription<custom_interfaces::msg::LoadcellState>::SharedPtr loadcell_data_subscriber_;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HomingNode>());
  rclcpp::shutdown();
  return 0;
}
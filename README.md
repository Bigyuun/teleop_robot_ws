# teleop_robot_ws
[MIDAS] Surgical robot development worksapce
This is the ROS2 project for the MIDAS surgical robot.



## Prerequisite
#### OS
Ubuntu 20.04.6 LTS
#### Software
- ROS2 Foxy (Focal)
  <https://docs.ros.org/en/foxy/index.html>
#### ROS2 packages 
- Rviz2
- teleop_twist_joy
- robot_state_publisher
- joint_state_publisher_gui

## System
#### System Configuration

#### Node
- **'surgical_robot_control_node'**
  - Publisher
    `kinematics_control_target_val`
  - Subscriber
    `/joy`
    `/motor_state`
    `/loadcell_data`
- **tcpclient**
  - Publisher
    `/motor_state`
  - Subscriber
    `kinematics_control_target_val`
- **loadcell_publisher**
  - Publisher
    `/loadcell_data`
- **surgical_robot_state_publisher**
  - Publisher
    `/tf`
    `/robot_description`
  - Subscriber
    `/joint_states`
- **joy_node**
  - Publisher
    `/joy`

#### Topic
- **Custom interfaces**
  - MotorCommand.msg
    |Type|Name|
    |:--|:--|
    |builtin_interfaces/Time|stamp|
    |int32[]|target_position|
    |int32[]|target_velocity_profile|
  - MotorState.msg
    |Type|Name|
    |:--|:--|
    |builtin_interfaces/Time|stamp|
    |int32[]|actual_position|
    |int32[]|actual_velocity|
    |int32[]|actual_acceleration|
    |int32[]|actual_torque|

## Execute
### TCP
```bash
  $ cd teleop_robot_ws
  $ source /opt/ros/{DISTRIBUTION}/setup.bash
  $ . install/setup.bash
  $ ros2 launch launch_pkg teleop_robot.launch.py
```

### Non-TCP
```bash
  $ cd teleop_robot_ws
  $ source /opt/ros/{DISTRIBUTION}/setup.bash
  $ . install/setup.bash
  $ ros2 launch launch_pkg demo_teleop_robot.launch.py
```

## Branch
- main
  > Use 5 motors. 4 for Directions(E,W,S,N) and the other for forceps.

- twin
  > Use 10 motors. It means 2 set of main branch system.



# teleop_robot_ws
[MIDAS] Surgical robot development worksapce
This is the ROS2 project for the MIDAS surgical robot.
---
## Index
1. [Prerequisite](#prerequisite)
2. [OS](#os)
3. [ROS2 packages](#ros2-packages)
<<<<<<< HEAD
4. [Installation](#installation)
5. [System Confituration](#system-confituration)
6. [Node](#node)
7. [Topic](#topic)
8. [Branch](#branch)
9. [Execute](#execute)

=======
4. [System Confituration](#system-confituration)
5. [Node](#node)
6. [Topic](#topic)
7. [Branch](#branch)
8. [Execute](#execute)
---
>>>>>>> be04d84714f6701617e4222d9800f11731269615
## Prerequisite
#### OS
Ubuntu 20.04.6 LTS
#### Software
- ROS2 Foxy (Focal)
  <https://docs.ros.org/en/foxy/index.html>
#### ROS2 packages 
##### Support
- Rviz2
- teleop_twist_joy
- robot_state_publisher
- joint_state_publisher_gui
##### Custom
- surgical_robot_control

<<<<<<< HEAD
## Installation
##### ROS2
- xacro
```bash
  $ sudo apt install ros-{distro}-xacro
```
- joint_state_publisher_gui
```bash
  $ sudo apt install ros-{distro}-joint-state-publisher-gui
```
##### python
- serial (pyserial)
```bash
  $ pip install pyserial
```
- parse
```bash
  $ pip install parse
```
=======
>>>>>>> be04d84714f6701617e4222d9800f11731269615
## System
#### System Configuration
![Image Description](https://github.com/Bigyuun/teleop_robot_ws/blob/main/media/ros2_system_config.png)

#### Node
- **'surgical_robot_control_node'**
  - Publisher<br/>
    `kinematics_control_target_val`<br/>
  - Subscriber<br/>
    `/joy`<br/>
    `/motor_state`<br/>
    `/loadcell_data`<br/>
- **tcpclient**
  - Publisher<br/>
    `/motor_state`<br/>
  - Subscriber<br/>
    `kinematics_control_target_val`<br/>
- **loadcell_publisher**
  - Publisher<br/>
    `/loadcell_data`<br/>
- **surgical_robot_state_publisher**
  - Publisher<br/>
    `/tf`<br/>
    `/robot_description`<br/>
  - Subscriber<br/>
    `/joint_states`<br/>
- **joy_node**
  - Publisher<br/>
    `/joy`<br/>

#### Topic
- **Custom interfaces**
  - MotorCommand.msg
    |Type|Name|
    |:--|:--|
    |builtin_interfaces/Time|stamp|
    |float32[]|threshold|
    |float32[]|data|
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
   

## Branch
- main
  > Use 5 motors. 4 for Directions(E,W,S,N) and the other for forceps.
```bash
  $ git clone -b main https://github.com/Bigyuun/teleop_robot_ws.git
```

- twin
  > Use 10 motors. It means 2 set of main branch system.
```bash
  $ git clone -b twin https://github.com/Bigyuun/teleop_robot_ws.git
```
  
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





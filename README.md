# teleop_robot_ws
[MIDAS] Surgical robot development worksapce

This is the ROS2 project for the MIDAS surgical robot.

## OS
Ubuntu 20.04.6 LTS

## Prerequisite packages
- ROS2 Foxy (Focal)
  - <https://docs.ros.org/en/foxy/index.html>

### ROS2 packages 
- Rviz2
- teleop_twist_joy

## Execute

'''bash
  $ cd teleop_robot_ws
  $ source /opt/ros/{DISTRIBUTION}/setup.bash
  $ . install/setup.bash
  $ ros2 launch launch_pkg teleop_robot.launch.py
'''

## Execute(Demo - No TCP coneection)
'''bash
  $ cd teleop_robot_ws
  $ source /opt/ros/{DISTRIBUTION}/setup.bash
  $ . install/setup.bash
  $ ros2 launch launch_pkg demo_teleop_robot.launch.py
'''

## Branch
- main
  > Use 5 motors. 4 for Directions(E,W,S,N) and the other for forceps.

- twin
  > Use 10 motors. It means 2 set of main branch system.



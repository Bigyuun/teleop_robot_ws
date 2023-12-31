cmake_minimum_required(VERSION 3.5)
project(tcp_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)

include_directories("./include/tcp_pkg/"
                    "/opt/ros/foxy/include"
                    "../surgical_robot_control/include/surgical_robot_control/"
)

# DY
# Build
add_executable(tcpclient src/main.cpp src/tcp_node.cpp)
ament_target_dependencies(tcpclient
  rclcpp 
  std_msgs 
  sensor_msgs 
  geometry_msgs
  custom_interfaces
)

add_executable(demo_node src/demo_node.cpp)
ament_target_dependencies(demo_node
  rclcpp 
  std_msgs 
  sensor_msgs 
  geometry_msgs
  custom_interfaces
)


# DY
# Install
install(TARGETS
  tcpclient
  demo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# DY
# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# DY
# Macro for ament package
ament_package()

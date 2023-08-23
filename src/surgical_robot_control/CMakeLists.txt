cmake_minimum_required(VERSION 3.5)
project(surgical_robot_control)

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
find_package(custom_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

# DY
include_directories(include
                    "include/surgical_robot_control"
                    "../tcp_pkg/include/tcp_pkg"
)

# DY
# Build
add_executable(surgical_robot_control_node src/main.cpp src/kinematics_control_node.cpp)
ament_target_dependencies(surgical_robot_control_node
  rclcpp 
  std_msgs 
  sensor_msgs 
  geometry_msgs
  custom_interfaces
)

# DY
# Install
install(TARGETS
surgical_robot_control_node
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(surgical_robot_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

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

ament_package()

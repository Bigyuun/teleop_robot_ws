cmake_minimum_required(VERSION 3.5)
project(tf2_pkg)

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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories(include
  "include/tf2_pkg"
  "/opt/ros/foxy/include"
)

add_executable(broadcaster src/broadcaster.cpp)
ament_target_dependencies(broadcaster
                          rclcpp
                          std_msgs
                          std_srvs
                          geometry_msgs
                          tf2
                          tf2_ros
                          tf2_geometry_msgs
                          )

add_executable(static_broadcaster src/static_broadcaster.cpp)
ament_target_dependencies(static_broadcaster
                          rclcpp
                          std_msgs
                          std_srvs
                          geometry_msgs
                          tf2
                          tf2_ros
                          tf2_geometry_msgs
                          )

add_executable(listener src/listener.cpp)
ament_target_dependencies(listener
                          rclcpp
                          std_msgs
                          std_srvs
                          geometry_msgs
                          tf2
                          tf2_ros
                          tf2_geometry_msgs
                          )


                          # Install
install(TARGETS
  broadcaster
  static_broadcaster
  listener
  DESTINATION lib/${PROJECT_NAME}
)

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

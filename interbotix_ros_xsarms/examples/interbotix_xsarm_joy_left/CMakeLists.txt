cmake_minimum_required(VERSION 3.5)
project(interbotix_xsarm_joy_left)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(interbotix_xs_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(ROS_DEPENDENCIES
  ament_index_cpp
  interbotix_xs_msgs
  rclcpp
  sensor_msgs
)

add_executable(xsarm_joy_left src/xsarm_joy_left.cpp)
ament_target_dependencies(xsarm_joy_left ${ROS_DEPENDENCIES})

install(
  TARGETS
    xsarm_joy_left
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/xsarm_robot_left.py
  DESTINATION
    lib/${PROJECT_NAME}
)
install(
  DIRECTORY
    config
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

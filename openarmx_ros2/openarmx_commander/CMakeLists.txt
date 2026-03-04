cmake_minimum_required(VERSION 3.8)
project(openarmx_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(openarmx_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(moveit_commander src/moveit_commander.cpp)
ament_target_dependencies(moveit_commander
  rclcpp
  moveit_ros_planning_interface
  example_interfaces
  openarmx_interfaces
  tf2_geometry_msgs
)

install(TARGETS
  moveit_commander
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)

project(resilient_manipulation)


if(
  CMAKE_COMPILER_IS_GNUCXX
  OR
  CMAKE_CXX_COMPILER_ID MATCHES "Clang"
)

  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )

endif()


find_package(
  ament_cmake REQUIRED
)

find_package(
  rclcpp REQUIRED
)

find_package(
  geometry_msgs REQUIRED
)

find_package(
  moveit_ros_planning_interface REQUIRED
)

find_package(
  moveit_msgs REQUIRED
)

find_package(
  shape_msgs REQUIRED
)


add_executable(
  planning_scene_initializer
  src/planning_scene_initializer.cpp
)

ament_target_dependencies(
  planning_scene_initializer

  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
)


add_executable(
  vision_motion_executor
  src/vision_motion_executor.cpp
)

ament_target_dependencies(
  vision_motion_executor

  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)


install(
  TARGETS

  planning_scene_initializer
  vision_motion_executor

  DESTINATION
  lib/${PROJECT_NAME}
)


ament_package()

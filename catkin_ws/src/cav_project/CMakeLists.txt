cmake_minimum_required(VERSION 3.0.2)
project(cav_project)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  ackermann_msgs
  message_generation
)

add_message_files(
  FILES
  limo_info.msg
  limo_info_array.msg
  QP_solution.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs sensor_msgs ackermann_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

cmake_minimum_required(VERSION 2.8.3)
project(darknet_rospy)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_sensor_msgs
  visualization_msgs
  darknet_rospy_msgs
  libuvc_camera
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


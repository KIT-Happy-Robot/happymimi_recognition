cmake_minimum_required(VERSION 3.0.2)
project(recognition_processing)

find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  geometry_msgs
  happymimi_recognition_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES recognition_processing
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

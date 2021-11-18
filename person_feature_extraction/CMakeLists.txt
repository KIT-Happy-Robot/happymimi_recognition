cmake_minimum_required(VERSION 3.0.2)
project(person_feature_extraction)

find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  geometry_msgs
  happymimi_recognition_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES person_feature_extraction
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

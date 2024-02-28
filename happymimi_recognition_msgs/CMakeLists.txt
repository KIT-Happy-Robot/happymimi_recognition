cmake_minimum_required(VERSION 3.0.2)
project(happymimi_recognition_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  actionlib_msgs
  happymimi_msgs
  message_generation
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  RecognitionList.srv
  RecognitionFind.srv
  RecognitionCount.srv
  RecognitionLocalize.srv
  MultipleLocalize.srv
  PositionEstimator.srv
  depth_meter.srv
  Clip.srv
  ITT.srv
  PositionEstimatorPCL.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  RecognitionProcessing.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
  happymimi_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES happymimi_recognition_msgs
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

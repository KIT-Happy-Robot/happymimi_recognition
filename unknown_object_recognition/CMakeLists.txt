cmake_minimum_required(VERSION 3.0.2)
project(unknown_object_recognition)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  cv_bridge
  geometry_msgs
  happymimi_recognition_msgs
  image_transport
  roscpp
  rospy
  std_msgs
  vision_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES unknown_object_recognition
#  CATKIN_DEPENDS actionlib cv_bridge geometry_msgs happymimi_recognition_msgs image_transport rcl_ros roscpp rospy std_msgs vision_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/unknown_object_recognition.cpp
# )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/unknown_object_recognition_node.cpp)

add_executable(downsampling src/downsampling.cpp)
add_executable(planar_segmentation src/planar_segmentation.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(downsampling ${catkin_LIBRARIES})
target_link_libraries(planar_segmentation ${catkin_LIBRARIES})

#############
## Install ##
#############
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  script/image_server.py
  script/image_module.py
  script/pc_module.py
  script/prompt_module.py
  script/clip_module.py
  script/table_object_server.py
  script/uor_yolo_server.py
  script/uor_gpt_server.py
  script/uor_clip_server.py
  script/uor_yolo_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

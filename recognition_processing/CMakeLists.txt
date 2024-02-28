cmake_minimum_required(VERSION 3.0.2)
project(recognition_processing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  cv_bridge
  happymimi_recognition_msgs
  #add pcl
  pcl_conversions pcl_ros
)

find_package(OpenCV REQUIRED)
#add pcl
find_package(PCL REQUIRED)
catkin_package(CATKIN_DEPENDS roscpp std_msgs pcl_conversions pcl_ros)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
#add pcl
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#add pcl
add_executable(point_sample point_cloud/point_sample.cpp)
add_executable(point_estimation point_cloud/point_estimation.cpp)
target_link_libraries(point_sample ${catkin_LIBRARIES})
target_link_libraries(point_estimation ${catkin_LIBRARIES})

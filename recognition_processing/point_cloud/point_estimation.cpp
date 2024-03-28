//ros
#include <ros/ros.h>
//math, opencv
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
//rviz
#include <visualization_msgs/Marker.h>
//pcl
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <geometry_msgs/Point.h>
// custom srv
#include <happymimi_recognition_msgs/PositionEstimatorPCL.h>

class ThreeDimensionalPositionEstimator
{
public:
  ThreeDimensionalPositionEstimator();
  ~ThreeDimensionalPositionEstimator(){};

private:
  ros::NodeHandle nh;
  /* -- topic -- */
  ros::Subscriber realsense_subscriber;
  ros::Subscriber realsense_point_sub;
  ros::Publisher realsense_point_cloud;
  ros::Publisher marker_pub;
  /* -- service -- */
  ros::ServiceServer estimate_server;
  /* -- member variables -- */
  sensor_msgs::ImageConstPtr depth_image;
  sensor_msgs::PointCloud2 point_data;
  /* -- member functions --*/
  void realSenseCB(const sensor_msgs::ImageConstPtr& ros_image);
  void pointCB(const sensor_msgs::PointCloud2 input_point);
  bool convertImage(const sensor_msgs::ImageConstPtr& input_image, cv::Mat &output_image);
  //bool getDepth(happymimi_recognition_msgs::PositionEstimator::Request &req, happymimi_recognition_msgs::PositionEstimator::Response &res);
};

ThreeDimensionalPositionEstimator::ThreeDimensionalPositionEstimator() : nh("")
{
  realsense_subscriber = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &ThreeDimensionalPositionEstimator::realSenseCB, this);
  realsense_point_sub  = nh.subscribe("/camera/depth/color/points", 1, &ThreeDimensionalPositionEstimator::pointCB, this);
  realsense_point_cloud = nh.advertise<geometry_msgs::Point>("nearest_point_topic", 1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("nearest_point_marker_topic", 1);
  //estimate_server = nh.advertiseService("/detect/depth", &ThreeDimensionalPositionEstimator::getDepth, this);
  //nh.getParam("/mimi_specification/Ground_Neck_Height", neck_height);
}

void ThreeDimensionalPositionEstimator::realSenseCB(const sensor_msgs::ImageConstPtr& ros_image){
    depth_image = ros_image;
}

void ThreeDimensionalPositionEstimator::pointCB(const sensor_msgs::PointCloud2 input_point){
    // Convert PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(input_point, *cloud);

    // Find nearest point (minimum X coordinate)
    pcl::PointXYZ nearest_point;
    nearest_point.z = std::numeric_limits<float>::max(); // Initialize with maximum possible value

    for (const auto& point : cloud->points) {
        pcl::PointXYZ point_xyz;
        point_xyz.x = point.x;
        point_xyz.y = point.y;
        point_xyz.z = point.z;

        if (point_xyz.z < nearest_point.z) {
            nearest_point = point_xyz;
        }
    }
    nearest_point.y = nearest_point.y + 0.02;
    // Publish the nearest point
    geometry_msgs::Point nearest_point_msg;
    nearest_point_msg.x = nearest_point.x;
    nearest_point_msg.y = nearest_point.y;
    nearest_point_msg.z = nearest_point.z;
    // Print the nearest point's coordinates (X, Y, Z)
    ROS_INFO("Nearest point coordinates: X = %f, Y = %f, Z = %f", nearest_point.x, nearest_point.y, nearest_point.z);
    realsense_point_cloud.publish(nearest_point_msg);

    // Publish a red marker at the nearest point for visualization in RViz
    /*
    visualization_msgs::Marker marker;
    marker.header = input_point.header;
    marker.ns = "nearest_point_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = nearest_point.x;
    marker.pose.position.y = nearest_point.y;
    marker.pose.position.z = nearest_point.z;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker_pub.publish(marker);
    */
}

bool ThreeDimensionalPositionEstimator::convertImage(const sensor_msgs::ImageConstPtr& input_image, cv::Mat &output_image){
  cv_bridge::CvImagePtr cv_ptr;
  try{
	// ros image msg -> cv_bridge -> cv::Mat
	cv_ptr = cv_bridge::toCvCopy(input_image, sensor_msgs::image_encodings::TYPE_16UC1);
	output_image = cv_ptr->image;

  }
  catch (cv_bridge::Exception& e){
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return false;
  }
  return true;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "three_dimensional_position_estimator");

  ThreeDimensionalPositionEstimator three_dimensional_position_estimator;

  ROS_INFO("Ready Point estimation Server");
  ros::spin();
  return 0;
}

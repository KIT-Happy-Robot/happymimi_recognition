#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <geometry_msgs/Point.h>
// custom srv
#include <happymimi_recognition_msgs/PositionEstimatorPCL.h>

//std::vector<pcl::PointXYZRGB> cluster_centers;  // Vector to store cluster centers
ros::Publisher nearest_point_pub;

void ThreeDimensionalPositionEstimator::realSenseCB(const sensor_msgs::ImageConstPtr& ros_image){
  depth_image = ros_image;
}

void Cloud_CB(const sensor_msgs::PointCloud2 input) {
    // Convert PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(input, *cloud);

    // Find nearest point (minimum Z coordinate)
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

    // Publish the nearest point
    geometry_msgs::Point nearest_point_msg;
    nearest_point_msg.x = nearest_point.x;
    nearest_point_msg.y = nearest_point.y;
    nearest_point_msg.z = nearest_point.z;
    // Print the nearest point's coordinates (Z, X, Y)
    ROS_INFO("Nearest point coordinates: X = %f, Y = %f, Z = %f", nearest_point.x, nearest_point.y, nearest_point.z);
    nearest_point_pub.publish(nearest_point_msg);
}

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "object_position_estimation");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 1, Cloud_CB);
  //ros::Subscriber sub_point = nh.subscribe("/output_points", 1, PointCloudConversion2D);
  //ros::ServiceServer srv = nh.advertiseService("point_estimation",Point_CB);
  // Create a ROS publisher for the output image
  //image_pub = nh.advertise<sensor_msgs::Image>("/output_image_topic", 1000);
  nearest_point_pub = nh.advertise<geometry_msgs::Point>("nearest_point_topic", 1);
  ROS_INFO("Ready to Server.");

  // Spin
  ros::spin();
  return 0;
}

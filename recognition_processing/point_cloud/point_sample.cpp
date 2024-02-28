#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

ros::Publisher pub;

void cloud_cb(sensor_msgs::PointCloud2 input) {
  // Convert PointCloud2 to PCL PointCloud with color information
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(input, *cloud);

  // Statistical Outlier Removal
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(50);  // Number of neighbors to analyze for each point
  sor.setStddevMulThresh(1.0);  // Standard deviation multiplier threshold
  sor.filter(*cloud_filtered);

  // Distance-based Masking
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  double min_distance = 0.001;  // Minimum distance
  double max_distance = 0.5;  // Maximum distance

  for (const auto& point : cloud_filtered->points) {
    double distance = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (distance >= min_distance && distance <= max_distance) {
      filtered_cloud->points.push_back(point);
    }
  }

  // Convert filtered PCL PointCloud back to PointCloud2
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*filtered_cloud, output);
  output.header = input.header;

  // Publish the filtered data
  pub.publish(output);
}

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "point_sample");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("camera/depth/color/points", 10, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("output_points", 10);

  // Spin
  ros::spin();
  return 0;
}

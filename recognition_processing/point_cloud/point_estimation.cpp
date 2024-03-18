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

std::vector<pcl::PointXYZRGB> cluster_centers;  // Vector to store cluster centers
ros::Publisher nearest_point_pub;

void Cloud_CB(sensor_msgs::PointCloud2 input) {
  // Convert PointCloud2 to PCL PointCloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(input, *cloud);

  // Voxel Grid Downsampling
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*cloud);

  // Euclidean Cluster Extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(0.02);  // 2cm
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

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

   // Print the nearest point's coordinates (Z, X, Y)
  ROS_INFO("Nearest point coordinates: Z = %f, X = %f, Y = %f", nearest_point.z, nearest_point.x, nearest_point.y);

  geometry_msgs::Point nearest_point_msg;
  nearest_point_msg.x = nearest_point.x;
  nearest_point_msg.y = nearest_point.y;
  nearest_point_msg.z = nearest_point.z;
  nearest_point_pub.publish(nearest_point_msg);
  // Clear the vector for the next set of data
  cluster_centers.clear();
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

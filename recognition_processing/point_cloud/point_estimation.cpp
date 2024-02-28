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

void cloud_cb(sensor_msgs::PointCloud2 input) {
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

  // Calculate center of each cluster
  for (const pcl::PointIndices& indices : cluster_indices) {
    pcl::PointXYZRGB centroid;
    centroid.x = 0.0;
    centroid.y = 0.0;
    centroid.z = 0.0;

    for (int index : indices.indices) {
      centroid.x += cloud->points[index].x;
      centroid.y += cloud->points[index].y;
      centroid.z += cloud->points[index].z;
    }

    centroid.x /= indices.indices.size();
    centroid.y /= indices.indices.size();
    centroid.z /= indices.indices.size();

    cluster_centers.push_back(centroid);
  }

   // Check if we have collected 10 data points
  if (cluster_centers.size() == 10) {
    // Calculate average of the collected data points
    pcl::PointXYZRGB average_center;
    average_center.x = 0.0;
    average_center.y = 0.0;
    average_center.z = 0.0;

    for (const pcl::PointXYZRGB& center : cluster_centers) {
      average_center.x += center.x;
      average_center.y += center.y;
      average_center.z += center.z;
    }

    average_center.x /= cluster_centers.size();
    average_center.y /= cluster_centers.size();
    average_center.z /= cluster_centers.size();

    // Display the average center
    ROS_INFO("Average Center: (%f, %f, %f)", average_center.x, average_center.y, average_center.z);

    // Clear the vector for the next set of data
    cluster_centers.clear();
  }
}


// Service callback to return the average position
bool point_cb(happymimi_recognition_msgs::PositionEstimatorPCL::Request& req,
              happymimi_recognition_msgs::PositionEstimatorPCL::Response& res) {
  if (cluster_centers.size() >= 1) {
    // Calculate average of the collected data points
    pcl::PointXYZRGB average_center;
    average_center.x = 0.0;
    average_center.y = 0.0;
    average_center.z = 0.0;

    for (const pcl::PointXYZRGB& center : cluster_centers) {
      average_center.x += center.x;
      average_center.y += center.y;
      average_center.z += center.z;
    }

    average_center.x /= cluster_centers.size();
    average_center.y /= cluster_centers.size();
    average_center.z /= cluster_centers.size();

    // Fill the response with the average position
    res.average_position.x = average_center.x;
    res.average_position.y = average_center.y;
    res.average_position.z = average_center.z;

    // Clear the vector for the next set of data
    cluster_centers.clear();

    return true;
  } else {
    ROS_WARN("Not enough data for average calculation.");
    return false;
  }
}

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "object_position_estimation");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/output_points", 1, cloud_cb);
  ros::ServiceServer srv = nh.advertiseService("point_estimation",point_cb);
  ROS_INFO("Ready to Server.");

  // Spin
  ros::spin();
  return 0;
}

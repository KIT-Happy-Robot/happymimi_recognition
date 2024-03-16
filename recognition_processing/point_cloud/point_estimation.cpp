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
ros::Publisher image_pub;

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
  // Clear the vector for the next set of data
  cluster_centers.clear();
}

void PointCloudConversion2D(sensor_msgs::PointCloud2 input){
  // Convert PointCloud2 to PCL PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(input, *cloud);

  // Initialize ROS Image message
  sensor_msgs::Image image_msg;
  image_msg.header = input.header;
  image_msg.height = 480; // Set image height
  image_msg.width = 640;  // Set image width
  image_msg.encoding = "rgb8"; // Set image encoding (monochrome, 8-bit)
  image_msg.step = image_msg.width; // Full row stride
  size_t image_size = image_msg.width * image_msg.height;
  image_msg.data.resize(image_size);

  // Project point cloud to 2D image
  for (const auto& point : cloud->points) {
      // Skip points outside image bounds
      if (point.x < 0 || point.x >= image_msg.width || point.y < 0 || point.y >= image_msg.height) {
          continue;
      }
      // Calculate index in image message data array
      size_t index = static_cast<size_t>(point.y) * image_msg.width + static_cast<size_t>(point.x);
      // Set pixel value (e.g., 255 for white)
      image_msg.data[index] = 255;
  }

  // Publish the image message
  image_pub.publish(image_msg);
}

// Service callback to return the average position
bool Point_CB(happymimi_recognition_msgs::PositionEstimatorPCL::Request& req,
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
  ros::Subscriber sub = nh.subscribe("/output_points", 1, Cloud_CB);
  //ros::Subscriber sub_point = nh.subscribe("/output_points", 1, PointCloudConversion2D);
  ros::ServiceServer srv = nh.advertiseService("point_estimation",Point_CB);
  // Create a ROS publisher for the output image
  //image_pub = nh.advertise<sensor_msgs::Image>("/output_image_topic", 1000);
  ROS_INFO("Ready to Server.");

  // Spin
  ros::spin();
  return 0;
}

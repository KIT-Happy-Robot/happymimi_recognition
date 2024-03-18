#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(PointCloud::ConstPtr cloud) {
    // Create a PCLVisualizer object
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0); // Set background color to black

    // Add point cloud data
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0); // Green
    viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    // Viewer options
    viewer->addCoordinateSystem(1.0); // Show XYZ axes
    viewer->initCameraParameters();   // Initialize camera parameters

    return (viewer);
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*cloud_msg, *cloud); // Convert PointCloud2 ROS message to PCL format

    // Visualize the point cloud
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = simpleVis(cloud);
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        ros::spinOnce(); // Handle ROS events
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_visualizer");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, cloudCallback);

    ros::spin();

    return 0;
}

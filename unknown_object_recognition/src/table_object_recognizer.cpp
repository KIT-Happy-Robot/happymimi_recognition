'''
ヘッド・アームカメラでテーブル平面上の物体の認識や重心3次元位置・姿勢を推定するノード
IN:
  - camera_name
  - mode (horizon, three degree)
OUT:
  - geometry_msgs/Poinst
Author: washio
'''
#include <string>  
#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <sensor_msgs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <happymimi_recognition_msgs/PositionEstimatorPCL.h>
// ds
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
// plane
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
// cls
#include <pcl/segmentation/extract_clusters.h> //
#include <pcl/segmentation/supervoxel_clustering.h> //
// viz
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <visualization_msgs/Marker.h>

class TableObjectRecognizer
{
private:
  ros::NodeHandle nh;
  ros::Subscriber pc1_sub;
  ros::Subscriber pc2_sub;
  ros::Publisher horizon1_pub;
  ros::Publisher horizon2_pub;
  ros::ServiceServer pppc_ss_;
  PEPCL = happymimi_recognition_msgs::PositionEstimatorPCL;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr head_pc_; // 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr camera1_pc_; // 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr camera2_pc_; // 
  pcl::visualization::PCLVisualizer::Ptr pppc_viewer(new pcl::visualization::PCLVisualizer("PPPC Viewer"));
  pcl::visualization::PCLVisualizer viewer{"pc_normals"}; //点群,法線の視覚化
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};// 点群
  pcl::PointCloud<pcl::PointNormal>::Ptr normals {new pcl::PointCloud<pcl::PointNormal>}; //点群+法線ベクトル
  double depth_thres_;
  double cls_three_torelace, cls_inlier_torelace;
  int cls_three_min_size, cls_three_max_size, cls_inlier_min_size, cls_inlier_min_size
  std::string filter_inlier_axis="x";
  std::map<std::string, std::vector<bool>> pcp_config;

public:
  PointCloudProcessor() {
    ROS_INFO("PointCloudProcessor: Initializing...");
    head_sub = nh.subscribe("/camera/depth/color/points", 1, &PointCloudProcessor::pc1CB, this);
    pc1_sub = nh.subscribe("/camera1/depth/color/points", 1, &PointCloudProcessor::pc1CB, this);
    pc2_sub = nh.subscribe("/camera2/depth/color/points", 1, &PointCloudProcessor::pc2CB, this);
    ros::Subscriber load_param_sub = nh.subscribe("/uor/pcp/load_param", 1, &PointCloudProcessor::loadParamCB, this);
    horizon_ss_ = nh_.advertiseService("/uor/pcp/horizon_centroids", &PointCloudProcessor::horizonRecSCB, this);

    head_pc_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    camera1_pc_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    camera2_pc_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    ///
    viewer.setBackgroundColor(1, 1, 1);
	  viewer.addCoordinateSystem(0.5, "axis");
    // params
    //nh.param<double>("/uor/pcp_config", horizon_flg, false);
    ros::param::get("/uor/pcp_config", pcp_config);
    // for (const auto& entry : pcp_config){
    //   ROS_INFO_STREAM("Category: " << entry.first);
    //   for (const auto& item : entry.second){ROS_INFO_STREAM("  - " << (item ? "True" : "False"))}
    // }
    bool distance_thres_flg, downsample_flg, pppc_flg, horizon_flg, horizon_centroids_flg, table_plane_flg, table_object_cluser_flg, foreground_centroid_flg;
    if (rosparam_shortcuts::get("debug", pcp_config, "distance_thres", distance_thres_flg) &&
        rosparam_shortcuts::get("debug", pcp_config, "downsample", downsample_flg) &&
        rosparam_shortcuts::get("debug", pcp_config, "pppc", pppc_flg) &&
        rosparam_shortcuts::get("debug", pcp_config, "horizon", horizon_flg) &&
        rosparam_shortcuts::get("debug", pcp_config, "horizon_centroids", horizon_centroids_flg) &&
        rosparam_shortcuts::get("debug", pcp_config, "table_plane", table_plane_flg) &&
        rosparam_shortcuts::get("debug", pcp_config, "table_object_cluser", table_object_cluster_flg) &&
        rosparam_shortcuts::get("debug", pcp_config, "foreground_centroid", foreground_centroid_flg)){ 
      ROS_INFO("Parameters successfully retrieved.")}
    else{ ROS_WARN("Failed to retrieve parameters.")}
    this->loadParameters()
    if (distance_thres_flg){printf("")}
    if (horizon_flg){
      //horizon_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 1);
      horizon1_pub = nh.advertise<sensor_msgs::PointCloud2>("/camera1/horizon_points", 1); ///
      horizon2_pub = nh.advertise<sensor_msgs::PointCloud2>("/camera2/horizon_points", 1); ///
      }
    ROS_INFO("PointCloudProcessor: I'm ready ...");
  }

  void loadParameters(){
    if (rosparam_shortcuts::get("thres", pcp_config, "depth", depth_thres) &&
        rosparam_shortcuts::get("clustering/three/tolerance", pcp_config, cls_three_torelace) &&
        rosparam_shortcuts::get("clustering/three/min_size", pcp_config, cls_three_min_size) &&
        rosparam_shortcuts::get("clustering/three/max_size", pcp_config, cls_three_max_size:) &&
        rosparam_shortcuts::get("clustering/inlier/tolerance", pcp_config, cls_inlier_torelace) &&
        rosparam_shortcuts::get("clustering/inlier/min_size", pcp_config, cls_inlier_min_size) &&
        rosparam_shortcuts::get("clustering/inlier/max_size", pcp_config, cls_inlier_max_size:) &&
        rosparam_shortcuts::get("thres", pcp_config, "depth", depth_thres)){ 
      ROS_INFO("Parameters successfully retrieved.")}
    else{ ROS_WARN("Failed to retrieve parameters."); return false}
  }

  void loadParamCB(std_msgs::Empty _){this->loadParam()}

  // Head camera
  void pc1CB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::fromROSMsg(*cloud_msg, *head_pc_msg);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    int width = cloud->width;
    int height = cloud->height;
    int y_index = height / 2;
    for (int x = 0; x < width; ++x) {
      pcl::PointXYZ point = cloud->at(x, y_index);
      if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)) { //有限の値
        filtered_cloud->push_back(point);
      }
    }
    // Publish the filtered point cloud
    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
    filtered_cloud_msg.header = cloud_msg->header;
    horizon1_pub.publish(filtered_cloud_msg);
  }

  void pc2CB(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::fromROSMsg(*msg, *camera2_pc_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    int width = cloud->width;
    int height = cloud->height;
    int y_index = height / 2;
    for (int x = 0; x < width; ++x) {
      pcl::PointXYZ point = cloud->at(x, y_index);
      if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)) { //有限の値
        filtered_cloud->push_back(point);
      }
    }
    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
    filtered_cloud_msg.header = msg->header;
    horizon2_pub.publish(filtered_cloud_msg);
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractInlierCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, std::string axis="x") {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (axis=="x"){
      int width = cloud->width;
      int height = cloud->height;
      int y_index = height / 2;
      for (int x = 0; x < width; ++x) {
        pcl::PointXYZ point = cloud->at(x, y_index);
        if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)) { //有限の値
          filtered_cloud->push_back(point);
        }
      }
    }
    // Publish the filtered point cloud
    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
    filtered_cloud_msg.header = cloud_msg->header;
    horizon1_pub.publish(filtered_cloud_msg);
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud) {
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(input_cloud);
        sor.setLeafSize(0.01f, 0.01f, 0.01f);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        sor.filter(*output_cloud);
        return output_cloud;
  }
  
  void visDownsample(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud){
    cloud = downsample(input_cloud);
    pppc_viewer->setBackgroundColor(0, 0, 0);
    pppc_viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
    pppc_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    pppc_viewer->spin();
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterByAxisDistance(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, std::string axis, double min, double max) {
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(input_cloud);
    pass.setFilterFieldName(axis);
    pass.setFilterLimits(min, max);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pass.filter(*output_cloud);
    return output_cloud;
  }

  // ユークリッド距離
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterByDepthDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, double distance_threshold) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices); //距離閾値内にある点群のインデックスのリスト
    pcl::ExtractIndices<pcl::PointXYZ> extract; //指定インデックスのセットを抽出するクラス
    for (size_t i = 0; i < input_cloud->points.size(); ++i) {
        if (std::sqrt(input_cloud->points[i].x * input_cloud->points[i].x +
                      input_cloud->points[i].y * input_cloud->points[i].y +
                      input_cloud->points[i].z * input_cloud->points[i].z) < distance_threshold) {
            inliers->indices.push_back(i);
        }
    }
    extract.setInputCloud(input_cloud); //抽出操作の対象を設定
    extract.setIndices(inliers); //抽出する点のインデックスを指定
    extract.setNegative(false); //F:指定したインデックスの点を抽出する | T:除外する
    extract.filter(*output_cloud); //実行
    return output_cloud;
  }

  void clusterInlierCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters) {
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cls_inlier_torelace); //Cluster tolerance: 2cm
    ec.setMinClusterSize(cls_inlier_min_size); //Minimum cluster size: 100 points
    ec.setMaxClusterSize(cls_inlier_max); //Maximum cluster size: 25000 points
    ec.setInputCloud(input_cloud);
    std::vector<pcl::PointIndices> cluster_indices; ///
    ec.extract(cluster_indices);
    for (const auto& indice : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point_index : indice.indices) {
            cluster->points.push_back(input_cloud->points[point_index]);
        }
        cluster->width = cluster->points.size(); //幅を点の数と同じに設定
        cluster->height = 1; //高さを1に設定
        cluster->is_dense = true; //クラスタが密な点群であることを示すフラグ
        clusters.push_back(cluster); //各クラスタはclusterスマートポインタを介して保持
    }
  }

  pcl::PointXYZ findShallowestCentroid(const std::vector<pcl::PointXYZ>& centroids) {
    float shallowest_depth = std::numeric_limits<float>::max();
    pcl::PointXYZ shallowest_centroid;
    for (const auto& centroid : centroids) {
        float depth = std::sqrt(centroid.x * centroid.x + centroid.y * centroid.y + centroid.z * centroid.z);
        if (depth < shallowest_depth) {
            shallowest_depth = depth;
            shallowest_centroid = centroid;
        }
    }
    return shallowest_centroid;
  }

  void horizonRecSCB(PEPCL::Request &req, PEPCL::Response &res){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_cloud = downsample(head_pc_);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inlier_cloud = extractInlierCloud(ds_cloud, filter_inlier_axis);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr depth_filterd_cloud = filterByDepthDistance(inlier_cloud, depth_thres_)
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    ClusterInlierCloud(depth_filterd_cloud, clusters) ///
    // Get the centroids of the clusters
    std::vector<pcl::PointXYZ> centroids;
    for (const auto& cluster : clusters) {
      pcl::PointXYZ centroid;
      pcl::computeCentroid(*cluster, centroid);
      centroids.push_back(centroid);
    }
    pcl::PointXYZ shallowest_centroid = findShallowestCentroid(centroids);
    geometry_msgs::Point shallowest_centroid_msg = pclPointXYZToGeometryMsg(shallowest_centroid); ///
    res.shallowest_centroid = shallowest_centroid_msg;
    return true;
  }
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "point_cloud_processor"); //"table_object_reconizer");
  PointCloudProcessor PCP;
  ros::spin ();// Spin
}

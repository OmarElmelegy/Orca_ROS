#include "orca_perception/length_measurement.h"

namespace orca_perception {

LengthMeasurement::LengthMeasurement(ros::NodeHandle& nh) : nh_(nh) {
  // Get parameters
  nh_.param<double>("min_distance", min_distance_, 0.1);
  nh_.param<double>("max_distance", max_distance_, 2.0);
  nh_.param<double>("cluster_tolerance", cluster_tolerance_, 0.02);
  nh_.param<int>("min_cluster_size", min_cluster_size_, 100);
  nh_.param<int>("max_cluster_size", max_cluster_size_, 25000);

  // Initialize subscribers and publishers
  cloud_sub_ = nh_.subscribe("/realsense/depth/points", 1, &LengthMeasurement::pointCloudCallback, this);
  length_pub_ = nh_.advertise<std_msgs::Float64>("object_length", 1);
  filtered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
  segmented_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("segmented_cloud", 1);

  ROS_INFO("Length measurement node initialized");
}

LengthMeasurement::~LengthMeasurement() {
}

void LengthMeasurement::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  // Convert ROS message to PCL point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*msg, *cloud);

  // Process point cloud to measure object length
  double length = processPointCloud(cloud);

  // Publish length
  std_msgs::Float64 length_msg;
  length_msg.data = length;
  length_pub_.publish(length_msg);

  ROS_INFO("Measured object length: %.3f meters", length);
}

double LengthMeasurement::processPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
  // Filter point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  filterPointCloud(cloud, filtered_cloud);

  // Publish filtered cloud
  sensor_msgs::PointCloud2 filtered_cloud_msg;
  pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
  filtered_cloud_msg.header.frame_id = "realsense_depth_frame";
  filtered_cloud_msg.header.stamp = ros::Time::now();
  filtered_cloud_pub_.publish(filtered_cloud_msg);

  // Segment objects
  std::vector<pcl::PointIndices> clusters;
  segmentObjects(filtered_cloud, clusters);

  // If no objects found, return 0
  if (clusters.empty()) {
    ROS_WARN("No objects found in point cloud");
    return 0.0;
  }

  // Find the largest cluster
  size_t largest_cluster_idx = 0;
  size_t largest_cluster_size = 0;
  for (size_t i = 0; i < clusters.size(); ++i) {
    if (clusters[i].indices.size() > largest_cluster_size) {
      largest_cluster_size = clusters[i].indices.size();
      largest_cluster_idx = i;
    }
  }

  // Extract the largest cluster
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(filtered_cloud);
  extract.setIndices(boost::make_shared<pcl::PointIndices>(clusters[largest_cluster_idx]));
  extract.setNegative(false);
  extract.filter(*segmented_cloud);

  // Publish segmented cloud
  sensor_msgs::PointCloud2 segmented_cloud_msg;
  pcl::toROSMsg(*segmented_cloud, segmented_cloud_msg);
  segmented_cloud_msg.header.frame_id = "realsense_depth_frame";
  segmented_cloud_msg.header.stamp = ros::Time::now();
  segmented_cloud_pub_.publish(segmented_cloud_msg);

  // Measure the length of the largest cluster
  double length = measureClusterLength(filtered_cloud, clusters[largest_cluster_idx]);

  return length;
}

void LengthMeasurement::filterPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered_cloud) {
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min_distance_, max_distance_);
  pass.filter(*filtered_cloud);
}

void LengthMeasurement::segmentObjects(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                                      std::vector<pcl::PointIndices>& clusters) {
  // Create a KD-Tree for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud);

  // Create the clustering object
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(clusters);
}

double LengthMeasurement::measureClusterLength(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                                             const pcl::PointIndices& cluster) {
  // Find the minimum and maximum points in each dimension
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double min_z = std::numeric_limits<double>::max();
  double max_x = -std::numeric_limits<double>::max();
  double max_y = -std::numeric_limits<double>::max();
  double max_z = -std::numeric_limits<double>::max();

  for (const auto& idx : cluster.indices) {
    const pcl::PointXYZRGB& point = cloud->points[idx];
    min_x = std::min(min_x, static_cast<double>(point.x));
    min_y = std::min(min_y, static_cast<double>(point.y));
    min_z = std::min(min_z, static_cast<double>(point.z));
    max_x = std::max(max_x, static_cast<double>(point.x));
    max_y = std::max(max_y, static_cast<double>(point.y));
    max_z = std::max(max_z, static_cast<double>(point.z));
  }

  // Calculate dimensions
  double length_x = max_x - min_x;
  double length_y = max_y - min_y;
  double length_z = max_z - min_z;

  // Return the maximum dimension as the length
  return std::max(std::max(length_x, length_y), length_z);
}

}  // namespace orca_perception

int main(int argc, char** argv) {
  ros::init(argc, argv, "length_measurement_node");
  ros::NodeHandle nh;

  orca_perception::LengthMeasurement length_measurement(nh);

  ros::spin();

  return 0;
}

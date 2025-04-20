#ifndef ORCA_PERCEPTION_LENGTH_MEASUREMENT_H
#define ORCA_PERCEPTION_LENGTH_MEASUREMENT_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>

namespace orca_perception {

/**
 * @brief Length measurement using point cloud data
 * 
 * This class implements a length measurement algorithm using
 * point cloud data from a depth camera. It segments objects
 * and measures their length.
 */
class LengthMeasurement {
public:
  /**
   * @brief Constructor
   * @param nh ROS node handle
   */
  LengthMeasurement(ros::NodeHandle& nh);

  /**
   * @brief Destructor
   */
  ~LengthMeasurement();

private:
  /**
   * @brief Callback for point cloud data
   * @param msg Point cloud message
   */
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

  /**
   * @brief Process point cloud to measure object length
   * @param cloud Input point cloud
   * @return Measured length in meters
   */
  double processPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

  /**
   * @brief Filter point cloud to remove noise and outliers
   * @param cloud Input point cloud
   * @param filtered_cloud Output filtered point cloud
   */
  void filterPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered_cloud);

  /**
   * @brief Segment objects from the point cloud
   * @param cloud Input point cloud
   * @param clusters Output clusters
   */
  void segmentObjects(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                     std::vector<pcl::PointIndices>& clusters);

  /**
   * @brief Measure the length of a cluster
   * @param cloud Input point cloud
   * @param cluster Cluster indices
   * @return Length in meters
   */
  double measureClusterLength(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                             const pcl::PointIndices& cluster);

  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher length_pub_;
  ros::Publisher filtered_cloud_pub_;
  ros::Publisher segmented_cloud_pub_;

  // Parameters
  double min_distance_;
  double max_distance_;
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
};

}  // namespace orca_perception

#endif  // ORCA_PERCEPTION_LENGTH_MEASUREMENT_H

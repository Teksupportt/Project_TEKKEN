#pragma once

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "perception_msgs/msg/target_detection.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace perception
{

class Lidar3DTracker : public rclcpp::Node
{
public:
  Lidar3DTracker();

private:
  void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Downsamples, ground-filters, then Euclidean-clusters the cloud, and
  // selects the closest cluster within a plausible target size.
  bool detectTarget(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    double & out_x,
    double & out_y,
    std::array<double, 4> & out_cov) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<perception_msgs::msg::TargetDetection>::SharedPtr detection_pub_;

  // Parameters
  std::string sensor_id_;
  double max_range_;
  double voxel_leaf_size_;
  double ground_z_threshold_;
  double cluster_tolerance_;
  int min_cluster_points_;
  int max_cluster_points_;
  double min_cluster_width_;
  double max_cluster_width_;
  double position_noise_stddev_;
};

}  // namespace perception

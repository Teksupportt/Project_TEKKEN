#include "perception/lidar_3d_tracker.hpp"

#include <cmath>
#include <limits>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

namespace perception
{

Lidar3DTracker::Lidar3DTracker()
: Node("lidar_3d_tracker")
{
  sensor_id_ = this->declare_parameter<std::string>("sensor_id", "lidar_3d_1");
  max_range_ = this->declare_parameter<double>("max_range", 8.0);
  voxel_leaf_size_ = this->declare_parameter<double>("voxel_leaf_size", 0.05);
  ground_z_threshold_ = this->declare_parameter<double>("ground_z_threshold", 0.1);
  cluster_tolerance_ = this->declare_parameter<double>("cluster_tolerance", 0.3);
  min_cluster_points_ = this->declare_parameter<int>("min_cluster_points", 10);
  max_cluster_points_ = this->declare_parameter<int>("max_cluster_points", 5000);
  min_cluster_width_ = this->declare_parameter<double>("min_cluster_width", 0.15);
  max_cluster_width_ = this->declare_parameter<double>("max_cluster_width", 1.0);
  position_noise_stddev_ = this->declare_parameter<double>("position_noise_stddev", 0.03);

  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points", rclcpp::SensorDataQoS(),
    std::bind(&Lidar3DTracker::onPointCloud, this, std::placeholders::_1));

  detection_pub_ = this->create_publisher<perception_msgs::msg::TargetDetection>(
    "detection", rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(), "lidar_3d_tracker started, sensor_id=%s", sensor_id_.c_str());
}

bool Lidar3DTracker::detectTarget(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  double & out_x,
  double & out_y,
  std::array<double, 4> & out_cov) const
{
  if (cloud->empty()) {
    return false;
  }

  // Range filter — discard anything beyond max_range_ before doing
  // any heavier processing, keeps downstream steps cheap.
  pcl::PointCloud<pcl::PointXYZ>::Ptr ranged(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> range_filter;
  range_filter.setInputCloud(cloud);
  range_filter.setFilterFieldName("x");
  range_filter.setFilterLimits(0.0, max_range_);
  range_filter.filter(*ranged);

  // Ground removal — simple z-height threshold rather than full plane
  // segmentation (e.g. RANSAC), cheaper and sufficient if the sensor's
  // mounting height/tilt is roughly known and fixed.
  pcl::PointCloud<pcl::PointXYZ>::Ptr above_ground(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> ground_filter;
  ground_filter.setInputCloud(ranged);
  ground_filter.setFilterFieldName("z");
  ground_filter.setFilterLimits(ground_z_threshold_, std::numeric_limits<float>::max());
  ground_filter.filter(*above_ground);

  // Voxel downsample — reduces point count before clustering, this is
  // the main lever for keeping 3D processing cheap.
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(above_ground);
  voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  voxel_filter.filter(*downsampled);

  if (downsampled->empty()) {
    return false;
  }

  // Euclidean clustering via KD-tree — PCL's standard, efficient approach
  // for grouping nearby 3D points into candidate objects.
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(downsampled);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
  clustering.setClusterTolerance(cluster_tolerance_);
  clustering.setMinClusterSize(min_cluster_points_);
  clustering.setMaxClusterSize(max_cluster_points_);
  clustering.setSearchMethod(tree);
  clustering.setInputCloud(downsampled);
  clustering.extract(cluster_indices);

  double best_range = std::numeric_limits<double>::max();
  bool found = false;
  double best_x = 0.0, best_y = 0.0;

  for (const auto & indices : cluster_indices) {
    // Bounding-box width in the horizontal plane, as a cheap
    // "opponent-sized" proxy, same idea as the 2D tracker's width filter.
    double min_x = std::numeric_limits<double>::max(), max_x = -std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max(), max_y = -std::numeric_limits<double>::max();
    double sum_x = 0.0, sum_y = 0.0;

    for (const auto idx : indices.indices) {
      const auto & pt = downsampled->points[idx];
      min_x = std::min(min_x, static_cast<double>(pt.x));
      max_x = std::max(max_x, static_cast<double>(pt.x));
      min_y = std::min(min_y, static_cast<double>(pt.y));
      max_y = std::max(max_y, static_cast<double>(pt.y));
      sum_x += pt.x;
      sum_y += pt.y;
    }

    const double width = std::max(max_x - min_x, max_y - min_y);
    if (width < min_cluster_width_ || width > max_cluster_width_) {
      continue;
    }

    const int count = static_cast<int>(indices.indices.size());
    const double cx = sum_x / count;
    const double cy = sum_y / count;
    const double range = std::sqrt(cx * cx + cy * cy);

    if (range < best_range) {
      best_range = range;
      best_x = cx;
      best_y = cy;
      found = true;
    }
  }

  if (!found) {
    return false;
  }

  out_x = best_x;
  out_y = best_y;

  // Simple isotropic covariance from a flat position noise assumption —
  // 3D lidar range precision doesn't have the same range/bearing coupling
  // a 2D scanning lidar has, so a flat model is a reasonable simplification.
  const double var = position_noise_stddev_ * position_noise_stddev_;
  out_cov = {var, 0.0, 0.0, var};

  return true;
}

void Lidar3DTracker::onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  perception_msgs::msg::TargetDetection detection;
  detection.header.stamp = msg->header.stamp;
  detection.header.frame_id = msg->header.frame_id;
  detection.sensor_id = sensor_id_;

  double x, y;
  std::array<double, 4> cov;
  if (detectTarget(cloud, x, y, cov)) {
    detection.valid = true;
    detection.position.x = x;
    detection.position.y = y;
    detection.position.z = 0.0;
    detection.covariance.data = cov;
  } else {
    detection.valid = false;
  }

  detection_pub_->publish(detection);
}

}  // namespace perception

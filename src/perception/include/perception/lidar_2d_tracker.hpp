#pragma once

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "perception_msgs/msg/target_detection.hpp"

namespace perception
{

// A simple 2D point, used internally for clustering.
struct Point2D
{
  double x;
  double y;
};

class Lidar2DTracker : public rclcpp::Node
{
public:
  Lidar2DTracker();

private:
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  // Converts raw ranges into Cartesian points, discarding out-of-range/invalid readings.
  std::vector<Point2D> scanToPoints(const sensor_msgs::msg::LaserScan & scan) const;

  // Groups points into clusters based on distance between consecutive points.
  // Returns clusters as lists of point indices into the input vector.
  std::vector<std::vector<size_t>> clusterPoints(const std::vector<Point2D> & points) const;

  // Picks the best candidate cluster (closest cluster within plausible target size),
  // returns its centroid and a simple range/bearing-based covariance.
  bool selectTargetCluster(
    const std::vector<Point2D> & points,
    const std::vector<std::vector<size_t>> & clusters,
    double & out_x,
    double & out_y,
    std::array<double, 4> & out_cov) const;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<perception_msgs::msg::TargetDetection>::SharedPtr detection_pub_;

  // Parameters
  std::string sensor_id_;
  double max_range_;
  double cluster_distance_threshold_;
  int min_cluster_points_;
  int max_cluster_points_;
  double min_cluster_width_;
  double max_cluster_width_;
  double range_noise_stddev_;
  double bearing_noise_stddev_;
};

}  // namespace perception

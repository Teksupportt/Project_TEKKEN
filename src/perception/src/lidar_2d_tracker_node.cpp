#include "perception/lidar_2d_tracker.hpp"

#include <cmath>
#include <limits>

namespace perception
{

Lidar2DTracker::Lidar2DTracker()
: Node("lidar_2d_tracker")
{
  sensor_id_ = this->declare_parameter<std::string>("sensor_id", "lidar_2d_1");
  max_range_ = this->declare_parameter<double>("max_range", 8.0);
  cluster_distance_threshold_ = this->declare_parameter<double>("cluster_distance_threshold", 0.3);
  min_cluster_points_ = this->declare_parameter<int>("min_cluster_points", 3);
  max_cluster_points_ = this->declare_parameter<int>("max_cluster_points", 60);
  min_cluster_width_ = this->declare_parameter<double>("min_cluster_width", 0.15);
  max_cluster_width_ = this->declare_parameter<double>("max_cluster_width", 1.0);
  range_noise_stddev_ = this->declare_parameter<double>("range_noise_stddev", 0.03);
  bearing_noise_stddev_ = this->declare_parameter<double>("bearing_noise_stddev", 0.02);

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
    std::bind(&Lidar2DTracker::onScan, this, std::placeholders::_1));

  detection_pub_ = this->create_publisher<perception_msgs::msg::TargetDetection>(
    "detection", rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(), "lidar_2d_tracker started, sensor_id=%s", sensor_id_.c_str());
}

std::vector<Point2D> Lidar2DTracker::scanToPoints(const sensor_msgs::msg::LaserScan & scan) const
{
  std::vector<Point2D> points;
  points.reserve(scan.ranges.size());

  double angle = scan.angle_min;
  for (const float range : scan.ranges) {
    if (std::isfinite(range) && range >= scan.range_min && range <= scan.range_max &&
      range <= max_range_)
    {
      points.push_back({range * std::cos(angle), range * std::sin(angle)});
    }
    angle += scan.angle_increment;
  }
  return points;
}

std::vector<std::vector<size_t>> Lidar2DTracker::clusterPoints(
  const std::vector<Point2D> & points) const
{
  std::vector<std::vector<size_t>> clusters;
  if (points.empty()) {
    return clusters;
  }

  std::vector<size_t> current_cluster{0};
  for (size_t i = 1; i < points.size(); ++i) {
    const double dx = points[i].x - points[i - 1].x;
    const double dy = points[i].y - points[i - 1].y;
    const double dist = std::sqrt(dx * dx + dy * dy);

    if (dist <= cluster_distance_threshold_) {
      current_cluster.push_back(i);
    } else {
      clusters.push_back(current_cluster);
      current_cluster = {i};
    }
  }
  clusters.push_back(current_cluster);

  return clusters;
}

bool Lidar2DTracker::selectTargetCluster(
  const std::vector<Point2D> & points,
  const std::vector<std::vector<size_t>> & clusters,
  double & out_x,
  double & out_y,
  std::array<double, 4> & out_cov) const
{
  double best_range = std::numeric_limits<double>::max();
  bool found = false;
  double best_x = 0.0, best_y = 0.0;

  for (const auto & cluster : clusters) {
    const int count = static_cast<int>(cluster.size());
    if (count < min_cluster_points_ || count > max_cluster_points_) {
      continue;
    }

    // Cluster width = distance between its first and last point, a cheap
    // proxy for "is this roughly opponent-sized" without full shape fitting.
    const auto & p_first = points[cluster.front()];
    const auto & p_last = points[cluster.back()];
    const double width = std::sqrt(
      std::pow(p_last.x - p_first.x, 2) + std::pow(p_last.y - p_first.y, 2));

    if (width < min_cluster_width_ || width > max_cluster_width_) {
      continue;
    }

    // Centroid of the cluster
    double sum_x = 0.0, sum_y = 0.0;
    for (const auto idx : cluster) {
      sum_x += points[idx].x;
      sum_y += points[idx].y;
    }
    const double cx = sum_x / count;
    const double cy = sum_y / count;
    const double range = std::sqrt(cx * cx + cy * cy);

    // Among valid candidates, pick the closest one — simplest reasonable
    // heuristic for "most likely the opponent directly in front," matches
    // the front-facing sensor assumption.
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

  // Approximate Cartesian covariance from range/bearing noise.
  // For small bearing noise this is a reasonable linear approximation
  // rather than a full nonlinear polar->Cartesian covariance transform,
  // which would need a real EKF-style Jacobian for correctness.
  const double bearing = std::atan2(out_y, out_x);
  const double sr2 = range_noise_stddev_ * range_noise_stddev_;
  const double sb2 = bearing_noise_stddev_ * bearing_noise_stddev_;
  const double range_val = best_range;

  const double var_x = sr2 * std::cos(bearing) * std::cos(bearing) +
    range_val * range_val * sb2 * std::sin(bearing) * std::sin(bearing);
  const double var_y = sr2 * std::sin(bearing) * std::sin(bearing) +
    range_val * range_val * sb2 * std::cos(bearing) * std::cos(bearing);
  const double cov_xy = (sr2 - range_val * range_val * sb2) *
    std::sin(bearing) * std::cos(bearing);

  out_cov = {var_x, cov_xy, cov_xy, var_y};
  return true;
}

void Lidar2DTracker::onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  const auto points = scanToPoints(*msg);
  const auto clusters = clusterPoints(points);

  perception_msgs::msg::TargetDetection detection;
  detection.header.stamp = msg->header.stamp;
  detection.header.frame_id = msg->header.frame_id;
  detection.sensor_id = sensor_id_;

  double x, y;
  std::array<double, 4> cov;
  if (selectTargetCluster(points, clusters, x, y, cov)) {
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

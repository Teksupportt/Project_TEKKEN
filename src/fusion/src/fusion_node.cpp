/*
Sensor fusion ROS2 node
Takes each sensor output topic, fuses them, and
determines where the target is using EKF,
then publishes detection to its own topic.
*/
#include "fusion/fusion_node.hpp"

#include <chrono>
#include <functional>

namespace fusion
{

FusionNode::FusionNode()
: Node("fusion_node")
{
  predict_rate_hz_ = this->declare_parameter<double>("predict_rate_hz", 40.0);
  q_pos_ = this->declare_parameter<double>("q_pos", 0.05);
  q_vel_ = this->declare_parameter<double>("q_vel", 0.5);
  frame_id_ = this->declare_parameter<std::string>("frame_id", "base_link");

  // Arbitrary list of sensor topics — e.g. two 2D lidars, one 3D lidar,
  // one cmaera, all facing forward. Add/remove sensors here without
  // touching fusion logic at all.
  const auto sensor_topics = this->declare_parameter<std::vector<std::string>>(
    "sensor_topics", std::vector<std::string>{"target/raw/lidar", "target/raw/camera"}); // Add/Remove here

  for (const auto & topic : sensor_topics) {
    auto sub = this->create_subscription<perception_msgs::msg::TargetDetection>(
      topic, rclcpp::SensorDataQoS(),
      [this, topic](const perception_msgs::msg::TargetDetection::SharedPtr msg) {
        onDetection(msg, topic);
      });
    subscriptions_.push_back(sub);
  }

  estimate_pub_ = this->create_publisher<perception_msgs::msg::TargetEstimate>(
    "target/fused", rclcpp::QoS(10));

  const auto period = std::chrono::duration<double>(1.0 / predict_rate_hz_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&FusionNode::onTimer, this));

  RCLCPP_INFO(
    this->get_logger(), "fusion_node started at %.1f Hz, tracking %zu sensor topic(s)",
    predict_rate_hz_, sensor_topics.size());
}

void FusionNode::onDetection(
  const perception_msgs::msg::TargetDetection::SharedPtr msg,
  const std::string & topic_name)
{
  if (msg->valid) {
    latest_by_topic_[topic_name] = msg;
  }
}

bool FusionNode::isFresh(
  const perception_msgs::msg::TargetDetection::SharedPtr & cached) const
{
  if (!cached) {
    return false;
  }
  const rclcpp::Time stamp(cached->header.stamp);
  const double age_sec = (this->now() - stamp).seconds();
  return age_sec <= kStalenessTimeoutSec;
}

void FusionNode::onTimer()
{
  const rclcpp::Time now = this->now();

  // --- Predict step ---
  double dt = 1.0 / predict_rate_hz_;
  if (have_last_predict_time_) {
    dt = (now - last_predict_time_).seconds();
  }
  last_predict_time_ = now;
  have_last_predict_time_ = true;

  if (ekf_.isInitialized()) {
    ekf_.predict(dt, q_pos_, q_vel_);
  }

  // --- Update step, looping over however many sensors are currently fresh ---
  std::vector<std::string> contributing_sensors;

  for (const auto & [topic_name, detection] : latest_by_topic_) {
    if (!isFresh(detection)) {
      continue;
    }
    double x, y;
    std::array<double, 4> cov;
    detectionToMeasurement(*detection, x, y, cov);
    ekf_.update(x, y, cov);
    contributing_sensors.push_back(detection->sensor_id);
  }

  // --- Publish ---
  EkfState state = ekf_.state();
  state.valid = ekf_.isInitialized() && !contributing_sensors.empty();

  auto estimate = stateToEstimate(state, contributing_sensors, now, frame_id_);
  estimate_pub_->publish(estimate);
}

}  // namespace fusion

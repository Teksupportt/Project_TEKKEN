#pragma once

#include <array>
#include <string>
#include <vector>

#include "rclcpp/time.hpp"
#include "common_msgs/msg/matrix2.hpp"
#include "common_msgs/msg/matrix4.hpp"
#include "perception_msgs/msg/target_detection.hpp"
#include "perception_msgs/msg/target_estimate.hpp"

namespace fusion
{

// ============================================================
// MATH SECTION — pure C++, no ROS types touched anywhere below
// ============================================================

// Plain-data EKF state, independent of ROS message types.
struct EkfState
{
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  std::array<double, 16> covariance{};  // row-major 4x4
  bool valid{false};
};

// Lightweight constant-velocity EKF, state = [x, y, vx, vy].
// Exploits the sparse/known structure of F and H (position-only
// measurement) to avoid pulling in a general-purpose matrix library
// for what is only ever a 4-state filter.
class ConstantVelocityEkf
{
public:
  ConstantVelocityEkf();

  // Reset/initialize filter state from a first measurement.
  void initialize(double x, double y, const std::array<double, 4> & initial_pos_cov);

  // Time-update step. dt in seconds. q_pos/q_vel are simple diagonal
  // process noise terms for position and velocity states.
  void predict(double dt, double q_pos, double q_vel);

  // Measurement-update step. meas_cov is row-major 2x2 [xx, xy, yx, yy].
  // Re-initializes the filter if it hasn't seen a measurement yet.
  void update(double meas_x, double meas_y, const std::array<double, 4> & meas_cov);

  const EkfState & state() const { return state_; }
  bool isInitialized() const { return initialized_; }

private:
  EkfState state_;
  bool initialized_{false};
};

// ============================================================
// ROS INTERFACE SECTION — the only place ROS message types and
// the pure-math EKF/EkfState types touch each other.
// ============================================================

// Extract position + 2x2 covariance from a single-sensor detection
// message, ready to hand to ConstantVelocityEkf::update().
void detectionToMeasurement(
  const perception_msgs::msg::TargetDetection & detection,
  double & out_x,
  double & out_y,
  std::array<double, 4> & out_cov);

// Package current filter state into the fused output message.
perception_msgs::msg::TargetEstimate stateToEstimate(
  const EkfState & state,
  const std::vector<std::string> & contributing_sensors,
  const rclcpp::Time & stamp,
  const std::string & frame_id);

}  // namespace fusion

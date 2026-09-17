/*
EKF node, Math heavy (because both motion and measurement model are linear, this is really just a linear Kalman filter, not extended):
1: Prediction Covariance: P' = F P F^T + Q
2: Innovation: y = z - Hx, where H selects (x, y) from the state
3: Measurement Covariance: S = H P H^T + R
4: Kalman Gain: K = P H^T S^-1
5: State Update: x = x + K * innovation
6: Covariance Update: P = P - K (H P), where H P is the top two rows of P
*/
#include "fusion/ekf.hpp"

#include <cmath>

namespace fusion
{

ConstantVelocityEkf::ConstantVelocityEkf()
{
  state_.covariance.fill(0.0);
}

void ConstantVelocityEkf::initialize(
  double x, double y, const std::array<double, 4> & initial_pos_cov)
{
  state_.x = x;
  state_.y = y;
  state_.vx = 0.0;
  state_.vy = 0.0;

  state_.covariance.fill(0.0);
  state_.covariance[0 * 4 + 0] = initial_pos_cov[0];
  state_.covariance[0 * 4 + 1] = initial_pos_cov[1];
  state_.covariance[1 * 4 + 0] = initial_pos_cov[2];
  state_.covariance[1 * 4 + 1] = initial_pos_cov[3];
  // No velocity info yet on first detection — start with large uncertainty
  state_.covariance[2 * 4 + 2] = 10.0;
  state_.covariance[3 * 4 + 3] = 10.0;

  state_.valid = true;
  initialized_ = true;
}

void ConstantVelocityEkf::predict(double dt, double q_pos, double q_vel)
{
  if (!initialized_) {
    return;
  }

  // State propagation: x' = x + vx*dt, y' = y + vy*dt, velocities unchanged
  state_.x += state_.vx * dt;
  state_.y += state_.vy * dt;

  auto & P = state_.covariance;  // row-major 4x4, index = row*4 + col

  /* F = [[1,0,dt,0],
          [0,1,0,dt],
          [0,0,1,0],
          [0,0,0,1]]
     P' = F P F^T, computed analytically using F's sparsity rather than
     a generic 4x4 multiply (cheaper, and avoids a matrix-library dependency).
  */

  // Step 1: FP = F * P
  std::array<double, 16> FP{};
  for (int col = 0; col < 4; ++col) {
    FP[0 * 4 + col] = P[0 * 4 + col] + dt * P[2 * 4 + col];
    FP[1 * 4 + col] = P[1 * 4 + col] + dt * P[3 * 4 + col];
    FP[2 * 4 + col] = P[2 * 4 + col];
    FP[3 * 4 + col] = P[3 * 4 + col];
  }

  // Step 2: FPFt = FP * F^T
  std::array<double, 16> FPFt{};
  for (int row = 0; row < 4; ++row) {
    FPFt[row * 4 + 0] = FP[row * 4 + 0] + dt * FP[row * 4 + 2];
    FPFt[row * 4 + 1] = FP[row * 4 + 1] + dt * FP[row * 4 + 3];
    FPFt[row * 4 + 2] = FP[row * 4 + 2];
    FPFt[row * 4 + 3] = FP[row * 4 + 3];
  }

  // Add process noise (simple independent diagonal model)
  FPFt[0 * 4 + 0] += q_pos;
  FPFt[1 * 4 + 1] += q_pos;
  FPFt[2 * 4 + 2] += q_vel;
  FPFt[3 * 4 + 3] += q_vel;

  P = FPFt;
}

void ConstantVelocityEkf::update(
  double meas_x, double meas_y, const std::array<double, 4> & meas_cov)
{
  if (!initialized_) {
    initialize(meas_x, meas_y, meas_cov);
    return;
  }

  auto & P = state_.covariance;

  // Innovation: y = z - Hx, where H selects (x, y) from the state
  const double innov_x = meas_x - state_.x;
  const double innov_y = meas_y - state_.y;

  // S = H P H^T + R. Since H just selects position, H P H^T is the
  // top-left 2x2 block of P.
  const double s00 = P[0 * 4 + 0] + meas_cov[0];
  const double s01 = P[0 * 4 + 1] + meas_cov[1];
  const double s10 = P[1 * 4 + 0] + meas_cov[2];
  const double s11 = P[1 * 4 + 1] + meas_cov[3];

  const double det = s00 * s11 - s01 * s10;
  if (std::abs(det) < 1e-9) {
    // Degenerate measurement covariance — skip this update rather than
    // divide by (near) zero.
    return;
  }
  const double inv_det = 1.0 / det;
  const double sinv00 =  s11 * inv_det;
  const double sinv01 = -s01 * inv_det;
  const double sinv10 = -s10 * inv_det;
  const double sinv11 =  s00 * inv_det;

  // K = P H^T S^-1. P H^T is just the left two columns of P (4x2).
  std::array<double, 8> K{};  // 4x2, row-major: K[row*2+col]
  for (int row = 0; row < 4; ++row) {
    const double p0 = P[row * 4 + 0];
    const double p1 = P[row * 4 + 1];
    K[row * 2 + 0] = p0 * sinv00 + p1 * sinv10;
    K[row * 2 + 1] = p0 * sinv01 + p1 * sinv11;
  }

  // State update: x = x + K * innovation
  state_.x  += K[0 * 2 + 0] * innov_x + K[0 * 2 + 1] * innov_y;
  state_.y  += K[1 * 2 + 0] * innov_x + K[1 * 2 + 1] * innov_y;
  state_.vx += K[2 * 2 + 0] * innov_x + K[2 * 2 + 1] * innov_y;
  state_.vy += K[3 * 2 + 0] * innov_x + K[3 * 2 + 1] * innov_y;

  // Covariance update: P = P - K (H P), where H P is the top two rows of P.
  std::array<double, 16> newP = P;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      const double hp0 = P[0 * 4 + col];
      const double hp1 = P[1 * 4 + col];
      newP[row * 4 + col] -= K[row * 2 + 0] * hp0 + K[row * 2 + 1] * hp1;
    }
  }
  P = newP;

  state_.valid = true;
}

// ============================================================
// ROS INTERFACE SECTION
// ============================================================

void detectionToMeasurement(
  const perception_msgs::msg::TargetDetection & detection,
  double & out_x,
  double & out_y,
  std::array<double, 4> & out_cov)
{
  out_x = detection.position.x;
  out_y = detection.position.y;
  out_cov = detection.covariance.data;
}

perception_msgs::msg::TargetEstimate stateToEstimate(
  const EkfState & state,
  const std::vector<std::string> & contributing_sensors,
  const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  perception_msgs::msg::TargetEstimate estimate;
  estimate.header.stamp = stamp;
  estimate.header.frame_id = frame_id;
  estimate.valid = state.valid;

  estimate.position.x = state.x;
  estimate.position.y = state.y;
  estimate.position.z = 0.0;

  estimate.velocity.x = state.vx;
  estimate.velocity.y = state.vy;
  estimate.velocity.z = 0.0;

  estimate.covariance.data = state.covariance;
  estimate.contributing_sensors = contributing_sensors;

  return estimate;
}

}  // namespace fusion

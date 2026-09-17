#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "perception_msgs/msg/target_estimate.hpp"
#include "heading_controller/rate_limiter.hpp"

namespace heading_controller
{

class HeadingControllerNode : public rclcpp::Node
{
public:
  HeadingControllerNode();

private:
  void onTargetEstimate(const perception_msgs::msg::TargetEstimate::SharedPtr msg);
  void onCmdVelIn(const geometry_msgs::msg::Twist::SharedPtr msg);
  void onTimer();

  // Computes the raw (pre-rate-limit) PD assist yaw command from the
  // current fused target estimate.
  double computeAssistYaw(double dt);

  rclcpp::Subscription<perception_msgs::msg::TargetEstimate>::SharedPtr target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_in_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  perception_msgs::msg::TargetEstimate::SharedPtr latest_target_;
  geometry_msgs::msg::Twist::SharedPtr latest_cmd_vel_in_;

  std::unique_ptr<RateLimiter> rate_limiter_;

  double previous_bearing_error_{0.0};
  bool have_previous_error_{false};
  rclcpp::Time last_cycle_time_;
  bool have_last_cycle_time_{false};

  // Parameters
  double control_rate_hz_;
  double deadzone_;
  double kp_;
  double kd_;
  double max_yaw_rate_;
  double max_yaw_accel_;

  // TODO(smoothing): replace hard switch below with proportional blending
  // between assist and manual yaw as |cmd_vel_in.angular.z| increases,
  // rather than an instant on/off at the deadzone edge.
};

}  // namespace heading_controller
#include "heading_controller/heading_controller_node.hpp"

#include <cmath>

namespace heading_controller
{

HeadingControllerNode::HeadingControllerNode()
: Node("heading_controller_node")
{
  control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 50.0);
  deadzone_ = this->declare_parameter<double>("deadzone", 0.05);
  kp_ = this->declare_parameter<double>("kp", 2.0);
  kd_ = this->declare_parameter<double>("kd", 0.3);
  max_yaw_rate_ = this->declare_parameter<double>("max_yaw_rate", 1.5);
  max_yaw_accel_ = this->declare_parameter<double>("max_yaw_accel", 3.0);

  rate_limiter_ = std::make_unique<RateLimiter>(max_yaw_rate_, max_yaw_accel_);

  target_sub_ = this->create_subscription<perception_msgs::msg::TargetEstimate>(
    "target/fused", rclcpp::QoS(10),
    std::bind(&HeadingControllerNode::onTargetEstimate, this, std::placeholders::_1));

  cmd_vel_in_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_in", rclcpp::QoS(10),
    std::bind(&HeadingControllerNode::onCmdVelIn, this, std::placeholders::_1));

  cmd_vel_out_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel_out", rclcpp::QoS(10));

  const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&HeadingControllerNode::onTimer, this));

  RCLCPP_INFO(this->get_logger(), "heading_controller_node started at %.1f Hz", control_rate_hz_);
}

void HeadingControllerNode::onTargetEstimate(
  const perception_msgs::msg::TargetEstimate::SharedPtr msg)
{
  latest_target_ = msg;
}

void HeadingControllerNode::onCmdVelIn(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_cmd_vel_in_ = msg;
}

double HeadingControllerNode::computeAssistYaw(double dt)
{
  // Bearing to target in the robot's own frame: atan2(y, x) gives the
  // angle to turn toward, since target position is already expressed
  // relative to the robot (base_link), per TargetEstimate's frame_id.
  const double bearing_error = std::atan2(latest_target_->position.y, latest_target_->position.x);

  double derivative = 0.0;
  if (have_previous_error_ && dt > 1e-6) {
    derivative = (bearing_error - previous_bearing_error_) / dt;
  }
  previous_bearing_error_ = bearing_error;
  have_previous_error_ = true;

  return kp_ * bearing_error + kd_ * derivative;
}

void HeadingControllerNode::onTimer()
{
  if (!latest_cmd_vel_in_) {
    // No teleop input has arrived yet at all — nothing safe to command.
    return;
  }

  const rclcpp::Time now = this->now();
  double dt = 1.0 / control_rate_hz_;
  if (have_last_cycle_time_) {
    dt = (now - last_cycle_time_).seconds();
  }
  last_cycle_time_ = now;
  have_last_cycle_time_ = true;

  geometry_msgs::msg::Twist out;
  // Linear always passes through untouched, in every branch.
  out.linear = latest_cmd_vel_in_->linear;

  const bool target_valid = latest_target_ && latest_target_->valid;
  const bool operator_is_turning = std::abs(latest_cmd_vel_in_->angular.z) > deadzone_;

  if (!target_valid || operator_is_turning) {
    // Assist off: full manual passthrough. Also keep the rate limiter's
    // reference synced to passthrough so re-entering assist later ramps
    // from wherever the operator last left yaw, not a stale value.
    out.angular.z = latest_cmd_vel_in_->angular.z;
    rate_limiter_->reset(out.angular.z);
    have_previous_error_ = false;  // avoid a derivative spike on re-entry
  } else {
    const double raw_assist = computeAssistYaw(dt);
    const double limited = rate_limiter_->limit(raw_assist, rate_limiter_->lastValue(), dt);
    out.angular.z = limited;
    rate_limiter_->reset(limited);
  }

  cmd_vel_out_pub_->publish(out);
}

}  // namespace heading_controller
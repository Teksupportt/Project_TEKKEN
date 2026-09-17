#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "perception_msgs/msg/target_detection.hpp"
#include "perception_msgs/msg/target_estimate.hpp"
#include "fusion/ekf.hpp"

namespace fusion
{

class FusionNode : public rclcpp::Node
{
public:
  FusionNode();

private:
  void onTimer();

  void onDetection(
    const perception_msgs::msg::TargetDetection::SharedPtr msg,
    const std::string & topic_name);

  bool isFresh(const perception_msgs::msg::TargetDetection::SharedPtr & cached) const;

  std::vector<rclcpp::Subscription<perception_msgs::msg::TargetDetection>::SharedPtr>
    subscriptions_;
  rclcpp::Publisher<perception_msgs::msg::TargetEstimate>::SharedPtr estimate_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Latest detection per sensor topic — any number of sensors supported
  std::map<std::string, perception_msgs::msg::TargetDetection::SharedPtr> latest_by_topic_;

  ConstantVelocityEkf ekf_;
  rclcpp::Time last_predict_time_;
  bool have_last_predict_time_{false};

  double predict_rate_hz_;
  double q_pos_;
  double q_vel_;

  static constexpr double kStalenessTimeoutSec = 0.3;

  std::string frame_id_;
};

}  // namespace fusion

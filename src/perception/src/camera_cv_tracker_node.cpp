#include "perception/camera_cv_tracker.hpp"

#include <cmath>

#include <cv_bridge/cv_bridge.h>

namespace perception
{

CameraCvTracker::CameraCvTracker()
: Node("camera_cv_tracker")
{
  sensor_id_ = this->declare_parameter<std::string>("sensor_id", "camera_1");

  hue_min_ = this->declare_parameter<int>("hue_min", 40);
  hue_max_ = this->declare_parameter<int>("hue_max", 80);
  sat_min_ = this->declare_parameter<int>("sat_min", 100);
  sat_max_ = this->declare_parameter<int>("sat_max", 255);
  val_min_ = this->declare_parameter<int>("val_min", 100);
  val_max_ = this->declare_parameter<int>("val_max", 255);

  min_blob_area_px_ = this->declare_parameter<int>("min_blob_area_px", 200);
  known_target_width_m_ = this->declare_parameter<double>("known_target_width_m", 0.3);
  position_noise_stddev_ = this->declare_parameter<double>("position_noise_stddev", 0.1);

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw", rclcpp::SensorDataQoS(),
    std::bind(&CameraCvTracker::onImage, this, std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info", rclcpp::SensorDataQoS(),
    std::bind(&CameraCvTracker::onCameraInfo, this, std::placeholders::_1));

  detection_pub_ = this->create_publisher<perception_msgs::msg::TargetDetection>(
    "detection", rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(), "camera_cv_tracker started, sensor_id=%s", sensor_id_.c_str());
}

void CameraCvTracker::onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  // K = [fx  0  cx]
  //     [ 0 fy  cy]
  //     [ 0  0   1]   (row-major, 9 elements)
  fx_ = msg->k[0];
  cx_ = msg->k[2];
  have_camera_info_ = true;
}

bool CameraCvTracker::detectBlob(
  const cv::Mat & bgr_image,
  double & out_pixel_cx,
  double & out_pixel_cy,
  double & out_pixel_width) const
{
  cv::Mat hsv;
  cv::cvtColor(bgr_image, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(
    hsv,
    cv::Scalar(hue_min_, sat_min_, val_min_),
    cv::Scalar(hue_max_, sat_max_, val_max_),
    mask);

  cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);
  cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  double best_area = 0.0;
  bool found = false;
  cv::Rect best_rect;

  for (const auto & contour : contours) {
    const double area = cv::contourArea(contour);
    if (area < min_blob_area_px_) {
      continue;
    }
    if (area > best_area) {
      best_area = area;
      best_rect = cv::boundingRect(contour);
      found = true;
    }
  }

  if (!found) {
    return false;
  }

  out_pixel_cx = best_rect.x + best_rect.width / 2.0;
  out_pixel_cy = best_rect.y + best_rect.height / 2.0;
  out_pixel_width = best_rect.width;
  return true;
}

void CameraCvTracker::pixelToPosition(
  double pixel_cx,
  double pixel_width,
  double & out_x,
  double & out_y,
  std::array<double, 4> & out_cov) const
{
  // Pinhole model: distance = (fx * real_width) / apparent_pixel_width
  const double distance = (fx_ * known_target_width_m_) / pixel_width;

  // True bearing angle using the calibrated principal point (cx_),
  // not an assumed image center — this is what fixes the earlier bug.
  const double bearing = std::atan2(cx_ - pixel_cx, fx_);

  out_x = distance * std::cos(bearing);
  out_y = distance * std::sin(bearing);

  const double var = position_noise_stddev_ * position_noise_stddev_;
  out_cov = {var, 0.0, 0.0, var};
}

void CameraCvTracker::onImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!have_camera_info_) {
    // Can't compute a metric position without calibrated intrinsics yet —
    // skip publishing until CameraInfo arrives at least once.
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge conversion failed: %s", e.what());
    return;
  }

  perception_msgs::msg::TargetDetection detection;
  detection.header.stamp = msg->header.stamp;
  detection.header.frame_id = msg->header.frame_id;
  detection.sensor_id = sensor_id_;

  double pixel_cx, pixel_cy, pixel_width;
  if (detectBlob(cv_ptr->image, pixel_cx, pixel_cy, pixel_width)) {
    double x, y;
    std::array<double, 4> cov;
    pixelToPosition(pixel_cx, pixel_width, x, y, cov);

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

#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "perception_msgs/msg/target_detection.hpp"

#include <opencv2/opencv.hpp>

namespace perception
{

class CameraCvTracker : public rclcpp::Node
{
public:
  CameraCvTracker();

private:
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg);
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  bool detectBlob(
    const cv::Mat & bgr_image,
    double & out_pixel_cx,
    double & out_pixel_cy,
    double & out_pixel_width) const;

  // Converts pixel centroid + apparent width into an (x, y) position
  // estimate in the camera frame, using calibrated intrinsics from
  // CameraInfo and a known real-world target width.
  void pixelToPosition(
    double pixel_cx,
    double pixel_width,
    double & out_x,
    double & out_y,
    std::array<double, 4> & out_cov) const;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<perception_msgs::msg::TargetDetection>::SharedPtr detection_pub_;

  // Calibrated intrinsics, populated from CameraInfo once received
  bool have_camera_info_{false};
  double fx_{0.0};
  double cx_{0.0};  // principal point x, i.e. true optical center in pixels

  // Parameters
  std::string sensor_id_;
  int hue_min_, hue_max_;
  int sat_min_, sat_max_;
  int val_min_, val_max_;
  int min_blob_area_px_;
  double known_target_width_m_;
  double position_noise_stddev_;
};

}  // namespace perception

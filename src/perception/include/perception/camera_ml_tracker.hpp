#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "perception_msgs/msg/target_detection.hpp"

#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

namespace perception
{

struct Detection
{
  float confidence;
  cv::Rect box;  // pixel-space bounding box
};

class CameraMlTracker : public rclcpp::Node
{
public:
  CameraMlTracker();

private:
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg);
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  // Preprocess: resize + normalize + HWC->CHW, returns input tensor data.
  std::vector<float> preprocess(const cv::Mat & bgr_image) const;

  // Runs inference, parses raw output into detections, returns the
  // single highest-confidence detection above the threshold, if any.
  bool runInference(const cv::Mat & bgr_image, Detection & out_detection);

  // Same pinhole-model conversion approach as camera_cv_tracker.
  void boxToPosition(
    const cv::Rect & box,
    double & out_x,
    double & out_y,
    std::array<double, 4> & out_cov) const;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<perception_msgs::msg::TargetDetection>::SharedPtr detection_pub_;

  bool have_camera_info_{false};
  double fx_{0.0};
  double cx_{0.0};

  // ONNX Runtime session state
  std::unique_ptr<Ort::Env> ort_env_;
  std::unique_ptr<Ort::Session> ort_session_;
  Ort::MemoryInfo memory_info_{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;

  // Parameters
  std::string sensor_id_;
  std::string model_path_;
  int input_width_;
  int input_height_;
  float confidence_threshold_;
  double known_target_width_m_;
  double position_noise_stddev_;
};

}  // namespace perception

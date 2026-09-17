#include "perception/camera_ml_tracker.hpp"

#include <cmath>

#include <cv_bridge/cv_bridge.h>

namespace perception
{

CameraMlTracker::CameraMlTracker()
: Node("camera_ml_tracker")
{
  sensor_id_ = this->declare_parameter<std::string>("sensor_id", "camera_ml_1");
  model_path_ = this->declare_parameter<std::string>("model_path", "");
  input_width_ = this->declare_parameter<int>("input_width", 320);
  input_height_ = this->declare_parameter<int>("input_height", 320);
  confidence_threshold_ = this->declare_parameter<double>("confidence_threshold", 0.5);
  known_target_width_m_ = this->declare_parameter<double>("known_target_width_m", 0.4);
  position_noise_stddev_ = this->declare_parameter<double>("position_noise_stddev", 0.15);

  if (model_path_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "model_path parameter is empty, node cannot run inference");
  } else {
    ort_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "camera_ml_tracker");

    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);  // keep CPU footprint small, tune if needed
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    ort_session_ = std::make_unique<Ort::Session>(
      *ort_env_, model_path_.c_str(), session_options);

    // Assumes single input, single output — typical for a simple YOLO-style
    // detector. Adjust if your specific model has a different signature.
    Ort::AllocatorWithDefaultOptions allocator;
    input_names_.push_back(ort_session_->GetInputNameAllocated(0, allocator).get());
    output_names_.push_back(ort_session_->GetOutputNameAllocated(0, allocator).get());

    RCLCPP_INFO(this->get_logger(), "Loaded ONNX model from %s", model_path_.c_str());
  }

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw", rclcpp::SensorDataQoS(),
    std::bind(&CameraMlTracker::onImage, this, std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info", rclcpp::SensorDataQoS(),
    std::bind(&CameraMlTracker::onCameraInfo, this, std::placeholders::_1));

  detection_pub_ = this->create_publisher<perception_msgs::msg::TargetDetection>(
    "detection", rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(), "camera_ml_tracker started, sensor_id=%s", sensor_id_.c_str());
}

void CameraMlTracker::onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  fx_ = msg->k[0];
  cx_ = msg->k[2];
  have_camera_info_ = true;
}

std::vector<float> CameraMlTracker::preprocess(const cv::Mat & bgr_image) const
{
  cv::Mat resized, rgb, float_img;
  cv::resize(bgr_image, resized, cv::Size(input_width_, input_height_));
  cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
  rgb.convertTo(float_img, CV_32F, 1.0 / 255.0);

  // HWC -> CHW, the standard layout expected by most ONNX vision models
  std::vector<float> input_data(3 * input_height_ * input_width_);
  std::vector<cv::Mat> channels(3);
  cv::split(float_img, channels);

  for (int c = 0; c < 3; ++c) {
    std::memcpy(
      input_data.data() + c * input_height_ * input_width_,
      channels[c].data,
      input_height_ * input_width_ * sizeof(float));
  }

  return input_data;
}

bool CameraMlTracker::runInference(const cv::Mat & bgr_image, Detection & out_detection)
{
  if (!ort_session_) {
    return false;
  }

  auto input_data = preprocess(bgr_image);
  std::array<int64_t, 4> input_shape{1, 3, input_height_, input_width_};

  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
    memory_info_, input_data.data(), input_data.size(),
    input_shape.data(), input_shape.size());

  std::vector<const char *> input_names_c{input_names_[0].c_str()};
  std::vector<const char *> output_names_c{output_names_[0].c_str()};

  auto output_tensors = ort_session_->Run(
    Ort::RunOptions{nullptr},
    input_names_c.data(), &input_tensor, 1,
    output_names_c.data(), 1);

  // Expected output layout: [N, 6] rows of (x1, y1, x2, y2, confidence, class)
  // in normalized [0,1] coordinates — a common convention, but verify against
  // your specific exported model and adjust parsing if it differs.
  const float * output_data = output_tensors[0].GetTensorData<float>();
  const auto output_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
  const int64_t num_detections = output_shape[0];

  float best_confidence = 0.0f;
  bool found = false;

  for (int64_t i = 0; i < num_detections; ++i) {
    const float x1 = output_data[i * 6 + 0];
    const float y1 = output_data[i * 6 + 1];
    const float x2 = output_data[i * 6 + 2];
    const float y2 = output_data[i * 6 + 3];
    const float confidence = output_data[i * 6 + 4];

    if (confidence < confidence_threshold_) {
      continue;
    }

    if (confidence > best_confidence) {
      best_confidence = confidence;
      out_detection.confidence = confidence;
      out_detection.box = cv::Rect(
        static_cast<int>(x1 * bgr_image.cols),
        static_cast<int>(y1 * bgr_image.rows),
        static_cast<int>((x2 - x1) * bgr_image.cols),
        static_cast<int>((y2 - y1) * bgr_image.rows));
      found = true;
    }
  }

  return found;
}

void CameraMlTracker::boxToPosition(
  const cv::Rect & box,
  double & out_x,
  double & out_y,
  std::array<double, 4> & out_cov) const
{
  const double pixel_cx = box.x + box.width / 2.0;
  const double pixel_width = box.width;

  const double distance = (fx_ * known_target_width_m_) / pixel_width;
  const double bearing = std::atan2(cx_ - pixel_cx, fx_);

  out_x = distance * std::cos(bearing);
  out_y = distance * std::sin(bearing);

  const double var = position_noise_stddev_ * position_noise_stddev_;
  out_cov = {var, 0.0, 0.0, var};
}

void CameraMlTracker::onImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!have_camera_info_ || !ort_session_) {
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

  Detection ml_detection;
  if (runInference(cv_ptr->image, ml_detection)) {
    double x, y;
    std::array<double, 4> cov;
    boxToPosition(ml_detection.box, x, y, cov);

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

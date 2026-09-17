#include "rclcpp/rclcpp.hpp"
#include "perception/camera_cv_tracker.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception::CameraCvTracker>());
  rclcpp::shutdown();
  return 0;
}

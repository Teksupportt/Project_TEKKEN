#include "rclcpp/rclcpp.hpp"
#include "perception/camera_ml_tracker.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception::CameraMlTracker>());
  rclcpp::shutdown();
  return 0;
}

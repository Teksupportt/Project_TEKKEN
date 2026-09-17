#include "rclcpp/rclcpp.hpp"
#include "perception/lidar_2d_tracker.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception::Lidar2DTracker>());
  rclcpp::shutdown();
  return 0;
}

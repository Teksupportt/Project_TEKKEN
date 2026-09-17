#include "rclcpp/rclcpp.hpp"
#include "perception/lidar_3d_tracker.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception::Lidar3DTracker>());
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "fusion/fusion_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fusion::FusionNode>());
  rclcpp::shutdown();
  return 0;
}

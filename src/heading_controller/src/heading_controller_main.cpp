#include "rclcpp/rclcpp.hpp"
#include "heading_controller/heading_controller_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<heading_controller::HeadingControllerNode>());
  rclcpp::shutdown();
  return 0;
}
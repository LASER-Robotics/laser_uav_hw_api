#include <rclcpp/rclcpp.hpp>

#include <laser_uav_hw_api/px4_api_node.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<laser_uav_hw_api::Px4ApiNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}

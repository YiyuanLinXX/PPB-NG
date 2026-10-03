#include "ppbng_hsi/hsi_production_node_factory.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(ppbng_hsi::make_hsi_production_node());
  rclcpp::shutdown();
  return 0;
}

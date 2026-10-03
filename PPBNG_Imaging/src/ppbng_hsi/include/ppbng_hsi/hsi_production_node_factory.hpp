#pragma once

#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

namespace ppbng_hsi
{

std::shared_ptr<rclcpp::Node> make_hsi_production_node(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

}  // namespace ppbng_hsi

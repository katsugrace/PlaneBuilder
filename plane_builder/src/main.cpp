// Copyright 2025 Victor Drobizov

#include <iostream>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("plane_builder");
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}

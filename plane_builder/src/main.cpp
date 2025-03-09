// Copyright 2025 Victor Drobizov

#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <plane_builder/plane_builder.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<plane_builder::PlaneBuilder>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}

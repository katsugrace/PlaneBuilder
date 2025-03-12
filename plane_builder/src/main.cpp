// Copyright 2025 Victor Drobizov

#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <plane_builder/plane_builder.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<plane_builder::PlaneBuilder>(node_options);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}

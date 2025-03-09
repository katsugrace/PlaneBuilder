// Copyright 2025 Victor Drobizov

#ifndef PLANE_BUILDER__PLANE_BUILDER_HPP_
#define PLANE_BUILDER__PLANE_BUILDER_HPP_

#include <rclcpp/rclcpp.hpp>

namespace plane_builder
{

class PlaneBuilder
  : public rclcpp::Node
{
public:
  explicit PlaneBuilder(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
};

}  // namespace plane_builder

#endif  // PLANE_BUILDER__PLANE_BUILDER_HPP_

// Copyright 2025 Victor Drobizov

#ifndef PLANE_BUILDER__PLANE_BUILDER_HPP_
#define PLANE_BUILDER__PLANE_BUILDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

namespace plane_builder
{

class PlaneBuilder
  : public rclcpp::Node
{
public:
  explicit PlaneBuilder(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

public:
  void SetHeaderFrame(const std::string & name) noexcept;
  std::string GetHeaderFrame() const noexcept;

  void SetChildFrame(const std::string & name) noexcept;
  std::string GetChildFrame() const noexcept;

public:
  bool AttachPoints(
    const Eigen::Vector3d & first,
    const Eigen::Vector3d & second,
    const Eigen::Vector3d & third);

private:
  std::string header_frame_;
  std::string child_frame_;
};

}  // namespace plane_builder

#endif  // PLANE_BUILDER__PLANE_BUILDER_HPP_

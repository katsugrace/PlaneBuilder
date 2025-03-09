// Copyright 2025 Victor Drobizov

#include <plane_builder/plane_builder.hpp>

namespace plane_builder
{

PlaneBuilder::PlaneBuilder(const rclcpp::NodeOptions & options)
: rclcpp::Node("plane_builder", options)
{}

void PlaneBuilder::SetHeaderFrame(const std::string & name) noexcept
{
  header_frame_ = name;
}

std::string PlaneBuilder::GetHeaderFrame() const noexcept
{
  return header_frame_;
}

void PlaneBuilder::SetChildFrame(const std::string & name) noexcept
{
  child_frame_ = name;
}

std::string PlaneBuilder::GetChildFrame() const noexcept
{
  return child_frame_;
}

bool PlaneBuilder::AttachPoints(
  const Eigen::Vector3d &,
  const Eigen::Vector3d &,
  const Eigen::Vector3d &)
{
  return true;
}

}  // namespace plane_builder

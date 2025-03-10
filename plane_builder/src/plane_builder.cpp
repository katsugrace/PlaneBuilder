// Copyright 2025 Victor Drobizov

#include <plane_builder/def.hpp>
#include <plane_builder/plane_builder.hpp>
#include <geometry_msgs/msg/transform.hpp>

namespace plane_builder
{

PlaneBuilder::PlaneBuilder(const rclcpp::NodeOptions & options)
: rclcpp::Node("plane_builder", options)
  , header_frame_(HEADER_FRAME)
  , child_frame_(CHILD_FRAME)
  , broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
{
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
    std::bind(&PlaneBuilder::publishPlaneCb, this));
}

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

void PlaneBuilder::publishPlaneCb()
{
  geometry_msgs::msg::TransformStamped transfrom_stamped;

  transfrom_stamped.header.stamp = this->now();
  transfrom_stamped.header.frame_id = header_frame_;
  transfrom_stamped.child_frame_id = child_frame_;

  broadcaster_->sendTransform(transfrom_stamped);
}

}  // namespace plane_builder

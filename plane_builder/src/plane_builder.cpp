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
  declareParameters();
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
  const Eigen::Vector3d & first_point,
  const Eigen::Vector3d & second_point,
  const Eigen::Vector3d & third_point)
{
  Eigen::Vector3d first_matrtix = second_point - first_point;
  Eigen::Vector3d second_matrtix = third_point - first_point;

  Eigen::Vector3d normal = first_matrtix.cross(second_matrtix);
  normal.normalize();

  auto quaterniond = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), normal);

  transfrom_stamped_.transform.rotation.x = quaterniond.x();
  transfrom_stamped_.transform.rotation.y = quaterniond.y();
  transfrom_stamped_.transform.rotation.z = quaterniond.z();
  transfrom_stamped_.transform.rotation.w = quaterniond.w();

  return true;
}

bool PlaneBuilder::AttachPoints(
  const geometry_msgs::msg::Point & first_point,
  const geometry_msgs::msg::Point & second_point,
  const geometry_msgs::msg::Point & third_point)
{
  return AttachPoints(
    Eigen::Vector3d(first_point.x, first_point.y, first_point.z),
    Eigen::Vector3d(second_point.x, second_point.y, second_point.z),
    Eigen::Vector3d(third_point.x, third_point.y, third_point.z));
}

void PlaneBuilder::publishPlaneCb()
{
  transfrom_stamped_.header.stamp = this->now();
  transfrom_stamped_.header.frame_id = header_frame_;
  transfrom_stamped_.child_frame_id = child_frame_;

  broadcaster_->sendTransform(transfrom_stamped_);
}

void PlaneBuilder::declareParameters()
{
  this->get_parameter("header_frame", header_frame_);
  this->get_parameter("child_frame", child_frame_);
}

}  // namespace plane_builder

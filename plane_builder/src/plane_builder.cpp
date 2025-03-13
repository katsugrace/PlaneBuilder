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
  Eigen::Vector3d first_vector = second_point - first_point;
  if (first_vector.norm() == 0) {
    RCLCPP_ERROR(this->get_logger(),
      "The first vector (second_point - first_point) is zero");
    return false;
  }

  Eigen::Vector3d second_vector = third_point - first_point;
  if (second_vector.norm() == 0) {
    RCLCPP_ERROR(this->get_logger(),
      "The second vector (third_point - first_point) is zero");
    return false;
  }

  Eigen::Vector3d normal = first_vector.cross(second_vector);
  if (normal.norm() == 0) {
    RCLCPP_ERROR(this->get_logger(),
      "The cross product of the two vectors is zero, "
      "indicating that the points are collinear");
    return false;
  }

  normal.normalize();

  auto quaterniond = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), normal);
  if (quaterniond.coeffs().hasNaN()) {
    RCLCPP_ERROR(this->get_logger(),
      "The computed quaternion contains NaN values");
    return false;
  }

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

void PlaneBuilder::SetPosition(const Eigen::Vector3d & position) noexcept
{
  transfrom_stamped_.transform.translation.x = position.x();
  transfrom_stamped_.transform.translation.y = position.y();
  transfrom_stamped_.transform.translation.z = position.z();
}

void PlaneBuilder::SetPosition(const geometry_msgs::msg::Point & position) noexcept
{
  transfrom_stamped_.transform.translation.x = position.x;
  transfrom_stamped_.transform.translation.y = position.y;
  transfrom_stamped_.transform.translation.z = position.z;
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

  this->get_parameter("initial.position.x", transfrom_stamped_.transform.translation.x);
  this->get_parameter("initial.position.y", transfrom_stamped_.transform.translation.y);
  this->get_parameter("initial.position.z", transfrom_stamped_.transform.translation.z);

  this->get_parameter("initial.quaternion.x", transfrom_stamped_.transform.rotation.x);
  this->get_parameter("initial.quaternion.y", transfrom_stamped_.transform.rotation.y);
  this->get_parameter("initial.quaternion.z", transfrom_stamped_.transform.rotation.z);
  this->get_parameter("initial.quaternion.w", transfrom_stamped_.transform.rotation.w);
}

}  // namespace plane_builder

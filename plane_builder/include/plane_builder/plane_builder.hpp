// Copyright 2025 Victor Drobizov

#ifndef PLANE_BUILDER__PLANE_BUILDER_HPP_
#define PLANE_BUILDER__PLANE_BUILDER_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <eigen3/Eigen/Dense>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>

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
    const Eigen::Vector3d & first_point,
    const Eigen::Vector3d & second_point,
    const Eigen::Vector3d & third_point);

private:
  void publishPlaneCb();
  void declareParameters();

private:
  std::string header_frame_;
  std::string child_frame_;

private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace plane_builder

#endif  // PLANE_BUILDER__PLANE_BUILDER_HPP_

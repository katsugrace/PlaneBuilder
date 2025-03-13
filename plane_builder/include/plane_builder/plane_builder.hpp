// Copyright 2025 Victor Drobizov

#ifndef PLANE_BUILDER__PLANE_BUILDER_HPP_
#define PLANE_BUILDER__PLANE_BUILDER_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <eigen3/Eigen/Dense>
#include <string>
#include <memory>

#include <geometry_msgs/msg/point.hpp>
#include <plane_builder_msgs/srv/attach_points.hpp>
#include <plane_builder_msgs/srv/set_position.hpp>
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

  bool AttachPoints(
    const geometry_msgs::msg::Point & first_point,
    const geometry_msgs::msg::Point & second_point,
    const geometry_msgs::msg::Point & third_point);

  void SetPosition(const Eigen::Vector3d & position) noexcept;
  void SetPosition(const geometry_msgs::msg::Point & position) noexcept;

private:
  void attachPointsCallback(
    const plane_builder_msgs::srv::AttachPoints::Request::SharedPtr request,
    plane_builder_msgs::srv::AttachPoints::Response::SharedPtr response);

  void setPositionCallback(
    const plane_builder_msgs::srv::SetPosition::Request::SharedPtr request,
    plane_builder_msgs::srv::SetPosition::Response::SharedPtr response);

private:
  void publishPlaneCb();
  void declareParameters();

private:
  std::string header_frame_;
  std::string child_frame_;

private:
  geometry_msgs::msg::TransformStamped transfrom_stamped_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<plane_builder_msgs::srv::AttachPoints>::SharedPtr attach_points_srv_;
  rclcpp::Service<plane_builder_msgs::srv::SetPosition>::SharedPtr set_position_srv_;
};

}  // namespace plane_builder

#endif  // PLANE_BUILDER__PLANE_BUILDER_HPP_

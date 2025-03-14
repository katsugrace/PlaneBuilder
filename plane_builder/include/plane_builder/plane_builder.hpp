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

/**
 * @class PlaneBuilder
 * @brief The class plane builder
 */
class PlaneBuilder
  : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Plane Builder object
   * @param options Parameters for the node
   */
  explicit PlaneBuilder(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

public:
  /**
   * @brief Set name for parent link
   * @param name The name parent link
   */
  void SetHeaderFrame(const std::string & name) noexcept;
  /**
   * @brief Get name for parent link
   * @return std::string
   */
  std::string GetHeaderFrame() const noexcept;
  /**
   * @brief Set name for plane link
   * @param name The name plane link
   */
  void SetChildFrame(const std::string & name) noexcept;
  /**
   * @brief Get name for plane link
   * @return std::string
   */
  std::string GetChildFrame() const noexcept;

public:
  /**
   * @brief Calculate a plane based on three points
   * @param first_point The first point for calculation
   * @param second_point The second point for calculation
   * @param third_point The third point for calculation
   * @return true or false
   */
  bool AttachPoints(
    const Eigen::Vector3d & first_point,
    const Eigen::Vector3d & second_point,
    const Eigen::Vector3d & third_point);
  /**
   * @brief Calculate a plane based on three points
   * @param first_point The first point for calculation
   * @param second_point The second point for calculation
   * @param third_point The third point for calculation
   * @return true or false
   */
  bool AttachPoints(
    const geometry_msgs::msg::Point & first_point,
    const geometry_msgs::msg::Point & second_point,
    const geometry_msgs::msg::Point & third_point);
  /**
   * @brief Set position for plane
   * @param position The position
   */
  void SetPosition(const Eigen::Vector3d & position) noexcept;
  /**
   * @brief Set position for plane
   * @param position The position
   */
  void SetPosition(const geometry_msgs::msg::Point & position) noexcept;

private:
  /**
   * @brief Calculate a plane based on three points through the service
   * @param request The request containing three points for plane calculation
   * @param response The response calculation result
   */
  void attachPointsCallback(
    const plane_builder_msgs::srv::AttachPoints::Request::SharedPtr request,
    plane_builder_msgs::srv::AttachPoints::Response::SharedPtr response);
  /**
   * @brief Set position for plane through the service
   * @param request The request containing position for plane
   * @param response The response set position result
   */
  void setPositionCallback(
    const plane_builder_msgs::srv::SetPosition::Request::SharedPtr request,
    plane_builder_msgs::srv::SetPosition::Response::SharedPtr response);

private:
  /**
   * @brief Publishing plane
   */
  void publishPlaneCb();
  /**
   * @brief Declaring default parameters from config
   */
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
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr plane_state_pub_;
};

}  // namespace plane_builder

#endif  // PLANE_BUILDER__PLANE_BUILDER_HPP_

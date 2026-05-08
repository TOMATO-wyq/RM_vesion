#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/time.hpp>

visualization_msgs::msg::Marker createSphereMarker(
    const Eigen::Vector3d& position,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    int id, float scale, float r, float g, float b, float a);

visualization_msgs::msg::Marker createBoxMarker(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    int id,
    float r, float g, float b, float a);


visualization_msgs::msg::Marker createArrowMarker(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    int id,
    float shaft_diameter,
    float head_diameter,
    float head_length,
    float r, float g, float b, float a);

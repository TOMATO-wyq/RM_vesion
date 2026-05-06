#pragma once

#include <Eigen/Core>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/time.hpp>
#include <string>

visualization_msgs::msg::Marker createSphereMarker(
    const Eigen::Vector3d& position,
    const rclcpp::Time& stamp,
    int id, float scale, float r, float g, float b, float a);

visualization_msgs::msg::Marker createCylinderMarker(
    const Eigen::Vector3d& center,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    int id, float diameter, float height,
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

visualization_msgs::msg::Marker createLineMarker(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    int id,
    float line_width,
    float r, float g, float b, float a);

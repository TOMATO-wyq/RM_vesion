#include "armor_plate_tracker/DebugTracker.hpp"

visualization_msgs::msg::Marker createSphereMarker(
    const Eigen::Vector3d& position,
    const rclcpp::Time& stamp,
    int id, float scale, float r, float g, float b, float a)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = stamp;
    marker.ns = "tracker_sphere";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    return marker;
}

visualization_msgs::msg::Marker createCylinderMarker(
    const Eigen::Vector3d& center,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    int id, float diameter, float height,
    float r, float g, float b, float a)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "tracker_axis";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = center.x();
    marker.pose.position.y = center.y();
    marker.pose.position.z = center.z();
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.scale.x = diameter;
    marker.scale.y = diameter;
    marker.scale.z = height;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    return marker;
}

visualization_msgs::msg::Marker createLineMarker(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    int id,
    float line_width,
    float r, float g, float b, float a)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "tracker_axis";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point p_start, p_end;
    p_start.x = start.x();
    p_start.y = start.y();
    p_start.z = start.z();
    p_end.x = end.x();
    p_end.y = end.y();
    p_end.z = end.z();
    marker.points.push_back(p_start);
    marker.points.push_back(p_end);

    marker.scale.x = line_width;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    return marker;
}

visualization_msgs::msg::Marker createArrowMarker(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    int id,
    float shaft_diameter,
    float head_diameter,
    float head_length,
    float r, float g, float b, float a)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "tracker_velocity";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point p_start, p_end;
    p_start.x = start.x();
    p_start.y = start.y();
    p_start.z = start.z();
    p_end.x = end.x();
    p_end.y = end.y();
    p_end.z = end.z();
    marker.points.push_back(p_start);
    marker.points.push_back(p_end);

    marker.scale.x = shaft_diameter;
    marker.scale.y = head_diameter;
    marker.scale.z = head_length;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    return marker;
}



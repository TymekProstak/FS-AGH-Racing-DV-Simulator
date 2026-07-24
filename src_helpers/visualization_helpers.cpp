#include "visualization_helpers.hpp"

#include <ros/time.h>

namespace lem_dynamics_sim_
{
namespace
{

std_msgs::ColorRGBA color(float red,
                          float green,
                          float blue,
                          float alpha)
{
    std_msgs::ColorRGBA result;
    result.r = red;
    result.g = green;
    result.b = blue;
    result.a = alpha;
    return result;
}

} // namespace

std_msgs::ColorRGBA ground_truth_cone_color(
    const std::string& cone_class,
    float alpha)
{
    if (cone_class == "yellow" || cone_class == "Y") {
        return color(1.0f, 0.9f, 0.0f, alpha);
    }
    if (cone_class == "blue" || cone_class == "B") {
        return color(0.1f, 0.3f, 1.0f, alpha);
    }
    if (cone_class == "orange" || cone_class == "O") {
        return color(1.0f, 0.4f, 0.0f, alpha);
    }
    return color(0.6f, 0.6f, 0.6f, alpha);
}

std_msgs::ColorRGBA camera_cone_color(
    const std::string& cone_class,
    float alpha)
{
    if (cone_class == "yellow") {
        return color(1.0f, 1.0f, 0.0f, alpha);
    }
    if (cone_class == "blue") {
        return color(0.0f, 0.3f, 1.0f, alpha);
    }
    if (cone_class == "orange") {
        return color(1.0f, 0.55f, 0.0f, alpha);
    }
    return color(0.7f, 0.7f, 0.7f, alpha);
}

std_msgs::ColorRGBA lidar_cone_color(
    const std::string& cone_class,
    float alpha)
{
    if (cone_class == "yellow") {
        return color(1.0f, 0.95f, 0.1f, alpha);
    }
    if (cone_class == "blue") {
        return color(0.1f, 0.6f, 1.0f, alpha);
    }
    if (cone_class == "orange") {
        return color(1.0f, 0.5f, 0.1f, alpha);
    }
    return color(0.8f, 0.8f, 0.8f, alpha);
}

visualization_msgs::Marker make_cone_marker(
    int id,
    const std::string& frame,
    double x,
    double y,
    double z,
    const std_msgs::ColorRGBA& marker_color,
    const ros::Duration& lifetime)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "cones";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z + 0.18;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.28;
    marker.scale.y = 0.28;
    marker.scale.z = 0.36;
    marker.color = marker_color;
    marker.lifetime = lifetime;
    marker.frame_locked = false;
    return marker;
}

} // namespace lem_dynamics_sim_

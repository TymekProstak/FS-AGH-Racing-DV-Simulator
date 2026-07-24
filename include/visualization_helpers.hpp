#pragma once

#include <ros/duration.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

#include <string>

namespace lem_dynamics_sim_
{

std_msgs::ColorRGBA ground_truth_cone_color(
    const std::string& cone_class,
    float alpha = 0.95f);

std_msgs::ColorRGBA camera_cone_color(
    const std::string& cone_class,
    float alpha = 0.95f);

std_msgs::ColorRGBA lidar_cone_color(
    const std::string& cone_class,
    float alpha = 0.95f);

visualization_msgs::Marker make_cone_marker(
    int id,
    const std::string& frame,
    double x,
    double y,
    double z,
    const std_msgs::ColorRGBA& color,
    const ros::Duration& lifetime);

} // namespace lem_dynamics_sim_

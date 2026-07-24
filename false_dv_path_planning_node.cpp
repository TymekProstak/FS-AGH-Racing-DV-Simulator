#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

struct XY
{
  double x = 0.0;
  double y = 0.0;
};

static bool parseTrackCsv(
    const std::string& file_path,
    std::vector<XY>& out_points,
    std::string& err)
{
  out_points.clear();

  std::ifstream in(file_path);

  if (!in.is_open())
  {
    err = "cannot open file: " + file_path;
    return false;
  }

  std::string line;

  while (std::getline(in, line))
  {
    if (line.find_first_not_of(" \t\r\n") == std::string::npos)
    {
      continue;
    }

    std::string normalized = line;

    for (char& c : normalized)
    {
      if (c == ';')
      {
        c = ',';
      }
    }

    std::stringstream ss(normalized);

    std::string token_x;
    std::string token_y;

    if (!std::getline(ss, token_x, ','))
    {
      continue;
    }

    if (!std::getline(ss, token_y, ','))
    {
      continue;
    }

    try
    {
      const double x = std::stod(token_x);
      const double y = std::stod(token_y);

      if (!std::isfinite(x) || !std::isfinite(y))
      {
        continue;
      }

      out_points.push_back({x, y});
    }
    catch (const std::exception&)
    {
      /* Usually CSV header, e.g. x,y or x;y. */
      continue;
    }
  }

  if (out_points.size() < 2)
  {
    err = "track csv has too few valid x,y points (<2): " + file_path;
    return false;
  }

  return true;
}

static geometry_msgs::Quaternion yawToQuaternion(
    const double yaw)
{
  geometry_msgs::Quaternion q;

  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);

  return q;
}

static double computeYawFromPoints(
    const std::vector<XY>& pts,
    const std::size_t i)
{
  if (pts.size() < 2)
  {
    return 0.0;
  }

  if (i == 0)
  {
    return std::atan2(
        pts[1].y - pts[0].y,
        pts[1].x - pts[0].x);
  }

  if (i + 1 >= pts.size())
  {
    return std::atan2(
        pts[i].y - pts[i - 1].y,
        pts[i].x - pts[i - 1].x);
  }

  return std::atan2(
      pts[i + 1].y - pts[i - 1].y,
      pts[i + 1].x - pts[i - 1].x);
}

static void appendFirstPointIfNeeded(
    std::vector<XY>& pts)
{
  if (pts.size() < 2)
  {
    return;
  }

  const XY& first = pts.front();
  const XY& last = pts.back();

  const double dist =
      std::hypot(last.x - first.x, last.y - first.y);

  if (dist > 1.0e-6)
  {
    pts.push_back(first);
  }
}

static geometry_msgs::PoseArray buildPoseArrayFromPoints(
    const std::vector<XY>& pts,
    const std::string& frame_id)
{
  geometry_msgs::PoseArray msg;

  msg.header.frame_id = frame_id;
  msg.header.stamp = ros::Time::now();

  msg.poses.reserve(pts.size());

  for (std::size_t i = 0; i < pts.size(); ++i)
  {
    geometry_msgs::Pose pose;

    pose.position.x = pts[i].x;
    pose.position.y = pts[i].y;
    pose.position.z = 0.0;

    pose.orientation = yawToQuaternion(
        computeYawFromPoints(pts, i));

    msg.poses.push_back(pose);
  }

  return msg;
}

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "false_dv_path_planning_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::Publisher path_pub =
      nh.advertise<geometry_msgs::PoseArray>(
          "/path_planning/path",
          1,
          true);

  double publish_rate_hz = 10.0;
  pnh.param("publish_rate", publish_rate_hz, 10.0);

  std::string frame_id = "map";
  pnh.param<std::string>("frame_id", frame_id, "map");

  std::string track_file;

  if (!pnh.getParam("track_file", track_file))
  {
    ROS_ERROR_STREAM(
        "[false_dv_path_planning_node] Missing required private param '~track_file'. "
        "Use one of the *_centerline.csv files from lem_simulator/tracks.");

    return 1;
  }

  bool append_first_point_if_closed = false;
  pnh.param(
      "append_first_point_if_closed",
      append_first_point_if_closed,
      false);

  std::vector<XY> pts;
  std::string err;

  if (!parseTrackCsv(track_file, pts, err))
  {
    ROS_ERROR_STREAM(
        "[false_dv_path_planning_node] Failed to load centerline: "
        << err);

    return 1;
  }

  if (append_first_point_if_closed)
  {
    appendFirstPointIfNeeded(pts);
  }

  geometry_msgs::PoseArray msg =
      buildPoseArrayFromPoints(pts, frame_id);

  double total_length_m = 0.0;

  for (std::size_t i = 1; i < pts.size(); ++i)
  {
    total_length_m +=
        std::hypot(
            pts[i].x - pts[i - 1].x,
            pts[i].y - pts[i - 1].y);
  }

  ROS_INFO_STREAM(
      "[false_dv_path_planning_node] Loaded centerline from: "
      << track_file);

  ROS_INFO_STREAM(
      "[false_dv_path_planning_node] Points: "
      << msg.poses.size()
      << " | total_length_m="
      << total_length_m
      << " | append_first_point_if_closed="
      << append_first_point_if_closed);

  ROS_INFO_STREAM(
      "[false_dv_path_planning_node] Publishing latched geometry_msgs/PoseArray on /path_planning/path");

  ros::Rate rate(std::max(0.1, publish_rate_hz));

  while (ros::ok())
  {
    msg.header.stamp = ros::Time::now();
    path_pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

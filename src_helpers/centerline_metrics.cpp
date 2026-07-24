#include "centerline_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace lem_dynamics_sim_
{
namespace
{

double wrap_angle(double angle_rad)
{
    return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

bool parse_xy(const std::string& line, double& x, double& y)
{
    std::string normalized = line;
    std::replace(normalized.begin(), normalized.end(), ';', ',');

    std::stringstream stream(normalized);
    std::string x_token;
    std::string y_token;
    if (!std::getline(stream, x_token, ',') ||
        !std::getline(stream, y_token, ',')) {
        return false;
    }

    try {
        std::size_t x_end = 0;
        std::size_t y_end = 0;
        x = std::stod(x_token, &x_end);
        y = std::stod(y_token, &y_end);
        return x_end > 0 && y_end > 0 &&
               std::isfinite(x) && std::isfinite(y);
    } catch (const std::exception&) {
        return false;
    }
}

} // namespace

void CenterlineMetrics::load_csv(const std::string& file_path)
{
    std::ifstream input(file_path);
    if (!input.is_open()) {
        throw std::runtime_error(
            "Could not open centerline file: " + file_path);
    }

    std::vector<Point> loaded_points;
    std::string line;
    while (std::getline(input, line)) {
        double x = 0.0;
        double y = 0.0;
        if (parse_xy(line, x, y)) {
            loaded_points.push_back({x, y});
        }
    }

    if (loaded_points.size() < 2) {
        throw std::runtime_error(
            "Centerline requires at least two valid points: " + file_path);
    }

    points_ = std::move(loaded_points);
}

CenterlineTrackingError CenterlineMetrics::evaluate(
    const double x_m,
    const double y_m,
    const double yaw_rad,
    const double body_vx_mps,
    const double body_vy_mps) const
{
    if (points_.size() < 2) {
        throw std::logic_error("Centerline metrics used before loading a CSV");
    }

    double closest_distance_squared =
        std::numeric_limits<double>::infinity();
    double closest_lateral_error = 0.0;
    double closest_tangent_x = 1.0;
    double closest_tangent_y = 0.0;

    for (std::size_t i = 0; i + 1 < points_.size(); ++i) {
        const Point& begin = points_[i];
        const Point& end = points_[i + 1];
        const double segment_x = end.x - begin.x;
        const double segment_y = end.y - begin.y;
        const double length_squared =
            segment_x * segment_x + segment_y * segment_y;
        if (length_squared <= 1.0e-12) {
            continue;
        }

        const double projection = std::clamp(
            ((x_m - begin.x) * segment_x +
             (y_m - begin.y) * segment_y) / length_squared,
            0.0,
            1.0);
        const double closest_x = begin.x + projection * segment_x;
        const double closest_y = begin.y + projection * segment_y;
        const double error_x = x_m - closest_x;
        const double error_y = y_m - closest_y;
        const double distance_squared =
            error_x * error_x + error_y * error_y;

        if (distance_squared >= closest_distance_squared) {
            continue;
        }

        const double segment_length = std::sqrt(length_squared);
        closest_distance_squared = distance_squared;
        closest_tangent_x = segment_x / segment_length;
        closest_tangent_y = segment_y / segment_length;
        closest_lateral_error =
            closest_tangent_x * error_y -
            closest_tangent_y * error_x;
    }

    const double centerline_yaw =
        std::atan2(closest_tangent_y, closest_tangent_x);
    const double velocity_x_global =
        body_vx_mps * std::cos(yaw_rad) -
        body_vy_mps * std::sin(yaw_rad);
    const double velocity_y_global =
        body_vx_mps * std::sin(yaw_rad) +
        body_vy_mps * std::cos(yaw_rad);

    CenterlineTrackingError result;
    result.lateral_error_m = closest_lateral_error;
    result.heading_error_rad = wrap_angle(yaw_rad - centerline_yaw);
    result.path_speed_mps =
        velocity_x_global * closest_tangent_x +
        velocity_y_global * closest_tangent_y;
    return result;
}

} // namespace lem_dynamics_sim_

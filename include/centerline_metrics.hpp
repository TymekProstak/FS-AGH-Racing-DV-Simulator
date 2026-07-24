#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace lem_dynamics_sim_
{

struct CenterlineTrackingError
{
    double lateral_error_m = 0.0;
    double heading_error_rad = 0.0;
    double path_speed_mps = 0.0;
};

class CenterlineMetrics
{
public:
    void load_csv(const std::string& file_path);

    CenterlineTrackingError evaluate(
        double x_m,
        double y_m,
        double yaw_rad,
        double body_vx_mps,
        double body_vy_mps) const;

    std::size_t point_count() const noexcept { return points_.size(); }

private:
    struct Point
    {
        double x = 0.0;
        double y = 0.0;
    };

    std::vector<Point> points_;
};

} // namespace lem_dynamics_sim_

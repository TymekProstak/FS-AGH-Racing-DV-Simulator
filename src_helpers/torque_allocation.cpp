#include "torque_allocation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace lem_dynamics_sim_
{
namespace
{

double drivetrain_ratio(const ParamBank& parameters)
{
    const double ratio = parameters.get("gear_ratio");
    return std::isfinite(ratio) && ratio > 0.0 ? ratio : 1.0;
}

double front_torque_fraction(const ParamBank& parameters,
                             double total_wheel_torque_nm)
{
    const char* key = total_wheel_torque_nm < 0.0
        ? "front_back_split_brake"
        : "front_back_split_drive";
    const double fraction = parameters.get(key);

    if (!std::isfinite(fraction) || fraction < 0.0 || fraction > 1.0) {
        throw std::runtime_error(
            std::string("Torque allocation: ") + key +
            " must be within [0, 1]");
    }
    return fraction;
}

double low_speed_turn_gain(double speed_mps,
                           double low_speed_mps,
                           double high_speed_mps,
                           double transition_gain,
                           double low_speed_gain)
{
    const double speed_span = high_speed_mps - low_speed_mps;
    if (!std::isfinite(speed_span) || std::abs(speed_span) < 1.0e-9) {
        return 1.0;
    }

    const double normalized =
        (2.0 * speed_mps - low_speed_mps - high_speed_mps) / speed_span;
    const double blend =
        0.5 * (1.0 - std::tanh(transition_gain * normalized));

    return std::isfinite(low_speed_gain)
        ? 1.0 + low_speed_gain * blend
        : 1.0;
}

} // namespace

double control_torque_to_wheel_torque(const ParamBank& parameters,
                                      double motor_torque_nm)
{
    return std::isfinite(motor_torque_nm)
        ? motor_torque_nm * drivetrain_ratio(parameters)
        : 0.0;
}

WheelTorqueCommand allocate_one_wheel_baseline_torque(
    const ParamBank& parameters,
    double total_motor_torque_nm)
{
    WheelTorqueCommand command;
    const double total_wheel_torque_nm =
        control_torque_to_wheel_torque(parameters, total_motor_torque_nm);
    const double front_fraction =
        front_torque_fraction(parameters, total_wheel_torque_nm);

    command.fl = 0.5 * total_wheel_torque_nm * front_fraction;
    command.fr = command.fl;
    command.rl = 0.5 * total_wheel_torque_nm * (1.0 - front_fraction);
    command.rr = command.rl;
    return command;
}

double compute_left_torque_vectoring_delta(
    const ParamBank& parameters,
    double longitudinal_speed_mps,
    double steering_angle_rad,
    double yaw_rate_radps)
{
    const double speed_mps = std::max(
        0.0,
        std::isfinite(longitudinal_speed_mps)
            ? longitudinal_speed_mps
            : 0.0);
    const double steer_rad =
        std::isfinite(steering_angle_rad) ? steering_angle_rad : 0.0;
    const double measured_yaw_rate =
        std::isfinite(yaw_rate_radps) ? yaw_rate_radps : 0.0;

    const double effective_wheelbase_m =
        parameters.get("w") +
        parameters.get("turn_radius_speed_gain") * speed_mps * speed_mps;
    if (!std::isfinite(effective_wheelbase_m) ||
        std::abs(effective_wheelbase_m) < 1.0e-9) {
        return 0.0;
    }

    const double turn_gain = low_speed_turn_gain(
        speed_mps,
        parameters.get("speed_blend_low"),
        parameters.get("speed_blend_high"),
        parameters.get("transition_gain"),
        parameters.get("low_speed_gain"));
    const double target_yaw_rate =
        speed_mps * std::sin(steer_rad) *
        (1.0 + parameters.get("oversteer_gain")) *
        turn_gain / effective_wheelbase_m;

    const double motor_gain_nm_per_radps =
        parameters.get("torque_vectoring_p_gain") *
        parameters.get("torque_vectoring_scale_nm");
    const double wheel_gain_nm_per_radps =
        motor_gain_nm_per_radps * drivetrain_ratio(parameters);
    const double delta =
        -wheel_gain_nm_per_radps * (target_yaw_rate - measured_yaw_rate);

    return std::isfinite(delta) ? delta : 0.0;
}

void apply_symmetric_torque_vectoring(
    WheelTorqueCommand& command,
    double left_wheel_delta_nm)
{
    command.fl += left_wheel_delta_nm;
    command.fr -= left_wheel_delta_nm;
    command.rl += left_wheel_delta_nm;
    command.rr -= left_wheel_delta_nm;
}

} // namespace lem_dynamics_sim_

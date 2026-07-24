#pragma once

#include "ParamBank.hpp"

namespace lem_dynamics_sim_
{

struct WheelTorqueCommand
{
    double fl = 0.0;
    double fr = 0.0;
    double rl = 0.0;
    double rr = 0.0;
};

double control_torque_to_wheel_torque(const ParamBank& parameters,
                                      double motor_torque_nm);

WheelTorqueCommand allocate_baseline_torque(
    const ParamBank& parameters,
    double total_motor_torque_nm);

double compute_left_torque_vectoring_delta(
    const ParamBank& parameters,
    double longitudinal_speed_mps,
    double steering_angle_rad,
    double yaw_rate_radps);

void apply_symmetric_torque_vectoring(
    WheelTorqueCommand& command,
    double left_wheel_delta_nm);

} // namespace lem_dynamics_sim_

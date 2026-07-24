#pragma once

#include "drivetrain.hpp"
#include "tire_model.hpp"
#include "steering_system.hpp"
#include "utilities.hpp"
#include "wheels_dynamis_model.hpp"

#include <algorithm>

namespace lem_dynamics_sim_
{

struct Log_Info_reduced
{
    double vx = 0.0;       // m/s
    double vy = 0.0;       // m/s
    double yaw_rate = 0.0; // rad/s
    double x = 0.0;        // m
    double y = 0.0;        // m
    double yaw = 0.0;      // rad

    double torque_total = 0.0; // Nm
    double rack_angle = 0.0;   // rad

    double kappa_fl = 0.0; // nondim
    double kappa_fr = 0.0; // nondim
    double kappa_rl = 0.0; // nondim
    double kappa_rr = 0.0; // nondim

    double time = 0.0; // s

    double rack_angle_request = 0.0; // rad
    double torque_request = 0.0;     // Nm

    double ax = 0.0; // m/s^2
    double ay = 0.0; // m/s^2
};


struct Log_Info_full
{
    double kappa_fl = 0.0; // nondim
    double kappa_fr = 0.0; // nondim
    double kappa_rl = 0.0; // nondim
    double kappa_rr = 0.0; // nondim

    double slip_angle_fl = 0.0; // rad
    double slip_angle_fr = 0.0; // rad
    double slip_angle_rl = 0.0; // rad
    double slip_angle_rr = 0.0; // rad
    double slip_angle_body = 0.0; // rad

    double fz_fl = 0.0; // N
    double fz_fr = 0.0; // N
    double fz_rl = 0.0; // N
    double fz_rr = 0.0; // N

    double fy_fl = 0.0; // N
    double fy_fr = 0.0; // N
    double fy_rl = 0.0; // N
    double fy_rr = 0.0; // N

    double fx_fl = 0.0; // N
    double fx_fr = 0.0; // N
    double fx_rl = 0.0; // N
    double fx_rr = 0.0; // N

    double torque_total = 0.0; // Nm

    double torque_fl = 0.0; // Nm
    double torque_fr = 0.0; // Nm
    double torque_rl = 0.0; // Nm
    double torque_rr = 0.0; // Nm

    double omega_fl = 0.0; // rad/s
    double omega_fr = 0.0; // rad/s
    double omega_rl = 0.0; // rad/s
    double omega_rr = 0.0; // rad/s

    double delta_left = 0.0;  // rad
    double delta_right = 0.0; // rad
    double rack_angle = 0.0;  // rad

    double ax = 0.0; // m/s^2
    double ay = 0.0; // m/s^2

    double yaw_rate = 0.0; // rad/s
    double vx = 0.0;       // m/s
    double vy = 0.0;       // m/s
    double x = 0.0;        // m
    double y = 0.0;        // m
    double yaw = 0.0;      // rad

    double power_total = 0.0; // W

    double rack_angle_request = 0.0; // rad
    double torque_request = 0.0;     // Nm

    double time = 0.0; // s

    double total_drag = 0.0;      // N
    double total_downforce = 0.0; // N
};


Log_Info_reduced log_info_reduced(
    const State& x,
    const Input& u,
    const ParamBank& P,
    int step_number
);


State model_derative(
    const ParamBank& P,
    const State& x,
    const Input& u
);


Log_Info_full log_info_full(
    const State& x,
    const Input& u,
    const ParamBank& P,
    int step_number
);


void rk4_sim_timestep(
    State& x,
    const Input& u,
    const ParamBank& P
);


void euler_sim_timestep(
    State& x,
    const Input& u,
    const ParamBank& P
);

} // namespace lem_dynamics_sim_

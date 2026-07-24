#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace lem_dynamics_sim_
{

struct State
{
    //// ------------------------------------------------------------------------
    //// Vehicle body state
    //// ------------------------------------------------------------------------

    double x;
    double y;
    double yaw;

    double vx;
    double vy;
    double yaw_rate;

    //// ------------------------------------------------------------------------
    //// Wheel angular speeds - AWD
    //// ------------------------------------------------------------------------

    double omega_fl;
    double omega_fr;
    double omega_rl;
    double omega_rr;

    //// ------------------------------------------------------------------------
    //// Steering states
    //// ------------------------------------------------------------------------

    double delta_left;
    double d_delta_left;

    double delta_right;
    double d_delta_right;

    double rack_angle;
    double d_rack_angle;

    //// ------------------------------------------------------------------------
    //// Wheel torques - AWD
    //// ------------------------------------------------------------------------

    double torque_fl;
    double torque_fr;
    double torque_rl;
    double torque_rr;

    //// ------------------------------------------------------------------------
    //// Longitudinal tire forces - AWD
    //// ------------------------------------------------------------------------

    double fx_fl;
    double fx_fr;
    double fx_rl;
    double fx_rr;

    //// ------------------------------------------------------------------------
    //// Lateral tire forces
    //// ------------------------------------------------------------------------

    double fy_fl;
    double fy_fr;
    double fy_rl;
    double fy_rr;

    //// ------------------------------------------------------------------------
    //// Total normal forces used by the tire model / debug
    //// ------------------------------------------------------------------------

    double N_fl;
    double N_fr;
    double N_rl;
    double N_rr;

    //// ------------------------------------------------------------------------
    //// Elastic lateral normal-load-transfer states [N]
    //// ------------------------------------------------------------------------

    double N_lat_el_fl;
    double N_lat_el_fr;
    double N_lat_el_rl;
    double N_lat_el_rr;

    //// ------------------------------------------------------------------------
    //// Elastic longitudinal normal-load-transfer states [N]
    //// ------------------------------------------------------------------------

    double N_long_el_fl;
    double N_long_el_fr;
    double N_long_el_rl;
    double N_long_el_rr;

    //// ------------------------------------------------------------------------
    //// Rates of elastic lateral transfer states [N/s]
    //// Required by the second-order roll load-transfer model.
    //// ------------------------------------------------------------------------

    double d_N_lat_el_fl;
    double d_N_lat_el_fr;
    double d_N_lat_el_rl;
    double d_N_lat_el_rr;

    //// ------------------------------------------------------------------------
    //// Rates of elastic longitudinal transfer states [N/s]
    //// Required by the second-order pitch load-transfer model.
    //// ------------------------------------------------------------------------

    double d_N_long_el_fl;
    double d_N_long_el_fr;
    double d_N_long_el_rl;
    double d_N_long_el_rr;

    //// ------------------------------------------------------------------------
    //// Delayed accelerations from previous dynamics step
    //// ------------------------------------------------------------------------

    double prev_ax;
    double prev_ay;

    State();
    explicit State(double value);
    explicit State(const std::vector<double>& values);

    void setZero();

    State operator+(const State& other) const;
    State& operator+=(const State& other);
    State operator*(double scalar) const;
    friend State operator*(double scalar, const State& s);
};


struct TireForcesBody
{
    double fx_fl;
    double fy_fl;

    double fx_fr;
    double fy_fr;

    double fx_rl;
    double fy_rl;

    double fx_rr;
    double fy_rr;
};

// Convert tire forces stored in wheel frames to the vehicle body frame.
// Rear steer is assumed to be zero.
TireForcesBody tire_forces_in_body_frame(const State& x);


struct Input
{
    //// ------------------------------------------------------------------------
    //// Independent wheel torque requests - AWD
    //// ------------------------------------------------------------------------

    double torque_request_fl;
    double torque_request_fr;
    double torque_request_rl;
    double torque_request_rr;

    //// ------------------------------------------------------------------------
    //// Steering request
    //// ------------------------------------------------------------------------

    double rack_angle_request;

    Input();

    Input(double torque_fl,
          double torque_fr,
          double torque_rl,
          double torque_rr,
          double rack);

    explicit Input(double value);
    explicit Input(const std::vector<double>& values);
};


void unwrap_angle(double& angle);

} // namespace lem_dynamics_sim_

#include "utilities.hpp"

#include <cmath>
#include <stdexcept>

namespace lem_dynamics_sim_
{

State::State()
    : State(0.0)
{
}


State::State(double value)
    : x(value),
      y(value),
      yaw(value),
      vx(value),
      vy(value),
      yaw_rate(value),

      omega_fl(value),
      omega_fr(value),
      omega_rl(value),
      omega_rr(value),

      delta_left(value),
      d_delta_left(value),
      delta_right(value),
      d_delta_right(value),

      rack_angle(value),
      d_rack_angle(value),

      torque_fl(value),
      torque_fr(value),
      torque_rl(value),
      torque_rr(value),

      fx_fl(value),
      fx_fr(value),
      fx_rl(value),
      fx_rr(value),

      fy_fl(value),
      fy_fr(value),
      fy_rl(value),
      fy_rr(value),

      N_fl(value),
      N_fr(value),
      N_rl(value),
      N_rr(value),

      N_lat_el_fl(value),
      N_lat_el_fr(value),
      N_lat_el_rl(value),
      N_lat_el_rr(value),

      N_long_el_fl(value),
      N_long_el_fr(value),
      N_long_el_rl(value),
      N_long_el_rr(value),

      d_N_lat_el_fl(value),
      d_N_lat_el_fr(value),
      d_N_lat_el_rl(value),
      d_N_lat_el_rr(value),

      d_N_long_el_fl(value),
      d_N_long_el_fr(value),
      d_N_long_el_rl(value),
      d_N_long_el_rr(value),

      prev_ax(value),
      prev_ay(value)
{
}


State::State(const std::vector<double>& values)
{
    constexpr std::size_t kStateSize = 50;

    if (values.size() != kStateSize)
    {
        throw std::invalid_argument(
            "Vector size must be 50 to initialize State."
        );
    }

    x = values[0];
    y = values[1];
    yaw = values[2];
    vx = values[3];
    vy = values[4];
    yaw_rate = values[5];

    omega_fl = values[6];
    omega_fr = values[7];
    omega_rl = values[8];
    omega_rr = values[9];

    delta_left = values[10];
    d_delta_left = values[11];
    delta_right = values[12];
    d_delta_right = values[13];

    rack_angle = values[14];
    d_rack_angle = values[15];

    torque_fl = values[16];
    torque_fr = values[17];
    torque_rl = values[18];
    torque_rr = values[19];

    fx_fl = values[20];
    fx_fr = values[21];
    fx_rl = values[22];
    fx_rr = values[23];

    fy_fl = values[24];
    fy_fr = values[25];
    fy_rl = values[26];
    fy_rr = values[27];

    N_fl = values[28];
    N_fr = values[29];
    N_rl = values[30];
    N_rr = values[31];

    N_lat_el_fl = values[32];
    N_lat_el_fr = values[33];
    N_lat_el_rl = values[34];
    N_lat_el_rr = values[35];

    N_long_el_fl = values[36];
    N_long_el_fr = values[37];
    N_long_el_rl = values[38];
    N_long_el_rr = values[39];

    d_N_lat_el_fl = values[40];
    d_N_lat_el_fr = values[41];
    d_N_lat_el_rl = values[42];
    d_N_lat_el_rr = values[43];

    d_N_long_el_fl = values[44];
    d_N_long_el_fr = values[45];
    d_N_long_el_rl = values[46];
    d_N_long_el_rr = values[47];

    prev_ax = values[48];
    prev_ay = values[49];
}


void State::setZero()
{
    *this = State(0.0);
}


State State::operator+(const State& other) const
{
    State result = *this;
    result += other;
    return result;
}


State& State::operator+=(const State& other)
{
    x += other.x;
    y += other.y;
    yaw += other.yaw;
    vx += other.vx;
    vy += other.vy;
    yaw_rate += other.yaw_rate;

    omega_fl += other.omega_fl;
    omega_fr += other.omega_fr;
    omega_rl += other.omega_rl;
    omega_rr += other.omega_rr;

    delta_left += other.delta_left;
    d_delta_left += other.d_delta_left;
    delta_right += other.delta_right;
    d_delta_right += other.d_delta_right;

    rack_angle += other.rack_angle;
    d_rack_angle += other.d_rack_angle;

    torque_fl += other.torque_fl;
    torque_fr += other.torque_fr;
    torque_rl += other.torque_rl;
    torque_rr += other.torque_rr;

    fx_fl += other.fx_fl;
    fx_fr += other.fx_fr;
    fx_rl += other.fx_rl;
    fx_rr += other.fx_rr;

    fy_fl += other.fy_fl;
    fy_fr += other.fy_fr;
    fy_rl += other.fy_rl;
    fy_rr += other.fy_rr;

    N_fl += other.N_fl;
    N_fr += other.N_fr;
    N_rl += other.N_rl;
    N_rr += other.N_rr;

    N_lat_el_fl += other.N_lat_el_fl;
    N_lat_el_fr += other.N_lat_el_fr;
    N_lat_el_rl += other.N_lat_el_rl;
    N_lat_el_rr += other.N_lat_el_rr;

    N_long_el_fl += other.N_long_el_fl;
    N_long_el_fr += other.N_long_el_fr;
    N_long_el_rl += other.N_long_el_rl;
    N_long_el_rr += other.N_long_el_rr;

    d_N_lat_el_fl += other.d_N_lat_el_fl;
    d_N_lat_el_fr += other.d_N_lat_el_fr;
    d_N_lat_el_rl += other.d_N_lat_el_rl;
    d_N_lat_el_rr += other.d_N_lat_el_rr;

    d_N_long_el_fl += other.d_N_long_el_fl;
    d_N_long_el_fr += other.d_N_long_el_fr;
    d_N_long_el_rl += other.d_N_long_el_rl;
    d_N_long_el_rr += other.d_N_long_el_rr;

    prev_ax += other.prev_ax;
    prev_ay += other.prev_ay;

    return *this;
}


State State::operator*(double scalar) const
{
    State result;

    result.x = x * scalar;
    result.y = y * scalar;
    result.yaw = yaw * scalar;
    result.vx = vx * scalar;
    result.vy = vy * scalar;
    result.yaw_rate = yaw_rate * scalar;

    result.omega_fl = omega_fl * scalar;
    result.omega_fr = omega_fr * scalar;
    result.omega_rl = omega_rl * scalar;
    result.omega_rr = omega_rr * scalar;

    result.delta_left = delta_left * scalar;
    result.d_delta_left = d_delta_left * scalar;
    result.delta_right = delta_right * scalar;
    result.d_delta_right = d_delta_right * scalar;

    result.rack_angle = rack_angle * scalar;
    result.d_rack_angle = d_rack_angle * scalar;

    result.torque_fl = torque_fl * scalar;
    result.torque_fr = torque_fr * scalar;
    result.torque_rl = torque_rl * scalar;
    result.torque_rr = torque_rr * scalar;

    result.fx_fl = fx_fl * scalar;
    result.fx_fr = fx_fr * scalar;
    result.fx_rl = fx_rl * scalar;
    result.fx_rr = fx_rr * scalar;

    result.fy_fl = fy_fl * scalar;
    result.fy_fr = fy_fr * scalar;
    result.fy_rl = fy_rl * scalar;
    result.fy_rr = fy_rr * scalar;

    result.N_fl = N_fl * scalar;
    result.N_fr = N_fr * scalar;
    result.N_rl = N_rl * scalar;
    result.N_rr = N_rr * scalar;

    result.N_lat_el_fl = N_lat_el_fl * scalar;
    result.N_lat_el_fr = N_lat_el_fr * scalar;
    result.N_lat_el_rl = N_lat_el_rl * scalar;
    result.N_lat_el_rr = N_lat_el_rr * scalar;

    result.N_long_el_fl = N_long_el_fl * scalar;
    result.N_long_el_fr = N_long_el_fr * scalar;
    result.N_long_el_rl = N_long_el_rl * scalar;
    result.N_long_el_rr = N_long_el_rr * scalar;

    result.d_N_lat_el_fl = d_N_lat_el_fl * scalar;
    result.d_N_lat_el_fr = d_N_lat_el_fr * scalar;
    result.d_N_lat_el_rl = d_N_lat_el_rl * scalar;
    result.d_N_lat_el_rr = d_N_lat_el_rr * scalar;

    result.d_N_long_el_fl = d_N_long_el_fl * scalar;
    result.d_N_long_el_fr = d_N_long_el_fr * scalar;
    result.d_N_long_el_rl = d_N_long_el_rl * scalar;
    result.d_N_long_el_rr = d_N_long_el_rr * scalar;

    result.prev_ax = prev_ax * scalar;
    result.prev_ay = prev_ay * scalar;

    return result;
}


State operator*(double scalar, const State& s)
{
    return s * scalar;
}


Input::Input()
    : torque_request_fl(0.0),
      torque_request_fr(0.0),
      torque_request_rl(0.0),
      torque_request_rr(0.0),
      rack_angle_request(0.0)
{
}


Input::Input(double torque_fl,
             double torque_fr,
             double torque_rl,
             double torque_rr,
             double rack)
    : torque_request_fl(torque_fl),
      torque_request_fr(torque_fr),
      torque_request_rl(torque_rl),
      torque_request_rr(torque_rr),
      rack_angle_request(rack)
{
}


Input::Input(double value)
    : torque_request_fl(value),
      torque_request_fr(value),
      torque_request_rl(value),
      torque_request_rr(value),
      rack_angle_request(value)
{
}


Input::Input(const std::vector<double>& values)
{
    if (values.size() != 5)
    {
        throw std::invalid_argument("Vector size must be 5 to initialize Input.");
    }

    torque_request_fl = values[0];
    torque_request_fr = values[1];
    torque_request_rl = values[2];
    torque_request_rr = values[3];
    rack_angle_request = values[4];
}


TireForcesBody tire_forces_in_body_frame(const State& x)
{
    TireForcesBody out{};

    const double c_fl = std::cos(x.delta_left);
    const double s_fl = std::sin(x.delta_left);
    const double c_fr = std::cos(x.delta_right);
    const double s_fr = std::sin(x.delta_right);

    out.fx_fl = x.fx_fl * c_fl - x.fy_fl * s_fl;
    out.fy_fl = x.fx_fl * s_fl + x.fy_fl * c_fl;

    out.fx_fr = x.fx_fr * c_fr - x.fy_fr * s_fr;
    out.fy_fr = x.fx_fr * s_fr + x.fy_fr * c_fr;

    out.fx_rl = x.fx_rl;
    out.fy_rl = x.fy_rl;

    out.fx_rr = x.fx_rr;
    out.fy_rr = x.fy_rr;

    return out;
}


void unwrap_angle(double& angle)
{
    constexpr double pi = 3.14159265358979323846;

    while (angle > pi)
    {
        angle -= 2.0 * pi;
    }

    while (angle < -pi)
    {
        angle += 2.0 * pi;
    }
}

} // namespace lem_dynamics_sim_

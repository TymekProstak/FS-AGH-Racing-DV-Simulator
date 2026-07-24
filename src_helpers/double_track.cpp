#include "double_track.hpp"
#include "suspension.hpp"

#include <algorithm>
#include <cmath>

namespace lem_dynamics_sim_
{

namespace
{

constexpr double kVThresholdSlipMps = 1.5;
constexpr double kEpsSmall = 1.0e-9;
constexpr double kFzMinN = 50.0;

struct WheelLocalVelocities
{
    double vx_rr = 0.0;
    double vy_rr = 0.0;

    double vx_rl = 0.0;
    double vy_rl = 0.0;

    double vx_fr = 0.0;
    double vy_fr = 0.0;

    double vx_fl = 0.0;
    double vy_fl = 0.0;
};

struct NormalLoads
{
    double fl = 0.0;
    double fr = 0.0;
    double rl = 0.0;
    double rr = 0.0;
};

struct ForceBalance
{
    double aero_drag_N = 0.0;
    double aero_downforce_front_N = 0.0;
    double aero_downforce_rear_N = 0.0;
    double aero_downforce_total_N = 0.0;
    double rolling_resistance_N = 0.0;
    double resistance_constant_N = 0.0;
    double resistance_linear_N = 0.0;
    double resistance_total_N = 0.0;
    double resistance_signed_N = 0.0;

    double Fx_total_N = 0.0;
    double Fy_total_N = 0.0;
    double Mz_Nm = 0.0;

    double ax_force_mps2 = 0.0;
    double ay_force_mps2 = 0.0;
};

double safeSignedVxForAlpha(double vx, double epsilon)
{
    const double eps = std::max(std::abs(epsilon), 1.0e-4);

    if (std::abs(vx) >= eps) {
        return vx;
    }

    return std::copysign(eps, vx == 0.0 ? 1.0 : vx);
}

double slipDenominatorAccStyle(double vx)
{
    const double v_abs = std::abs(vx);

    const double denom_low =
        0.5 * (
            kVThresholdSlipMps +
            (vx * vx) / kVThresholdSlipMps
        );

    const double denom_high = v_abs;

    if (v_abs > kVThresholdSlipMps) {
        return std::max(denom_high, kEpsSmall);
    }

    return std::max(denom_low, kEpsSmall);
}

double kappaAccStyle(double vx_wheel_frame, double omega, double R)
{
    const double denom = slipDenominatorAccStyle(vx_wheel_frame);

    const double kappa =
        (omega * R - vx_wheel_frame) / denom;

    return std::clamp(kappa, -0.99, 0.99);
}

double resistanceSign(double vx)
{
    if (vx > 0.0) {
        return 1.0;
    }

    if (vx < 0.0) {
        return -1.0;
    }

    return 0.0;
}

WheelLocalVelocities computeWheelLocalVelocities(const State& x,
                                                 const ParamBank& P)
{
    WheelLocalVelocities out;

    const double r_rear = P.get("r_rear");
    const double r_front = P.get("r_front");

    const double ang_f = P.get("angle_construction_front");
    const double ang_r = P.get("angle_construction_rear");

    out.vx_rr = x.vx + x.yaw_rate * r_rear * std::sin(ang_r);
    out.vy_rr = x.vy - x.yaw_rate * r_rear * std::cos(ang_r);

    out.vx_rl = x.vx - x.yaw_rate * r_rear * std::sin(ang_r);
    out.vy_rl = x.vy - x.yaw_rate * r_rear * std::cos(ang_r);

    out.vx_fr = x.vx + x.yaw_rate * r_front * std::sin(ang_f);
    out.vy_fr = x.vy + x.yaw_rate * r_front * std::cos(ang_f);

    out.vx_fl = x.vx - x.yaw_rate * r_front * std::sin(ang_f);
    out.vy_fl = x.vy + x.yaw_rate * r_front * std::cos(ang_f);

    {
        const double vx0 = out.vx_rr;
        const double vy0 = out.vy_rr;

        out.vx_rr = vx0;
        out.vy_rr = vy0;
    }

    {
        const double vx0 = out.vx_rl;
        const double vy0 = out.vy_rl;

        out.vx_rl = vx0;
        out.vy_rl = vy0;
    }

    {
        const double c = std::cos(x.delta_right);
        const double s = std::sin(x.delta_right);

        const double vx0 = out.vx_fr;
        const double vy0 = out.vy_fr;

        out.vx_fr =  c * vx0 + s * vy0;
        out.vy_fr = -s * vx0 + c * vy0;
    }

    {
        const double c = std::cos(x.delta_left);
        const double s = std::sin(x.delta_left);

        const double vx0 = out.vx_fl;
        const double vy0 = out.vy_fl;

        out.vx_fl =  c * vx0 + s * vy0;
        out.vy_fl = -s * vx0 + c * vy0;
    }

    return out;
}

NormalLoads computeNormalLoads(const State& x, const ParamBank& P)
{
    (void)P;

    NormalLoads N;

    N.fl = std::max(x.N_fl, kFzMinN);
    N.fr = std::max(x.N_fr, kFzMinN);
    N.rl = std::max(x.N_rl, kFzMinN);
    N.rr = std::max(x.N_rr, kFzMinN);

    return N;
}

ForceBalance computeForceBalance(const State& x, const ParamBank& P)
{
    ForceBalance out;

    const double m = P.get("m");

    const double v_abs = std::abs(x.vx);

    //// ------------------------------------------------------------------------
    //// Longitudinal resistance model matched to dv_control/longitudinal_utils:
    ////
    ////     F_roll = Cr * m * g
    ////     F_drag = Cd * |vx|^2
    ////     F_extra = resistance_constant_N
    ////             + resistance_linear_N_per_mps * |vx|
    ////     F_res = F_roll + F_drag + F_extra
    ////
    //// Important: I do not add downforce to rolling resistance here, because
    //// dv_control currently uses only Cr*m*g in computeResistanceFeedforwardTorqueNm().
    //// This keeps the simulator resistance consistent with the controller
    //// feedforward / 1D speed MPC model.
    //// ------------------------------------------------------------------------

    const bool aero_enabled =
        P.get("aero_package_enabled") > 0.5;

    out.aero_drag_N =
        aero_enabled
            ? P.get("Cd") * v_abs * v_abs
            : 0.0;

    out.aero_downforce_front_N =
        aero_enabled
            ? P.get("Cl1") * v_abs * v_abs
            : 0.0;

    out.aero_downforce_rear_N =
        aero_enabled
            ? P.get("Cl2") * v_abs * v_abs
            : 0.0;

    out.aero_downforce_total_N =
        out.aero_downforce_front_N +
        out.aero_downforce_rear_N;

    out.rolling_resistance_N =
        P.get("Cr") * P.get("m") * P.get("g");

    out.resistance_constant_N =
        P.get("resistance_constant_N");

    out.resistance_linear_N =
        P.get("resistance_linear_N_per_mps") * v_abs;

    out.resistance_total_N =
        out.aero_drag_N +
        out.rolling_resistance_N +
        out.resistance_constant_N +
        out.resistance_linear_N;

    out.resistance_signed_N =
        out.resistance_total_N * resistanceSign(x.vx);

    // Tire-force states are expressed in wheel frames. Use the shared
    // transform so body dynamics and suspension use exactly the same forces.
    const TireForcesBody force_body =
        tire_forces_in_body_frame(x);

    const double Fx_fl_body = force_body.fx_fl;
    const double Fy_fl_body = force_body.fy_fl;

    const double Fx_fr_body = force_body.fx_fr;
    const double Fy_fr_body = force_body.fy_fr;

    const double Fx_rl_body = force_body.fx_rl;
    const double Fy_rl_body = force_body.fy_rl;

    const double Fx_rr_body = force_body.fx_rr;
    const double Fy_rr_body = force_body.fy_rr;

    out.Fx_total_N =
        Fx_fl_body
        + Fx_fr_body
        + Fx_rl_body
        + Fx_rr_body
        - out.resistance_signed_N;

    out.Fy_total_N =
        Fy_fl_body
        + Fy_fr_body
        + Fy_rl_body
        + Fy_rr_body;

    const double l_r = P.get("l_r");
    const double l_f = P.get("l_f");

    const double t_front = P.get("t_front");
    const double t_rear = P.get("t_rear");

    out.Mz_Nm =
        l_f * Fy_fl_body
        - (t_front / 2.0) * Fx_fl_body;

    out.Mz_Nm +=
        l_f * Fy_fr_body
        - (-t_front / 2.0) * Fx_fr_body;

    out.Mz_Nm +=
        (-l_r) * Fy_rl_body
        - (t_rear / 2.0) * Fx_rl_body;

    out.Mz_Nm +=
        (-l_r) * Fy_rr_body
        - (-t_rear / 2.0) * Fx_rr_body;

    out.ax_force_mps2 =
        out.Fx_total_N / std::max(m, kEpsSmall);

    out.ay_force_mps2 =
        out.Fy_total_N / std::max(m, kEpsSmall);

    return out;
}

Input applyTorqueAndPowerLimit(const State& x,
                               const Input& u,
                               const ParamBank& P)
{
    Input u_limited = u;

    const double T_min = P.get("min_torque");
    const double T_max = P.get("max_torque");

    const double P_rec_total = P.get("P_min_recup");
    const double P_drv_total = P.get("P_max_drive");

    const double P_rec_per_wheel =
        0.25 * std::abs(P_rec_total);

    const double P_drv_per_wheel =
        0.25 * std::abs(P_drv_total);

    const double omega_eps = 15.0;

    auto limit_one_wheel =
        [&](double T_request,
            double omega) -> double
    {
        const double omega_safe =
            std::max(std::abs(omega), omega_eps);

        const double T_min_power =
            -P_rec_per_wheel / omega_safe;

        const double T_max_power =
            P_drv_per_wheel / omega_safe;

        const double T_lower =
            std::max(T_min, T_min_power);

        const double T_upper =
            std::min(T_max, T_max_power);

        return std::clamp(
            T_request,
            T_lower,
            T_upper
        );
    };

    u_limited.torque_request_fl =
        limit_one_wheel(
            u.torque_request_fl,
            x.omega_fl
        );

    u_limited.torque_request_fr =
        limit_one_wheel(
            u.torque_request_fr,
            x.omega_fr
        );

    u_limited.torque_request_rl =
        limit_one_wheel(
            u.torque_request_rl,
            x.omega_rl
        );

    u_limited.torque_request_rr =
        limit_one_wheel(
            u.torque_request_rr,
            x.omega_rr
        );

    return u_limited;
}

} // anonymous namespace

State model_derative_without_tire_and_wheel(const ParamBank& P,
                                           const State& x,
                                           const Input& u)
{
    const ForceBalance fb = computeForceBalance(x, P);

    State temp;
    temp.setZero();

    temp.x =
        x.vx * std::cos(x.yaw)
        - x.vy * std::sin(x.yaw);

    temp.y =
        x.vy * std::cos(x.yaw)
        + x.vx * std::sin(x.yaw);

    temp.yaw =
        x.yaw_rate;

    temp.vx =
        fb.ax_force_mps2
        + x.vy * x.yaw_rate;

    temp.vy =
        fb.ay_force_mps2
        - x.vx * x.yaw_rate;

    temp.yaw_rate =
        fb.Mz_Nm / P.get("Iz");

    temp.prev_ax =
        (
            fb.ax_force_mps2
            - x.prev_ax
        ) / P.get("simulation_time_step");

    temp.prev_ay =
        (
            fb.ay_force_mps2
            - x.prev_ay
        ) / P.get("simulation_time_step");

    temp += normal_forces(P, x, u);
    temp += derative_steering(P, x, u);
    temp += derative_drivetrain(P, x, u);

    return temp;
}

State model_derative_tire_and_wheel_only(const ParamBank& P,
                                         const State& x,
                                         const Input& u)
{
    State temp;
    temp.setZero();

    temp += derative_tire_model(P, x, u);
    temp += derative_wheels_dynamics_model(P, x, u);

    return temp;
}

State model_derative(const ParamBank& P, const State& x, const Input& u)
{
    State temp =
        model_derative_without_tire_and_wheel(P, x, u);

    temp +=
        model_derative_tire_and_wheel_only(P, x, u);

    return temp;
}

Log_Info_full log_info_full(const State& x,
                            const Input& u,
                            const ParamBank& P,
                            int step_number)
{
    Log_Info_full info{};

    const double epsilon = P.get("epsilon");
    const double R = P.get("R");

    const WheelLocalVelocities wheel =
        computeWheelLocalVelocities(x, P);

    const NormalLoads N =
        computeNormalLoads(x, P);

    const ForceBalance fb =
        computeForceBalance(x, P);

    const Input u_limited =
        applyTorqueAndPowerLimit(x, u, P);

    const double kappa_fl =
        kappaAccStyle(wheel.vx_fl, x.omega_fl, R);

    const double kappa_fr =
        kappaAccStyle(wheel.vx_fr, x.omega_fr, R);

    const double kappa_rl =
        kappaAccStyle(wheel.vx_rl, x.omega_rl, R);

    const double kappa_rr =
        kappaAccStyle(wheel.vx_rr, x.omega_rr, R);

    info.kappa_fl = kappa_fl;
    info.kappa_fr = kappa_fr;
    info.kappa_rl = kappa_rl;
    info.kappa_rr = kappa_rr;

    const double slip_angle_fr =
        -std::atan2(
            wheel.vy_fr,
            safeSignedVxForAlpha(wheel.vx_fr, epsilon)
        );

    const double slip_angle_fl =
        -std::atan2(
            wheel.vy_fl,
            safeSignedVxForAlpha(wheel.vx_fl, epsilon)
        );

    const double slip_angle_rr =
        -std::atan2(
            wheel.vy_rr,
            safeSignedVxForAlpha(wheel.vx_rr, epsilon)
        );

    const double slip_angle_rl =
        -std::atan2(
            wheel.vy_rl,
            safeSignedVxForAlpha(wheel.vx_rl, epsilon)
        );

    info.slip_angle_fl = slip_angle_fl;
    info.slip_angle_fr = slip_angle_fr;
    info.slip_angle_rl = slip_angle_rl;
    info.slip_angle_rr = slip_angle_rr;

    info.slip_angle_body =
        std::atan2(
            x.vy,
            safeSignedVxForAlpha(x.vx, epsilon)
        );

    info.fz_fl = N.fl;
    info.fz_fr = N.fr;
    info.fz_rl = N.rl;
    info.fz_rr = N.rr;

    info.fy_fl = x.fy_fl;
    info.fy_fr = x.fy_fr;
    info.fy_rl = x.fy_rl;
    info.fy_rr = x.fy_rr;

    info.fx_fl = x.fx_fl;
    info.fx_fr = x.fx_fr;
    info.fx_rl = x.fx_rl;
    info.fx_rr = x.fx_rr;

    info.power_total =
        (
            x.torque_fl * x.omega_fl
            + x.torque_fr * x.omega_fr
            + x.torque_rl * x.omega_rl
            + x.torque_rr * x.omega_rr
        );

    info.torque_total =
        x.torque_fl
        + x.torque_fr
        + x.torque_rl
        + x.torque_rr;

    info.torque_fl = x.torque_fl;
    info.torque_fr = x.torque_fr;
    info.torque_rl = x.torque_rl;
    info.torque_rr = x.torque_rr;

    info.omega_fl = x.omega_fl;
    info.omega_fr = x.omega_fr;
    info.omega_rl = x.omega_rl;
    info.omega_rr = x.omega_rr;

    info.delta_left = x.delta_left;

    info.delta_right = x.delta_right;

    info.rack_angle = x.rack_angle;

    info.ax = fb.ax_force_mps2;
    info.ay = fb.ay_force_mps2;

    info.yaw_rate = x.yaw_rate;
    info.vx = x.vx;
    info.vy = x.vy;

    info.time =
        step_number * P.get("simulation_time_step");

    info.rack_angle_request = u.rack_angle_request;

    info.torque_request =
        u_limited.torque_request_fl
        + u_limited.torque_request_fr
        + u_limited.torque_request_rl
        + u_limited.torque_request_rr;

    info.x = x.x;
    info.y = x.y;
    info.yaw = x.yaw;

    info.total_drag =
        fb.resistance_total_N;

    info.total_downforce =
        fb.aero_downforce_total_N;

    return info;
}

void euler_sim_timestep(State& x, const Input& u, const ParamBank& P)
{
    Input u_limited =
        applyTorqueAndPowerLimit(x, u, P);

    const double dt =
        P.get("simulation_time_step");

    //// ------------------------------------------------------------------------
    //// Split integration:
    ////
    //// Slow part:
    ////     body motion, yaw dynamics, load transfer, steering, drivetrain torque
    ////     actuator dynamics.
    ////
    //// Fast part:
    ////     tire force relaxation + wheel angular dynamics.
    ////
    //// The stiff loop is:
    ////     omega -> kappa -> Fx -> omega
    ////
    //// Therefore I integrate only tire + wheel dynamics with smaller local
    //// substeps. I do not reduce the global simulator step, so the rest of the
    //// model keeps the same CPU cost.
    //// ------------------------------------------------------------------------

    State dx_slow =
        model_derative_without_tire_and_wheel(P, x, u_limited);

    x += dx_slow * dt;

    constexpr int tire_wheel_substeps = 8;

    const double dt_sub =
        dt / static_cast<double>(tire_wheel_substeps);

    for (int i = 0; i < tire_wheel_substeps; ++i) {
        State dx_fast =
            model_derative_tire_and_wheel_only(P, x, u_limited);

        x += dx_fast * dt_sub;
    }

    unwrap_angle(x.yaw);

    if (x.d_rack_angle > P.get("max_steering_angle_rate")) {
        x.d_rack_angle = P.get("max_steering_angle_rate");
        x.d_delta_left = P.get("max_steering_angle_rate");
        x.d_delta_right = P.get("max_steering_angle_rate");
    }

    if (x.d_rack_angle < P.get("min_steering_angle_rate")) {
        x.d_rack_angle = P.get("min_steering_angle_rate");
        x.d_delta_left = P.get("min_steering_angle_rate");
        x.d_delta_right = P.get("min_steering_angle_rate");
    }

    if (x.rack_angle >= P.get("max_steer")) {
        x.d_delta_left = std::min(0.0, x.d_delta_left);
        x.d_delta_right = std::min(0.0, x.d_delta_right);
        x.d_rack_angle = std::min(0.0, x.d_rack_angle);
    }

    if (x.rack_angle <= P.get("min_steer")) {
        x.d_delta_left = std::max(0.0, x.d_delta_left);
        x.d_delta_right = std::max(0.0, x.d_delta_right);
        x.d_rack_angle = std::max(0.0, x.d_rack_angle);
    }

    x.rack_angle =
        std::clamp(
            x.rack_angle,
            P.get("min_steer"),
            P.get("max_steer")
        );
}

// void rk4_sim_timestep(State& x, const Input& u, const ParamBank& P)
// {
//     double dt = P.get("simulation_time_step");
//
//     State k1 = model_derative(P, x, u);
//     State k2 = model_derative(P, x + 0.5 * dt * k1, u);
//     State k3 = model_derative(P, x + 0.5 * dt * k2, u);
//     State k4 = model_derative(P, x + dt * k3, u);
//
//     x += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
//
//     unwrap_angle(x.yaw);
// }

} // namespace lem_dynamics_sim_

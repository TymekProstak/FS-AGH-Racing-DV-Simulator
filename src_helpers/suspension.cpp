#include "suspension.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lem_dynamics_sim_
{

namespace
{

constexpr double kEps = 1.0e-9;

struct WheelLoads
{
    double rr = 0.0;
    double rl = 0.0;
    double fr = 0.0;
    double fl = 0.0;
};

struct NormalForceTargets
{
    WheelLoads direct;       // static + aero + unsprung + geometric, instant
    WheelLoads lat_elastic;  // target elastic roll transfer [N]
    WheelLoads long_elastic; // target elastic pitch transfer [N]
};

inline WheelLoads operator+(const WheelLoads& a, const WheelLoads& b)
{
    WheelLoads out;
    out.rr = a.rr + b.rr;
    out.rl = a.rl + b.rl;
    out.fr = a.fr + b.fr;
    out.fl = a.fl + b.fl;
    return out;
}

inline bool allFinite(const WheelLoads& a)
{
    return std::isfinite(a.rr)
        && std::isfinite(a.rl)
        && std::isfinite(a.fr)
        && std::isfinite(a.fl);
}

inline WheelLoads getLatElasticState(const State& x)
{
    return WheelLoads{
        x.N_lat_el_rr,
        x.N_lat_el_rl,
        x.N_lat_el_fr,
        x.N_lat_el_fl
    };
}

inline WheelLoads getLongElasticState(const State& x)
{
    return WheelLoads{
        x.N_long_el_rr,
        x.N_long_el_rl,
        x.N_long_el_fr,
        x.N_long_el_fl
    };
}

inline WheelLoads getLatElasticRateState(const State& x)
{
    return WheelLoads{
        x.d_N_lat_el_rr,
        x.d_N_lat_el_rl,
        x.d_N_lat_el_fr,
        x.d_N_lat_el_fl
    };
}

inline WheelLoads getLongElasticRateState(const State& x)
{
    return WheelLoads{
        x.d_N_long_el_rr,
        x.d_N_long_el_rl,
        x.d_N_long_el_fr,
        x.d_N_long_el_fl
    };
}

inline WheelLoads getTotalNormalState(const State& x)
{
    return WheelLoads{x.N_rr, x.N_rl, x.N_fr, x.N_fl};
}

inline bool totalNormalLoadsLookUninitialized(const State& x)
{
    const double sum_abs =
        std::abs(x.N_rr)
        + std::abs(x.N_rl)
        + std::abs(x.N_fr)
        + std::abs(x.N_fl);

    return sum_abs < 1.0e-9;
}

void addLongitudinalTransfer(WheelLoads& loads, double dFz_front_to_rear)
{
    // Positive dFz means load moves from front axle to rear axle.
    loads.fr -= 0.5 * dFz_front_to_rear;
    loads.fl -= 0.5 * dFz_front_to_rear;
    loads.rr += 0.5 * dFz_front_to_rear;
    loads.rl += 0.5 * dFz_front_to_rear;
}

void addLateralTransferFront(WheelLoads& loads, double dFz_inside_to_right)
{
    // ay > 0 means acceleration to the left, therefore right is outside.
    loads.fr += dFz_inside_to_right;
    loads.fl -= dFz_inside_to_right;
}

void addLateralTransferRear(WheelLoads& loads, double dFz_inside_to_right)
{
    loads.rr += dFz_inside_to_right;
    loads.rl -= dFz_inside_to_right;
}

NormalForceTargets computeNormalForceTargets(const ParamBank& P,
                                             const State& x,
                                             double ax,
                                             double ay)
{
    NormalForceTargets target;

    const double m = P.get("m");
    const double g = P.get("g");
    const double L = P.get("wheelbase");

    const double l_f = P.get("l_f");
    const double l_r = P.get("l_r");

    const double t_front = P.get("t_front");
    const double t_rear = P.get("t_rear");

    const double m_sprung = P.get("m_sprung");
    const double h_sprung = P.get("h_sprung");

    const double m_unsprung_front = P.get("m_unsprung_front");
    const double m_unsprung_rear = P.get("m_unsprung_rear");
    const double h_unsprung_front = P.get("h_unsprung_front");
    const double h_unsprung_rear = P.get("h_unsprung_rear");

    const double h_rc_front = P.get("h_rc_front");
    const double h_rc_rear = P.get("h_rc_rear");

    const double lambda_phi = P.get("lambda_phi_elastic_lateral");

    if (!std::isfinite(L) || L <= kEps
        || !std::isfinite(t_front) || t_front <= kEps
        || !std::isfinite(t_rear) || t_rear <= kEps
        || !std::isfinite(lambda_phi)
        || lambda_phi < 0.0 || lambda_phi > 1.0)
    {
        throw std::runtime_error(
            "computeNormalForceTargets: invalid vehicle/load-transfer parameters"
        );
    }

    // Tire states are stored in wheel frames. Convert once and use only
    // body-frame forces in the load-transfer equations.
    const TireForcesBody force_body =
        tire_forces_in_body_frame(x);

    const double Fx_front_body =
        force_body.fx_fl + force_body.fx_fr;

    const double Fx_rear_body =
        force_body.fx_rl + force_body.fx_rr;

    const double Fy_front_body =
        force_body.fy_fl + force_body.fy_fr;

    const double Fy_rear_body =
        force_body.fy_rl + force_body.fy_rr;

    // ========================================================================
    // 1. Static loads and aerodynamic downforce
    // ========================================================================

    const double Fz_front_static = m * g * l_r / L;
    const double Fz_rear_static = m * g * l_f / L;

    const bool aero_enabled = P.get("aero_package_enabled") > 0.5;
    const double v2 = x.vx * x.vx;

    const double Fz_aero_front =
        aero_enabled ? P.get("Cl1") * v2 : 0.0;

    const double Fz_aero_rear =
        aero_enabled ? P.get("Cl2") * v2 : 0.0;

    target.direct.fr += 0.5 * (Fz_front_static + Fz_aero_front);
    target.direct.fl += 0.5 * (Fz_front_static + Fz_aero_front);

    target.direct.rr += 0.5 * (Fz_rear_static + Fz_aero_rear);
    target.direct.rl += 0.5 * (Fz_rear_static + Fz_aero_rear);

    // ========================================================================
    // 2. Longitudinal transfer
    //
    // Total target is acceleration-based. Unsprung and pitch-center geometry
    // are direct. The remaining sprung contribution is the target of the
    // second-order elastic pitch-transfer state.
    // ========================================================================

    const double dFz_long_unsprung =
        ax * (
            m_unsprung_front * h_unsprung_front
            + m_unsprung_rear * h_unsprung_rear
        ) / L;

    // Tire forces spent accelerating the unsprung axle assemblies do not pass
    // through the sprung-mass pitch geometry.
    const double Fx_sprung_front =
        Fx_front_body - m_unsprung_front * ax;

    const double Fx_sprung_rear =
        Fx_rear_body - m_unsprung_rear * ax;

    const double dFz_long_geo =
        P.get("pitch_geo_gain_front") * Fx_sprung_front
        + P.get("pitch_geo_gain_rear") * Fx_sprung_rear;

    const double dFz_long_direct =
        dFz_long_unsprung + dFz_long_geo;

    addLongitudinalTransfer(target.direct, dFz_long_direct);

    const double dFz_long_sprung_ss =
        m_sprung * ax * h_sprung / L;

    const double dFz_long_elastic_target =
        dFz_long_sprung_ss - dFz_long_geo;

    addLongitudinalTransfer(
        target.long_elastic,
        dFz_long_elastic_target
    );

    // ========================================================================
    // 3. Lateral transfer
    //
    // Per axle:
    //   direct = unsprung transfer + force-based roll-center transfer
    //   elastic target = remaining sprung roll moment, split by roll stiffness
    // ========================================================================

    const double Fy_sprung_front =
        Fy_front_body - m_unsprung_front * ay;

    const double Fy_sprung_rear =
        Fy_rear_body - m_unsprung_rear * ay;

    const double dFz_lat_unsprung_front =
        m_unsprung_front * ay * h_unsprung_front / t_front;

    const double dFz_lat_unsprung_rear =
        m_unsprung_rear * ay * h_unsprung_rear / t_rear;

    const double dFz_lat_geo_front =
        Fy_sprung_front * h_rc_front / t_front;

    const double dFz_lat_geo_rear =
        Fy_sprung_rear * h_rc_rear / t_rear;

    addLateralTransferFront(
        target.direct,
        dFz_lat_unsprung_front + dFz_lat_geo_front
    );

    addLateralTransferRear(
        target.direct,
        dFz_lat_unsprung_rear + dFz_lat_geo_rear
    );

    const double M_roll_sprung_ss =
        m_sprung * ay * h_sprung;

    const double M_roll_geo =
        Fy_sprung_front * h_rc_front
        + Fy_sprung_rear * h_rc_rear;

    const double M_roll_elastic_target =
        M_roll_sprung_ss - M_roll_geo;

    const double dFz_lat_elastic_front =
        lambda_phi * M_roll_elastic_target / t_front;

    const double dFz_lat_elastic_rear =
        (1.0 - lambda_phi) * M_roll_elastic_target / t_rear;

    addLateralTransferFront(
        target.lat_elastic,
        dFz_lat_elastic_front
    );

    addLateralTransferRear(
        target.lat_elastic,
        dFz_lat_elastic_rear
    );

    if (!allFinite(target.direct)
        || !allFinite(target.lat_elastic)
        || !allFinite(target.long_elastic))
    {
        throw std::runtime_error(
            "computeNormalForceTargets: non-finite normal-force target"
        );
    }

    return target;
}

inline double secondOrderAcceleration(double target,
                                      double position,
                                      double rate,
                                      double omega,
                                      double zeta)
{
    return omega * omega * (target - position)
        - 2.0 * zeta * omega * rate;
}

} // namespace


State normal_forces(const ParamBank& P, const State& x, const Input& u)
{
    (void)u;

    State dx;
    dx.setZero();

    const double dt = P.get("simulation_time_step");

    if (!std::isfinite(dt) || dt <= 0.0) {
        throw std::runtime_error(
            "normal_forces: simulation_time_step must be positive"
        );
    }

    // One-step delayed accelerations avoid the algebraic loop:
    // N -> tire forces -> acceleration -> N.
    const double ax = x.prev_ax;
    const double ay = x.prev_ay;

    const NormalForceTargets target =
        computeNormalForceTargets(P, x, ax, ay);

    const WheelLoads N_total_current = getTotalNormalState(x);
    const WheelLoads N_lat_current = getLatElasticState(x);
    const WheelLoads N_long_current = getLongElasticState(x);
    const WheelLoads dN_lat_current = getLatElasticRateState(x);
    const WheelLoads dN_long_current = getLongElasticRateState(x);

    const double omega_phi = P.get("omega_phi");
    const double omega_theta = P.get("omega_theta");
    const double zeta_phi = P.get("zeta_phi_for_load_transfer");
    const double zeta_theta = P.get("zeta_theta_for_load_transfer");

    // ========================================================================
    // Initialization: place total and elastic states directly at equilibrium.
    // ========================================================================

    if (totalNormalLoadsLookUninitialized(x)) {
        const WheelLoads N_total_target =
            target.direct + target.lat_elastic + target.long_elastic;

        dx.N_rr = (N_total_target.rr - x.N_rr) / dt;
        dx.N_rl = (N_total_target.rl - x.N_rl) / dt;
        dx.N_fr = (N_total_target.fr - x.N_fr) / dt;
        dx.N_fl = (N_total_target.fl - x.N_fl) / dt;

        dx.N_lat_el_rr = (target.lat_elastic.rr - x.N_lat_el_rr) / dt;
        dx.N_lat_el_rl = (target.lat_elastic.rl - x.N_lat_el_rl) / dt;
        dx.N_lat_el_fr = (target.lat_elastic.fr - x.N_lat_el_fr) / dt;
        dx.N_lat_el_fl = (target.lat_elastic.fl - x.N_lat_el_fl) / dt;

        dx.N_long_el_rr = (target.long_elastic.rr - x.N_long_el_rr) / dt;
        dx.N_long_el_rl = (target.long_elastic.rl - x.N_long_el_rl) / dt;
        dx.N_long_el_fr = (target.long_elastic.fr - x.N_long_el_fr) / dt;
        dx.N_long_el_fl = (target.long_elastic.fl - x.N_long_el_fl) / dt;

        dx.d_N_lat_el_rr = -x.d_N_lat_el_rr / dt;
        dx.d_N_lat_el_rl = -x.d_N_lat_el_rl / dt;
        dx.d_N_lat_el_fr = -x.d_N_lat_el_fr / dt;
        dx.d_N_lat_el_fl = -x.d_N_lat_el_fl / dt;

        dx.d_N_long_el_rr = -x.d_N_long_el_rr / dt;
        dx.d_N_long_el_rl = -x.d_N_long_el_rl / dt;
        dx.d_N_long_el_fr = -x.d_N_long_el_fr / dt;
        dx.d_N_long_el_fl = -x.d_N_long_el_fl / dt;

        return dx;
    }

    // ========================================================================
    // Second-order elastic roll transfer
    // ========================================================================

    dx.N_lat_el_rr = dN_lat_current.rr;
    dx.N_lat_el_rl = dN_lat_current.rl;
    dx.N_lat_el_fr = dN_lat_current.fr;
    dx.N_lat_el_fl = dN_lat_current.fl;

    dx.d_N_lat_el_rr = secondOrderAcceleration(
        target.lat_elastic.rr, N_lat_current.rr, dN_lat_current.rr,
        omega_phi, zeta_phi
    );
    dx.d_N_lat_el_rl = secondOrderAcceleration(
        target.lat_elastic.rl, N_lat_current.rl, dN_lat_current.rl,
        omega_phi, zeta_phi
    );
    dx.d_N_lat_el_fr = secondOrderAcceleration(
        target.lat_elastic.fr, N_lat_current.fr, dN_lat_current.fr,
        omega_phi, zeta_phi
    );
    dx.d_N_lat_el_fl = secondOrderAcceleration(
        target.lat_elastic.fl, N_lat_current.fl, dN_lat_current.fl,
        omega_phi, zeta_phi
    );

    // ========================================================================
    // Second-order elastic pitch transfer
    // ========================================================================

    dx.N_long_el_rr = dN_long_current.rr;
    dx.N_long_el_rl = dN_long_current.rl;
    dx.N_long_el_fr = dN_long_current.fr;
    dx.N_long_el_fl = dN_long_current.fl;

    dx.d_N_long_el_rr = secondOrderAcceleration(
        target.long_elastic.rr, N_long_current.rr, dN_long_current.rr,
        omega_theta, zeta_theta
    );
    dx.d_N_long_el_rl = secondOrderAcceleration(
        target.long_elastic.rl, N_long_current.rl, dN_long_current.rl,
        omega_theta, zeta_theta
    );
    dx.d_N_long_el_fr = secondOrderAcceleration(
        target.long_elastic.fr, N_long_current.fr, dN_long_current.fr,
        omega_theta, zeta_theta
    );
    dx.d_N_long_el_fl = secondOrderAcceleration(
        target.long_elastic.fl, N_long_current.fl, dN_long_current.fl,
        omega_theta, zeta_theta
    );

    // Reconstruct the total normal loads from the next Euler values of the
    // elastic states. The direct part remains algebraic/instantaneous.
    const WheelLoads N_lat_next{
        N_lat_current.rr + dt * dx.N_lat_el_rr,
        N_lat_current.rl + dt * dx.N_lat_el_rl,
        N_lat_current.fr + dt * dx.N_lat_el_fr,
        N_lat_current.fl + dt * dx.N_lat_el_fl
    };

    const WheelLoads N_long_next{
        N_long_current.rr + dt * dx.N_long_el_rr,
        N_long_current.rl + dt * dx.N_long_el_rl,
        N_long_current.fr + dt * dx.N_long_el_fr,
        N_long_current.fl + dt * dx.N_long_el_fl
    };

    const WheelLoads N_total_next =
        target.direct + N_lat_next + N_long_next;

    dx.N_rr = (N_total_next.rr - N_total_current.rr) / dt;
    dx.N_rl = (N_total_next.rl - N_total_current.rl) / dt;
    dx.N_fr = (N_total_next.fr - N_total_current.fr) / dt;
    dx.N_fl = (N_total_next.fl - N_total_current.fl) / dt;

    return dx;
}

} // namespace lem_dynamics_sim_
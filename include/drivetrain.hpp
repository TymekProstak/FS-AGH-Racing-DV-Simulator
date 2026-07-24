#pragma once

#include "ParamBank.hpp"
#include "utilities.hpp"

// Zakładam układ 1 rzędu dla każdego koła osobno.
// Input ma osobny torque request na każde koło:
// torque_request_fl, torque_request_fr, torque_request_rl, torque_request_rr

namespace lem_dynamics_sim_
{

inline State derative_drivetrain(const ParamBank& P,
                                 const State& x,
                                 const Input& u)
{
    State temp;
    temp.setZero();

    const double tau = P.get("drivetrain_timescale");

    temp.torque_fl =
        (u.torque_request_fl - x.torque_fl) / tau;

    temp.torque_fr =
        (u.torque_request_fr - x.torque_fr) / tau;

    temp.torque_rl =
        (u.torque_request_rl - x.torque_rl) / tau;

    temp.torque_rr =
        (u.torque_request_rr - x.torque_rr) / tau;

    return temp;
}

} // namespace lem_dynamics_sim_

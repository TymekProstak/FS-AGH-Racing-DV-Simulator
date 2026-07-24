#pragma once

#include "utilities.hpp"
#include "ParamBank.hpp"

#include <cmath>

namespace lem_dynamics_sim_{

    struct delta_angels{
        double delta_left;
        double delta_right;
    };

    /*
        rack_angle tutaj oznacza:
            delta_bicycle / delta_center [rad]

        Dopasowanie z geometrii:
            delta_right = d + 0.09298947992255 * d^2 + 0.00324282679268
            delta_left  = d - 0.09298947992255 * d^2 - 0.00324282679268

        Zakres dopasowania:
            około +/- 0.4528 rad, czyli +/- 25.94 deg

        Dla Twojego max_steer = 0.38 rad jesteśmy w zakresie.
    */

    inline delta_angels anit_akerman(
        const double rack_angle,
        const ParamBank& P
    ){
        (void)P;

        const double d =
            rack_angle;

        const double ackermann_quad_coeff =
            9.298947992255e-02;

        const double toe_offset_rad =
            3.242826792680e-03;

        const double d2 =
            d * d;

        delta_angels out;

        out.delta_left =
            d
            - ackermann_quad_coeff * d2
            - toe_offset_rad;

        out.delta_right =
            d
            + ackermann_quad_coeff * d2
            + toe_offset_rad;

        return out;
    }

    /*
        Pochodna po rack_angle / delta_bicycle:

            d(delta_right)/d(d) = 1 + 2 * k * d
            d(delta_left)/d(d)  = 1 - 2 * k * d

        Stały toe_offset znika w pochodnej.
    */

    inline delta_angels derative_with_respect_to_rack_angle_anti_akerman(
        const double rack_angle,
        const ParamBank& P
    ){
        (void)P;

        const double d =
            rack_angle;

        const double ackermann_quad_coeff =
            9.298947992255e-02;

        delta_angels out;

        out.delta_left =
            1.0
            - 2.0 * ackermann_quad_coeff * d;

        out.delta_right =
            1.0
            + 2.0 * ackermann_quad_coeff * d;

        return out;
    }

    State derative_steering(
        const ParamBank& P,
        const State& x,
        const Input& u
    );

}

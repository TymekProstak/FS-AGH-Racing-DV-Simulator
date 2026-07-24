#include "tire_model.hpp"

#include <algorithm>
#include <cmath>

namespace lem_dynamics_sim_
{

State derative_tire_model(const ParamBank& P, const State& x, const Input& u)
{
    State temp;
    temp.setZero();

    (void)u;

    // ============================================================
    // 1) Parametry numerical safety + relaksacje
    // ============================================================

    const double epsilon = P.get("epsilon");

    const double cx = P.get("cx");
    const double dx = P.get("dx");
    const double cy = P.get("cy");
    const double dy = P.get("dy");

    const double dt = P.get("simulation_time_step");

    // ============================================================
    // 2) Geometria kół + promień
    // ============================================================

    const double r_rear  = P.get("r_rear");
    const double r_front = P.get("r_front");

    const double ang_f = P.get("angle_construction_front");
    const double ang_r = P.get("angle_construction_rear");

    const double R = P.get("R");

    // ============================================================
    // 3) MF 6.1 - parametry no camber
    // ============================================================

    // -------- LONGITUDINAL MF6.1 --------

    const double pCx1 = P.get("pCx1");

    const double pDx1 = P.get("pDx1");
    const double pDx2 = P.get("pDx2");

    const double pEx1 = P.get("pEx1");
    const double pEx2 = P.get("pEx2");
    const double pEx3 = P.get("pEx3");
    const double pEx4 = P.get("pEx4");

    const double pKx1 = P.get("pKx1");
    const double pKx2 = P.get("pKx2");
    const double pKx3 = P.get("pKx3");

    const double pHx1 = P.get("pHx1");
    const double pHx2 = P.get("pHx2");

    const double pVx1 = P.get("pVx1");
    const double pVx2 = P.get("pVx2");

    const double lambda_x = P.get("lambda_x");

    // -------- LATERAL MF6.1 --------

    const double pCy1 = P.get("pCy1");

    const double pDy1 = P.get("pDy1");
    const double pDy2 = P.get("pDy2");

    const double pEy1 = P.get("pEy1");
    const double pEy2 = P.get("pEy2");
    const double pEy3 = P.get("pEy3");

    const double pKy1 = P.get("pKy1");
    const double pKy2 = P.get("pKy2");
    const double pKy4 = P.get("pKy4");

    const double pHy1 = P.get("pHy1");
    const double pHy2 = P.get("pHy2");

    const double pVy1 = P.get("pVy1");
    const double pVy2 = P.get("pVy2");

    const double lambda_y = P.get("lambda_y");

    const double N0 = P.get("N0");

    // ============================================================
    // 4) Kinematyka kół
    // ============================================================

    double vx_rr = x.vx + x.yaw_rate * r_rear  * std::sin(ang_r);
    double vy_rr = x.vy - x.yaw_rate * r_rear  * std::cos(ang_r);

    double vx_rl = x.vx - x.yaw_rate * r_rear  * std::sin(ang_r);
    double vy_rl = x.vy - x.yaw_rate * r_rear  * std::cos(ang_r);

    double vx_fr = x.vx + x.yaw_rate * r_front * std::sin(ang_f);
    double vy_fr = x.vy + x.yaw_rate * r_front * std::cos(ang_f);

    double vx_fl = x.vx - x.yaw_rate * r_front * std::sin(ang_f);
    double vy_fl = x.vy + x.yaw_rate * r_front * std::cos(ang_f);

    // ============================================================
    // 5) Rzut prędkości do ramy koła
    //
    // Po tym bloku:
    //      vx_* = prędkość lokalna wzdłuż osi koła,
    //      vy_* = prędkość lokalna poprzecznie do osi koła.
    // ============================================================

    // RR, tył bez skrętu
    {
        const double c = 1.0;
        const double s = 0.0;

        const double vx0 = vx_rr;
        const double vy0 = vy_rr;

        vx_rr =  c * vx0 + s * vy0;
        vy_rr = -s * vx0 + c * vy0;
    }

    // RL, tył bez skrętu
    {
        const double c = 1.0;
        const double s = 0.0;

        const double vx0 = vx_rl;
        const double vy0 = vy_rl;

        vx_rl =  c * vx0 + s * vy0;
        vy_rl = -s * vx0 + c * vy0;
    }

    // FR, przód ze skrętem
    {
        const double c = std::cos(x.delta_right);
        const double s = std::sin(x.delta_right);

        const double vx0 = vx_fr;
        const double vy0 = vy_fr;

        vx_fr =  c * vx0 + s * vy0;
        vy_fr = -s * vx0 + c * vy0;
    }

    // FL, przód ze skrętem
    {
        const double c = std::cos(x.delta_left);
        const double s = std::sin(x.delta_left);

        const double vx0 = vx_fl;
        const double vy0 = vy_fl;

        vx_fl =  c * vx0 + s * vy0;
        vy_fl = -s * vx0 + c * vy0;
    }

    // ============================================================
    // 6) Clampy / epsy / mianownik slip ratio / prędkość relaksacji
    // ============================================================

    const double EPS_VX_ALPHA = std::max(epsilon, 1.0e-4);

    /*
        Minimalna długość relaksacji jest celowo rozdzielona:
            - lateral zostawiam praktycznie jak wcześniej,
            - longitudinal daję większe minimum, bo oscylacje były głównie
              w slip ratio / Fx, a nie w slip angle / Fy.

        To NIE jest sztuczne zwiększanie bezwładności koła. Koło może mieć
        fizyczne I_tire, a numeryczna/efektywna gładkość Fx pochodzi z
        relaksacji siły opony.
    */
    const double L_RELAX_MIN_ALPHA = 0.01;
    const double L_RELAX_MIN_KAPPA = 0.08;

    /*
        Lokalny substep relaksacji sił opony.

        Uwaga praktyczna:
        ponieważ w tej funkcji kinematyka koła i Fmf są zamrożone na cały
        główny krok, substep nie zastępuje pełnego substepu układu
        omega-kappa-Fx. Daje jednak bezpieczniejszą strukturę kodu pod dalsze
        rozszerzenie i ogranicza ryzyko agresywnej jednopróbkowej aktualizacji.
    */
    const int TIRE_RELAX_SUBSTEPS = 4;

    const double Z_MAX = 50.0;

    // Trzymam 1.5, żeby zgadzało się z generatorem map.
    // Docelowo najlepiej wynieść to do configa jako v_threshold.
    const double V_THRESHOLD_SLIP = 1.5;

    auto safe_vx_signed = [&](double vx) -> double
    {
        if (std::abs(vx) >= EPS_VX_ALPHA)
        {
            return vx;
        }

        return std::copysign(EPS_VX_ALPHA, (vx == 0.0 ? 1.0 : vx));
    };

    auto relaxation_speed = [&](double vx) -> double
    {
        return std::sqrt(
            vx * vx +
            V_THRESHOLD_SLIP * V_THRESHOLD_SLIP
        );
    };

    auto slip_denominator_numeric = [&](double vx) -> double
    {
        const double v_abs = std::abs(vx);

        const double denom_low =
            0.5 * (V_THRESHOLD_SLIP + (vx * vx) / V_THRESHOLD_SLIP);

        const double denom_high = v_abs;

        if (v_abs > V_THRESHOLD_SLIP)
        {
            return std::max(denom_high, 1.0e-9);
        }

        return std::max(denom_low, 1.0e-9);
    };

    const double vx_rr_den_kappa = slip_denominator_numeric(vx_rr);
    const double vx_rl_den_kappa = slip_denominator_numeric(vx_rl);
    const double vx_fr_den_kappa = slip_denominator_numeric(vx_fr);
    const double vx_fl_den_kappa = slip_denominator_numeric(vx_fl);

    // ============================================================
    // 7) Obciążenia normalne
    // ============================================================
    //
    // Normal forces są liczone przez osobny model zawieszenia /
    // normal_forces i przechowywane w State.
    //
    // Tire model NIE przelicza tutaj transferu masy.
    //
    // Używam:
    //      x.N_fl, x.N_fr, x.N_rl, x.N_rr
    //
    // Czyli:
    //      static + direct/geometric instant
    //      + elastic lateral relaxed
    //      + elastic longitudinal relaxed
    //
    // ============================================================

    const double FZ_MIN = 50.0;

    double N_fl = std::max(x.N_fl, FZ_MIN);
    double N_fr = std::max(x.N_fr, FZ_MIN);
    double N_rl = std::max(x.N_rl, FZ_MIN);
    double N_rr = std::max(x.N_rr, FZ_MIN);

    // ============================================================
    // 8) Kąty poślizgu
    // ============================================================

    double alpha_fr = -std::atan2(vy_fr, safe_vx_signed(vx_fr));
    double alpha_fl = -std::atan2(vy_fl, safe_vx_signed(vx_fl));

    double alpha_rr = -std::atan2(vy_rr, safe_vx_signed(vx_rr));
    double alpha_rl = -std::atan2(vy_rl, safe_vx_signed(vx_rl));

    alpha_fr = std::clamp(alpha_fr, -M_PI / 2.0 + 0.3, M_PI / 2.0 - 0.3);
    alpha_fl = std::clamp(alpha_fl, -M_PI / 2.0 + 0.3, M_PI / 2.0 - 0.3);
    alpha_rr = std::clamp(alpha_rr, -M_PI / 2.0 + 0.3, M_PI / 2.0 - 0.3);
    alpha_rl = std::clamp(alpha_rl, -M_PI / 2.0 + 0.3, M_PI / 2.0 - 0.3);

    // ============================================================
    // 9) Slip ratio
    // ============================================================
    //
    // 4WD / AWD:
    //      każde koło ma własny slip ratio,
    //      bo każde koło może mieć własny moment napędowy / hamujący.
    //
    // Mianownik jest zawsze dodatni i miękki dla małych prędkości.
    // ============================================================

    const double kappa_fl =
        std::clamp((x.omega_fl * R - vx_fl) / vx_fl_den_kappa, -0.99, 0.99);

    const double kappa_fr =
        std::clamp((x.omega_fr * R - vx_fr) / vx_fr_den_kappa, -0.99, 0.99);

    const double kappa_rl =
        std::clamp((x.omega_rl * R - vx_rl) / vx_rl_den_kappa, -0.99, 0.99);

    const double kappa_rr =
        std::clamp((x.omega_rr * R - vx_rr) / vx_rr_den_kappa, -0.99, 0.99);

    // ============================================================
    // 10) dfz
    // ============================================================

    const double dfz_fl = (N_fl - N0) / N0;
    const double dfz_fr = (N_fr - N0) / N0;
    const double dfz_rl = (N_rl - N0) / N0;
    const double dfz_rr = (N_rr - N0) / N0;

    // ============================================================
    // 11) Helper MF6.1 pure + pochodna dF/dx
    // ============================================================

    auto mf61_pure =
        [&](double x_in,
            double C,
            double D,
            double B,
            double E,
            double Sv,
            double& F,
            double& dFdx)
    {
        (void)Sv;

        const double Bx = B * x_in;
        const double atanBx = std::atan(Bx);

        const double phi = Bx - E * (Bx - atanBx);
        const double atanPhi = std::atan(phi);

        F = D * std::sin(C * atanPhi);

        const double d_atanPhi_dphi = 1.0 / (1.0 + phi * phi);
        const double d_atanBx_dBx = 1.0 / (1.0 + Bx * Bx);

        const double dphi_dx = B - E * (B - B * d_atanBx_dBx);

        dFdx =
            D *
            std::cos(C * atanPhi) *
            C *
            d_atanPhi_dphi *
            dphi_dx;
    };

    // ============================================================
    // 12) Pure Fx0(kappa), Fy0(alpha) + coupling elipsą
    // ============================================================

    auto compute_wheel_forces =
        [&](double Fz,
            double dfz,
            double kappa,
            double alpha,
            bool allow_longitudinal,
            double& Fx,
            double& Fy,
            double& dFx_dkappa,
            double& dFy_dalpha,
            double& mu_x,
            double& mu_y)
    {
        // ------------------------------------------------------------
        // LONG pure
        // ------------------------------------------------------------

        double Cx = 0.0;
        double mux = 0.0;
        double Dx = 0.0;
        double Kxk = 0.0;
        double Bx = 0.0;
        double Ex = 0.0;
        double Shx = 0.0;
        double Svx = 0.0;

        {
            Cx = pCx1;

            mux = lambda_x * (pDx1 + pDx2 * dfz);
            mux = std::max(mux, 1.0e-6);

            Dx = mux * Fz;

            Kxk =
                Fz *
                (pKx1 + pKx2 * dfz) *
                std::exp(pKx3 * dfz);

            Shx = pHx1 + pHx2 * dfz;
            Svx = Fz * (pVx1 + pVx2 * dfz);

            Svx = 0.0;

            const double denom = std::max(Cx * Dx, 1.0e-9);
            Bx = Kxk / denom;

            const double E0 = pEx1 + pEx2 * dfz + pEx3 * dfz * dfz;

            auto smooth_sign = [&](double xx) -> double
            {
                const double s = 50.0;
                return std::tanh(s * xx);
            };

            Ex = E0 * (1.0 - pEx4 * smooth_sign(kappa + Shx));
        }

        double Fx0 = 0.0;
        double dFx0_dk = 0.0;

        if (allow_longitudinal)
        {
            const double kx = kappa + Shx;

            mf61_pure(kx, Cx, Dx, Bx, Ex, Svx, Fx0, dFx0_dk);

            double Fx_bias = 0.0;
            double dtmp = 0.0;

            const double kx0 = Shx;
            mf61_pure(kx0, Cx, Dx, Bx, Ex, Svx, Fx_bias, dtmp);

            Fx0 -= Fx_bias;
        }

        // ------------------------------------------------------------
        // LAT pure
        // ------------------------------------------------------------

        double Cy = 0.0;
        double muy = 0.0;
        double Dy = 0.0;
        double Kya = 0.0;
        double By = 0.0;
        double Ey = 0.0;
        double Shy = 0.0;
        double Svy = 0.0;

        {
            Cy = pCy1;

            muy = lambda_y * (pDy1 + pDy2 * dfz);
            muy = std::max(muy, 1.0e-6);

            Dy = muy * Fz;

            Kya =
                pKy1 *
                N0 *
                std::sin(pKy4 * std::atan(Fz / (pKy2 * N0)));

            const double denom = std::max(Cy * Dy, 1.0e-9);
            By = Kya / denom;

            Shy = pHy1 + pHy2 * dfz;
            Svy = Fz * (pVy1 + pVy2 * dfz);

            Svy = 0.0;

            auto smooth_sign = [&](double xx) -> double
            {
                const double s = 50.0;
                return std::tanh(s * xx);
            };

            Ey =
                (pEy1 + pEy2 * dfz) *
                (1.0 - pEy3 * smooth_sign(alpha + Shy));
        }

        const double ay = alpha + Shy;

        double Fy0 = 0.0;
        double dFy0_da = 0.0;

        mf61_pure(ay, Cy, Dy, By, Ey, Svy, Fy0, dFy0_da);

        double Fy_bias = 0.0;
        double dtmp2 = 0.0;

        const double ay0 = Shy;
        mf61_pure(ay0, Cy, Dy, By, Ey, Svy, Fy_bias, dtmp2);

        Fy0 -= Fy_bias;

        // ------------------------------------------------------------
        // Coupling ellipse
        // ------------------------------------------------------------

        const double FxMax = mux * Fz;
        const double FyMax = muy * Fz;

        const double eps = 1.0e-9;

        const double FxMaxSafe = std::max(std::abs(FxMax), eps);
        const double FyMaxSafe = std::max(std::abs(FyMax), eps);

        const double axu = Fx0 / FxMaxSafe;
        const double ayu = Fy0 / FyMaxSafe;

        const double s = std::sqrt(axu * axu + ayu * ayu);

        if (s <= 1.0 || !std::isfinite(s))
        {
            Fx = Fx0;
            Fy = Fy0;
        }
        else
        {
            const double invs = 1.0 / s;

            Fx = Fx0 * invs;
            Fy = Fy0 * invs;
        }

        // ------------------------------------------------------------
        // Pochodne do relaksacji
        // ------------------------------------------------------------

        dFx_dkappa = dFx0_dk;
        dFy_dalpha = dFy0_da;

        mu_x = mux;
        mu_y = muy;
    };

    // ============================================================
    // 13) Model Coulombowski
    // ============================================================
    //
    // NA RAZIE WYŁĄCZONE.
    //
    // Zostawiam cały blok w komentarzu, żeby można było później
    // wrócić do blendowania Coulomb -> MF6.1 przy małych prędkościach.
    //
    // Obecnie testuję zachowanie samej Pacejki / MF6.1 również przy
    // małych prędkościach. Stabilność slip ratio przy vx ~ 0 zapewnia
    // miękki mianownik slip_denominator_numeric().
    // ============================================================

    /*
    auto compute_coulomb_forces =
        [&](double Fz,
            double dfz,
            double vx_loc,
            double vy_loc,
            double omega,
            double& Fxc,
            double& Fyc)
    {
        const double vsx = omega * R - vx_loc;
        const double vsy = -vy_loc;

        const double mu_x_coulomb =
            std::max(lambda_x * (pDx1 + pDx2 * dfz), 1.0e-6);

        const double mu_y_coulomb =
            std::max(lambda_y * (pDy1 + pDy2 * dfz), 1.0e-6);

        const double Fx_max = std::max(mu_x_coulomb * Fz, 1.0e-3);
        const double Fy_max = std::max(mu_y_coulomb * Fz, 1.0e-3);

        const double K_stiff = 800.0;

        Fxc = K_stiff * vsx;
        Fyc = K_stiff * vsy;

        Fxc = std::clamp(Fxc, -Fx_max, Fx_max);
        Fyc = std::clamp(Fyc, -Fy_max, Fy_max);

        const double nx = Fxc / Fx_max;
        const double ny = Fyc / Fy_max;
        const double norm = std::sqrt(nx * nx + ny * ny);

        if (norm > 1.0 && std::isfinite(norm))
        {
            Fxc /= norm;
            Fyc /= norm;
        }
    };
    */

    // ============================================================
    // 14) Zmienne dla MF6.1
    // ============================================================

    double Fx_fl = 0.0;
    double Fy_fl = 0.0;
    double dFx_fl_dk = 0.0;
    double dFy_fl_da = 0.0;
    double mux_fl = 0.0;
    double muy_fl = 0.0;

    double Fx_fr = 0.0;
    double Fy_fr = 0.0;
    double dFx_fr_dk = 0.0;
    double dFy_fr_da = 0.0;
    double mux_fr = 0.0;
    double muy_fr = 0.0;

    double Fx_rl = 0.0;
    double Fy_rl = 0.0;
    double dFx_rl_dk = 0.0;
    double dFy_rl_da = 0.0;
    double mux_rl = 0.0;
    double muy_rl = 0.0;

    double Fx_rr = 0.0;
    double Fy_rr = 0.0;
    double dFx_rr_dk = 0.0;
    double dFy_rr_da = 0.0;
    double mux_rr = 0.0;
    double muy_rr = 0.0;

    // ============================================================
    // 15) Obliczanie sił z MF6.1 / Pacejki
    // ============================================================
    //
    // 4WD / AWD:
    //      allow_longitudinal = true dla wszystkich 4 kół.
    // ============================================================

    compute_wheel_forces(
        N_fl,
        dfz_fl,
        kappa_fl,
        alpha_fl,
        true,
        Fx_fl,
        Fy_fl,
        dFx_fl_dk,
        dFy_fl_da,
        mux_fl,
        muy_fl
    );

    compute_wheel_forces(
        N_fr,
        dfz_fr,
        kappa_fr,
        alpha_fr,
        true,
        Fx_fr,
        Fy_fr,
        dFx_fr_dk,
        dFy_fr_da,
        mux_fr,
        muy_fr
    );

    compute_wheel_forces(
        N_rl,
        dfz_rl,
        kappa_rl,
        alpha_rl,
        true,
        Fx_rl,
        Fy_rl,
        dFx_rl_dk,
        dFy_rl_da,
        mux_rl,
        muy_rl
    );

    compute_wheel_forces(
        N_rr,
        dfz_rr,
        kappa_rr,
        alpha_rr,
        true,
        Fx_rr,
        Fy_rr,
        dFx_rr_dk,
        dFy_rr_da,
        mux_rr,
        muy_rr
    );

    // ============================================================
    // 16) Brak blendowania Coulomb -> MF
    // ============================================================
    //
    // Poprzednio można było robić:
    //      niskie prędkości  -> Coulomb,
    //      wysokie prędkości -> MF6.1,
    //      przejście         -> blend.
    //
    // Obecnie:
    //      wszystkie prędkości -> MF6.1.
    //
    // Jeżeli będę chciał wrócić do Coulomba, to odkomentuję blok
    // compute_coulomb_forces() z sekcji 13 i tutaj dodam blendowanie.
    // ============================================================

    /*
    // Przykładowy szkielet przyszłego blendowania dla 4WD:
    //
    // double Fxc_fl = 0.0;
    // double Fyc_fl = 0.0;
    // double Fxc_fr = 0.0;
    // double Fyc_fr = 0.0;
    // double Fxc_rl = 0.0;
    // double Fyc_rl = 0.0;
    // double Fxc_rr = 0.0;
    // double Fyc_rr = 0.0;
    //
    // compute_coulomb_forces(N_fl, dfz_fl, vx_fl, vy_fl, x.omega_fl, Fxc_fl, Fyc_fl);
    // compute_coulomb_forces(N_fr, dfz_fr, vx_fr, vy_fr, x.omega_fr, Fxc_fr, Fyc_fr);
    // compute_coulomb_forces(N_rl, dfz_rl, vx_rl, vy_rl, x.omega_rl, Fxc_rl, Fyc_rl);
    // compute_coulomb_forces(N_rr, dfz_rr, vx_rr, vy_rr, x.omega_rr, Fxc_rr, Fyc_rr);
    //
    // const double v_blend_start = 0.5;
    // const double v_blend_end = 2.0;
    //
    // auto blend_weight_mf = [&](double vx_loc) -> double
    // {
    //     const double v = std::abs(vx_loc);
    //     const double r =
    //         (v - v_blend_start) /
    //         std::max(v_blend_end - v_blend_start, 1.0e-9);
    //
    //     return std::clamp(r, 0.0, 1.0);
    // };
    //
    // const double w_mf_fl = blend_weight_mf(vx_fl);
    // const double w_mf_fr = blend_weight_mf(vx_fr);
    // const double w_mf_rl = blend_weight_mf(vx_rl);
    // const double w_mf_rr = blend_weight_mf(vx_rr);
    //
    // Fx_fl = w_mf_fl * Fx_fl + (1.0 - w_mf_fl) * Fxc_fl;
    // Fy_fl = w_mf_fl * Fy_fl + (1.0 - w_mf_fl) * Fyc_fl;
    //
    // Fx_fr = w_mf_fr * Fx_fr + (1.0 - w_mf_fr) * Fxc_fr;
    // Fy_fr = w_mf_fr * Fy_fr + (1.0 - w_mf_fr) * Fyc_fr;
    //
    // Fx_rl = w_mf_rl * Fx_rl + (1.0 - w_mf_rl) * Fxc_rl;
    // Fy_rl = w_mf_rl * Fy_rl + (1.0 - w_mf_rl) * Fyc_rl;
    //
    // Fx_rr = w_mf_rr * Fx_rr + (1.0 - w_mf_rr) * Fxc_rr;
    // Fy_rr = w_mf_rr * Fy_rr + (1.0 - w_mf_rr) * Fyc_rr;
    */

    // ============================================================
    // 17) Długości relaksacji dla MF
    // ============================================================

    const double Vx_rr = relaxation_speed(vx_rr);
    const double Vx_rl = relaxation_speed(vx_rl);
    const double Vx_fl = relaxation_speed(vx_fl);
    const double Vx_fr = relaxation_speed(vx_fr);

    double dyn_L_alpha_fl = Vx_fl * dy / cy + (1.0 / cy) * std::abs(dFy_fl_da);
    double dyn_L_alpha_fr = Vx_fr * dy / cy + (1.0 / cy) * std::abs(dFy_fr_da);
    double dyn_L_alpha_rl = Vx_rl * dy / cy + (1.0 / cy) * std::abs(dFy_rl_da);
    double dyn_L_alpha_rr = Vx_rr * dy / cy + (1.0 / cy) * std::abs(dFy_rr_da);

    double dyn_L_kappa_fl = Vx_fl * dx / cx + (1.0 / cx) * std::abs(dFx_fl_dk);
    double dyn_L_kappa_fr = Vx_fr * dx / cx + (1.0 / cx) * std::abs(dFx_fr_dk);
    double dyn_L_kappa_rl = Vx_rl * dx / cx + (1.0 / cx) * std::abs(dFx_rl_dk);
    double dyn_L_kappa_rr = Vx_rr * dx / cx + (1.0 / cx) * std::abs(dFx_rr_dk);

    // ============================================================
    // 18) Integracja relaksacji MF-only
    // ============================================================

    const double inv_dt = (dt > 1.0e-6) ? (1.0 / dt) : 0.0;

    auto calc_mf_relax_deriv =
        [&](double F_state,
            double Fmf,
            double vx_loc,
            double L,
            double L_min) -> double
    {
        if (dt <= 1.0e-9)
        {
            return 0.0;
        }

        const double V =
            relaxation_speed(vx_loc);

        const double Ls =
            std::max(L, L_min);

        const int n_substeps =
            std::max(1, TIRE_RELAX_SUBSTEPS);

        const double dt_sub =
            dt / static_cast<double>(n_substeps);

        double F =
            F_state;

        for (int i = 0; i < n_substeps; ++i)
        {
            const double z =
                std::min((V / Ls) * dt_sub, Z_MAX);

            const double one_minus_a =
                -std::expm1(-z);

            F +=
                (Fmf - F) * one_minus_a;
        }

        return (F - F_state) * inv_dt;
    };

    // ============================================================
    // 19) Pochodne stanów sił opony
    // ============================================================

    temp.fy_fl = calc_mf_relax_deriv(x.fy_fl, Fy_fl, vx_fl, dyn_L_alpha_fl, L_RELAX_MIN_ALPHA);
    temp.fy_fr = calc_mf_relax_deriv(x.fy_fr, Fy_fr, vx_fr, dyn_L_alpha_fr, L_RELAX_MIN_ALPHA);
    temp.fy_rl = calc_mf_relax_deriv(x.fy_rl, Fy_rl, vx_rl, dyn_L_alpha_rl, L_RELAX_MIN_ALPHA);
    temp.fy_rr = calc_mf_relax_deriv(x.fy_rr, Fy_rr, vx_rr, dyn_L_alpha_rr, L_RELAX_MIN_ALPHA);

    // 4WD / AWD:
    // każde koło ma aktywną siłę wzdłużną.
    temp.fx_fl = calc_mf_relax_deriv(x.fx_fl, Fx_fl, vx_fl, dyn_L_kappa_fl, L_RELAX_MIN_KAPPA);
    temp.fx_fr = calc_mf_relax_deriv(x.fx_fr, Fx_fr, vx_fr, dyn_L_kappa_fr, L_RELAX_MIN_KAPPA);
    temp.fx_rl = calc_mf_relax_deriv(x.fx_rl, Fx_rl, vx_rl, dyn_L_kappa_rl, L_RELAX_MIN_KAPPA);
    temp.fx_rr = calc_mf_relax_deriv(x.fx_rr, Fx_rr, vx_rr, dyn_L_kappa_rr, L_RELAX_MIN_KAPPA);

    return temp;
}

} // namespace lem_dynamics_sim_
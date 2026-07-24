#pragma once

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace lem_dynamics_sim_
{

struct ParamBank
{
  std::vector<std::string> names;
  std::unordered_map<std::string, int> idx;
  std::vector<double> values;

  int add(const std::string& name, double val)
  {
    auto it = idx.find(name);

    if (it != idx.end()) {
      values[it->second] = val;
      return it->second;
    }

    const int k = static_cast<int>(names.size());
    names.push_back(name);
    idx[name] = k;
    values.push_back(val);
    return k;
  }

  int i(const std::string& name) const
  {
    auto it = idx.find(name);
    if (it == idx.end()) {
      throw std::runtime_error("ParamBank: missing key '" + name + "'");
    }
    return it->second;
  }

  double get(const std::string& name) const
  {
    return values.at(i(name));
  }

  void set(const std::string& name, double v)
  {
    values.at(i(name)) = v;
  }

  size_t size() const
  {
    return values.size();
  }
};

inline double jsonToDouble(const nlohmann::json& v, const std::string& path)
{
  if (v.is_boolean()) {
    return v.get<bool>() ? 1.0 : 0.0;
  }

  if (!v.is_number()) {
    throw std::runtime_error("JSON: key '" + path + "' is not numeric/bool");
  }

  return v.get<double>();
}

inline double JgetReq(const nlohmann::json& J, const std::string& path)
{
  const auto pos = path.find('.');

  if (pos == std::string::npos) {
    if (!J.contains(path)) {
      throw std::runtime_error("JSON: missing required key '" + path + "'");
    }
    return jsonToDouble(J.at(path), path);
  }

  const std::string head = path.substr(0, pos);
  const std::string tail = path.substr(pos + 1);

  if (!J.contains(head)) {
    throw std::runtime_error("JSON: missing object '" + head + "'");
  }

  return JgetReq(J.at(head), tail);
}

inline double JgetReqSafe(const nlohmann::json& J, const std::string& path)
{
  try {
    return JgetReq(J, path);
  }
  catch (const std::exception& e) {
    std::cerr << "\n[JSON ERROR] while reading path: "
              << path
              << "\n  what(): "
              << e.what()
              << "\n"
              << std::endl;
    throw;
  }
}

inline double JgetOpt(const nlohmann::json& J,
                      const std::string& path,
                      double def)
{
  try {
    return JgetReq(J, path);
  }
  catch (...) {
    return def;
  }
}

namespace
{
constexpr double kPi = 3.14159265358979323846;

inline void requirePositive(double value, const std::string& name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error("ParamBank: " + name + " must be positive");
  }
}

inline void requireNonNegative(double value, const std::string& name)
{
  if (!std::isfinite(value) || value < 0.0) {
    throw std::runtime_error("ParamBank: " + name + " must be non-negative");
  }
}

inline void requireInRange01(double value, const std::string& name)
{
  if (!std::isfinite(value) || value < 0.0 || value > 1.0) {
    throw std::runtime_error("ParamBank: " + name + " must be in [0, 1]");
  }
}
} // namespace

inline ParamBank build_param_bank(const nlohmann::json& J)
{
  ParamBank P;

  // --------------------------------------------------------------------------
  // Total vehicle mass, static distribution and unambiguous CG distances.
  // l_f = CG -> front axle, l_r = CG -> rear axle.
  // --------------------------------------------------------------------------

  P.add("g", JgetReqSafe(J, "vehicle.g"));
  P.add("m", JgetReqSafe(J, "vehicle.m"));
  P.add("m_rear", JgetReqSafe(J, "vehicle.m_rear"));

  const double g = P.get("g");
  const double m = P.get("m");
  const double m_rear = P.get("m_rear");
  const double m_front = m - m_rear;

  requirePositive(g, "vehicle.g");
  requirePositive(m, "vehicle.m");

  if (m_rear <= 0.0 || m_rear >= m) {
    throw std::runtime_error(
        "ParamBank: vehicle.m_rear must be in range (0, vehicle.m)"
    );
  }

  P.add("m_front", m_front);

  P.add("h", JgetReqSafe(J, "vehicle.h"));
  P.add("w", JgetReqSafe(J, "vehicle.w"));
  P.add("wheelbase", P.get("w"));

  const double h_CG = P.get("h");
  const double L = P.get("wheelbase");

  requirePositive(h_CG, "vehicle.h");
  requirePositive(L, "vehicle.w / wheelbase");

  const double lambda_rear_static = m_rear / m;
  const double lambda_front_static = m_front / m;

  P.add("lambda_front_static", lambda_front_static);
  P.add("lambda_rear_static", lambda_rear_static);

  const double l_f = lambda_rear_static * L;
  const double l_r = lambda_front_static * L;

  P.add("l_f", l_f);
  P.add("l_r", l_r);

  P.add("t_front", JgetReqSafe(J, "vehicle.T_front"));
  P.add("t_rear", JgetReqSafe(J, "vehicle.T_rear"));

  const double t_front = P.get("t_front");
  const double t_rear = P.get("t_rear");

  requirePositive(t_front, "vehicle.T_front");
  requirePositive(t_rear, "vehicle.T_rear");

  // --------------------------------------------------------------------------
  // Unsprung masses and sprung-mass CG reconstructed from the total CG.
  // --------------------------------------------------------------------------

  P.add("m_unsprung_front_per_wheel",
        JgetReqSafe(J, "vehicle.unsprung_mass_front_per_wheel_kg"));
  P.add("m_unsprung_rear_per_wheel",
        JgetReqSafe(J, "vehicle.unsprung_mass_rear_per_wheel_kg"));
  P.add("h_unsprung_front",
        JgetReqSafe(J, "vehicle.unsprung_cg_height_front_m"));
  P.add("h_unsprung_rear",
        JgetReqSafe(J, "vehicle.unsprung_cg_height_rear_m"));

  requireNonNegative(P.get("m_unsprung_front_per_wheel"),
                     "vehicle.unsprung_mass_front_per_wheel_kg");
  requireNonNegative(P.get("m_unsprung_rear_per_wheel"),
                     "vehicle.unsprung_mass_rear_per_wheel_kg");
  requireNonNegative(P.get("h_unsprung_front"),
                     "vehicle.unsprung_cg_height_front_m");
  requireNonNegative(P.get("h_unsprung_rear"),
                     "vehicle.unsprung_cg_height_rear_m");

  const double m_unsprung_front =
      2.0 * P.get("m_unsprung_front_per_wheel");
  const double m_unsprung_rear =
      2.0 * P.get("m_unsprung_rear_per_wheel");
  const double m_unsprung = m_unsprung_front + m_unsprung_rear;
  const double m_sprung = m - m_unsprung;

  const double m_sprung_front = m_front - m_unsprung_front;
  const double m_sprung_rear = m_rear - m_unsprung_rear;

  if (m_sprung <= 0.0 || m_sprung_front <= 0.0 || m_sprung_rear <= 0.0) {
    throw std::runtime_error(
        "ParamBank: unsprung masses leave a non-positive sprung mass"
    );
  }

  const double h_sprung =
      (
          m * h_CG
          - m_unsprung_front * P.get("h_unsprung_front")
          - m_unsprung_rear * P.get("h_unsprung_rear")
      ) / m_sprung;

  if (!std::isfinite(h_sprung) || h_sprung <= 0.0) {
    throw std::runtime_error(
        "ParamBank: derived sprung-mass CG height must be positive"
    );
  }

  const double l_f_sprung = (m_sprung_rear / m_sprung) * L;
  const double l_r_sprung = (m_sprung_front / m_sprung) * L;

  P.add("m_unsprung_front", m_unsprung_front);
  P.add("m_unsprung_rear", m_unsprung_rear);
  P.add("m_unsprung", m_unsprung);
  P.add("m_sprung", m_sprung);
  P.add("m_sprung_front", m_sprung_front);
  P.add("m_sprung_rear", m_sprung_rear);
  P.add("h_sprung", h_sprung);
  P.add("l_f_sprung", l_f_sprung);
  P.add("l_r_sprung", l_r_sprung);

  // --------------------------------------------------------------------------
  // Roll geometry: MATLAB RTE values, applied to sprung lateral forces.
  // --------------------------------------------------------------------------

  P.add("h_rc_front",
        JgetReqSafe(J, "vehicle.roll_center_front_m"));
  P.add("h_rc_rear",
        JgetReqSafe(J, "vehicle.roll_center_rear_m"));

  const double h_RC_f = P.get("h_rc_front");
  const double h_RC_r = P.get("h_rc_rear");

  requireNonNegative(h_RC_f, "vehicle.roll_center_front_m");
  requireNonNegative(h_RC_r, "vehicle.roll_center_rear_m");

  const double h_roll_axis_sprung_cg =
      h_RC_f + (l_f_sprung / L) * (h_RC_r - h_RC_f);

  P.add("h_roll_axis_sprung_cg", h_roll_axis_sprung_cg);
  P.add("h_elastic_roll_arm_sprung", h_sprung - h_roll_axis_sprung_cg);

  // Compatibility aliases for code/logging that still uses the old names.
  P.add("h1_roll", h_RC_f);
  P.add("h2_roll", h_RC_r);
  P.add("h_roll_axis_cg", h_roll_axis_sprung_cg);
  P.add("h_elastic_roll_arm", h_sprung - h_roll_axis_sprung_cg);

  // --------------------------------------------------------------------------
  // Pitch geometry from MATLAB pitch-center coordinates.
  // PC_x is measured rearwards from the front axle.
  // The gains map sprung longitudinal axle force [N] directly to geometric
  // front-to-rear load transfer [N]. Both gains are positive for a pitch center
  // above ground and between the axles.
  // --------------------------------------------------------------------------

  P.add("pitch_center_x_from_front",
        JgetReqSafe(J, "vehicle.pitch_center_x_from_front_m"));
  P.add("pitch_center_height",
        JgetReqSafe(J, "vehicle.pitch_center_height_m"));

  const double pc_x = P.get("pitch_center_x_from_front");
  const double pc_z = P.get("pitch_center_height");

  if (!std::isfinite(pc_x) || pc_x <= 0.0 || pc_x >= L) {
    throw std::runtime_error(
        "ParamBank: pitch center x must lie strictly between the axles"
    );
  }
  requireNonNegative(pc_z, "vehicle.pitch_center_height_m");

  P.add("pitch_geo_gain_front", pc_z / pc_x);
  P.add("pitch_geo_gain_rear", pc_z / (L - pc_x));

  // --------------------------------------------------------------------------
  // RTE 3.5 conventional suspension.
  //
  // Corner spring convention:
  //
  //     MR_spring = wheel travel / spring travel
  //     k_wheel   = k_spring / MR_spring^2
  //
  // ARB convention used by the supplied MATLAB parameters:
  //
  //     MR_ARB = left-right wheel-travel difference / ARB spring travel
  //
  // The JSON stores the physical linear ARB spring stiffness [N/m] and its
  // motion ratio. The equivalent axle roll stiffness is derived once here:
  //
  //     K_phi_ARB = k_ARB * track^2 / MR_ARB^2
  // --------------------------------------------------------------------------

  P.add("spring_rate_front_N_per_m",
        JgetReqSafe(J, "vehicle.spring_rate_front_N_per_m"));
  P.add("spring_rate_rear_N_per_m",
        JgetReqSafe(J, "vehicle.spring_rate_rear_N_per_m"));

  P.add("spring_motion_ratio_front",
        JgetReqSafe(
            J,
            "vehicle.spring_motion_ratio_front_wheel_over_spring"
        ));
  P.add("spring_motion_ratio_rear",
        JgetReqSafe(
            J,
            "vehicle.spring_motion_ratio_rear_wheel_over_spring"
        ));

  P.add("arb_stiffness_front_N_per_m",
        JgetReqSafe(J, "vehicle.arb_stiffness_front_N_per_m"));
  P.add("arb_stiffness_rear_N_per_m",
        JgetReqSafe(J, "vehicle.arb_stiffness_rear_N_per_m"));

  P.add("arb_motion_ratio_front",
        JgetReqSafe(
            J,
            "vehicle.arb_motion_ratio_front_wheel_difference_over_arb"
        ));
  P.add("arb_motion_ratio_rear",
        JgetReqSafe(
            J,
            "vehicle.arb_motion_ratio_rear_wheel_difference_over_arb"
        ));

  const double spring_rate_front = P.get("spring_rate_front_N_per_m");
  const double spring_rate_rear = P.get("spring_rate_rear_N_per_m");
  const double spring_mr_front = P.get("spring_motion_ratio_front");
  const double spring_mr_rear = P.get("spring_motion_ratio_rear");

  const double arb_stiffness_front =
      P.get("arb_stiffness_front_N_per_m");
  const double arb_stiffness_rear =
      P.get("arb_stiffness_rear_N_per_m");
  const double arb_mr_front = P.get("arb_motion_ratio_front");
  const double arb_mr_rear = P.get("arb_motion_ratio_rear");

  requirePositive(
      spring_rate_front,
      "vehicle.spring_rate_front_N_per_m"
  );
  requirePositive(
      spring_rate_rear,
      "vehicle.spring_rate_rear_N_per_m"
  );
  requirePositive(
      spring_mr_front,
      "vehicle.spring_motion_ratio_front_wheel_over_spring"
  );
  requirePositive(
      spring_mr_rear,
      "vehicle.spring_motion_ratio_rear_wheel_over_spring"
  );
  requireNonNegative(
      arb_stiffness_front,
      "vehicle.arb_stiffness_front_N_per_m"
  );
  requireNonNegative(
      arb_stiffness_rear,
      "vehicle.arb_stiffness_rear_N_per_m"
  );
  requirePositive(
      arb_mr_front,
      "vehicle.arb_motion_ratio_front_wheel_difference_over_arb"
  );
  requirePositive(
      arb_mr_rear,
      "vehicle.arb_motion_ratio_rear_wheel_difference_over_arb"
  );

  const double wheel_rate_front_N_per_m =
      spring_rate_front / (spring_mr_front * spring_mr_front);

  const double wheel_rate_rear_N_per_m =
      spring_rate_rear / (spring_mr_rear * spring_mr_rear);

  P.add("wheel_rate_front_N_per_m", wheel_rate_front_N_per_m);
  P.add("wheel_rate_rear_N_per_m", wheel_rate_rear_N_per_m);
  P.add("wheel_rate_front_N_per_mm", wheel_rate_front_N_per_m / 1000.0);
  P.add("wheel_rate_rear_N_per_mm", wheel_rate_rear_N_per_m / 1000.0);

  const double K_phi_springs_front =
      wheel_rate_front_N_per_m * t_front * t_front / 2.0;

  const double K_phi_springs_rear =
      wheel_rate_rear_N_per_m * t_rear * t_rear / 2.0;

  const double K_phi_arb_front =
      arb_stiffness_front * t_front * t_front
      / (arb_mr_front * arb_mr_front);

  const double K_phi_arb_rear =
      arb_stiffness_rear * t_rear * t_rear
      / (arb_mr_rear * arb_mr_rear);

  const double K_phi_front =
      K_phi_springs_front + K_phi_arb_front;

  const double K_phi_rear =
      K_phi_springs_rear + K_phi_arb_rear;

  const double K_phi_total =
      K_phi_front + K_phi_rear;

  requirePositive(K_phi_front, "derived K_phi_front");
  requirePositive(K_phi_rear, "derived K_phi_rear");
  requirePositive(K_phi_total, "derived K_phi_total");

  // Front share of the elastic roll moment.
  const double lambda_phi_elastic_lateral =
      K_phi_front / K_phi_total;

  // Pitch does not twist an ideal ARB because the left and right wheels of
  // each axle move in phase. Therefore K_theta contains corner springs only.
  const double K_theta =
      2.0 * wheel_rate_front_N_per_m * l_f_sprung * l_f_sprung
      + 2.0 * wheel_rate_rear_N_per_m * l_r_sprung * l_r_sprung;

  requirePositive(K_theta, "derived K_theta");

  P.add("K_phi_springs_front", K_phi_springs_front);
  P.add("K_phi_springs_rear", K_phi_springs_rear);
  P.add("K_phi_arb_front", K_phi_arb_front);
  P.add("K_phi_arb_rear", K_phi_arb_rear);
  P.add("K_phi_front", K_phi_front);
  P.add("K_phi_rear", K_phi_rear);
  P.add("K_phi_total", K_phi_total);
  P.add("lambda_phi_elastic_lateral", lambda_phi_elastic_lateral);
  P.add("K_theta", K_theta);

  // --------------------------------------------------------------------------
  // Inertias are final, pre-scaled values stored directly in JSON.
  // No mass-based inertia scaling is performed at runtime.
  // --------------------------------------------------------------------------

  P.add("Iphi", JgetReqSafe(J, "vehicle.Iphi"));
  P.add("Itheta", JgetReqSafe(J, "vehicle.Itheta"));
  P.add("Iz", JgetReqSafe(J, "vehicle.Iz"));

  const double I_phi = P.get("Iphi");
  const double I_theta = P.get("Itheta");

  requirePositive(I_phi, "vehicle.Iphi");
  requirePositive(I_theta, "vehicle.Itheta");
  requirePositive(P.get("Iz"), "vehicle.Iz");

  const double omega_phi =
      std::sqrt(K_phi_total / I_phi);

  const double omega_theta =
      std::sqrt(K_theta / I_theta);

  const double f_phi =
      omega_phi / (2.0 * kPi);

  const double f_theta =
      omega_theta / (2.0 * kPi);

  P.add("omega_phi", omega_phi);
  P.add("omega_theta", omega_theta);
  P.add("f_phi", f_phi);
  P.add("f_theta", f_theta);
  P.add("roll_natural_frequency_hz", f_phi);
  P.add("pitch_natural_frequency_hz", f_theta);

  P.add("damper_low_speed_damping_ratio",
        JgetReqSafe(J, "vehicle.damper_low_speed_damping_ratio"));

  const double zeta_low =
      P.get("damper_low_speed_damping_ratio");

  requirePositive(
      zeta_low,
      "vehicle.damper_low_speed_damping_ratio"
  );

  P.add("zeta_phi_for_load_transfer", zeta_low);
  P.add("zeta_theta_for_load_transfer", zeta_low);

  // Compatibility/debug values only. The suspension model integrates the
  // full second-order equations using omega and zeta.
  P.add("tau_lat_elastic", 1.0 / (zeta_low * omega_phi));
  P.add("tau_long_elastic", 1.0 / (zeta_low * omega_theta));

  P.add("angle_construction_front", std::atan((0.5 * t_front) / l_f));
  P.add("angle_construction_rear", std::atan((0.5 * t_rear) / l_r));

  P.add("K1", JgetOpt(J, "vehicle.K1", 0.0));
  P.add("K2", JgetOpt(J, "vehicle.K2", 0.0));

  P.add("Cd", JgetReqSafe(J, "vehicle.Cd"));
  P.add("Cl1", JgetReqSafe(J, "vehicle.Cl1"));
  P.add("Cl2", JgetReqSafe(J, "vehicle.Cl2"));

  const double rolling_resistance_coeff =
      JgetOpt(J, "vehicle.rolling_resistance_coeff",
             JgetReqSafe(J, "vehicle.Cr"));

  P.add("Cr", rolling_resistance_coeff);
  P.add("rolling_resistance_coeff", rolling_resistance_coeff);
  P.add("resistance_constant_N",
        JgetOpt(J, "vehicle.resistance_constant_N", 0.0));
  P.add("resistance_linear_N_per_mps",
        JgetOpt(J, "vehicle.resistance_linear_N_per_mps", 0.0));

  P.add("aero_package_enabled",
        JgetOpt(J, "vehicle.aero_package_enabled", 0.0));

  P.add("R", JgetReqSafe(J, "vehicle.R"));
  P.add("I_tire", JgetReqSafe(J, "vehicle.I_tire"));
  P.add("r_front", JgetReqSafe(J, "vehicle.r_front"));
  P.add("r_rear", JgetReqSafe(J, "vehicle.r_rear"));

  requirePositive(P.get("R"), "vehicle.R");
  requirePositive(P.get("I_tire"), "vehicle.I_tire");

  P.add("pCx1", JgetReqSafe(J, "tire.pCx1"));
  P.add("pDx1", JgetReqSafe(J, "tire.pDx1"));
  P.add("pDx2", JgetReqSafe(J, "tire.pDx2"));
  P.add("pEx1", JgetReqSafe(J, "tire.pEx1"));
  P.add("pEx2", JgetReqSafe(J, "tire.pEx2"));
  P.add("pEx3", JgetReqSafe(J, "tire.pEx3"));
  P.add("pEx4", JgetReqSafe(J, "tire.pEx4"));
  P.add("pKx1", JgetReqSafe(J, "tire.pKx1"));
  P.add("pKx2", JgetReqSafe(J, "tire.pKx2"));
  P.add("pKx3", JgetReqSafe(J, "tire.pKx3"));
  P.add("pHx1", JgetReqSafe(J, "tire.pHx1"));
  P.add("pHx2", JgetReqSafe(J, "tire.pHx2"));
  P.add("pVx1", JgetReqSafe(J, "tire.pVx1"));
  P.add("pVx2", JgetReqSafe(J, "tire.pVx2"));
  P.add("lambda_x", JgetReqSafe(J, "tire.lambda_x"));

  P.add("pCy1", JgetReqSafe(J, "tire.pCy1"));
  P.add("pDy1", JgetReqSafe(J, "tire.pDy1"));
  P.add("pDy2", JgetReqSafe(J, "tire.pDy2"));
  P.add("pEy1", JgetReqSafe(J, "tire.pEy1"));
  P.add("pEy2", JgetReqSafe(J, "tire.pEy2"));
  P.add("pEy3", JgetReqSafe(J, "tire.pEy3"));
  P.add("pKy1", JgetReqSafe(J, "tire.pKy1"));
  P.add("pKy2", JgetReqSafe(J, "tire.pKy2"));
  P.add("pKy4", JgetReqSafe(J, "tire.pKy4"));
  P.add("pHy1", JgetReqSafe(J, "tire.pHy1"));
  P.add("pHy2", JgetReqSafe(J, "tire.pHy2"));
  P.add("pVy1", JgetReqSafe(J, "tire.pVy1"));
  P.add("pVy2", JgetReqSafe(J, "tire.pVy2"));
  P.add("lambda_y", JgetReqSafe(J, "tire.lambda_y"));

  P.add("N0", JgetReqSafe(J, "tire.N0"));
  P.add("epsilon", JgetReqSafe(J, "tire.epsilon"));

  P.add("relax_time_slip_angle_first_guees", JgetReqSafe(J, "tire.relax_length_slip_angle_piorek"));
  P.add("relax_time_slip_ratio_first_guees", 0.4 * P.get("relax_time_slip_angle_first_guees"));

  P.add("cy", JgetReqSafe(J, "tire.cy_first_guees"));
  P.add("dy", 0.3 * P.get("cy") * P.get("relax_time_slip_angle_first_guees"));

  P.add("cx", JgetReqSafe(J, "tire.cx_first_guees"));
  P.add("dx", 0.3 * P.get("cx") * P.get("relax_time_slip_ratio_first_guees"));

  P.add("P_max_drive", JgetReqSafe(J, "drivetrain.P_max_drive"));
  P.add("P_min_recup", JgetReqSafe(J, "drivetrain.P_min_recup"));
  P.add("drivetrain_timescale", JgetReqSafe(J, "drivetrain.drivetrain_timescale"));
  P.add("max_torque", JgetReqSafe(J, "drivetrain.max_torque"));
  P.add("min_torque", JgetReqSafe(J, "drivetrain.min_torque"));
  P.add("gear_ratio", JgetOpt(J, "drivetrain.gear_ratio", 1.0));

  P.add("natural_frequency_steering_system", JgetReqSafe(J, "steering_system.natural_frequency_steering_system"));
  P.add("steering_system_damping", JgetReqSafe(J, "steering_system.steering_system_damping"));
  P.add("max_steer", JgetReqSafe(J, "steering_system.max_steer"));
  P.add("min_steer", JgetReqSafe(J, "steering_system.min_steer"));
  P.add("max_steering_angle_rate", JgetReqSafe(J, "steering_system.max_steering_angle_rate"));
  P.add("min_steering_angle_rate", JgetReqSafe(J, "steering_system.min_steering_angle_rate"));

  P.add("simulation_time_step", JgetReqSafe(J, "simulation.time_step"));
  requirePositive(P.get("simulation_time_step"), "simulation.time_step");

  P.add("roll_inclination_of_world", JgetReqSafe(
      J, "simulation.roll_inclination_of_world_rad"));
  P.add("pitch_inclination_of_world", JgetReqSafe(
      J, "simulation.pitch_inclination_of_world_rad"));

  P.add("torque_vectoring_p_gain", JgetReqSafe(
      J, "torque_allocation_and_vectoring.p_gain"));
  P.add("speed_blend_low", JgetReqSafe(
      J, "torque_allocation_and_vectoring.speed_blend_low_mps"));
  P.add("speed_blend_high", JgetReqSafe(
      J, "torque_allocation_and_vectoring.speed_blend_high_mps"));
  P.add("turn_radius_speed_gain", JgetReqSafe(
      J, "torque_allocation_and_vectoring.turn_radius_speed_gain"));
  P.add("oversteer_gain", JgetReqSafe(
      J, "torque_allocation_and_vectoring.oversteer_gain"));
  P.add("transition_gain", JgetReqSafe(
      J, "torque_allocation_and_vectoring.transition_gain"));
  P.add("low_speed_gain", JgetReqSafe(
      J, "torque_allocation_and_vectoring.low_speed_gain"));
  P.add("torque_vectoring_scale_nm", JgetReqSafe(
      J, "torque_allocation_and_vectoring.vectoring_scale_motor_nm"));
  P.add("torque_vectoring_max_motor_delta_nm", JgetReqSafe(
      J, "torque_allocation_and_vectoring.max_motor_delta_nm"));
  P.add("front_back_split_drive", JgetReqSafe(
      J, "torque_allocation_and_vectoring.front_fraction_drive"));
  P.add("front_back_split_brake", JgetReqSafe(
      J, "torque_allocation_and_vectoring.front_fraction_brake"));

  requireNonNegative(
      P.get("low_speed_gain"),
      "torque_allocation_and_vectoring.low_speed_gain"
  );

  P.add("control_to_dv_board_read_time_step", JgetReqSafe(J, "dv_board.control_input_read_period_s"));
  P.add("dv_board_to_main_time_step", JgetReqSafe(J, "dv_board.dv_board_to_main_time_step"));
  P.add("wheel_encoder_reading_time_step", JgetReqSafe(J, "dv_board.wheel_encoder_reading_time_step"));
  P.add("dv_board_data_publishing_time_step", JgetReqSafe(J, "dv_board.dv_board_data_publishing_time_step"));
  P.add("steering_command_read_time_step", JgetReqSafe(J, "dv_board.steering_command_read_period_s"));
  P.add("steer_publishing_time_step", JgetReqSafe(J, "dv_board.steer_publishing_time_step"));

  P.add("main_loop_time_step", JgetReqSafe(J, "main.main_loop_time_step"));
  P.add("main_computation_delay_s", JgetReqSafe(J, "main.main_computation_delay_s"));
  P.add("main_imu_read_time_step", JgetReqSafe(J, "main.main_imu_read_time_step"));

  P.add("state_estimator_position_noise_std", JgetReqSafe(
      J, "gaussian_state_estimator.position_noise_std_m"));
  P.add("state_estimator_yaw_noise_std", JgetReqSafe(
      J, "gaussian_state_estimator.yaw_noise_std_rad"));
  P.add("state_estimator_yaw_rate_noise_std", JgetReqSafe(
      J, "gaussian_state_estimator.yaw_rate_noise_std_rad_per_s"));
  P.add("state_estimator_speed_noise_std", JgetReqSafe(
      J, "gaussian_state_estimator.speed_noise_std_mps"));
  P.add("state_estimator_frequency_hz", JgetReqSafe(
      J, "gaussian_state_estimator.frequency_hz"));
  P.add("calibration_time", JgetReqSafe(
      J, "gaussian_state_estimator.calibration_time_s"));
  P.add("state_estimator_pose_yaw_delay_s", JgetReqSafe(
      J, "gaussian_state_estimator.pose_yaw_delay_s"));
  P.add("state_estimator_speed_delay_s", JgetReqSafe(
      J, "gaussian_state_estimator.speed_delay_s"));
  P.add("state_estimator_position_bias_rw", JgetReqSafe(
      J, "gaussian_state_estimator.position_bias_rw_std_m_sqrt_s"));
  P.add("state_estimator_yaw_bias_rw", JgetReqSafe(
      J, "gaussian_state_estimator.yaw_bias_rw_std_rad_sqrt_s"));
  P.add("state_estimator_speed_bias_rw", JgetReqSafe(
      J, "gaussian_state_estimator.speed_bias_rw_std_mps_sqrt_s"));
  P.add("state_estimator_yaw_rate_bias_rw", JgetReqSafe(
      J, "gaussian_state_estimator.yaw_rate_bias_rw_std_rad_per_s_sqrt_s"));

  P.add("imu_accelerometer_rate_hz", JgetReqSafe(J, "imu.accelerometer_rate_hz"));
  P.add("imu_gyroscope_rate_hz", JgetReqSafe(J, "imu.gyroscope_rate_hz"));
  P.add("imu_buffer_sample_rate_hz", JgetReqSafe(J, "imu.buffer_sample_rate_hz"));
  P.add("imu_samples_per_output", JgetReqSafe(J, "imu.samples_per_output"));
  P.add("imu_accelerometer_bandwidth_hz", JgetReqSafe(J, "imu.accelerometer_bandwidth_hz"));
  P.add("imu_gyroscope_bandwidth_hz", JgetReqSafe(J, "imu.gyroscope_bandwidth_hz"));
  P.add("imu_gyroscope_noise_std", JgetReqSafe(J, "imu.gyroscope_noise_std_rad_per_s"));
  P.add("imu_accelerometer_noise_std", JgetReqSafe(J, "imu.accelerometer_noise_std_mps2"));
  P.add("imu_gyroscope_bias_rw_std", JgetReqSafe(J, "imu.gyroscope_bias_rw_std_rad_per_s_sqrt_s"));
  P.add("imu_accelerometer_bias_rw_std", JgetReqSafe(J, "imu.accelerometer_bias_rw_std_mps2_sqrt_s"));

  P.add("steering_actuator_noise_std_rad", JgetReqSafe(
      J, "steering_system.steering_noise_actuator_std_deg") *
      kPi / 180.0);
  P.add("steering_bias_rad", JgetReqSafe(
      J, "steering_system.steering_bias_deg") * kPi / 180.0);
  P.add("steering_encoder_noise_std_rad", JgetReqSafe(
      J, "steering_system.steering_noise_encoder_std_deg") *
      kPi / 180.0);

  P.add("camera_horizontal_fov_rad", JgetReqSafe(
      J, "camera.horizontal_fov_rad"));
  P.add("camera_vertical_fov_rad", JgetReqSafe(
      J, "camera.vertical_fov_rad"));
  P.add("z_camera_to_cog", JgetReqSafe(J, "camera.z_camera_to_cog"));
  P.add("x_camera_to_cog", JgetReqSafe(J, "camera.x_camera_to_cog"));
  P.add("y_camera_to_cog", JgetReqSafe(J, "camera.y_camera_to_cog"));
  P.add("cone_height", JgetReqSafe(J, "camera.cone_height"));
  P.add("frames_per_second", JgetReqSafe(J, "camera.frames_per_second"));
  P.add("mean_time_of_vision_execuction", JgetReqSafe(J, "camera.mean_time_of_vision_execuction"));
  P.add("var_of_vision_time_execution", JgetReqSafe(J, "camera.var_of_vision_time_execution"));
  P.add("camera_range", JgetReqSafe(J, "camera.camera_range"));
  P.add("vision_noise_a", JgetReqSafe(
      J, "camera.position_noise_scale_m"));
  P.add("vision_noise_b", JgetReqSafe(
      J, "camera.position_noise_distance_gain_per_m"));

  P.add("lidar_max_range_m", JgetReqSafe(J, "lidar.max_range_m"));
  P.add("lidar_columns_per_scan", JgetReqSafe(J, "lidar.columns_per_scan"));
  P.add("lidar_rotation_rate_hz", JgetReqSafe(J, "lidar.rotation_rate_hz"));
  P.add("lidar_azimuth_window_rad", JgetReqSafe(
      J, "lidar.azimuth_window_rad"));
  P.add("lidar_range_noise_sigma_m", JgetReqSafe(J, "lidar.range_noise_sigma_m"));
  P.add("lidar_azimuth_noise_base_rad", JgetReqSafe(
      J, "lidar.azimuth_noise_base_rad"));
  P.add("z_lidar_to_cog", JgetReqSafe(J, "lidar.z_lidar_to_cog"));
  P.add("x_lidar_to_cog", JgetReqSafe(J, "lidar.x_lidar_to_cog"));
  P.add("y_lidar_to_cog", JgetReqSafe(J, "lidar.y_lidar_to_cog"));
  P.add("lidar_pitch_rad", JgetReqSafe(J, "lidar.pitch_rad"));
  P.add("lidar_use_motion_distortion", JgetReqSafe(J, "lidar.use_motion_distortion"));

  const double lidar_delta_azimuth_rad =
      2.0 * kPi / P.get("lidar_columns_per_scan");
  P.add("lidar_delta_azimuth_rad", lidar_delta_azimuth_rad);

  const double lidar_scan_period_s = 1.0 / P.get("lidar_rotation_rate_hz");
  P.add("lidar_scan_period_s", lidar_scan_period_s);

  const double lidar_roi_period_s =
      (P.get("lidar_azimuth_window_rad") / (2.0 * kPi)) *
      lidar_scan_period_s;

  P.add("lidar_roi_period_s", lidar_roi_period_s);
  P.add("lidar_azimuth_quantization_sigma_rad", P.get("lidar_delta_azimuth_rad") / std::sqrt(12.0));

  P.add("lidar_execution_time_mean", JgetReqSafe(J, "lidar.mean_time_of_lidar_execution"));
  P.add("lidar_execution_time_var", JgetReqSafe(J, "lidar.var_of_lidar_time_execution"));
  P.add("lidar_frames_per_second", JgetReqSafe(J, "lidar.frames_per_second"));

  P.add("perception_dropout_enabled", JgetReqSafe(
      J, "perception_errors.dropout_enabled"));
  P.add("perception_dropout_probability", JgetReqSafe(
      J, "perception_errors.cone_dropout_probability"));
  P.add("perception_false_positives_enabled", JgetReqSafe(
      J, "perception_errors.false_positives_enabled"));
  P.add("perception_false_positive_mean_count", JgetReqSafe(
      J, "perception_errors.false_positive_mean_count"));
  P.add("perception_false_positive_min_range_m", JgetReqSafe(
      J, "perception_errors.false_positive_min_range_m"));
  P.add("perception_false_positive_max_range_m", JgetReqSafe(
      J, "perception_errors.false_positive_max_range_m"));
  P.add("perception_false_positive_lateral_fraction", JgetReqSafe(
      J, "perception_errors.false_positive_lateral_fraction"));

  requireInRange01(
      P.get("perception_dropout_probability"),
      "perception_errors.cone_dropout_probability");
  requireNonNegative(
      P.get("perception_false_positive_mean_count"),
      "perception_errors.false_positive_mean_count");
  requireNonNegative(
      P.get("perception_false_positive_min_range_m"),
      "perception_errors.false_positive_min_range_m");
  requireNonNegative(
      P.get("perception_false_positive_max_range_m"),
      "perception_errors.false_positive_max_range_m");
  requireNonNegative(
      P.get("perception_false_positive_lateral_fraction"),
      "perception_errors.false_positive_lateral_fraction");
  if (P.get("perception_false_positive_max_range_m") <
      P.get("perception_false_positive_min_range_m")) {
    throw std::runtime_error(
        "ParamBank: perception false-positive max range must be "
        "greater than or equal to min range");
  }

  P.add("metrics_sideslip_threshold_rad", JgetReqSafe(
      J, "metrics.sideslip_threshold_rad"));
  P.add("metrics_minimum_speed_mps_for_sideslip", JgetReqSafe(
      J, "metrics.minimum_speed_mps_for_sideslip"));
  requireNonNegative(
      P.get("metrics_sideslip_threshold_rad"),
      "metrics.sideslip_threshold_rad");
  requireNonNegative(
      P.get("metrics_minimum_speed_mps_for_sideslip"),
      "metrics.minimum_speed_mps_for_sideslip");

  return P;
}

} // namespace lem_dynamics_sim_

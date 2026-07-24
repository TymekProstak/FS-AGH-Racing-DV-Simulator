#pragma once

#include "ParamBank.hpp"
#include "dv_interfaces/Imu.h"
#include "runtime_helpers.hpp"
#include "utilities.hpp"

#include <optional>
#include <random>

namespace lem_dynamics_sim_
{

class ImuSimulator
{
public:
    void configure(const ParamBank& parameters,
                   double simulation_step_s,
                   std::mt19937& phase_rng);

    std::optional<dv_interfaces::Imu> update(
        int simulation_step,
        const State& state,
        const ParamBank& parameters);

private:
    void read_accelerometer(const State& state,
                            const ParamBank& parameters);
    void read_gyroscope(const State& state,
                        const ParamBank& parameters);
    std::optional<dv_interfaces::Imu> sample_output();
    double standard_normal_noise();

    PeriodicTimer accelerometer_timer_;
    PeriodicTimer gyroscope_timer_;
    PeriodicTimer output_sample_timer_;

    double accelerometer_period_s_ = 0.0;
    double gyroscope_period_s_ = 0.0;
    int samples_per_output_ = 1;

    double accelerometer_bias_x_ = 0.0;
    double accelerometer_bias_y_ = 0.0;
    double accelerometer_bias_z_ = 0.0;
    double gyroscope_bias_x_ = 0.0;
    double gyroscope_bias_y_ = 0.0;
    double gyroscope_bias_z_ = 0.0;

    double accelerometer_x_ = 0.0;
    double accelerometer_y_ = 0.0;
    double accelerometer_z_ = 0.0;
    double gyroscope_x_ = 0.0;
    double gyroscope_y_ = 0.0;
    double gyroscope_z_ = 0.0;
    bool has_accelerometer_sample_ = false;
    bool has_gyroscope_sample_ = false;

    double filtered_accelerometer_x_ = 0.0;
    double filtered_accelerometer_y_ = 0.0;
    double filtered_accelerometer_z_ = 0.0;
    double filtered_gyroscope_x_ = 0.0;
    double filtered_gyroscope_y_ = 0.0;
    double filtered_gyroscope_z_ = 0.0;
    bool accelerometer_filter_initialized_ = false;
    bool gyroscope_filter_initialized_ = false;

    double sum_accelerometer_x_ = 0.0;
    double sum_accelerometer_y_ = 0.0;
    double sum_accelerometer_z_ = 0.0;
    double sum_gyroscope_x_ = 0.0;
    double sum_gyroscope_y_ = 0.0;
    double sum_gyroscope_z_ = 0.0;
    int output_sample_count_ = 0;

    std::mt19937 noise_rng_{std::random_device{}()};
    std::normal_distribution<double> standard_normal_{0.0, 1.0};
};

} // namespace lem_dynamics_sim_

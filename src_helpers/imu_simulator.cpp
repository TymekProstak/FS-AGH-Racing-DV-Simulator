#include "imu_simulator.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

namespace lem_dynamics_sim_
{

void ImuSimulator::configure(const ParamBank& parameters,
                                   double simulation_step_s,
                                   std::mt19937& phase_rng)
{
    accelerometer_period_s_ =
        1.0 / parameters.get("imu_accelerometer_rate_hz");
    gyroscope_period_s_ =
        1.0 / parameters.get("imu_gyroscope_rate_hz");
    const double buffer_sample_period_s =
        1.0 / parameters.get("imu_buffer_sample_rate_hz");
    samples_per_output_ = std::max(
        1,
        static_cast<int>(std::round(
            parameters.get("imu_samples_per_output"))));

    accelerometer_timer_.configure(
        accelerometer_period_s_, simulation_step_s, phase_rng);
    gyroscope_timer_.configure(
        gyroscope_period_s_, simulation_step_s, phase_rng);
    output_sample_timer_.configure(
        buffer_sample_period_s, simulation_step_s, phase_rng);
}

std::optional<dv_interfaces::Imu> ImuSimulator::update(
    int simulation_step,
    const State& state,
    const ParamBank& parameters)
{
    if (accelerometer_timer_.due(simulation_step)) {
        read_accelerometer(state, parameters);
    }
    if (gyroscope_timer_.due(simulation_step)) {
        read_gyroscope(state, parameters);
    }
    if (output_sample_timer_.due(simulation_step)) {
        return sample_output();
    }
    return std::nullopt;
}

void ImuSimulator::read_accelerometer(
    const State& state,
    const ParamBank& parameters)
{
    const double sqrt_dt = std::sqrt(accelerometer_period_s_);
    const double bias_rw =
        parameters.get("imu_accelerometer_bias_rw_std");
    accelerometer_bias_x_ += bias_rw * sqrt_dt * gaussian_noise();
    accelerometer_bias_y_ += bias_rw * sqrt_dt * gaussian_noise();
    accelerometer_bias_z_ += bias_rw * sqrt_dt * gaussian_noise();

    const double roll = parameters.get("roll_inclination_of_world");
    const double pitch = parameters.get("pitch_inclination_of_world");
    const double yaw = state.yaw;

    const Eigen::AngleAxisd roll_rotation(
        roll, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch_rotation(
        pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw_rotation(
        yaw, Eigen::Vector3d::UnitZ());
    const Eigen::Matrix3d body_to_world =
        (pitch_rotation * roll_rotation * yaw_rotation).toRotationMatrix();
    const Eigen::Vector3d gravity_body =
        body_to_world.transpose() *
        Eigen::Vector3d(0.0, 0.0, -parameters.get("g"));

    const double true_x = state.prev_ax - gravity_body.x();
    const double true_y = state.prev_ay - gravity_body.y();
    const double true_z = -gravity_body.z();
    const double bandwidth_hz =
        parameters.get("imu_accelerometer_bandwidth_hz");
    const double tau = 1.0 / (2.0 * M_PI * bandwidth_hz);
    const double alpha = accelerometer_period_s_ /
        (tau + accelerometer_period_s_);

    if (!accelerometer_filter_initialized_) {
        filtered_accelerometer_x_ = true_x;
        filtered_accelerometer_y_ = true_y;
        filtered_accelerometer_z_ = true_z;
        accelerometer_filter_initialized_ = true;
    } else {
        filtered_accelerometer_x_ +=
            alpha * (true_x - filtered_accelerometer_x_);
        filtered_accelerometer_y_ +=
            alpha * (true_y - filtered_accelerometer_y_);
        filtered_accelerometer_z_ +=
            alpha * (true_z - filtered_accelerometer_z_);
    }

    const double noise_std =
        parameters.get("imu_accelerometer_noise_std");
    accelerometer_x_ = filtered_accelerometer_x_ +
        accelerometer_bias_x_ + noise_std * gaussian_noise();
    accelerometer_y_ = filtered_accelerometer_y_ +
        accelerometer_bias_y_ + noise_std * gaussian_noise();
    accelerometer_z_ = filtered_accelerometer_z_ +
        accelerometer_bias_z_ + noise_std * gaussian_noise();
    has_accelerometer_sample_ = true;
}

void ImuSimulator::read_gyroscope(
    const State& state,
    const ParamBank& parameters)
{
    const double sqrt_dt = std::sqrt(gyroscope_period_s_);
    const double bias_rw = parameters.get("imu_gyroscope_bias_rw_std");
    gyroscope_bias_x_ += bias_rw * sqrt_dt * gaussian_noise();
    gyroscope_bias_y_ += bias_rw * sqrt_dt * gaussian_noise();
    gyroscope_bias_z_ += bias_rw * sqrt_dt * gaussian_noise();

    constexpr double true_x = 0.0;
    constexpr double true_y = 0.0;
    const double true_z = state.yaw_rate;
    const double bandwidth_hz =
        parameters.get("imu_gyroscope_bandwidth_hz");
    const double tau = 1.0 / (2.0 * M_PI * bandwidth_hz);
    const double alpha =
        gyroscope_period_s_ / (tau + gyroscope_period_s_);

    if (!gyroscope_filter_initialized_) {
        filtered_gyroscope_x_ = true_x;
        filtered_gyroscope_y_ = true_y;
        filtered_gyroscope_z_ = true_z;
        gyroscope_filter_initialized_ = true;
    } else {
        filtered_gyroscope_x_ +=
            alpha * (true_x - filtered_gyroscope_x_);
        filtered_gyroscope_y_ +=
            alpha * (true_y - filtered_gyroscope_y_);
        filtered_gyroscope_z_ +=
            alpha * (true_z - filtered_gyroscope_z_);
    }

    const double noise_std = parameters.get("imu_gyroscope_noise_std");
    gyroscope_x_ = filtered_gyroscope_x_ +
        gyroscope_bias_x_ + noise_std * gaussian_noise();
    gyroscope_y_ = filtered_gyroscope_y_ +
        gyroscope_bias_y_ + noise_std * gaussian_noise();
    gyroscope_z_ = filtered_gyroscope_z_ +
        gyroscope_bias_z_ + noise_std * gaussian_noise();
    has_gyroscope_sample_ = true;
}

std::optional<dv_interfaces::Imu> ImuSimulator::sample_output()
{
    if (!has_accelerometer_sample_ || !has_gyroscope_sample_) {
        return std::nullopt;
    }

    sum_accelerometer_x_ += accelerometer_x_;
    sum_accelerometer_y_ += accelerometer_y_;
    sum_accelerometer_z_ += accelerometer_z_;
    sum_gyroscope_x_ += gyroscope_x_;
    sum_gyroscope_y_ += gyroscope_y_;
    sum_gyroscope_z_ += gyroscope_z_;
    ++output_sample_count_;

    if (output_sample_count_ < samples_per_output_) {
        return std::nullopt;
    }

    const double divisor = static_cast<double>(output_sample_count_);
    dv_interfaces::Imu message;
    message.gyro.x = sum_gyroscope_x_ / divisor;
    message.gyro.y = sum_gyroscope_y_ / divisor;
    message.gyro.z = -sum_gyroscope_z_ / divisor;

    // Preserve the IMU sensor-axis convention used by the vehicle stack.
    message.acc.x = sum_accelerometer_y_ / divisor;
    message.acc.y = sum_accelerometer_x_ / divisor;
    message.acc.z = sum_accelerometer_z_ / divisor;

    sum_accelerometer_x_ = 0.0;
    sum_accelerometer_y_ = 0.0;
    sum_accelerometer_z_ = 0.0;
    sum_gyroscope_x_ = 0.0;
    sum_gyroscope_y_ = 0.0;
    sum_gyroscope_z_ = 0.0;
    output_sample_count_ = 0;
    return message;
}

double ImuSimulator::gaussian_noise()
{
    return standard_normal_(noise_rng_);
}

} // namespace lem_dynamics_sim_

#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>
#include <random>
#include <stdexcept>
#include <utility>

namespace lem_dynamics_sim_
{

class PeriodicTimer
{
public:
    PeriodicTimer() = default;

    void configure(double period_s,
                   double simulation_step_s,
                   std::mt19937& phase_rng,
                   bool randomize_phase = true)
    {
        if (!std::isfinite(period_s) || period_s <= 0.0) {
            throw std::invalid_argument("PeriodicTimer: period must be positive");
        }
        if (!std::isfinite(simulation_step_s) || simulation_step_s <= 0.0) {
            throw std::invalid_argument(
                "PeriodicTimer: simulation step must be positive");
        }

        period_steps_ = std::max(
            1,
            static_cast<int>(std::round(period_s / simulation_step_s)));

        if (randomize_phase && period_steps_ > 1) {
            std::uniform_int_distribution<int> phase(0, period_steps_ - 1);
            phase_step_ = phase(phase_rng);
        } else {
            phase_step_ = 0;
        }
    }

    bool due(int simulation_step) const
    {
        return period_steps_ > 0 &&
               simulation_step % period_steps_ == phase_step_;
    }

    int period_steps() const { return period_steps_; }

private:
    int period_steps_ = 1;
    int phase_step_ = 0;
};

template <typename T>
class ValueCache
{
public:
    void write(const T& value)
    {
        value_ = value;
        has_value_ = true;
    }

    void write(T&& value)
    {
        value_ = std::move(value);
        has_value_ = true;
    }

    bool has_value() const { return has_value_; }

    const T& value() const
    {
        if (!has_value_) {
            throw std::logic_error("ValueCache: read before first write");
        }
        return value_;
    }

    T value_or(const T& fallback) const
    {
        return has_value_ ? value_ : fallback;
    }

private:
    T value_{};
    bool has_value_ = false;
};

template <typename T>
class DelayedQueue
{
public:
    void push(int ready_step, const T& value)
    {
        queue_.push_back({ready_step, value});
    }

    void push(int ready_step, T&& value)
    {
        queue_.push_back({ready_step, std::move(value)});
    }

    std::optional<T> take_latest_ready(int current_step)
    {
        std::optional<T> latest;

        while (!queue_.empty() && queue_.front().ready_step <= current_step) {
            latest = std::move(queue_.front().value);
            queue_.pop_front();
        }

        return latest;
    }

    void clear() { queue_.clear(); }

private:
    struct Entry
    {
        int ready_step;
        T value;
    };

    std::deque<Entry> queue_;
};

} // namespace lem_dynamics_sim_

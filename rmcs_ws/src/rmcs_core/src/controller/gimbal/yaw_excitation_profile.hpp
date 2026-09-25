#pragma once

#include <cmath>
#include <numbers>
#include <initializer_list>
#include <stdexcept>

namespace rmcs_core::controller::gimbal {

// Seconds and radians. Sampling this profile does not depend on executor frequency.
struct YawExcitationProfile {
    double delay_s = 5.0;
    double duration_s = 40.0;
    double ramp_s = 3.0;
    double amplitude_rad = 0.50;
    double max_velocity_rad_s = 3.79;
    double start_frequency_hz = 0.10;
    double end_frequency_hz = 5.0;

    struct Sample {
        // 1: waiting, 2: running, 3: finished. 0 is reserved for inactive.
        double state, offset_rad, velocity_rad_s, frequency_hz;
        double acceleration_rad_s2 = 0;
    };

    void validate() const {
        for (double value : {delay_s, duration_s, ramp_s, amplitude_rad,
                             start_frequency_hz, end_frequency_hz, max_velocity_rad_s}) {
            if (!std::isfinite(value))
                throw std::invalid_argument("yaw excitation parameters must be finite");
        }
        if (delay_s < 0 || duration_s <= 0 || ramp_s <= 0 || ramp_s > duration_s / 2
            || amplitude_rad < 0 || amplitude_rad >= std::numbers::pi
            || start_frequency_hz <= 0 || end_frequency_hz <= 0 || max_velocity_rad_s <= 0)
            throw std::invalid_argument("invalid yaw excitation duration, ramp, amplitude or frequency");
        if (velocity_budget_discriminant() <= 0 || velocity_headroom() <= 0)
            throw std::invalid_argument("reduce amplitude or increase ramp/duration for velocity limit");
    }

    Sample sample(double elapsed_s) const {
        if (!std::isfinite(elapsed_s) || elapsed_s < delay_s)
            return {1, 0, 0, 0};
        const double t = elapsed_s - delay_s;
        if (t >= duration_s)
            return {3, 0, 0, 0};
        const double slope = (end_frequency_hz - start_frequency_hz) / duration_s;
        const double frequency = start_frequency_hz + slope * t;
        const double phase = 2 * std::numbers::pi * (start_frequency_hz * t + slope * t * t / 2);
        double envelope = 1, envelope_rate = 0, envelope_acceleration = 0;
        if (t < ramp_s) {
            const double a = std::numbers::pi * t / ramp_s;
            envelope = (1 - std::cos(a)) / 2;
            envelope_rate = std::numbers::pi * std::sin(a) / (2 * ramp_s);
            envelope_acceleration = std::numbers::pi * std::numbers::pi * std::cos(a)
                                  / (2 * ramp_s * ramp_s);
        } else if (t > duration_s - ramp_s) {
            const double a = std::numbers::pi * (duration_s - t) / ramp_s;
            envelope = (1 - std::cos(a)) / 2;
            envelope_rate = -std::numbers::pi * std::sin(a) / (2 * ramp_s);
            envelope_acceleration = std::numbers::pi * std::numbers::pi * std::cos(a)
                                  / (2 * ramp_s * ramp_s);
        }
        // Smooth frequency-dependent amplitude. Reserve speed for both envelope
        // and amplitude derivatives, not just the sinusoidal carrier A*omega.
        const double budget = (velocity_headroom() + std::sqrt(velocity_budget_discriminant())) / 2;
        const double omega = 2 * std::numbers::pi * frequency;
        const double denominator = budget + amplitude_rad * omega;
        const double amplitude = amplitude_rad * budget / denominator;
        const double amplitude_rate = -amplitude_rad * amplitude_rad * budget
                                    * (2 * std::numbers::pi * slope) / (denominator * denominator);
        const double omega_rate = 2 * std::numbers::pi * slope;
        const double amplitude_acceleration = 2 * amplitude_rad * amplitude_rad * amplitude_rad
            * budget * omega_rate * omega_rate / (denominator * denominator * denominator);
        const double h = amplitude * envelope;
        const double h_rate = amplitude_rate * envelope + amplitude * envelope_rate;
        const double h_acceleration = amplitude_acceleration * envelope
            + 2 * amplitude_rate * envelope_rate + amplitude * envelope_acceleration;
        return {2, amplitude * envelope * std::sin(phase),
                (amplitude_rate * envelope + amplitude * envelope_rate) * std::sin(phase)
                    + amplitude * envelope * omega * std::cos(phase),
                frequency,
                (h_acceleration - h * omega * omega) * std::sin(phase)
                    + (2 * h_rate * omega + h * omega_rate) * std::cos(phase)};
    }

private:
    double velocity_headroom() const {
        return max_velocity_rad_s - amplitude_rad * std::numbers::pi / (2 * ramp_s);
    }
    double velocity_budget_discriminant() const {
        const double derivative_bound = amplitude_rad * amplitude_rad * 2 * std::numbers::pi
            * std::abs(end_frequency_hz - start_frequency_hz) / duration_s;
        return velocity_headroom() * velocity_headroom() - 4 * derivative_bound;
    }
};

} // namespace rmcs_core::controller::gimbal

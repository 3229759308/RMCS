#pragma once

#include "controller/gimbal/yaw_excitation_profile.hpp"

#include <algorithm>
#include <vector>

namespace rmcs_core::controller::gimbal {

// Version 2 protocol. Analytic trajectories retain full unwrapped yaw displacement.
// Stage: 1 prepare, 2 initial hold, 3 speed ladder, 4 acceleration comparison,
//        5 chirp, 6 held-out mixed trajectory, 7 final hold, 8 completed, 9 manual.
class YawTestSequence {
public:
    struct Config {
        double initial_hold_s = 10, final_hold_s = 10;
        std::vector<double> ladder_speeds{0.3, 1.0, 2.0, 3.0};
        double ladder_hold_s = 5, ladder_stop_s = 2, ladder_acceleration = 2;
        std::vector<double> accelerations{1.0, 3.0};
        double acceleration_speed = 2, acceleration_hold_s = 2;
        int acceleration_repeats = 2;
        std::vector<double> mixed_speeds{0.25, 1.5, -0.5, 0, -2.5, 2.5, 0.75, 0};
        double mixed_transition_s = 2.5, mixed_hold_s = 2.5;
    };
    struct Sample {
        double state = 0, angle = 0, velocity = 0, acceleration = 0, frequency = 0;
        int stage = 0, segment = 0;
        double stage_elapsed = 0;
    };

    void configure(const Config& config, const YawExcitationProfile& chirp) {
        chirp.validate();
        for (double v : {config.initial_hold_s, config.final_hold_s, config.ladder_hold_s,
                         config.ladder_stop_s, config.acceleration_hold_s, config.mixed_hold_s})
            if (!std::isfinite(v) || v < 0) throw std::invalid_argument("invalid test hold duration");
        for (double v : {config.ladder_acceleration, config.acceleration_speed, config.mixed_transition_s})
            if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("invalid test transition");
        if (config.ladder_speeds.empty() || config.accelerations.empty() || config.mixed_speeds.empty()
            || config.mixed_speeds.back() != 0 || config.acceleration_repeats < 1
            || config.acceleration_repeats > 100)
            throw std::invalid_argument("invalid test sequence; mixed trajectory must end at zero speed");
        for (double v : config.ladder_speeds)
            if (!std::isfinite(v) || v <= 0 || v > chirp.max_velocity_rad_s)
                throw std::invalid_argument("ladder speed outside reference speed limit");
        for (double v : config.mixed_speeds)
            if (!std::isfinite(v) || std::abs(v) > chirp.max_velocity_rad_s)
                throw std::invalid_argument("mixed speed outside reference speed limit");
        for (double v : config.accelerations)
            if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("invalid acceleration");
        if (config.acceleration_speed > chirp.max_velocity_rad_s)
            throw std::invalid_argument("acceleration test speed outside reference speed limit");

        chirp_ = chirp;
        delay_s_ = chirp.delay_s;
        chirp_.delay_s = 0;
        pieces_.clear(); total_s_ = angle_ = velocity_ = 0;
        add(2, config.initial_hold_s, 0);
        for (double speed : config.ladder_speeds) {
            for (double sign : {1.0, -1.0}) {
                ramp(3, sign * speed, config.ladder_acceleration);
                add(3, config.ladder_hold_s, sign * speed);
                ramp(3, 0, config.ladder_acceleration);
                add(3, config.ladder_stop_s, 0);
            }
        }
        for (double acceleration : config.accelerations) {
            for (int repeat = 0; repeat < config.acceleration_repeats; ++repeat) {
                for (double target : {config.acceleration_speed, -config.acceleration_speed, 0.0}) {
                    ramp(4, target, acceleration);
                    add(4, config.acceleration_hold_s, target);
                }
            }
        }
        add(5, chirp.duration_s, 0, true);
        for (double target : config.mixed_speeds) {
            add(6, config.mixed_transition_s, target);
            add(6, config.mixed_hold_s, target);
        }
        add(7, config.final_hold_s, 0);
    }

    double duration() const { return delay_s_ + total_s_; }
    const std::vector<double> boundaries() const {
        std::vector<double> result{delay_s_};
        for (const auto& p : pieces_) result.push_back(delay_s_ + p.start + p.duration);
        return result;
    }

    Sample sample(double elapsed) const {
        if (!std::isfinite(elapsed) || elapsed < delay_s_)
            return {1, 0, 0, 0, 0, 1, 0, std::max(0.0, elapsed)};
        const double t = elapsed - delay_s_;
        if (t >= total_s_)
            return {3, angle_, 0, 0, 0, 8, static_cast<int>(pieces_.size()), t-total_s_};
        for (size_t i = 0; i < pieces_.size(); ++i) {
            const auto& p = pieces_[i];
            if (t >= p.start + p.duration) continue;
            const double u = t-p.start;
            if (p.chirp) {
                const auto c = chirp_.sample(u);
                return {2, p.angle+c.offset_rad, c.velocity_rad_s, c.acceleration_rad_s2,
                        c.frequency_hz, p.stage, static_cast<int>(i+1), t-p.stage_start};
            }
            const double phase = std::numbers::pi*u/p.duration;
            const double dv = p.end_velocity-p.velocity;
            return {2,
                p.angle+p.velocity*u+dv*(u-p.duration/std::numbers::pi*std::sin(phase))/2,
                p.velocity+dv*(1-std::cos(phase))/2,
                dv*std::numbers::pi/(2*p.duration)*std::sin(phase),
                0, p.stage, static_cast<int>(i+1), t-p.stage_start};
        }
        return {3, angle_, 0, 0, 0, 8, static_cast<int>(pieces_.size()), 0};
    }

private:
    struct Piece {
        int stage;
        double start, stage_start, duration, angle, velocity, end_velocity;
        bool chirp;
    };
    void ramp(int stage, double target, double acceleration) {
        // Half-cosine velocity transition: peak acceleration exactly matches the setting.
        add(stage, std::numbers::pi*std::abs(target-velocity_)/(2*acceleration), target);
    }
    void add(int stage, double duration, double target, bool chirp = false) {
        if (duration <= 0) return;
        const double stage_start = pieces_.empty() || pieces_.back().stage != stage
            ? total_s_ : pieces_.back().stage_start;
        pieces_.push_back({stage,total_s_,stage_start,duration,angle_,velocity_,target,chirp});
        if (!chirp) angle_ += (velocity_+target)*duration/2;
        velocity_ = target;
        total_s_ += duration;
    }
    YawExcitationProfile chirp_;
    std::vector<Piece> pieces_;
    double delay_s_ = 0, total_s_ = 0, angle_ = 0, velocity_ = 0;
};

// Joystick trajectory uses elapsed time, not a presumed executor update rate.
struct ManualYawTrajectory {
    double angle = 0, velocity = 0, acceleration = 0;
    void update(double target, double dt, double acceleration_limit) {
        acceleration = 0;
        if (dt <= 0) return;
        const double delta = target-velocity;
        if (delta == 0) { angle += velocity*dt; return; }
        const double a = std::copysign(acceleration_limit, delta);
        const double ramp_time = std::min(dt,std::abs(delta)/acceleration_limit);
        angle += velocity*ramp_time + a*ramp_time*ramp_time/2 + target*(dt-ramp_time);
        velocity += a*ramp_time;
        if (ramp_time < dt) velocity = target;
        else acceleration = a;
    }
};

} // namespace rmcs_core::controller::gimbal

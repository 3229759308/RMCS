#pragma once

#include <cmath>
#include <numbers>
#include <vector>

namespace rmcs_core::controller::gimbal {

// Normalized command protocol shared by direct torque and velocity-only tests.
// Stages: 10 zero baseline, 11 small signal/ladder, 12 reversals,
// 13 smooth ramps, 14 chirp, 15 held-out mixed commands, 16 final zero, 17 complete,
// 18 peak torque pulses (torque mode only; scaled separately from rated torque).
class YawSingleLoopSequence {
public:
    struct Sample {
        double value = 0, frequency = 0, stage_elapsed = 0;
        int stage = 17, segment = 0;
    };
    explicit YawSingleLoopSequence(bool velocity_mode = false) {
        // Speed plateaus allow steady-state measurement; torque pulses stay short.
        const double hold = velocity_mode ? 3.0 : 0.5;
        const double stop = velocity_mode ? 2.0 : 1.5;
        const double reversal = velocity_mode ? 1.0 : 0.4;
        add(10, 3, 0);
        for (double level : {0.05, 0.15, 0.4, 0.7, 1.0})
            for (double sign : {1.0, -1.0}) {
                add(11, hold, sign*level);
                add(11, stop, 0);
            }
        for (int i = 0; i < 3; ++i) {
            add(12, reversal, 0.7);
            add(12, reversal, -0.7);
            add(12, velocity_mode ? 2.0 : 1.2, 0);
        }
        for (double target : {1.0, -1.0, 0.0}) add(13, velocity_mode ? 4.0 : 2.0, target, true);
        add(14, 20, 0);
        for (double target : {0.2, -0.6, 0.0, 0.9, -0.3, 0.0}) {
            add(15, 0.6, target);
            add(15, 1.4, 0);
        }
        if (!velocity_mode)
            for (double sign : {1.0, -1.0}) {
                add(18, 0.5, sign);
                add(18, 1.5, 0);
            }
        add(16, 3, 0);
    }
    double duration() const { return duration_; }
    Sample sample(double t) const {
        if (!std::isfinite(t) || t < 0) return {};
        for (size_t i = 0; i < pieces_.size(); ++i) {
            const auto& p = pieces_[i];
            if (t >= p.start+p.duration) continue;
            const double u = t-p.start;
            double value = p.target, frequency = 0;
            if (p.ramp)
                value = p.previous+(p.target-p.previous)*(1-std::cos(std::numbers::pi*u/p.duration))/2;
            if (p.stage == 14) {
                frequency = 0.1+0.245*u; // 0.1 -> 5 Hz, smooth zero endpoints.
                const double envelope = std::pow(std::sin(std::numbers::pi*u/p.duration), 2);
                value = envelope*std::sin(2*std::numbers::pi*(0.1*u+0.245*u*u/2));
            }
            return {value, frequency, t-p.stage_start, p.stage, static_cast<int>(i+1)};
        }
        return {};
    }
private:
    struct Piece {
        int stage;
        double start, stage_start, duration, previous, target;
        bool ramp;
    };
    void add(int stage, double duration, double target, bool ramp = false) {
        const double stage_start = pieces_.empty() || pieces_.back().stage != stage
            ? duration_ : pieces_.back().stage_start;
        pieces_.push_back({stage, duration_, stage_start, duration,
                          pieces_.empty() ? 0 : pieces_.back().target, target, ramp});
        duration_ += duration;
    }
    std::vector<Piece> pieces_;
    double duration_ = 0;
};
} // namespace rmcs_core::controller::gimbal

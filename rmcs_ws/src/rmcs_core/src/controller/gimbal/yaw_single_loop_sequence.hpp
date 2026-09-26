#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

namespace rmcs_core::controller::gimbal {

// Normalized command protocol shared by direct torque and velocity-only tests.
// Stages: 10 zero baseline, 11 small signal/ladder, 12 reversals,
// 13 smooth ramps, 14 chirp, 15 held-out mixed commands, 16 final zero, 17 complete,
// 18 high-torque ladder pulses (scaled by peak torque); torque-only stages below
// use rated torque: 19 fixed sine, 20 multisine, 21 biased sine,
// 22 direct steps, 23 variable-width pulses, 24 triangular ramps.
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
        // Torque ladder: 0.05 through 2.5 N*m at the default rated amplitude.
        const std::vector<double> levels = velocity_mode
            ? std::vector<double>{0.05, 0.15, 0.4, 0.7, 1.0}
            : std::vector<double>{0.02, 0.04, 0.05, 0.08, 0.10, 0.15, 0.20, 0.25,
                                  0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.0};
        for (double level : levels)
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
            // 3.0, 3.5, 4.0 and 4.5 N*m at the default peak amplitude.
            for (double level : {2.0/3.0, 7.0/9.0, 8.0/9.0, 1.0})
                for (double sign : {1.0, -1.0}) {
                    add(18, 0.5, sign*level);
                    add(18, 1.5, 0);
                }
        if (!velocity_mode) {
            // Four cycles each: first/last cycle fade, two full-amplitude cycles.
            for (double amplitude : {0.2, 0.5, 1.0})
                for (double frequency : {0.25, 0.5, 1.0, 2.0, 5.0}) {
                    wave(19, 4/frequency, 2, amplitude, frequency);
                    add(19, 1, 0);
                }
            wave(20, 12, 3, 1, 0); // 0.5, 1.5, 3.5 Hz weighted multisine.
            add(20, 1, 0);
            for (double sign : {1.0, -1.0}) {
                wave(21, 6, 2, 0.3, 1, sign*0.2);
                add(21, 1, 0);
            }
            for (double level : {0.2, 0.5, 1.0}) {
                for (double target : {level, level/2, -level, -level/2, 0.0})
                    add(22, 0.5, target);
                add(22, 1, 0);
            }
            for (double width : {0.1, 0.25, 1.0})
                for (double sign : {1.0, -1.0}) {
                    add(23, width, sign*0.5);
                    add(23, 1, 0);
                }
            for (double target : {1.0, -1.0, 0.0}) {
                add(24, 2, target);
                pieces_.back().shape = 4;
            }
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
            if (p.shape == 4) value = p.previous+(p.target-p.previous)*u/p.duration;
            if (p.shape == 2 || p.shape == 3) {
                const double fade = p.shape == 2 ? 1/p.frequency : 1;
                const double edge = std::clamp(std::min(u,p.duration-u)/fade,0.0,1.0);
                const double envelope = (1-std::cos(std::numbers::pi*edge))/2;
                if (p.shape == 2) {
                    frequency = p.frequency;
                    value = envelope*(p.bias+p.amplitude*std::sin(2*std::numbers::pi*frequency*u));
                } else {
                    value = envelope*p.amplitude*(0.5*std::sin(2*std::numbers::pi*0.5*u)
                        +0.3*std::sin(2*std::numbers::pi*1.5*u)
                        +0.2*std::sin(2*std::numbers::pi*3.5*u));
                }
            }
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
        int shape = 0;
        double amplitude = 0, frequency = 0, bias = 0;
    };
    void add(int stage, double duration, double target, bool ramp = false) {
        const double stage_start = pieces_.empty() || pieces_.back().stage != stage
            ? duration_ : pieces_.back().stage_start;
        pieces_.push_back({stage, duration_, stage_start, duration,
                          pieces_.empty() ? 0 : pieces_.back().target, target, ramp});
        duration_ += duration;
    }
    void wave(int stage, double duration, int shape, double amplitude, double frequency,
              double bias = 0) {
        add(stage,duration,0);
        auto& p = pieces_.back();
        p.shape = shape;
        p.amplitude = amplitude;
        p.frequency = frequency;
        p.bias = bias;
    }
    std::vector<Piece> pieces_;
    double duration_ = 0;
};
} // namespace rmcs_core::controller::gimbal

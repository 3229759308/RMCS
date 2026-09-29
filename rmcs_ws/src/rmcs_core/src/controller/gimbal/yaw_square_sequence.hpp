#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace rmcs_core::controller::gimbal {

// Protocol 8 (seven step groups): stage 30 preparation at -A, 31 step response,
// 33 center dwell. Preparation and dwell are never response windows.
class YawSquareSequence {
public:
    struct Group { double amplitude_deg, frequency; int cycles, stage; };
    struct Sample {
        double state = 1, angle = 0, frequency = 0, stage_elapsed = 0;
        int stage = 1, group = 0, cycle = 0, edge = 0;
        double amplitude_deg = 0, edge_time = -1;
    };
    void configure(double delay, double prepare, double dwell, int selected = 0) {
        for (double v : {delay, prepare, dwell})
            if (!std::isfinite(v) || v < 0) throw std::invalid_argument("invalid square timing");
        if (prepare < 0.2 || selected < 0 || selected > 7)
            throw std::invalid_argument("square prepare must be >=0.2 s; group must be 0..7");
        delay_ = delay; prepare_ = prepare; dwell_ = dwell; selected_ = selected;
        groups_.clear();
        for (double a : {5., 10., 15.})
            for (double f : {0.2, 0.5}) groups_.push_back({a, f, 8, 31});
        groups_.push_back({30., 0.2, 8, 31});
    }
    const std::vector<Group>& groups() const { return groups_; }
    double duration() const {
        double t = delay_;
        for (size_t i = 0; i < groups_.size(); ++i)
            if (!selected_ || selected_ == static_cast<int>(i+1))
                t += prepare_ + groups_[i].cycles/groups_[i].frequency + dwell_;
        return t;
    }
    Sample sample(double elapsed) const {
        if (!std::isfinite(elapsed) || elapsed < delay_) return {};
        double start = delay_;
        for (size_t i = 0; i < groups_.size(); ++i) {
            if (selected_ && selected_ != static_cast<int>(i+1)) continue;
            const auto& g = groups_[i];
            const double active = g.cycles/g.frequency;
            const double end = start + prepare_ + active + dwell_;
            if (elapsed >= end) { start = end; continue; }
            Sample s;
            s.group = static_cast<int>(i+1); s.amplitude_deg = g.amplitude_deg;
            s.frequency = g.frequency;
            const double a = g.amplitude_deg*std::numbers::pi/180;
            if (elapsed < start + prepare_) {
                s.stage = 30; s.angle = -a; s.stage_elapsed = elapsed-start;
            } else if (elapsed < start + prepare_ + active) {
                s.state = 2; s.stage = g.stage;
                s.stage_elapsed = elapsed-start-prepare_;
                const int half = std::min(2*g.cycles-1,
                    static_cast<int>(std::floor(s.stage_elapsed*2*g.frequency + 1e-10)));
                s.angle = half%2 == 0 ? a : -a;
                s.cycle = half/2+1; s.edge = half+1;
                s.edge_time = start+prepare_+half/(2*g.frequency);
            } else {
                s.stage = 33; s.stage_elapsed = elapsed-start-prepare_-active;
            }
            return s;
        }
        Sample s; s.state = 3; s.stage = 8; return s;
    }
private:
    std::vector<Group> groups_;
    double delay_ = 5, prepare_ = 5, dwell_ = 2;
    int selected_ = 0;
};
} // namespace rmcs_core::controller::gimbal

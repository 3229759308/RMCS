#include "controller/gimbal/yaw_square_sequence.hpp"
#include <iostream>

using rmcs_core::controller::gimbal::YawSquareSequence;
static void check(bool ok) { if (!ok) throw std::runtime_error("square sequence regression"); }
int main() {
    YawSquareSequence all;
    all.configure(5, 5, 2);
    check(all.groups().size() == 22);
    check(std::abs(all.duration() - (575.0 + 2.0/3.0)) < 1e-9);
    double start = 5;
    for (int id = 1; id <= 22; ++id) {
        const auto g = all.groups()[id-1];
        check(g.amplitude_deg >= 5 && g.frequency >= 0.2);
        check(g.cycles == (id <= 12 ? 8 : 10));
        check(g.stage == (id <= 12 ? 31 : 32));
        const double a = g.amplitude_deg*std::numbers::pi/180;
        const double half = 0.5/g.frequency;
        check(all.sample(start+0.01).stage == 30);
        check(all.sample(start+4.99).angle == -a);
        int positive = 0, negative = 0;
        for (int edge = 0; edge < 2*g.cycles; ++edge) {
            const double time = start+5+edge*half;
            const auto s = all.sample(time+1e-8);
            check(s.state == 2 && s.stage == g.stage && s.group == id);
            check(s.edge == edge+1 && s.cycle == edge/2+1);
            check(std::abs(s.edge_time-time) < 1e-9 && s.frequency == g.frequency);
            check(s.angle == (edge%2 ? -a : a));
            check(all.sample(time+half-1e-7).angle == s.angle);
            check(std::abs(s.angle-all.sample(time-1e-7).angle) > 1.99*a);
            if (edge%2) ++negative; else ++positive;
        }
        check(positive == g.cycles && negative == g.cycles);
        const double end = start+5+g.cycles/g.frequency;
        check(all.sample(end+1e-8).stage == 33 && all.sample(end+1e-8).angle == 0);
        YawSquareSequence single;
        single.configure(5, 5, 2, id);
        check(single.sample(10.01).group == id);
        check(single.sample(single.duration()+1e-8).state == 3);
        start = end+2;
    }
    check(std::abs(start-all.duration()) < 1e-9);
    check(all.sample(start+1e-8).state == 3);
    check(all.sample(4.99).state == 1);
    for (int id : {-1, 23}) {
        bool rejected = false;
        try { all.configure(5,5,2,id); } catch (const std::invalid_argument&) { rejected = true; }
        check(rejected);
    }
    std::cout << "22 groups: amplitudes, frequencies, complete windows, selection and completion passed\n";
}

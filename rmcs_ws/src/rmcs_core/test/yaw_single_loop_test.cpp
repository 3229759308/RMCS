#include "controller/gimbal/yaw_excitation.cpp"
#include "controller/gimbal/deformable_infantry_gimbal_controller.cpp"
#include "component_fixture.hpp"

#include <iostream>
#include <set>

static void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
static void near(double actual, double expected, const char* message) {
    check(std::isfinite(actual) && std::abs(actual-expected) < 1e-8, message);
}

int main(int argc, char** argv) {
    using namespace rmcs_core::controller::gimbal;
    using rmcs_executor::Executor;
    using rmcs_msgs::Switch;
    YawSingleLoopSequence sequence;
    near(sequence.duration(), 74, "74 second supplementary protocol");
    YawSingleLoopSequence velocity_sequence{true};
    near(velocity_sequence.duration(),112,"speed protocol includes longer steady-state plateaus");
    near(velocity_sequence.sample(5.99).value,0.05,"speed small-signal plateau lasts three seconds");
    std::set<int> stages;
    bool positive = false, negative = false;
    for (double t = 0; t < sequence.duration(); t += 0.0037) {
        const auto sample = sequence.sample(t);
        stages.insert(sample.stage);
        check(std::isfinite(sample.value) && std::abs(sample.value) <= 1, "bounded normalized command");
        check(sample.frequency >= 0 && sample.frequency <= 5, "chirp frequency bounds");
        check(sample.segment > 0 && sample.stage_elapsed >= 0, "stage metadata");
        positive |= sample.value > 0.99;
        negative |= sample.value < -0.99;
    }
    check(stages == std::set<int>({10,11,12,13,14,15,16,18}) && positive && negative,
          "all supplementary scenarios and both signs covered");
    near(sequence.sample(3.1).value, 0.05, "small positive pulse");
    near(sequence.sample(3.6).value, 0, "free decay between pulses");
    near(sequence.sample(5.1).value, -0.05, "small negative pulse");
    near(sequence.sample(23.1).value, 0.7, "positive reversal");
    near(sequence.sample(23.5).value, -0.7, "negative reversal without intervening zero");
    for (double t : {29.0,31.0,33.0,35.0})
        check(std::abs(sequence.sample(t+1e-7).value-sequence.sample(t-1e-7).value) < 1e-5,
              "ramp and chirp endpoints continuous");
    for (double t : {-1.0,74.0,1000.0,std::numeric_limits<double>::quiet_NaN()})
        near(sequence.sample(t).value, 0, "outside sequence gives zero, never repeats");

    rclcpp::init(argc, argv);
    {
        rmcs_executor::Component::initializing_component_name = "single_loop_generator";
        YawExcitation generator;
        rmcs_executor::Component::initializing_component_name = "gimbal_test";
        DeformableInfantryGimbalController controller;
        Switch left = Switch::DOWN, right = Switch::DOWN;
        rmcs_description::Tf tf;
        tf.set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(0.0);
        tf.set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(0.0);
        tf.set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(Eigen::Quaterniond::Identity());
        Eigen::Vector2d joystick{1.0,1.0};
        double pitch = 0, yaw_rate = 0, pitch_rate = 0;
        for (rmcs_executor::Component* component : {static_cast<rmcs_executor::Component*>(&generator),
                                                   static_cast<rmcs_executor::Component*>(&controller)}) {
            Executor::bind(*component, "/remote/switch/left", left);
            Executor::bind(*component, "/remote/switch/right", right);
            Executor::bind(*component, "/remote/joystick/left", joystick);
            Executor::bind(*component, "/tf", tf);
        }
        Executor::bind(controller, "/gimbal/yaw/velocity_imu", yaw_rate);
        Executor::bind(controller, "/gimbal/pitch/angle", pitch);
        Executor::bind(controller, "/gimbal/pitch/velocity_imu", pitch_rate);
        const std::string prefix = "/gimbal/yaw/excitation/";
        auto signal = [&](const std::string& name) -> double& {
            return Executor::output<double>(generator,prefix+name);
        };
        auto output = [&](const std::string& name) -> double& {
            return Executor::output<double>(controller,"/gimbal/yaw/"+name);
        };
        Executor::bind(controller, prefix+"selected", Executor::output<bool>(generator,prefix+"selected"));
        Executor::bind(controller, prefix+"direction", Executor::output<Eigen::Vector3d>(generator,prefix+"direction"));
        for (const auto* name : {"mode","state","session_id","torque_reference_nm","velocity_rad_s"})
            Executor::bind(controller,prefix+name,signal(name));
        auto now = std::chrono::steady_clock::time_point{};
        auto step = [&](double dt = 0) {
            now += std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(dt));
            generator.update_at(now);
            controller.update();
        };
        auto disabled = [&] {
            check(std::isnan(output("control_torque")), "yaw disabled");
            check(std::isnan(Executor::output<double>(controller,"/gimbal/pitch/control_torque")), "pitch disabled");
        };
        auto restart = [&](Switch target) {
            left = right = Switch::DOWN;
            step(); disabled();
            left = target; right = Switch::MIDDLE;
            yaw_rate = 0;
            step();
            near(signal("state"),1,"re-entry starts full delay");
            near(output("control_torque"),0,"preparation has zero yaw torque");
        };
        step(); disabled();
        restart(Switch::UP);
        near(signal("mode"),3,"dedicated torque entry");
        near(signal("protocol_version"),3,"supplement protocol version");
        near(signal("planned_duration_s"),79,"delay plus full protocol");
        step(4.999); near(signal("state"),1,"wait full five seconds");
        step(0.001); near(signal("state"),2,"baseline begins after delay");
        step(3.1);
        near(signal("torque_reference_nm"),0.125,"physical torque reference scaling");
        near(output("control_torque"),0.125,"direct torque bypasses both yaw PIDs");
        yaw_rate = -0.6;
        controller.update();
        near(output("control_torque"),0.125,"torque independent of measured speed");
        check(std::isnan(output("control_angle_error")),"no position-loop error in supplementary mode");
        near(Executor::output<double>(controller,"/gimbal/pitch/control_angle_error"),
             -5*std::numbers::pi/180,"pitch remains five degrees despite joystick");
        step(2); near(output("control_torque"),-0.125,"negative torque pulse reaches motor command");
        for (double torque : {2.5,-2.5,4.5,-4.5,5.0,-5.0}) {
            signal("torque_reference_nm") = torque;
            controller.update(); near(output("control_torque"),torque,"direct torque has no supplementary clamp");
        }
        restart(Switch::UP);
        step(72.1); // 5 s preparation + 67.1 s: positive peak stage.
        near(signal("stage"),18,"separate peak torque stage");
        near(output("control_torque"),4.5,"positive peak torque pulse");
        step(2); near(output("control_torque"),-4.5,"negative peak torque pulse");
        step(100); near(signal("state"),3,"completion state");
        near(output("control_torque"),0,"completed torque mode gives zero");
        step(100); near(signal("state"),3,"does not automatically repeat");

        const double previous_session = signal("session_id");
        yaw_rate = 0;
        left = Switch::MIDDLE;
        step(); near(signal("session_id"),previous_session+1,"mode switch starts new session");
        near(signal("mode"),4,"dedicated velocity entry");
        near(signal("planned_duration_s"),117,"speed test delay and extended plateaus");
        near(signal("state"),1,"torque to velocity restarts delay");
        step(8.1);
        near(signal("velocity_rad_s"),0.05,"velocity reference scaling");
        near(output("control_torque"),0.65,"single velocity PID without position cascade");
        near(output("supplement/velocity_error_rad_s"),0.05,"velocity error recorded");
        signal("velocity_rad_s") = -0.05;
        controller.update(); check(output("control_torque") < 0,"speed reversal drives negative torque");
        signal("velocity_rad_s") = 1;
        controller.update(); near(output("control_torque"),13,"velocity PID has no supplementary torque clamp");
        signal("velocity_rad_s") = 0;
        controller.update(); near(output("control_torque"),0.02,"original PID integral is retained");
        step(200); near(output("control_torque"),0,"completed speed test releases yaw torque");
        near(signal("state"),3,"velocity test completes once");

        for (Switch target : {Switch::UP,Switch::MIDDLE}) {
            restart(target); step(8.1);
            for (double speed : {4.0,-4.0,6.0,-6.0}) {
                yaw_rate = speed;
                step();
                near(signal("state"),2,"no supplementary overspeed trip");
                check(std::isfinite(output("control_torque")),"controller continues at high speed");
                if (target == Switch::UP)
                    near(output("control_torque"),0.125,"direct torque unaffected by speed");
            }
            yaw_rate = std::numeric_limits<double>::quiet_NaN();
            step(); near(signal("state"),2,"generator has no feedback fault latch");
            yaw_rate = 0;
            step(); check(std::isfinite(output("control_torque")),"feedback recovery needs no re-entry");
            left = right = Switch::DOWN;
            controller.update(); disabled(); // priority over stale excitation inputs
            step(); near(signal("mode"),0,"double down exits supplementary mode");
            restart(target); step(8.1);
            right = Switch::UNKNOWN;
            controller.update(); disabled();
            step(); near(signal("mode"),0,"remote loss exits supplementary mode");
        }
        // Exercise every sample through the real controller using irregular virtual time steps.
        for (Switch target : {Switch::UP,Switch::MIDDLE}) {
            restart(target);
            const double duration = signal("planned_duration_s");
            std::set<int> integrated_stages;
            for (double elapsed = 0; elapsed < duration+0.03; elapsed += 0.0173) {
                step(0.0173);
                integrated_stages.insert(static_cast<int>(signal("stage")));
                check(std::isfinite(output("control_torque")),"entire protocol produces finite torque");
                if (target == Switch::UP)
                    near(output("control_torque"),signal("torque_reference_nm"),"torque reference passes through unchanged");
                check(std::isnan(signal("reference_yaw_rad")),"supplement has no angle reference");
                check(std::isfinite(Executor::output<double>(controller,"/gimbal/pitch/control_torque")),
                      "pitch stays controlled throughout both protocols");
            }
            for (int stage = 10; stage <= 17; ++stage)
                check(integrated_stages.contains(stage),"every supplementary stage reaches the controller");
            near(signal("state"),3,"full integrated protocol completes");
            near(output("control_torque"),0,"full integrated protocol ends with zero torque");
        }
        restart(Switch::UP);
        tf.set_transform<rmcs_description::PitchLink,rmcs_description::OdomImu>(
            Eigen::Quaterniond{Eigen::AngleAxisd{std::numbers::pi/2,Eigen::Vector3d::UnitY()}});
        step(); disabled(); near(signal("state"),4,"original invalid-heading behavior retained");
        tf.set_transform<rmcs_description::PitchLink,rmcs_description::OdomImu>(Eigen::Quaterniond::Identity());
        step(); disabled();
        restart(Switch::UP);
        right = Switch::UP; // ordinary manual control resumes from current pose
        step(); near(signal("mode"),0,"leaving supplement restores original mode");
        check(std::isfinite(output("control_torque")),"ordinary controller remains usable");
    }
    for (const auto* name : {"bad_torque","bad_speed","bad_peak","bad_switch"}) {
        rmcs_executor::Component::initializing_component_name = name;
        bool rejected = false;
        try { YawExcitation invalid; } catch (const std::invalid_argument&) { rejected = true; }
        check(rejected,"invalid supplementary configuration must be rejected");
    }
    rclcpp::shutdown();
    std::cout << "single torque and velocity loop sequence/controller checks passed\n";
}

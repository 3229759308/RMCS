#include "controller/gimbal/deformable_infantry_gimbal_controller.cpp"
#include "component_fixture.hpp"

#include <iostream>

static void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
static void near(double actual, double expected) {
    check(std::isfinite(actual) && std::abs(actual - expected) < 1e-9, "numeric mismatch");
}

static void test_arithmetic() {
    using namespace yaw_experiment;
    AccelFeedforwardConfig c;
    validate(c);
    auto run = [&](double alpha) { return evaluate(c, true, true, 1, 2, true, alpha); };
    check(!run(3).active, "default off");
    c.enabled = true;
    check(!run(3).active, "zero scale off");
    c.scale = 1;
    near(run(3).applied_nm, 0.521334);
    near(run(-3).applied_nm, -0.521334);
    near(run(81.729).applied_nm, 0.6);
    near(run(-81.729).applied_nm, -0.6);
    check(run(81.729).clipped, "chirp clipped");
    c.scale = 0.25;
    near(run(3).applied_nm, 0.1303335);
    check(std::signbit(add_to_pid(-0.0, {})), "signed zero retained");
    const double nan = std::numeric_limits<double>::quiet_NaN();
    check(std::isnan(add_to_pid(nan, run(3))), "NaN retained");
    c.inertia_kg_m2 = std::numeric_limits<double>::max();
    check(!run(81.729).reference_valid, "overflow rejected");
    for (int field = 0; field < 3; ++field) {
        for (double bad : {-1.0, nan, std::numeric_limits<double>::infinity()}) {
            auto invalid = AccelFeedforwardConfig{};
            if (field == 0) invalid.inertia_kg_m2 = bad;
            if (field == 1) invalid.scale = bad;
            if (field == 2) invalid.limit_nm = bad;
            bool rejected = false;
            try { validate(invalid); } catch (const std::invalid_argument&) { rejected = true; }
            check(rejected, "invalid configuration rejected");
        }
    }
    for (auto invalid : {AccelFeedforwardConfig{true, 0, 1, .6},
                         AccelFeedforwardConfig{true, .173778, 2, .6},
                         AccelFeedforwardConfig{true, .173778, 1, 0}}) {
        bool rejected = false;
        try { validate(invalid); } catch (const std::invalid_argument&) { rejected = true; }
        check(rejected, "configuration boundary rejected");
    }
}

int main(int argc, char** argv) {
    test_arithmetic();
    rclcpp::init(argc, argv);
    {
        using namespace rmcs_core::controller::gimbal;
        using rmcs_executor::Executor;
        using rmcs_msgs::Switch;
        rmcs_executor::Component::initializing_component_name = "gimbal_test";
        DeformableInfantryGimbalController controller;
        Switch left = Switch::UP, right = Switch::DOWN;
        rmcs_description::Tf tf;
        tf.set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(0.0);
        tf.set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(0.0);
        tf.set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(Eigen::Quaterniond::Identity());
        Eigen::Vector2d joystick = Eigen::Vector2d::Zero();
        Eigen::Vector3d direction{std::cos(0.01), std::sin(0.01), 0};
        double pitch = 0, yaw_rate = .01, pitch_rate = 0;
        double mode = 1, state = 2, session = 1, alpha = 3, velocity = .1, torque = .2;
        bool selected = true;
        Executor::bind(controller, "/remote/switch/left", left);
        Executor::bind(controller, "/remote/switch/right", right);
        Executor::bind(controller, "/remote/joystick/left", joystick);
        Executor::bind(controller, "/tf", tf);
        Executor::bind(controller, "/gimbal/yaw/velocity_imu", yaw_rate);
        Executor::bind(controller, "/gimbal/pitch/angle", pitch);
        Executor::bind(controller, "/gimbal/pitch/velocity_imu", pitch_rate);
        const std::string prefix = "/gimbal/yaw/excitation/";
        Executor::bind(controller, prefix + "selected", selected);
        Executor::bind(controller, prefix + "direction", direction);
        Executor::bind(controller, prefix + "mode", mode);
        Executor::bind(controller, prefix + "state", state);
        Executor::bind(controller, prefix + "session_id", session);
        Executor::bind(controller, prefix + "velocity_rad_s", velocity);
        Executor::bind(controller, prefix + "torque_reference_nm", torque);
        auto out = [&](const std::string& name) -> double& {
            return Executor::output<double>(controller, "/gimbal/yaw/" + name);
        };
        auto cleared = [&] {
            for (const char* name : {"active", "torque_raw_nm", "torque_applied_nm", "clipped"})
                near(out(std::string("ff/") + name), 0);
        };
        auto disabled = [&] {
            cleared();
            check(std::isnan(out("control_torque")), "original disable semantics");
            check(std::isnan(out("ff/pid_torque_nm")), "no stale PID diagnostics");
            check(std::isnan(out("ff/velocity_command_rad_s")), "no stale velocity diagnostics");
        };
        controller.update();
        cleared();
        near(out("ff/reference_valid"), 0);
        check(std::isnan(out("ff/acceleration_used_rad_s2")), "unconnected reference logged");
        Executor::bind(controller, prefix + "acceleration_rad_s2", alpha);
        // Independent original PID cascade checks both off-path equivalence and one update/cycle.
        rmcs_core::controller::pid::PidCalculator angle_pid{10, 0, 0}, velocity_pid{13, .02, 0};
        session += 1;
        const bool enabled = out("ff/enabled") != 0 && out("ff/scale") > 0;
        for (double acceleration : {3., -3., 81.729, -81.729, 0., 3.}) {
            alpha = acceleration;
            controller.update();
            const double command = angle_pid.update(out("control_angle_error"));
            const double expected_pid = velocity_pid.update(command - yaw_rate);
            const double expected_ff = enabled
                ? std::clamp(out("ff/scale") * .173778 * alpha, -.6, .6) : 0;
            near(out("ff/velocity_command_rad_s"), command);
            near(out("ff/velocity_error_rad_s"), command - yaw_rate);
            near(out("ff/pid_torque_nm"), expected_pid);
            near(out("ff/torque_applied_nm"), expected_ff);
            near(out("control_torque"), expected_pid + expected_ff);
            near(out("ff/active"), enabled);
        }
        for (double s : {0., 1., 3., 4.}) {
            state = s; controller.update(); cleared();
        }
        state = 2;
        for (double m : {2., 3., 4.}) {
            mode = m; controller.update(); cleared();
            if (m == 3) near(out("control_torque"), torque);
            if (m != 2) check(std::isnan(out("ff/pid_torque_nm")), "supplement has no normal PID diagnostic");
        }
        mode = 1;
        for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()}) {
            alpha = bad; controller.update(); cleared(); near(out("ff/reference_valid"), 0);
        }
        alpha = 3;
        controller.update(); near(out("ff/active"), enabled);
        direction.setZero(); controller.update(); disabled();
        direction = Eigen::Vector3d{1, .01, 0};
        session += 1; controller.update(); near(out("ff/active"), enabled);
        yaw_rate = std::numeric_limits<double>::quiet_NaN();
        controller.update(); disabled();
        yaw_rate = .01;
        for (Switch sw : {Switch::DOWN, Switch::UNKNOWN}) {
            left = sw; controller.update(); disabled();
            left = Switch::UP; session += 1; controller.update(); near(out("ff/active"), enabled);
        }
        selected = false; right = Switch::UP;
        controller.update(); cleared();
        // Toggle the existing pitch hold switch while stale standard references remain bound.
        left = Switch::DOWN; right = Switch::MIDDLE; controller.update();
        right = Switch::UP; controller.update(); cleared();
        check(std::isnan(out("ff/pid_torque_nm")), "hold excludes normal diagnostic");
        selected = true; session += 1; left = Switch::UP; right = Switch::DOWN;
        controller.update(); near(out("ff/active"), enabled);
    }
    rclcpp::shutdown();
    std::cout << "PASS: yaw inertia arithmetic and controller wiring/reset/PID checks\n";
}

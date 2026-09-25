#include "controller/gimbal/yaw_excitation.cpp"
#include "controller/gimbal/deformable_infantry_gimbal_controller.cpp"

#include <cstdlib>
#include <iostream>
#include <thread>

// Standalone executor fixture: uses the same interface binding as the real executor,
// but never creates a hardware component or starts its update thread.
namespace rmcs_executor {
class Executor {
public:
    template <typename T>
    static void bind(Component& component, const std::string& name, T& value) {
        for (auto& input : component.input_list_) {
            if (input.name == name && input.type == typeid(T)) {
                input.bind(input.binding, &value);
                return;
            }
        }
        throw std::runtime_error("missing input: " + name);
    }
    template <typename T>
    static T& output(Component& component, const std::string& name) {
        for (auto& output : component.output_list_)
            if (output.name == name && output.type == typeid(T))
                return *static_cast<T*>(output.binding);
        throw std::runtime_error("missing output: " + name);
    }
};
}

static void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

int main(int argc, char** argv) {
    using namespace rmcs_core::controller::gimbal;
    using rmcs_executor::Executor;
    using rmcs_msgs::Switch;
    YawExcitationProfile profile;
    profile.validate();
    check(profile.sample(4.999).state == 1, "must wait full default five seconds");
    check(profile.sample(5).state == 2, "starts at five seconds");
    check(profile.sample(45).state == 3 && profile.sample(100).offset_rad == 0,
          "completion holds center without repeating");
    for (double t = 0; t < 46; t += 0.0073) {
        const auto sample = profile.sample(t);
        check(std::abs(sample.offset_rad) <= profile.amplitude_rad, "amplitude bound");
        check(std::abs(sample.velocity_rad_s) <= profile.max_velocity_rad_s, "reference speed bound");
        const double derivative = (profile.sample(t + 1e-6).offset_rad
                                   - profile.sample(t - 1e-6).offset_rad) / 2e-6;
        check(std::abs(derivative - sample.velocity_rad_s) < 1e-5,
              "analytic reference velocity matches irregular-time samples");
    }
    auto invalid = profile;
    invalid.ramp_s = 0;
    bool rejected = false;
    try { invalid.validate(); } catch (const std::invalid_argument&) { rejected = true; }
    check(rejected, "invalid profile must be rejected");

    YawTestSequence sequence;
    sequence.configure(YawTestSequence::Config{}, profile);
    check(sequence.duration() > 230 && sequence.duration() < 250, "standard duration about four minutes");
    bool visited[10]{};
    double max_angle = 0;
    for (double t = 0; t < sequence.duration()+1; t += 0.0113) {
        const auto s = sequence.sample(t);
        visited[s.stage] = true;
        max_angle = std::max(max_angle,std::abs(s.angle));
        check(std::abs(s.velocity) <= profile.max_velocity_rad_s+1e-9, "sequence reference speed bounded");
        const auto before = sequence.sample(t-1e-6), after = sequence.sample(t+1e-6);
        check(std::abs((after.angle-before.angle)/2e-6-s.velocity) < 2e-5,
              "unwrapped angle derivative matches velocity");
        check(std::abs((after.velocity-before.velocity)/2e-6-s.acceleration) < 2e-4,
              "reference velocity derivative matches acceleration");
    }
    for (int stage = 1; stage <= 8; ++stage) check(visited[stage], "all protocol stages visited");
    check(max_angle > 2*std::numbers::pi, "multiple turns supported without angle clipping");
    for (double t : sequence.boundaries()) {
        const auto before = sequence.sample(t-1e-7), after = sequence.sample(t+1e-7);
        check(std::abs(after.angle-before.angle) < 1e-5, "angle continuous at boundaries");
        check(std::abs(after.velocity-before.velocity) < 1e-5, "velocity continuous at boundaries");
    }
    check(sequence.sample(sequence.duration()+100).angle == sequence.sample(sequence.duration()).angle,
          "completion holds final angle, no restart or return-to-zero jump");
    ManualYawTrajectory single, partitioned;
    single.update(2.0,2.0,3.0);
    for (double dt : {0.013,0.137,0.35,0.4,1.1}) partitioned.update(2.0,dt,3.0);
    check(std::abs(single.angle-partitioned.angle)<1e-10 && single.velocity==partitioned.velocity,
          "manual integration independent of sampling intervals");
    check(std::abs(single.angle-10.0/3)<1e-10, "manual trajectory exact ramp plus cruise integral");
    auto bad_config = YawTestSequence::Config{};
    bad_config.mixed_speeds.back() = 1;
    rejected = false;
    try { sequence.configure(bad_config,profile); } catch (const std::invalid_argument&) { rejected=true; }
    check(rejected, "mixed trajectory must finish at rest");

    rclcpp::init(argc, argv);
    {
        rmcs_executor::Component::initializing_component_name = "yaw_excitation_test";
        YawExcitation generator;
        rmcs_executor::Component::initializing_component_name = "gimbal_test";
        DeformableInfantryGimbalController controller;
        Switch left = Switch::DOWN, right = Switch::DOWN;
        rmcs_description::Tf tf;
        tf.set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(0.0);
        tf.set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(0.0);
        tf.set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
            Eigen::Quaterniond::Identity());
        Eigen::Vector2d joystick = Eigen::Vector2d::Zero();
        double pitch = 0, yaw_rate = 0, pitch_rate = 0;
        for (rmcs_executor::Component* component : {static_cast<rmcs_executor::Component*>(&generator),
                                                   static_cast<rmcs_executor::Component*>(&controller)}) {
            Executor::bind(*component, "/remote/switch/left", left);
            Executor::bind(*component, "/remote/switch/right", right);
            Executor::bind(*component, "/tf", tf);
        }
        Executor::bind(controller, "/remote/joystick/left", joystick);
        Executor::bind(generator, "/remote/joystick/left", joystick);
        Executor::bind(controller, "/gimbal/pitch/angle", pitch);
        Executor::bind(controller, "/gimbal/yaw/velocity_imu", yaw_rate);
        Executor::bind(controller, "/gimbal/pitch/velocity_imu", pitch_rate);
        const std::string prefix = "/gimbal/yaw/excitation/";
        auto& selected = Executor::output<bool>(generator, prefix + "selected");
        auto& direction = Executor::output<Eigen::Vector3d>(generator, prefix + "direction");
        auto& state = Executor::output<double>(generator, prefix + "state");
        Executor::bind(controller, prefix + "selected", selected);
        Executor::bind(controller, prefix + "direction", direction);
        auto& session = Executor::output<double>(generator, prefix + "session_id");
        Executor::bind(controller, prefix + "session_id", session);
        auto step = [&] { generator.update(); controller.update(); };
        auto disabled = [&] {
            check(std::isnan(Executor::output<double>(controller, "/gimbal/yaw/control_torque")),
                  "yaw must be disabled");
            check(std::isnan(Executor::output<double>(controller, "/gimbal/pitch/control_torque")),
                  "pitch must be disabled");
        };
        step(); disabled(); check(state == 0, "double-down inactive");
        left = Switch::UP;
        step(); check(state == 1 && selected, "test entry waits");
        const double elevation = 5.0 * std::numbers::pi / 180.0;
        check(direction.isApprox(Eigen::Vector3d{std::cos(elevation), 0, std::sin(elevation)}),
              "wait holds yaw heading and commands five degrees upward");
        check(std::abs(Executor::output<double>(controller, "/gimbal/pitch/control_angle_error")
                       + elevation) < 1e-6, "upward target has negative pitch error");
        right = Switch::UNKNOWN;
        step(); disabled(); check(state == 0, "remote loss cancels wait");
        right = Switch::DOWN;
        step(); check(state == 1, "re-entry restarts delay");
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        step(); check(state == 2, "excitation begins after configured delay");
        check(std::abs(direction.z() - std::sin(elevation)) < 1e-12,
              "yaw sweep preserves five-degree elevation");
        check(std::isfinite(Executor::output<double>(controller, "/gimbal/yaw/control_torque")),
              "existing PID receives excitation");
        left = Switch::DOWN;
        // Controller must disable independently even before generator processes cancellation.
        controller.update(); disabled();
        step(); check(state == 0 && !selected, "double-down cancels active excitation");
        left = Switch::UP;
        step(); check(state == 1, "restart needs a fresh delay");
        std::this_thread::sleep_for(std::chrono::milliseconds(90));
        step(); check(state == 3, "test finishes once");
        check(std::abs(direction.z() - std::sin(elevation)) < 1e-12,
              "finished hold preserves five-degree elevation");
        step(); check(state == 3, "holding switch does not restart");
        const double old_session = session;
        left = Switch::MIDDLE;
        step(); check(selected && state==1 && session==old_session+1, "manual test starts fresh wait");
        joystick << 1.0, 0.6; // Pitch stick must not change the fixed elevation.
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        step(); check(state==2, "manual starts after delay");
        check(Executor::output<double>(generator,prefix+"stage")==9, "manual stage logged");
        check(Executor::output<double>(generator,prefix+"velocity_rad_s")>0, "manual yaw follows stick");
        check(std::abs(direction.z()-std::sin(elevation))<1e-12, "manual pitch stick ignored");
        left = Switch::DOWN;
        controller.update(); disabled();
        step(); check(!selected, "manual cancelled by double down");
        left = Switch::UP; right = Switch::MIDDLE;
        step(); check(!selected, "other switch combinations keep original controller");
        right = Switch::DOWN;
        step(); check(state==1 && selected, "automatic re-entry resets wait");
        right = Switch::UNKNOWN;
        step(); disabled();
    }
    rclcpp::shutdown();
    std::cout << "yaw excitation profile and controller integration checks passed\n";
}

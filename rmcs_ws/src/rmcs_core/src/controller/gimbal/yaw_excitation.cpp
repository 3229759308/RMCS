#include "controller/gimbal/yaw_test_sequence.hpp"
#include "controller/gimbal/yaw_single_loop_sequence.hpp"

#include <chrono>
#include <limits>
#include <string>

#include <eigen3/Eigen/Geometry>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::gimbal {

// Test reference generator; the gimbal controller owns motor commands and PID gains.
class YawExcitation : public rmcs_executor::Component, public rclcpp::Node {
public:
    YawExcitation()
        : Node(get_component_name(),
               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        get_parameter_or("enabled", enabled_, true);
        get_parameter_or("standard_test", standard_test_, true);
        get_parameter_or("delay_s", profile_.delay_s, profile_.delay_s);
        get_parameter_or("duration_s", profile_.duration_s, profile_.duration_s);
        get_parameter_or("ramp_s", profile_.ramp_s, profile_.ramp_s);
        get_parameter_or("amplitude_rad", profile_.amplitude_rad, profile_.amplitude_rad);
        get_parameter_or("start_frequency_hz", profile_.start_frequency_hz, profile_.start_frequency_hz);
        get_parameter_or("end_frequency_hz", profile_.end_frequency_hz, profile_.end_frequency_hz);
        get_parameter_or("max_velocity_rad_s", profile_.max_velocity_rad_s, profile_.max_velocity_rad_s);
        profile_.validate();
        get_parameter_or("pitch_up_deg", pitch_up_deg_, 5.0);
        get_parameter_or("manual_acceleration_rad_s2", manual_acceleration_, 3.0);
        get_parameter_or("manual_deadband", manual_deadband_, 0.02);
        if (!std::isfinite(pitch_up_deg_) || std::abs(pitch_up_deg_) >= 90.0
            || !std::isfinite(manual_acceleration_) || manual_acceleration_ <= 0
            || !std::isfinite(manual_deadband_) || manual_deadband_ < 0 || manual_deadband_ >= 1)
            throw std::invalid_argument("invalid pitch or manual trajectory parameters");
        YawTestSequence::Config config;
        get_parameter_or("initial_hold_s", config.initial_hold_s, config.initial_hold_s);
        get_parameter_or("final_hold_s", config.final_hold_s, config.final_hold_s);
        get_parameter_or("ladder_speeds_rad_s", config.ladder_speeds, config.ladder_speeds);
        get_parameter_or("ladder_hold_s", config.ladder_hold_s, config.ladder_hold_s);
        get_parameter_or("ladder_stop_s", config.ladder_stop_s, config.ladder_stop_s);
        get_parameter_or("ladder_acceleration_rad_s2", config.ladder_acceleration, config.ladder_acceleration);
        get_parameter_or("acceleration_levels_rad_s2", config.accelerations, config.accelerations);
        get_parameter_or("acceleration_speed_rad_s", config.acceleration_speed, config.acceleration_speed);
        get_parameter_or("acceleration_hold_s", config.acceleration_hold_s, config.acceleration_hold_s);
        get_parameter_or("acceleration_repeats", config.acceleration_repeats, config.acceleration_repeats);
        get_parameter_or("mixed_speeds_rad_s", config.mixed_speeds, config.mixed_speeds);
        get_parameter_or("mixed_transition_s", config.mixed_transition_s, config.mixed_transition_s);
        get_parameter_or("mixed_hold_s", config.mixed_hold_s, config.mixed_hold_s);
        // Sweep-only compatibility mode does not need a standard sequence.
        if (standard_test_) sequence_.configure(config, profile_);
        test_left_ = switch_parameter("switch_left", "up");
        test_right_ = switch_parameter("switch_right", "down");
        manual_left_ = switch_parameter("manual_switch_left", "middle");
        manual_right_ = switch_parameter("manual_switch_right", "down");
        if ((test_left_ == manual_left_ && test_right_ == manual_right_)
            || double_down(test_left_,test_right_) || double_down(manual_left_,manual_right_))
            throw std::invalid_argument("test switches must be distinct and cannot be double-down");

        get_parameter_or("supplement_enabled", supplement_enabled_, false);
        get_parameter_or("supplement_torque_nm", supplement_torque_, 2.5);
        get_parameter_or("supplement_peak_torque_nm", supplement_peak_torque_, 4.5);
        get_parameter_or("supplement_velocity_rad_s", supplement_velocity_, 1.0);
        for (double value : {supplement_torque_, supplement_peak_torque_, supplement_velocity_})
            if (!std::isfinite(value) || value <= 0)
                throw std::invalid_argument("supplement amplitudes must be finite and positive");
        const auto conflicts = [](auto left, auto right) {
            return right == rmcs_msgs::Switch::MIDDLE
                && (left == rmcs_msgs::Switch::UP || left == rmcs_msgs::Switch::MIDDLE);
        };
        if (supplement_enabled_ && (conflicts(test_left_, test_right_)
                                   || conflicts(manual_left_, manual_right_)))
            throw std::invalid_argument("supplement switch conflicts with existing test");
        register_output(prefix_+"torque_reference_nm", torque_reference_, 0.0);
        register_input("/remote/switch/left", left_);
        register_input("/remote/switch/right", right_);
        register_input("/remote/joystick/left", joystick_);
        register_input("/tf", tf_);
        register_output(prefix_+"selected", selected_, false);
        register_output(prefix_+"direction", direction_, Eigen::Vector3d::UnitX().eval());
        register_output(prefix_+"state", state_, 0.0);
        register_output(prefix_+"elapsed_s", elapsed_, 0.0);
        register_output(prefix_+"offset_rad", offset_, 0.0);
        register_output(prefix_+"velocity_rad_s", velocity_, 0.0);
        register_output(prefix_+"acceleration_rad_s2", acceleration_, 0.0);
        register_output(prefix_+"frequency_hz", frequency_, 0.0);
        register_output(prefix_+"mode", mode_output_, 0.0); // 0 off, 1 auto, 2 manual, 3 torque, 4 velocity
        register_output(prefix_+"stage", stage_, 0.0);
        register_output(prefix_+"segment", segment_, 0.0);
        register_output(prefix_+"stage_elapsed_s", stage_elapsed_, 0.0);
        register_output(prefix_+"validation_segment", validation_segment_, 0.0);
        register_output(prefix_+"protocol_version", protocol_version_, supplement_enabled_ ? 5.0 : standard_test_ ? 2.0 : 1.0);
        register_output(prefix_+"session_id", session_id_, 0.0);
        register_output(prefix_+"reference_yaw_rad", reference_yaw_, kNaN);
        register_output(prefix_+"pitch_target_up_deg", pitch_target_, pitch_up_deg_);
        register_output(prefix_+"pitch_actual_up_deg", pitch_actual_, kNaN);
        register_output(prefix_+"planned_duration_s", planned_duration_, 0.0);
        // Numeric mirrors: ValueCollector supports only double signals.
        register_output(prefix_+"switch_left", logged_left_, 0.0);
        register_output(prefix_+"switch_right", logged_right_, 0.0);
    }

    void update() override { update_at(std::chrono::steady_clock::now()); }

    // Explicit monotonic timestamp also permits deterministic hardware-free protocol tests.
    void update_at(std::chrono::steady_clock::time_point now) {
        *logged_left_ = static_cast<double>(*left_);
        *logged_right_ = static_cast<double>(*right_);
        const auto current = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, *tf_);
        const bool attitude_valid = current->allFinite() && current->head<2>().norm() >= 1e-6;
        *pitch_actual_ = attitude_valid
            ? std::asin(std::clamp(current->normalized().z(), -1.0, 1.0))*180/std::numbers::pi : kNaN;
        *torque_reference_ = 0;
        const int requested = !enabled_ ? 0
            : supplement_enabled_ && *right_ == rmcs_msgs::Switch::MIDDLE
                && *left_ == rmcs_msgs::Switch::UP ? 3
            : supplement_enabled_ && *right_ == rmcs_msgs::Switch::MIDDLE
                && *left_ == rmcs_msgs::Switch::MIDDLE ? 4
            : (*left_ == test_left_ && *right_ == test_right_) ? 1
            : (*left_ == manual_left_ && *right_ == manual_right_) ? 2 : 0;
        if (!requested) {
            mode_ = 0;
            *selected_ = false;
            *state_ = *elapsed_ = *offset_ = *velocity_ = *acceleration_ = *frequency_ = 0;
            *stage_ = *segment_ = *stage_elapsed_ = *mode_output_ = *validation_segment_ = 0;
            *reference_yaw_ = kNaN;
            *planned_duration_ = 0;
            return;
        }
        if (requested != mode_) {
            mode_ = requested;
            *mode_output_ = mode_;
            *selected_ = true;
            ++*session_id_;
            *state_ = 1;
            *elapsed_ = *offset_ = *velocity_ = *acceleration_ = *frequency_ = 0;
            *stage_ = 1; *segment_ = *stage_elapsed_ = *validation_segment_ = 0;
            *reference_yaw_ = kNaN;
            started_at_ = now;
            previous_elapsed_ = 0;
            manual_ = {};
            *planned_duration_ = mode_ == 1
                ? (standard_test_ ? sequence_.duration() : profile_.delay_s+profile_.duration_s)
                : mode_ >= 3 ? profile_.delay_s+single_loop_sequence().duration() : 0;
            if (!attitude_valid) { fault(); return; }
            const double elevation = pitch_up_deg_*std::numbers::pi/180;
            const auto heading = current->head<2>().normalized().eval();
            initial_yaw_ = std::atan2(heading.y(),heading.x());
            initial_direction_ = {std::cos(elevation)*heading.x(),
                                  std::cos(elevation)*heading.y(),std::sin(elevation)};
        }
        if (*state_ == 4) return; // Fault remains latched until leaving/changing test mode.
        if (!attitude_valid) { fault(); return; }
        *elapsed_ = std::chrono::duration<double>(now-started_at_).count();
        if (mode_ >= 3) {
            const double t = *elapsed_-profile_.delay_s;
            const auto s = single_loop_sequence().sample(t);
            *state_ = t < 0 ? 1 : t >= single_loop_sequence().duration() ? 3 : 2;
            *stage_ = t < 0 ? 1 : s.stage;
            *segment_ = s.segment; *stage_elapsed_ = t < 0 ? *elapsed_ : s.stage_elapsed;
            *acceleration_ = kNaN; // Step commands have no finite acceleration reference.
            *frequency_ = s.frequency; *validation_segment_ = s.stage == 15 ? 1 : 0;
            *torque_reference_ = mode_ == 3
                ? s.value*(s.stage == 18 ? supplement_peak_torque_ : supplement_torque_) : 0;
            *velocity_ = mode_ == 4 ? s.value*supplement_velocity_ : 0;
            // Follow measured heading for pitch solving; yaw position is bypassed.
            initial_yaw_ = std::atan2(current->y(), current->x());
            const double elevation = pitch_up_deg_*std::numbers::pi/180;
            initial_direction_ = {std::cos(elevation)*std::cos(initial_yaw_),
                                  std::cos(elevation)*std::sin(initial_yaw_), std::sin(elevation)};
        } else if (mode_ == 1 && standard_test_) {
            const auto s = sequence_.sample(*elapsed_);
            *state_ = s.state; *offset_ = s.angle; *velocity_ = s.velocity;
            *acceleration_ = s.acceleration; *frequency_ = s.frequency;
            *stage_ = s.stage; *segment_ = s.segment; *stage_elapsed_ = s.stage_elapsed;
            *validation_segment_ = s.stage == 6 ? 1.0 : 0.0;
        } else if (mode_ == 1) {
            const auto s = profile_.sample(*elapsed_);
            *state_ = s.state; *offset_ = s.offset_rad; *velocity_ = s.velocity_rad_s;
            *acceleration_ = s.acceleration_rad_s2; *frequency_ = s.frequency_hz;
            *stage_ = s.state == 1 ? 1 : s.state == 2 ? 5 : 8;
            *stage_elapsed_ = std::max(0.0,*elapsed_-profile_.delay_s);
        } else {
            if (!joystick_->allFinite()) { fault(); return; }
            const bool waiting = *elapsed_ < profile_.delay_s;
            const double stick = std::clamp(joystick_->y(),-1.0,1.0);
            const double target = std::abs(stick) < manual_deadband_ ? 0
                : std::copysign((std::abs(stick)-manual_deadband_)/(1-manual_deadband_),stick)
                  * profile_.max_velocity_rad_s;
            if (!waiting)
                manual_.update(target,*elapsed_-std::max(previous_elapsed_,profile_.delay_s),manual_acceleration_);
            *state_ = waiting ? 1 : 2;
            *stage_ = waiting ? 1 : 9;
            *stage_elapsed_ = waiting ? *elapsed_ : *elapsed_-profile_.delay_s;
            *offset_ = manual_.angle; *velocity_ = manual_.velocity; *acceleration_ = manual_.acceleration;
        }
        previous_elapsed_ = *elapsed_;
        *reference_yaw_ = mode_ >= 3 ? kNaN : initial_yaw_+*offset_;
        *direction_ = Eigen::AngleAxisd{std::remainder(*offset_,2*std::numbers::pi),Eigen::Vector3d::UnitZ()}
                    * initial_direction_;
    }

private:
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    static bool double_down(rmcs_msgs::Switch l, rmcs_msgs::Switch r) {
        return l == rmcs_msgs::Switch::DOWN && r == rmcs_msgs::Switch::DOWN;
    }
    rmcs_msgs::Switch switch_parameter(const std::string& name, const std::string& fallback) {
        std::string value;
        get_parameter_or(name,value,fallback);
        if (value == "up") return rmcs_msgs::Switch::UP;
        if (value == "middle") return rmcs_msgs::Switch::MIDDLE;
        if (value == "down") return rmcs_msgs::Switch::DOWN;
        throw std::invalid_argument("test switches must be up, middle or down");
    }
    void fault() {
        *state_ = 4;
        *torque_reference_ = 0;
        *velocity_ = *acceleration_ = *frequency_ = 0;
        *reference_yaw_ = kNaN;
        direction_->setConstant(kNaN); // Controller clears both axes' commands.
    }
    using Clock = std::chrono::steady_clock;
    const std::string prefix_ = "/gimbal/yaw/excitation/";
    YawExcitationProfile profile_;
    YawTestSequence sequence_;
    const YawSingleLoopSequence& single_loop_sequence() const {
        return mode_ == 4 ? velocity_sequence_ : torque_sequence_;
    }
    YawSingleLoopSequence torque_sequence_, velocity_sequence_{true};
    bool supplement_enabled_ = false;
    double supplement_torque_ = 2.5, supplement_peak_torque_ = 4.5, supplement_velocity_ = 1;
    OutputInterface<double> torque_reference_;
    ManualYawTrajectory manual_;
    bool enabled_ = true, standard_test_ = true;
    int mode_ = 0;
    double pitch_up_deg_ = 5, manual_acceleration_ = 3, manual_deadband_ = .02;
    double initial_yaw_ = 0, previous_elapsed_ = 0;
    rmcs_msgs::Switch test_left_, test_right_, manual_left_, manual_right_;
    Clock::time_point started_at_;
    Eigen::Vector3d initial_direction_ = Eigen::Vector3d::UnitX();
    InputInterface<rmcs_msgs::Switch> left_, right_;
    InputInterface<Eigen::Vector2d> joystick_;
    InputInterface<rmcs_description::Tf> tf_;
    OutputInterface<bool> selected_;
    OutputInterface<Eigen::Vector3d> direction_;
    OutputInterface<double> state_, elapsed_, offset_, velocity_, acceleration_, frequency_;
    OutputInterface<double> mode_output_, stage_, segment_, stage_elapsed_, validation_segment_;
    OutputInterface<double> protocol_version_, session_id_, reference_yaw_, pitch_target_, pitch_actual_;
    OutputInterface<double> planned_duration_, logged_left_, logged_right_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::YawExcitation, rmcs_executor::Component)

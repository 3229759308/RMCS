#include <cmath>
#include <chrono>
#include <algorithm>
#include <cstdlib>
#include <limits>
#include <memory>
#include <numbers>
#include <string>
#include <stdexcept>

#include <eigen3/Eigen/Dense>
#include <fmt/chrono.h>
#include <fmt/format.h>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/shoot_mode.hpp>
#include <rmcs_msgs/shoot_status.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/fps_counter.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller {

class GantryController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GantryController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        horizontal_max_velocity_ = get_parameter("horizontal_max_velocity").as_double();
        up_max_velocity_ = get_parameter("up_max_velocity").as_double();
        joystick_deadzone_ = get_parameter("joystick_deadzone").as_double();
        if (!std::isfinite(horizontal_max_velocity_) || horizontal_max_velocity_ <= 0.0
            || !std::isfinite(up_max_velocity_) || up_max_velocity_ <= 0.0
            || !std::isfinite(joystick_deadzone_) || joystick_deadzone_ < 0.0
            || joystick_deadzone_ >= 1.0)
            throw std::runtime_error("Invalid gantry velocity limits or joystick deadzone");

        // Defaults live here so existing launch/config files need no changes.
        const auto parameter = [this](const char* name, double value) {
            if (!has_parameter(name))
                declare_parameter<double>(name, value);
            return get_parameter(name).as_double();
        };
        homing_speed_ = parameter("homing_speed", 1.0);
        homing_velocity_threshold_ = parameter("homing_velocity_threshold", 0.02);
        homing_torque_threshold_ = parameter("homing_torque_threshold", 0.01);
        homing_confirm_time_ = parameter("homing_confirm_time", 0.1);
        homing_timeout_ = parameter("homing_timeout", 30.0);
        if (!std::isfinite(homing_speed_) || homing_speed_ <= 0.0
            || homing_speed_ > horizontal_max_velocity_
            || !std::isfinite(homing_velocity_threshold_) || homing_velocity_threshold_ <= 0.0
            || homing_velocity_threshold_ >= homing_speed_
            || !std::isfinite(homing_torque_threshold_) || homing_torque_threshold_ <= 0.0
            || !std::isfinite(homing_confirm_time_) || homing_confirm_time_ <= 0.0
            || !std::isfinite(homing_timeout_) || homing_timeout_ <= homing_confirm_time_)
            throw std::runtime_error("Invalid gantry homing parameters");

        pitch_target_rate_ = parameter("pitch_target_rate", 0.1);
        pitch_max_offset_ = parameter("pitch_max_offset", 0.5);
        pitch_velocity_direction_ = parameter("pitch_velocity_direction", 1.0);
        if (!std::isfinite(pitch_target_rate_) || pitch_target_rate_ <= 0.0
            || !std::isfinite(pitch_max_offset_) || pitch_max_offset_ <= 0.0
            || pitch_max_offset_ >= std::numbers::pi
            || (pitch_velocity_direction_ != 1.0 && pitch_velocity_direction_ != -1.0))
            throw std::runtime_error("Invalid gantry pitch parameters");

        sync_kp_ = get_parameter("sync_kp").as_double();
        max_sync_velocity_ = get_parameter("max_sync_velocity").as_double();
        if (!std::isfinite(sync_kp_) || sync_kp_ < 0.0
            || !std::isfinite(max_sync_velocity_) || max_sync_velocity_ < 0.0)
            throw std::runtime_error("Invalid gantry synchronization parameters");

        register_input("/dart/left_motor/velocity", left_motor_velocity_);
        register_input("/dart/left_motor/angle", left_motor_angle_);
        register_input("/dart/left_motor/torque", left_motor_torque_);
        register_output( "/gantry/left_motor/requested_velocity", left_motor_control_velocity_, nan_);

        register_input("/dart/right_motor/velocity", right_motor_velocity_);
        register_input("/dart/right_motor/angle", right_motor_angle_);
        register_input("/dart/right_motor/torque", right_motor_torque_);
        register_output( "/gantry/right_motor/requested_velocity", right_motor_control_velocity_, nan_);

        register_input("/dart/up_motor/velocity", up_motor_velocity_);
        register_input("/dart/up_motor/angle", up_motor_angle_);
        register_input("/dart/up_motor/torque", up_motor_torque_);
        register_output( "/dart/up_motor/control_velocity", up_motor_control_velocity_, nan_);

        register_input("/gantry/imu/roll", imu_roll_);
        register_input("/predefined/update_rate", update_rate_);
        register_output("/gantry/pitch/measurement", pitch_measurement_, nan_);
        register_output("/gantry/pitch/target", pitch_target_, nan_);
        register_output("/gantry/pitch/enabled", pitch_enabled_, false);

        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        velocity_output_ = create_partner_component<VelocityOutput>(
            get_component_name() + "_velocity_output", horizontal_max_velocity_, pitch_velocity_direction_);

    }

    void update() override {
        using rmcs_msgs::Switch;
        *pitch_enabled_ = false;
        *pitch_measurement_ = nan_;
        *pitch_target_ = nan_;
        const bool pitch_mode = *switch_left_ == Switch::UP && *switch_right_ == Switch::MIDDLE;
        if (!pitch_mode)
            pitch_active_ = false;

        if (!joystick_left_->allFinite() || !joystick_right_->allFinite() || *switch_left_ == Switch::UNKNOWN ||*switch_right_ == Switch::UNKNOWN ||
            (*switch_left_ == Switch::DOWN &&*switch_right_ == Switch::DOWN)) {
            homing_active_ = false;
            pitch_active_ = false;
            both_up_previous_ = (*switch_left_ == Switch::UP && *switch_right_ == Switch::UP);
            *left_motor_control_velocity_ = nan_;
            *right_motor_control_velocity_ = nan_;
            *up_motor_control_velocity_ = nan_;
            return;
        }

        const bool both_up = *switch_left_ == Switch::UP && *switch_right_ == Switch::UP;
        if (both_up && !both_up_previous_) {
            left_homing_ = {};
            right_homing_ = {};
            homing_started_ = Clock::now();
            homing_active_ = true;
            RCLCPP_INFO(logger_, "Gantry homing started at %.3f rad/s downward", homing_speed_);
        }
        both_up_previous_ = both_up;
        if (both_up) {
            update_homing();
            return;
        }
        // Leaving both-UP cancels an incomplete attempt without changing the last zero pair.
        homing_active_ = false;

        // Manual modes remain available before homing, but synchronized mode needs valid zeros.
        const bool both_middle = *switch_left_ == Switch::MIDDLE && *switch_right_ == Switch::MIDDLE;
        if ((both_middle || pitch_mode) && !zero_valid_) {
            pitch_active_ = false;
            *left_motor_control_velocity_ = 0.0;
            *right_motor_control_velocity_ = 0.0;
            *up_motor_control_velocity_ = 0.0;
            return;
        }

        // The remote interface exposes normalized x/y axes in [-1, 1].
        const auto apply_deadzone = [this](double value) {
            value = std::clamp(value, -1.0, 1.0);
            if (std::abs(value) <= joystick_deadzone_)
                return 0.0;
            return std::copysign(
                (std::abs(value) - joystick_deadzone_) / (1.0 - joystick_deadzone_), value);
        };
        if (pitch_mode) {
            update_pitch(apply_deadzone(joystick_right_->x()));
            return;
        }

        // Left stick trims each motor independently; right stick commands common motion.
        // Reverse both right-stick axes to match the mechanism's physical direction.
        const double common_command = -apply_deadzone(joystick_right_->x());
        *left_motor_control_velocity_ = std::clamp(
            common_command + apply_deadzone(joystick_left_->x()), -1.0, 1.0)
            * horizontal_max_velocity_;
        *right_motor_control_velocity_ = std::clamp(
            common_command + apply_deadzone(joystick_left_->y()), -1.0, 1.0)
            * horizontal_max_velocity_;
        *up_motor_control_velocity_ = -apply_deadzone(joystick_right_->y()) * up_max_velocity_;

        // Both motor angles must increase in the same direction of gantry travel.
        // Use continuous output-shaft angles, not wrapped single-turn differences.
        if (both_middle)
            apply_sync_correction();
    }  

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    // Separate output stage gives the executor an acyclic, same-cycle chain:
    // GantryController -> pitch PID -> VelocityOutput -> motor velocity PIDs.
    class VelocityOutput : public rmcs_executor::Component {
    public:
        VelocityOutput(double limit, double direction) : limit_(limit), direction_(direction) {
            register_input("/gantry/left_motor/requested_velocity", left_request_);
            register_input("/gantry/right_motor/requested_velocity", right_request_);
            register_input("/gantry/pitch/enabled", pitch_enabled_);
            register_input("/gantry/pitch/control_velocity", pitch_velocity_);
            register_output("/dart/left_motor/control_velocity", left_output_, nan_);
            register_output("/dart/right_motor/control_velocity", right_output_, nan_);
        }

        void update() override {
            const double common = *pitch_enabled_ ? direction_ * *pitch_velocity_ : 0.0;
            *left_output_ = mix(*left_request_, common);
            *right_output_ = mix(*right_request_, common);
        }

    private:
        double mix(double request, double common) const {
            if (!std::isfinite(request) || !std::isfinite(common))
                return nan_;
            return std::clamp(request + common, -limit_, limit_);
        }
        double limit_, direction_;
        InputInterface<double> left_request_, right_request_, pitch_velocity_;
        InputInterface<bool> pitch_enabled_;
        OutputInterface<double> left_output_, right_output_;
    };

    void apply_sync_correction() {
        if (!std::isfinite(*left_motor_angle_) || !std::isfinite(*right_motor_angle_)) {
            *left_motor_control_velocity_ = nan_;
            *right_motor_control_velocity_ = nan_;
            return;
        }
        const double sync_error = (*left_motor_angle_ - left_zero_angle_)
                                - (*right_motor_angle_ - right_zero_angle_);
        const double correction = std::clamp(
            sync_kp_ * sync_error, -max_sync_velocity_, max_sync_velocity_);
        *left_motor_control_velocity_ = std::clamp(
            *left_motor_control_velocity_ - correction,
            -horizontal_max_velocity_, horizontal_max_velocity_);
        *right_motor_control_velocity_ = std::clamp(
            *right_motor_control_velocity_ + correction,
            -horizontal_max_velocity_, horizontal_max_velocity_);
    }

    void update_pitch(double joystick) {
        *left_motor_control_velocity_ = 0.0;
        *right_motor_control_velocity_ = 0.0;
        *up_motor_control_velocity_ = 0.0;
        if (!std::isfinite(*imu_roll_) || !std::isfinite(*left_motor_angle_)
            || !std::isfinite(*right_motor_angle_) || !std::isfinite(*update_rate_)
            || *update_rate_ <= 0.0) {
            pitch_active_ = false;
            *left_motor_control_velocity_ = nan_;
            *right_motor_control_velocity_ = nan_;
            return;
        }
        if (!pitch_active_) {
            pitch_origin_ = pitch_feedback_ = pitch_target_angle_ = *imu_roll_;
            pitch_active_ = true;
        } else {
            // Keep roll continuous across +/-pi; the motor sync angles are already multi-turn.
            pitch_feedback_ += std::remainder(*imu_roll_ - last_imu_roll_, 2.0 * std::numbers::pi);
            pitch_target_angle_ = std::clamp(
                pitch_target_angle_ + joystick * pitch_target_rate_ / *update_rate_,
                pitch_origin_ - pitch_max_offset_, pitch_origin_ + pitch_max_offset_);
        }
        last_imu_roll_ = *imu_roll_;
        *pitch_measurement_ = pitch_feedback_;
        *pitch_target_ = pitch_target_angle_;
        *pitch_enabled_ = true;
        apply_sync_correction();
    }

    using Clock = std::chrono::steady_clock;
    struct HomingMotor {
        bool moved = false;
        bool confirming = false;
        bool done = false;
        Clock::time_point stopped_since{};
        double zero = 0.0;
    };

    void update_homing_motor(
        HomingMotor& state, double velocity, double torque, double angle,
        OutputInterface<double>& command, Clock::time_point now) {
        if (state.done) {
            *command = 0.0;
            return;
        }
        *command = -homing_speed_;
        // Require observed downward motion first: startup standstill is not a zero.
        if (velocity < -homing_velocity_threshold_)
            state.moved = true;
        if (!state.moved || std::abs(velocity) > homing_velocity_threshold_
            || std::abs(torque) < homing_torque_threshold_) {
            state.confirming = false;
            return;
        }
        if (!state.confirming) {
            state.confirming = true;
            state.stopped_since = now;
        }
        if (std::chrono::duration<double>(now - state.stopped_since).count() >= homing_confirm_time_) {
            state.zero = angle;
            state.done = true;
            *command = 0.0;
        }
    }

    void update_homing() {
        *left_motor_control_velocity_ = 0.0;
        *right_motor_control_velocity_ = 0.0;
        *up_motor_control_velocity_ = 0.0;
        if (!homing_active_)
            return;
        const auto now = Clock::now();
        if (!std::isfinite(*left_motor_angle_) || !std::isfinite(*right_motor_angle_)
            || !std::isfinite(*left_motor_velocity_) || !std::isfinite(*right_motor_velocity_)
            || !std::isfinite(*left_motor_torque_) || !std::isfinite(*right_motor_torque_)
            || std::chrono::duration<double>(now - homing_started_).count() >= homing_timeout_) {
            homing_active_ = false;
            RCLCPP_WARN(logger_, "Gantry homing aborted: invalid feedback or timeout; re-enter both-UP to retry");
            return;
        }
        update_homing_motor(left_homing_, *left_motor_velocity_, *left_motor_torque_,
                            *left_motor_angle_, left_motor_control_velocity_, now);
        update_homing_motor(right_homing_, *right_motor_velocity_, *right_motor_torque_,
                            *right_motor_angle_, right_motor_control_velocity_, now);
        if (left_homing_.done && right_homing_.done) {
            left_zero_angle_ = left_homing_.zero;
            right_zero_angle_ = right_homing_.zero;
            zero_valid_ = true;
            homing_active_ = false;
            RCLCPP_INFO(logger_, "Gantry zero recorded: left=%.6f, right=%.6f rad",
                        left_zero_angle_, right_zero_angle_);
        }
    }

    std::shared_ptr<VelocityOutput> velocity_output_;
    InputInterface<double> imu_roll_, update_rate_;
    OutputInterface<double> pitch_measurement_, pitch_target_;
    OutputInterface<bool> pitch_enabled_;
    double pitch_target_rate_, pitch_max_offset_, pitch_velocity_direction_;
    double pitch_origin_ = 0.0, pitch_feedback_ = 0.0, pitch_target_angle_ = 0.0;
    double last_imu_roll_ = 0.0;
    bool pitch_active_ = false;

    rclcpp::Logger logger_;
    double homing_speed_;
    double homing_velocity_threshold_;
    double homing_torque_threshold_;
    double homing_confirm_time_;
    double homing_timeout_;
    HomingMotor left_homing_;
    HomingMotor right_homing_;
    Clock::time_point homing_started_{};
    bool homing_active_ = false;
    bool both_up_previous_ = false;
    bool zero_valid_ = false;
    double horizontal_max_velocity_;
    double up_max_velocity_;
    double joystick_deadzone_;
    double left_zero_angle_ = 0.0;
    double right_zero_angle_ = 0.0;
    double sync_kp_;
    double max_sync_velocity_;

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

    InputInterface<double> up_motor_velocity_;
    InputInterface<double> up_motor_angle_;
    InputInterface<double> up_motor_torque_;
    OutputInterface<double> up_motor_control_velocity_;
    



    InputInterface<double> left_motor_velocity_;
    InputInterface<double> left_motor_angle_;
    InputInterface<double> left_motor_torque_;
    OutputInterface<double> left_motor_control_velocity_;
    
    InputInterface<double> right_motor_velocity_;
    InputInterface<double> right_motor_angle_;
    InputInterface<double> right_motor_torque_;
    OutputInterface<double> right_motor_control_velocity_;
};

} // namespace rmcs_core::controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::GantryController, rmcs_executor::Component)

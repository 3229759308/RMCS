#include <cmath>
#include <chrono>
#include <numbers>
#include <algorithm>
#include <cstdlib>
#include <limits>
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

#include "controller/pid/pid_calculator.hpp"

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
        pitch_pid_ = pid::make_pid_calculator(*this, "pitch_");
        pitch_target_rate_ = get_parameter("pitch_target_rate").as_double();
        pitch_target_span_ = get_parameter("pitch_target_span").as_double();
        pitch_motor_direction_ = get_parameter("pitch_motor_direction").as_double();
        if (!std::isfinite(pitch_target_rate_) || pitch_target_rate_ <= 0.0
            || !std::isfinite(pitch_target_span_) || pitch_target_span_ <= 0.0
            || pitch_target_span_ >= std::numbers::pi
            || (pitch_motor_direction_ != 1.0 && pitch_motor_direction_ != -1.0))
            throw std::runtime_error("Invalid pitch target rate, span or motor direction");
        register_input("/gantry/imu/pitch", imu_pitch_);
        register_input("/predefined/timestamp", timestamp_);
        register_output("/gantry/pitch/target", pitch_target_output_, nan_);
        register_output("/gantry/pitch/error", pitch_error_output_, nan_);

        left_velocity_pid_ = pid::make_pid_calculator(*this, "left_velocity_");
        right_velocity_pid_ = pid::make_pid_calculator(*this, "right_velocity_");
        up_velocity_pid_ = pid::make_pid_calculator(*this, "up_velocity_");
        register_output("/dart/left_motor/control_torque", left_motor_control_torque_, nan_);
        register_output("/dart/right_motor/control_torque", right_motor_control_torque_, nan_);
        register_output("/dart/up_motor/control_torque", up_motor_control_torque_, nan_);

        horizontal_max_velocity_ = get_parameter("horizontal_max_velocity").as_double();
        up_max_velocity_ = get_parameter("up_max_velocity").as_double();
        joystick_deadzone_ = get_parameter("joystick_deadzone").as_double();
        if (!std::isfinite(horizontal_max_velocity_) || horizontal_max_velocity_ <= 0.0
            || !std::isfinite(up_max_velocity_) || up_max_velocity_ <= 0.0
            || !std::isfinite(joystick_deadzone_) || joystick_deadzone_ < 0.0
            || joystick_deadzone_ >= 1.0)
            throw std::runtime_error("Invalid gantry velocity limits or joystick deadzone");

        left_zero_angle_ = get_parameter("left_zero_angle").as_double();
        right_zero_angle_ = get_parameter("right_zero_angle").as_double();
        sync_kp_ = get_parameter("sync_kp").as_double();
        sync_velocity_kp_ = get_parameter("sync_velocity_kp").as_double();
        max_sync_velocity_ = get_parameter("max_sync_velocity").as_double();
        if (!std::isfinite(left_zero_angle_) || !std::isfinite(right_zero_angle_)
            || !std::isfinite(sync_kp_) || sync_kp_ < 0.0
            || !std::isfinite(sync_velocity_kp_) || sync_velocity_kp_ < 0.0
            || !std::isfinite(max_sync_velocity_) || max_sync_velocity_ < 0.0)
            throw std::runtime_error("Invalid gantry synchronization parameters");

        register_input("/dart/left_motor/velocity", left_motor_velocity_);
        register_input("/dart/left_motor/angle", left_motor_angle_);
        register_input("/dart/left_motor/torque", left_motor_torque_);
        register_output( "/dart/left_motor/control_velocity", left_motor_control_velocity_, nan_);

        register_input("/dart/right_motor/velocity", right_motor_velocity_);
        register_input("/dart/right_motor/angle", right_motor_angle_);
        register_input("/dart/right_motor/torque", right_motor_torque_);
        register_output( "/dart/right_motor/control_velocity", right_motor_control_velocity_, nan_);

        register_input("/dart/up_motor/velocity", up_motor_velocity_);
        register_input("/dart/up_motor/angle", up_motor_angle_);
        register_input("/dart/up_motor/torque", up_motor_torque_);
        register_output( "/dart/up_motor/control_velocity", up_motor_control_velocity_, nan_);

        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);

    }

    void update() override {
        update_velocity_targets();
        const auto calculate_torque = [](pid::PidCalculator& calculator, double target, double actual) {
            if (!std::isfinite(target) || !std::isfinite(actual)) {
                calculator.reset();
                return nan_;
            }
            return calculator.update(target - actual);
        };
        *left_motor_control_torque_ = calculate_torque(
            left_velocity_pid_, *left_motor_control_velocity_, *left_motor_velocity_);
        *right_motor_control_torque_ = calculate_torque(
            right_velocity_pid_, *right_motor_control_velocity_, *right_motor_velocity_);
        *up_motor_control_torque_ = calculate_torque(
            up_velocity_pid_, *up_motor_control_velocity_, *up_motor_velocity_);
    }

private:
    void update_velocity_targets() {
        using rmcs_msgs::Switch;

        if (!joystick_left_->allFinite() || !joystick_right_->allFinite() || *switch_left_ == Switch::UNKNOWN ||*switch_right_ == Switch::UNKNOWN ||
            (*switch_left_ == Switch::DOWN &&*switch_right_ == Switch::DOWN)) {
            reset_pitch();
            *left_motor_control_velocity_ = nan_;
            *right_motor_control_velocity_ = nan_;
            *up_motor_control_velocity_ = nan_;
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

        const bool automatic = *switch_left_ == Switch::MIDDLE && *switch_right_ == Switch::MIDDLE;
        if (!automatic)
            reset_pitch();

        // Both motor angles must increase in the same direction of gantry travel.
        // Use continuous output-shaft angles, not wrapped single-turn differences.
        if (automatic) {
            if (!std::isfinite(*left_motor_angle_) || !std::isfinite(*right_motor_angle_)
                || !std::isfinite(*left_motor_velocity_) || !std::isfinite(*right_motor_velocity_)
                || !std::isfinite(*imu_pitch_)) {
                reset_pitch();
                *left_motor_control_velocity_ = nan_;
                *right_motor_control_velocity_ = nan_;
                return;
            }
            double dt = 0.0;
            if (!pitch_active_) {
                // Capture the measured angle on each entry; no configured initial setpoint.
                pitch_measured_ = pitch_previous_raw_ = *imu_pitch_;
                pitch_target_ = pitch_reference_ = pitch_measured_;
                pitch_pid_.reset();
                left_velocity_pid_.reset();
                right_velocity_pid_.reset();
                pitch_active_ = true;
            } else {
                pitch_measured_ += std::remainder(
                    *imu_pitch_ - pitch_previous_raw_, 2.0 * std::numbers::pi);
                pitch_previous_raw_ = *imu_pitch_;
                dt = std::clamp(std::chrono::duration<double>(*timestamp_ - pitch_timestamp_).count(),
                                0.0, 0.01);
            }
            pitch_timestamp_ = *timestamp_;
            // Positive pitch demand maps to negative motor velocity with the current installation.
            pitch_target_ = std::clamp(
                pitch_target_ + pitch_motor_direction_ * common_command * pitch_target_rate_ * dt,
                pitch_reference_ - pitch_target_span_, pitch_reference_ + pitch_target_span_);
            *pitch_target_output_ = pitch_target_;
            *pitch_error_output_ = pitch_target_ - pitch_measured_;
            const double pitch_velocity = pitch_motor_direction_ * pitch_pid_.update(*pitch_error_output_);
            // Automatic mode owns both motors; independent left-stick trim remains manual-only.
            *left_motor_control_velocity_ = pitch_velocity;
            *right_motor_control_velocity_ = pitch_velocity;

            const double sync_error = (*left_motor_angle_ - left_zero_angle_)
                                    - (*right_motor_angle_ - right_zero_angle_);
            // Relative encoder displacement supplies the accumulated speed-difference term
            // without a separate drifting integrator or duplicate position compensation.
            const double speed_difference = *left_motor_velocity_ - *right_motor_velocity_;
            const double correction = std::clamp(
                sync_kp_ * sync_error + sync_velocity_kp_ * speed_difference,
                -max_sync_velocity_, max_sync_velocity_);
            *left_motor_control_velocity_ = std::clamp(
                *left_motor_control_velocity_ - correction,
                -horizontal_max_velocity_, horizontal_max_velocity_);
            *right_motor_control_velocity_ = std::clamp(
                *right_motor_control_velocity_ + correction,
                -horizontal_max_velocity_, horizontal_max_velocity_);
        }
    }

    void reset_pitch() {
        if (pitch_active_) {
            left_velocity_pid_.reset();
            right_velocity_pid_.reset();
        }
        pitch_active_ = false;
        pitch_pid_.reset();
        *pitch_target_output_ = nan_;
        *pitch_error_output_ = nan_;
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    pid::PidCalculator pitch_pid_;
    InputInterface<double> imu_pitch_;
    InputInterface<std::chrono::steady_clock::time_point> timestamp_;
    OutputInterface<double> pitch_target_output_, pitch_error_output_;
    bool pitch_active_ = false;
    double pitch_target_rate_, pitch_target_span_, pitch_motor_direction_;
    double pitch_measured_ = 0.0, pitch_previous_raw_ = 0.0;
    double pitch_target_ = 0.0, pitch_reference_ = 0.0;
    std::chrono::steady_clock::time_point pitch_timestamp_{};
    pid::PidCalculator left_velocity_pid_, right_velocity_pid_, up_velocity_pid_;
    OutputInterface<double> left_motor_control_torque_, right_motor_control_torque_, up_motor_control_torque_;
    rclcpp::Logger logger_;
    double horizontal_max_velocity_;
    double up_max_velocity_;
    double joystick_deadzone_;
    double left_zero_angle_;
    double right_zero_angle_;
    double sync_kp_;
    double sync_velocity_kp_;
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

#include <cmath>
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

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);

    }

    void update() override {
        using rmcs_msgs::Switch;

        if (!joystick_right_->allFinite() || *switch_left_ == Switch::UNKNOWN ||*switch_right_ == Switch::UNKNOWN ||
            (*switch_left_ == Switch::DOWN &&*switch_right_ == Switch::DOWN)) {
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
        const double horizontal_velocity =
            apply_deadzone(joystick_right_->x()) * horizontal_max_velocity_;
        *left_motor_control_velocity_ = horizontal_velocity;
        *right_motor_control_velocity_ = horizontal_velocity;
        *up_motor_control_velocity_ = apply_deadzone(joystick_right_->y()) * up_max_velocity_;
    }  

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    rclcpp::Logger logger_;
    double horizontal_max_velocity_;
    double up_max_velocity_;
    double joystick_deadzone_;

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

#include <cmath>

#include <limits>
#include <string>

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

class Task3MotorController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Task3MotorController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        
        max_velocity_ = get_parameter("max_velocity").as_double();
        register_input("/task3_motor/velocity", task3_motor_velocity_);
        register_output( "/task3_motor/control_velocity", task3_motor_control_velocity_, nan_);
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
    }

    void update() override {
        using rmcs_msgs::Switch;

        if (*switch_left_ == Switch::UNKNOWN ||*switch_right_ == Switch::UNKNOWN ||
            (*switch_left_ == Switch::DOWN &&*switch_right_ == Switch::DOWN)) {
            *task3_motor_control_velocity_ = nan_;
            return;
        }
        const double axis = joystick_right_->x();
        *task3_motor_control_velocity_ = axis * max_velocity_;

    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    rclcpp::Logger logger_;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

    double max_velocity_;

    InputInterface<double> task3_motor_velocity_;
    OutputInterface<double> task3_motor_control_velocity_;

    OutputInterface<double> filtered_velocity_;


};

} // namespace rmcs_core::controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::Task3MotorController, rmcs_executor::Component)

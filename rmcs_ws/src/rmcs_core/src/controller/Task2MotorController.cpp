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
#include "filter/low_pass_filter.hpp"

namespace rmcs_core::controller {

class Task2MotorController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Task2MotorController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        
        target_velocity_ = get_parameter("target_velocity").as_double();
        register_input("/task2_motor/velocity", task2_motor_velocity_);
        register_output( "/task2_motor/control_velocity", task2_motor_control_velocity_, nan_);
        register_output("/task2_motor/filtered_velocity", filtered_velocity_,nan_);
        }

    void update() override {
    *filtered_velocity_ =
        velocity_filter_.update(*task2_motor_velocity_);
    *task2_motor_control_velocity_ = target_velocity_;
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    rclcpp::Logger logger_;


    double target_velocity_;

    InputInterface<double> task2_motor_velocity_;
    OutputInterface<double> task2_motor_control_velocity_;

    rmcs_core::filter::LowPassFilter<1> velocity_filter_{0.1};
    OutputInterface<double> filtered_velocity_;


};

} // namespace rmcs_core::controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::Task2MotorController, rmcs_executor::Component)

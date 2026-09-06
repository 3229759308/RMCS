#include <atomic>
#include <cmath>
#include <limits>
#include <numbers>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/float64.hpp>

namespace rmcs_core::controller {

class Task3MotorController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Task3MotorController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}
                  .automatically_declare_parameters_from_overrides(true)) {

        register_input("/task3_motor/angle", task3_motor_angle_);

        register_output(
            "/task3_motor/control_angle",
            task3_motor_control_angle_,
            nan_);

        register_output(
            "/task3_motor/target_angle",
            target_angle_output_,
            nan_);

        command_angle_subscription_ =
            create_subscription<std_msgs::msg::Float64>(
                "/task3_motor/command_angle",
                rclcpp::QoS(1),
                [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
                    if (!std::isfinite(msg->data)) {
                        RCLCPP_WARN(
                            get_logger(),
                            "Ignoring non-finite target angle");
                        return;
                    }

                    command_angle_.store(
                        msg->data, std::memory_order_relaxed);
                });
    }

    void update() override {
        constexpr double full_turn = 2.0 * std::numbers::pi;
        constexpr double target_change_threshold = 1e-6;
        constexpr double same_angle_tolerance = 1e-6;

        const double actual_angle = *task3_motor_angle_;
        const double target_angle =
            command_angle_.load(std::memory_order_relaxed);

        // 未收到目标，或者反馈无效时，不进行控制。
        if (!std::isfinite(actual_angle) ||
            !std::isfinite(target_angle)) {
            *task3_motor_control_angle_ = nan_;
            *target_angle_output_ = nan_;
            goal_valid_ = false;
            return;
        }

        // 根据单圈反馈的变化量，更新本次行程剩余误差。
        if (goal_valid_) {
            const double moved =
                std::remainder(
                    actual_angle - previous_angle_, full_turn);

            remaining_error_ -= moved;
        }

        previous_angle_ = actual_angle;

        // 新目标才重新规划；重复发送相同目标不会重启行程。
        const bool new_target =
            !goal_valid_ ||
            std::abs(std::remainder(
                target_angle - accepted_target_, full_turn))
                > target_change_threshold;

        if (new_target) {
            const double short_error =
                std::remainder(
                    target_angle - actual_angle, full_turn);

            if (std::abs(short_error) <= same_angle_tolerance) {
                remaining_error_ = 0.0;
            } else if (short_error > 0.0) {
                remaining_error_ = short_error - full_turn;
            } else {
                remaining_error_ = short_error + full_turn;
            }

            accepted_target_ = target_angle;
            goal_valid_ = true;
        }

        // 剩余误差不能再归一化，否则会丢失优弧行程。
        *task3_motor_control_angle_ = remaining_error_;
        *target_angle_output_ = target_angle;
    }

private:
    static constexpr double nan_ =
        std::numeric_limits<double>::quiet_NaN();

    // 订阅回调与控制更新之间传递目标角度。
    std::atomic<double> command_angle_{nan_};

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        command_angle_subscription_;

    InputInterface<double> task3_motor_angle_;
    OutputInterface<double> task3_motor_control_angle_;
    OutputInterface<double> target_angle_output_;

    bool goal_valid_ = false;
    double previous_angle_ = 0.0;
    double accepted_target_ = 0.0;
    double remaining_error_ = 0.0;
};

} // namespace rmcs_core::controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::Task3MotorController,
    rmcs_executor::Component)
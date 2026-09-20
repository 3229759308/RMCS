#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <cerrno>
#include <cstddef>
#include <cstring>
#include <memory>
#include <span>
#include <utility>

#include <eigen3/Eigen/Dense>
#include <librmcs/board/c_board.hpp>
#include <librmcs/data/datas.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/board_clock.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/ring_buffer.hpp>
#include <std_msgs/msg/int32.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/remote_control.hpp"
#include "hardware/device/supercap.hpp"

namespace rmcs_core::hardware {

class GantryControlHardware
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {

public:
    GantryControlHardware()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
        , gantry_control_hardware_command_(
              create_partner_component<GantryControlHardwareCommand>(get_component_name() + "_command", *this))
        ,left_motor_(*this, *gantry_control_hardware_command_, "/dart/left_motor")
        ,right_motor_(*this, *gantry_control_hardware_command_, "/dart/right_motor")
        ,up_motor_(*this, *gantry_control_hardware_command_, "/dart/up_motor")
        ,dr16_{}
        {
        left_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 3}.enable_multi_turn_angle());

        right_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 2}.enable_multi_turn_angle());
        
        up_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 1}.enable_multi_turn_angle());
        
        // Gantry roll = sensor pitch, gantry pitch = sensor roll, yaw unchanged; radians.
        const auto unavailable = std::numeric_limits<double>::quiet_NaN();
        register_output("/gantry/imu/roll", imu_roll_, unavailable);
        register_output("/gantry/imu/pitch", imu_pitch_, unavailable);
        register_output("/gantry/imu/yaw", imu_yaw_, unavailable);

        board_ = std::make_unique<librmcs::board::CBoard>(
            *this, get_parameter("board_serial").as_string());

        remote_control_ = std::make_unique<device::RemoteControl>(*this);
        remote_control_->register_dr16(&dr16_);

        }
    GantryControlHardware(const GantryControlHardware&) = delete;
    GantryControlHardware& operator=(const GantryControlHardware&) = delete;
    GantryControlHardware(GantryControlHardware&&) = delete;
    GantryControlHardware& operator=(GantryControlHardware&&) = delete;

    ~GantryControlHardware() override = default;

    void update() override {
        update_motors();
        update_imu();
        
        dr16_.update_status();
        remote_control_->update();
    }

    void command_update() {
        auto builder = board_->start_transmit();

        builder.can_transmit(
            Spec::kCans.kCan1,
            {
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                        up_motor_.generate_command(),
                        right_motor_.generate_command(),
                        left_motor_.generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                    }
                        .as_bytes(),
            });
    }

private:
    void update_imu() {
        if (!imu_accelerometer_received_.load(std::memory_order_acquire)
            || !imu_gyroscope_received_.load(std::memory_order_acquire))
            return;

        // Match the 2025 implementation: one Mahony step per 1000 Hz executor update.
        bmi088_.update_status();
        const auto q = Eigen::Quaterniond{
            bmi088_.q0(), bmi088_.q1(), bmi088_.q2(), bmi088_.q3()}.normalized();
        // Swap output angle labels only, preserving sensor coordinates during fusion.
        *imu_pitch_ = std::atan2(
            2.0 * (q.w() * q.x() + q.y() * q.z()),
            1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
        *imu_roll_ = std::asin(std::clamp(
            2.0 * (q.w() * q.y() - q.z() * q.x()), -1.0, 1.0));
        *imu_yaw_ = std::atan2(
            2.0 * (q.w() * q.z() + q.x() * q.y()),
            1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    }

    void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
        bmi088_.store_accelerometer_status(data.x, data.y, data.z);
        imu_accelerometer_received_.store(true, std::memory_order_release);
    }

    void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
        bmi088_.store_gyroscope_status(data.x, data.y, data.z);
        imu_gyroscope_received_.store(true, std::memory_order_release);
    }

    void update_motors() {
    up_motor_.update_status();
    right_motor_.update_status();
    left_motor_.update_status();
}
    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;
        
        if (can == Spec::kCans.kCan1) {
            auto can_id = data.can_id;
            if (can_id == 0x201) {
                up_motor_.store_status(data.can_data);
           }
            if (can_id == 0x202) {
                right_motor_.store_status(data.can_data);
            }
            if (can_id == 0x203) {
                left_motor_.store_status(data.can_data);
            }
        }
    }

    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart == Spec::kUarts.kDbus) {
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
        }
    }

private:
    rclcpp::Logger logger_;


    class GantryControlHardwareCommand : public rmcs_executor::Component {
    public:
        explicit GantryControlHardwareCommand(GantryControlHardware& gantry_control_hardware)
            : gantry_control_hardware_(gantry_control_hardware) {}

        void update() override { gantry_control_hardware_.command_update(); }

    private:
        GantryControlHardware& gantry_control_hardware_;
    };
    std::shared_ptr<GantryControlHardwareCommand> gantry_control_hardware_command_;

    device::DjiMotor left_motor_;
    device::DjiMotor right_motor_;
    device::DjiMotor up_motor_;

    device::Dr16 dr16_;
    std::unique_ptr<device::RemoteControl> remote_control_;
    // Legacy Mahony gains; sample frequency matches gantry_control.yaml update_rate.
    device::Bmi088 bmi088_{1000.0, 0.2, 0.0};
    std::atomic<bool> imu_accelerometer_received_{false};
    std::atomic<bool> imu_gyroscope_received_{false};
    OutputInterface<double> imu_roll_;
    OutputInterface<double> imu_pitch_;
    OutputInterface<double> imu_yaw_;

    // Destroy the board first so callbacks stop before their state is destroyed.
    std::unique_ptr<librmcs::board::CBoard> board_;
};

}  // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::GantryControlHardware, rmcs_executor::Component)

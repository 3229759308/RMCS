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

#include "hardware/device/bmi088_ekf.hpp"
#include "hardware/device/board_clock_lifter.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/remote_control.hpp"
#include "hardware/device/supercap.hpp"

namespace rmcs_core::hardware {

class DartPitchHardware
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {

public:
    DartPitchHardware()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
        , dart_pitch_hardware_command_(
              create_partner_component<DartPitchHardwareCommand>(get_component_name() + "_command", *this))
        ,dart_pitch_hardware_left_motor_(*this, *dart_pitch_hardware_command_, "/dart/left_motor")
        ,dart_pitch_hardware_right_motor_(*this, *dart_pitch_hardware_command_, "/dart/right_motor")
        ,dr16_{}
        {
        dart_pitch_hardware_left_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}.set_reduction_ratio(1.).enable_multi_turn_angle());//未知id随便使用

        dart_pitch_hardware_right_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 2}.set_reduction_ratio(1.).enable_multi_turn_angle());
        
        
        board_ = std::make_unique<librmcs::board::CBoard>(
            *this, get_parameter("board_serial").as_string());

        remote_control_ = std::make_unique<device::RemoteControl>(*this);
        remote_control_->register_dr16(&dr16_);

        }
    DartPitchHardware(const DartPitchHardware&) = delete;
    DartPitchHardware& operator=(const DartPitchHardware&) = delete;
    DartPitchHardware(DartPitchHardware&&) = delete;
    DartPitchHardware& operator=(DartPitchHardware&&) = delete;

    ~DartPitchHardware() override = default;

    void update() override {
        update_motors();
        
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
                        dart_pitch_hardware_left_motor_.generate_command(),
                        dart_pitch_hardware_right_motor_.generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                    }
                        .as_bytes(),
            });
    }

private:
    void update_motors() {
    dart_pitch_hardware_left_motor_.update_status();
    dart_pitch_hardware_right_motor_.update_status();
}
    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;
        
        if (can == Spec::kCans.kCan1) {
            auto can_id = data.can_id;
            if (can_id == 0x201) {
                dart_pitch_hardware_left_motor_.store_status(data.can_data);
            }
            if (can_id == 0x202) {
                dart_pitch_hardware_right_motor_.store_status(data.can_data);
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

    std::unique_ptr<librmcs::board::CBoard> board_;

    class DartPitchHardwareCommand : public rmcs_executor::Component {
    public:
        explicit DartPitchHardwareCommand(DartPitchHardware& dart_pitch_hardware)
            : dart_pitch_hardware_(dart_pitch_hardware) {}

        void update() override { dart_pitch_hardware_.command_update(); }

    private:
        DartPitchHardware& dart_pitch_hardware_;
    };
    std::shared_ptr<DartPitchHardwareCommand> dart_pitch_hardware_command_;

    device::DjiMotor dart_pitch_hardware_left_motor_;
    device::DjiMotor dart_pitch_hardware_right_motor_;

    device::Dr16 dr16_;
    std::unique_ptr<device::RemoteControl> remote_control_;
};

}  // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DartPitchHardware, rmcs_executor::Component)

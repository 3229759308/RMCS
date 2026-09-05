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

class Task2Motor
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {
public:
    Task2Motor()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
        , task2_command_(
              create_partner_component<Task2Command>(get_component_name() + "_command", *this))
        ,task2_motor_(*this, *task2_command_, "/task2_motor") 
        ,dr16_{}
        {
        task2_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 3}.set_reduction_ratio(1.));
        
        board_ = std::make_unique<librmcs::board::CBoard>(
            *this, get_parameter("board_serial").as_string());

        remote_control_ = std::make_unique<device::RemoteControl>(*this);
        remote_control_->register_dr16(&dr16_);

        }
    Task2Motor(const Task2Motor&) = delete;
    Task2Motor& operator=(const Task2Motor&) = delete;
    Task2Motor(Task2Motor&&) = delete;
    Task2Motor& operator=(Task2Motor&&) = delete;

    ~Task2Motor() override = default;

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
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                        task2_motor_.generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                    }
                        .as_bytes(),
            });
    }

private:
    void update_motors() {
    task2_motor_.update_status();
}

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;
        
        if (can == Spec::kCans.kCan1) {
            auto can_id = data.can_id;
            if (can_id == 0x203) {
                task2_motor_.store_status(data.can_data);
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

    class Task2Command : public rmcs_executor::Component {
    public:
        explicit Task2Command(Task2Motor& task2_motor)
            : task2_motor_(task2_motor) {}

        void update() override { task2_motor_.command_update(); }

    private:
        Task2Motor& task2_motor_;
    };
    std::shared_ptr<Task2Command> task2_command_;

    device::DjiMotor task2_motor_;

    device::Dr16 dr16_;
    std::unique_ptr<device::RemoteControl> remote_control_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Task2Motor, rmcs_executor::Component)

#include <cmath>
#include <algorithm>
#include <cstdlib>
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

class DartPitchController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartPitchController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        register_input("/dart/left_motor/velocity", dart_left_motor_velocity_);
        register_input("/dart/left_motor/angle", dart_left_motor_angle_);
        register_input("/dart/left_motor/torque", dart_left_motor_torque_);
        register_output( "/dart/left_motor/control_velocity", dart_left_motor_control_velocity_, nan_);

        register_input("/dart/right_motor/velocity", dart_right_motor_velocity_);
        register_input("/dart/right_motor/angle", dart_right_motor_angle_);
        register_input("/dart/right_motor/torque", dart_right_motor_torque_);
        register_output( "/dart/right_motor/control_velocity", dart_right_motor_control_velocity_, nan_);
        

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);

        threshold_ = get_parameter("threshold").as_double();
        dart_velocity_ =get_parameter("dart_velocity").as_double();
        sync_kp_ =get_parameter("sync_kp").as_double();
        max_sync_velocity_ =get_parameter("max_sync_velocity").as_double();

    }

    void update() override {
        using rmcs_msgs::Switch;

        if (*switch_left_ == Switch::UNKNOWN ||*switch_right_ == Switch::UNKNOWN ||
            (*switch_left_ == Switch::DOWN &&*switch_right_ == Switch::DOWN)) {
            *dart_left_motor_control_velocity_ = nan_;
            *dart_right_motor_control_velocity_ = nan_;
            return;
        }

        else if (*switch_left_ == Switch::UP &&*switch_right_ == Switch::UP) {//双上找零
            dart_motor_find_zero();
            return;
        }

        if(is_dart_motor_found_zero_ ==DartMotorFoundZeroState::FOUND){
            double joystick = (*joystick_right_).x();
            if (std::abs(joystick) < 0.05){
                joystick = 0.0;
            }

            double target_velocity = joystick * dart_velocity_;
            double sync_error =(*dart_left_motor_angle_ - left_zero_angle) - (*dart_right_motor_angle_ - right_zero_angle);
            double sync_velocity =sync_kp_ * sync_error;
            if(std::abs(sync_error)<0.01){
                sync_velocity=0;
            }
            sync_velocity = std::clamp(sync_velocity,-max_sync_velocity_,max_sync_velocity_);
            *dart_left_motor_control_velocity_ = target_velocity - sync_velocity;
            *dart_right_motor_control_velocity_= target_velocity + sync_velocity;
            return;
        }
        *dart_left_motor_control_velocity_ = 0.0;
        *dart_right_motor_control_velocity_ = 0.0;

    }

private:
    void dart_motor_find_zero() {

        if (is_dart_motor_found_zero_ == DartMotorFoundZeroState::FOUND) {
            *dart_left_motor_control_velocity_ = 0.0;
            *dart_right_motor_control_velocity_ = 0.0;
            return;
        }

        if (is_dart_motor_found_zero_ == DartMotorFoundZeroState::NOT_FOUND) {
            is_dart_motor_found_zero_ = DartMotorFoundZeroState::FINDING;
            *dart_left_motor_control_velocity_ = 1.0;//无实物，不知道具体该给多少速度，反正是个小的速度
            *dart_right_motor_control_velocity_ = 1.0;
            return;
        }

        if (is_dart_motor_found_zero_ == DartMotorFoundZeroState::FINDING) {
            if(!isleft_zerofind){
            bool left_still = std::abs(*dart_left_motor_velocity_) < 0.1 &&std::abs(*dart_left_motor_torque_) > threshold_;
                if(!left_still)
                {
                    left_zero_count_ = 0;
                }

                if(left_still)
                {
                    left_zero_count_++;
                }

                if(left_zero_count_ >= 100){
                    *dart_left_motor_control_velocity_=0.0;
                    left_zero_angle=*dart_left_motor_angle_;
                    isleft_zerofind = true;
                }
            }

            if(!isright_zerofind){
            bool right_still = std::abs(*dart_right_motor_velocity_) < 0.1 &&std::abs(*dart_right_motor_torque_) > threshold_;
                if(!right_still)
                {
                    right_zero_count_ = 0;
                }

                if(right_still)
                {
                    right_zero_count_++;
                }

                if(right_zero_count_ >= 100){
                    *dart_right_motor_control_velocity_=0.0;
                    right_zero_angle=*dart_right_motor_angle_;
                    isright_zerofind = true;
                }
            }

            if(isleft_zerofind&&isright_zerofind){
                is_dart_motor_found_zero_ = DartMotorFoundZeroState::FOUND;
            }
        }
    }

    

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    rclcpp::Logger logger_;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

    enum class DartMotorFoundZeroState {
        NOT_FOUND,
        FINDING,
        FOUND
    };

    DartMotorFoundZeroState is_dart_motor_found_zero_ = DartMotorFoundZeroState::NOT_FOUND;
    double threshold_;
    double dart_velocity_;

    InputInterface<double> dart_left_motor_velocity_;
    InputInterface<double> dart_left_motor_angle_;
    InputInterface<double> dart_left_motor_torque_;
    OutputInterface<double> dart_left_motor_control_velocity_;
    bool isleft_zerofind=false;
    double left_zero_angle;
    int left_zero_count_ = 0;
    
    InputInterface<double> dart_right_motor_velocity_;
    InputInterface<double> dart_right_motor_angle_;
    InputInterface<double> dart_right_motor_torque_;
    OutputInterface<double> dart_right_motor_control_velocity_;
    bool isright_zerofind=false;
    double right_zero_angle;
    int right_zero_count_ = 0;

    double sync_kp_;//简易单p差速器
    double max_sync_velocity_;
};

} // namespace rmcs_core::controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::DartPitchController, rmcs_executor::Component)

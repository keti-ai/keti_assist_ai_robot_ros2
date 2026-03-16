#include "kaair_driver/tool/tool_hw_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pluginlib/class_list_macros.hpp>
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace kaair_driver {

    CallbackReturn ToolHwInterface::on_init(const hardware_interface::HardwareInfo & info) {
        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;

        clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

        // 1. 파라미터 로드 (urdf/ros2_control.xacro에서 설정한 값들)
        device_name_ = info.hardware_parameters.at("usb_port");
        baudrate_ = std::stoi(info.hardware_parameters.at("baud_rate"));
        
        // 2. 조인트 개수만큼 버퍼 초기화
        hw_states_.resize(info.joints.size(), 0.0);
        hw_commands_.resize(info.joints.size(), 0.0);
        
        // 3. 다이나믹셀 객체 생성
        koras_hw_ = std::make_unique<KorasHw>(device_name_, baudrate_);

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> ToolHwInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_states_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_states_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[0].name, hardware_interface::HW_IF_EFFORT, &hw_states_[2]));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ToolHwInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_commands_[0]));
        return command_interfaces;
    }

    hardware_interface::CallbackReturn ToolHwInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        
        if(!koras_hw_->open_port()) return hardware_interface::CallbackReturn::ERROR;

        koras_hw_->gripperInit();
        koras_hw_->setMotorSpeedPercent(100);
        koras_hw_->setMotorTorquePercent(50);
        koras_hw_->setFingerPositionMeter(0.05);

        RCLCPP_INFO(rclcpp::get_logger("ToolHwInterface"), "Tool Hardware activated. Moving to 0.05m initial position.");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn ToolHwInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        koras_hw_->close_port();
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type ToolHwInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period) {
        KorasHw::Status data_out;
        if (!koras_hw_->readStatus(data_out)){
            read_error_count_++;

            // 연속 에러가 허용 범위 내라면 OK를 리턴해서 루프를 살림
            if (read_error_count_ < MAX_READ_ERRORS) {
                // 너무 자주 찍히지 않게 Throttle 로그 사용
                RCLCPP_WARN_THROTTLE(
                    rclcpp::get_logger("HeadHwInterface"),
                    *clock_, 500, // 0.5초 간격으로 경고
                    "Sync Read Failed (%d/%d). Using last valid state.", 
                    read_error_count_, MAX_READ_ERRORS
                );
                
                // 중요: 여기서 ERROR를 리턴하지 않고 OK를 리턴함!
                // hw_states_에는 이전 루프에서 읽었던 값이 그대로 남아있어 안전함.
                return hardware_interface::return_type::OK;
            } 
            else {
                // 연속 실패 횟수를 초과한 경우 (진짜 문제 발생)
                RCLCPP_ERROR(rclcpp::get_logger("HeadHwInterface"), 
                            "Critical: Consecutive sync read failures! Stopping hardware.");
                return hardware_interface::return_type::ERROR;
            }
        }

        // 2. 읽기 성공 시 카운터 초기화
        if (read_error_count_ > 0) {
            RCLCPP_INFO(rclcpp::get_logger("HeadHwInterface"), "Communication recovered.");
        }
        read_error_count_ = 0;


        // ==========================================================
        // 3. 상태값(States) 업데이트 (위치, 속도, 전류)
        // ==========================================================
        
        // [State 0] 위치 (Position): m 단위 (구조체에서 이미 변환된 값 사용)
        hw_states_[0] = data_out.finger_position_m;

        // [State 1] 속도 (Velocity): m/s 단위 (수치 미분)
        double dt = period.seconds();
        if (dt > 0.0) {
            hw_states_[1] = (hw_states_[0] - last_position_m_) / dt;
        } else {
            hw_states_[1] = 0.0;
        }
        // 다음 루프의 미분 계산을 위해 현재 위치 갱신
        last_position_m_ = hw_states_[0];

        // [State 2] 전류/토크 (Effort): mA -> A 로 단위 변환
        // (주의: 역방향 전류로 음수가 나올 수 있으므로 int16_t로 먼저 캐스팅한 후 double로 변환)
        hw_states_[2] = static_cast<double>(static_cast<int16_t>(data_out.motor_current_mA)) / 1000.0;

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ToolHwInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    // 컨트롤러에서 내려온 목표 라디안을 다이나믹셀로 전송
        double target_position = hw_commands_[0];

        koras_hw_->setFingerPositionMeter(target_position);


        return hardware_interface::return_type::OK;
    }

}

PLUGINLIB_EXPORT_CLASS(kaair_driver::ToolHwInterface, hardware_interface::SystemInterface)
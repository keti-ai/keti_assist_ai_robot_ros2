#include "kaair_driver/lift/lift_hw_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace kaair_driver {

    CallbackReturn LiftHwInterface::on_init(const hardware_interface:: HardwareInfo & info){
        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;

        if (info.joints.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("LiftHwInterface"), "LiftHwInterface expects exactly 1 joint!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

        
        // 1. URDF 파라미터 로드 (.count()를 사용하여 xacro에 값이 있을 때만 덮어씌움)
        if (info.hardware_parameters.count("usb_port")) 
            config.usb_port = info.hardware_parameters.at("usb_port");
        if (info.hardware_parameters.count("baud_rate")) 
            config.baud_rate = std::stoi(info.hardware_parameters.at("baud_rate"));
        if (info.hardware_parameters.count("offset_position")) 
            config.offset_position = std::stod(info.hardware_parameters.at("offset_position"));
        if (info.hardware_parameters.count("encoder_ppr")) 
            config.encoder_ppr = std::stoi(info.hardware_parameters.at("encoder_ppr"));
        if (info.hardware_parameters.count("lead_pitch")) 
            config.lead_pitch = std::stod(info.hardware_parameters.at("lead_pitch"));
        if (info.hardware_parameters.count("reduction_ratio")) 
            config.reduction_ratio = std::stod(info.hardware_parameters.at("reduction_ratio"));
        if (info.hardware_parameters.count("max_rpm")) 
            config.max_rpm = std::stoi(info.hardware_parameters.at("max_rpm"));

        // boolean 값 파싱 (문자열 "true"와 비교)
        if (info.hardware_parameters.count("limit_direction")) 
            config.limit_direction = (info.hardware_parameters.at("limit_direction") == "true");
        if (info.hardware_parameters.count("motor_direction")) 
            config.motor_direction = (info.hardware_parameters.at("motor_direction") == "true");

        // 조인트 태그(<joint name="...">) 내부에 있는 모터 ID 로드
        if (info.joints[0].parameters.count("id")) {
            config.motor_id = std::stoi(info.joints[0].parameters.at("id"));
        }

        // 2. 조인트 개수만큼 버퍼 초기화
        hw_states_.resize(3, 0.0);
        hw_commands_.resize(1, 0.0);


        md485_hw_ = std::make_unique<MD485Hw>(config);

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> LiftHwInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
    }
    return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> LiftHwInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    }
    return command_interfaces;
    }

    CallbackReturn LiftHwInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        
        if(!md485_hw_->connect()) return CallbackReturn::ERROR;

        md485_hw_->clear_alarm();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        md485_hw_->set_max_rpm(config.max_rpm);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));


        RCLCPP_INFO(rclcpp::get_logger("LiftHwInterface"), "Hardware activated. ");
        
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn LiftHwInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    
        md485_hw_->stop_brake();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        md485_hw_->disconnect();

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type LiftHwInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        // 컨트롤러에서 내려온 목표 라디안을 다이나믹셀로 전송
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type LiftHwInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        RosDataPayload ros_main;
        
        if(!md485_hw_->read_ros_state(ros_main)){
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

        hw_states_[0]=ros_main.position;
        hw_states_[1]=ros_main.velocity;
        hw_states_[2]=ros_main.effort;

        // 4. 모니터링 로그 (디버깅 시 매우 유용)
        RCLCPP_INFO_THROTTLE(
            rclcpp::get_logger("LiftHwInterface"),
            *clock_, 1000,
            "Lift Status | Cmd: %.3f m | Pos: %.3f m, Vel: %.3f m/s, Effort: %.2f A",
            hw_commands_[0], ros_main.position, ros_main.velocity, ros_main.effort
        );


        return hardware_interface::return_type::OK;
    }
}

// 플러그인 등록
PLUGINLIB_EXPORT_CLASS(kaair_driver::LiftHwInterface, hardware_interface::SystemInterface)
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
        hw_states_.resize(info.joints.size(), 0.0);
        hw_commands_.resize(info.joints.size(), 0.0);


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
        
        
        RCLCPP_INFO(rclcpp::get_logger("LiftHwInterface"), "Hardware activated. ");
        
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn LiftHwInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type LiftHwInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        // 컨트롤러에서 내려온 목표 라디안을 다이나믹셀로 전송
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type LiftHwInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        return hardware_interface::return_type::OK;
    }
}

// 플러그인 등록
PLUGINLIB_EXPORT_CLASS(kaair_driver::LiftHwInterface, hardware_interface::SystemInterface)
#include "kaair_driver/head/head_hw_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace kaair_driver {

CallbackReturn HeadHwInterface::on_init(const hardware_interface::HardwareInfo & info) {
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;

  // 1. 파라미터 로드 (urdf/ros2_control.xacro에서 설정한 값들)
  device_name_ = info.hardware_parameters.at("usb_port");
  baudrate_ = std::stoi(info.hardware_parameters.at("baud_rate"));
  
  // 2. 조인트 개수만큼 버퍼 초기화
  hw_states_.resize(info.joints.size(), 0.0);
  hw_commands_.resize(info.joints.size(), 0.0);
  
  for (const auto & joint : info.joints) {
    dxl_ids_.push_back(std::stoi(joint.parameters.at("id")));
  }

  // 3. 다이나믹셀 객체 생성
  dxl_hw_ = std::make_unique<DxlHw>(device_name_, baudrate_);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HeadHwInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HeadHwInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

CallbackReturn HeadHwInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!dxl_hw_->open_port()) return CallbackReturn::ERROR;
  if (!dxl_hw_->set_torque(dxl_ids_, true)) return CallbackReturn::ERROR;
  
  // 현재 위치를 초기 명령값으로 설정 (갑작스런 튀튀 방지)
  dxl_hw_->sync_read_radian(dxl_ids_, hw_states_);
  hw_commands_ = hw_states_;
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn HeadHwInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  dxl_hw_->set_torque(dxl_ids_, false);
  dxl_hw_->close_port();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type HeadHwInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  // 다이나믹셀에서 현재 라디안 읽어오기
  if (!dxl_hw_->sync_read_radian(dxl_ids_, hw_states_)) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HeadHwInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  // 컨트롤러에서 내려온 목표 라디안을 다이나믹셀로 전송
  if (!dxl_hw_->sync_write_radian(dxl_ids_, hw_commands_)) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

} // namespace kaair_driver

// 플러그인 등록
PLUGINLIB_EXPORT_CLASS(kaair_driver::HeadHwInterface, hardware_interface::SystemInterface)
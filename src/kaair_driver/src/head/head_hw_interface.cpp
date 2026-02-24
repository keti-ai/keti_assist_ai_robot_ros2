#include "kaair_driver/head/head_hw_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace kaair_driver {

CallbackReturn HeadHwInterface::on_init(const hardware_interface::HardwareInfo & info) {
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;

  clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

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
  
  // 1. 토크를 켜기 전/후에 현재 위치를 먼저 파악 (상태 동기화)
  dxl_hw_->sync_read_radian(dxl_ids_, hw_states_);

  // 2. 초기화 속도 및 가속도 설정 (값은 100~200 사이 추천)
    // dxl_ids_는 std::vector<uint8_t> 여야 합니다.
    uint32_t init_vel = 50;
    dxl_hw_->write_profile_velocity(dxl_ids_, init_vel);

  // 2. 명령 버퍼를 0.0으로 강제 초기화
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);

  // 3. 토크 ON (이제 write 함수가 호출되면서 0,0으로 서서히 또는 설정된 속도로 이동 시작)
  if (!dxl_hw_->set_torque(dxl_ids_, true)) return CallbackReturn::ERROR;
  
  RCLCPP_INFO(rclcpp::get_logger("HeadHwInterface"), "Hardware activated. Moving to [0, 0] initial position.");
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn HeadHwInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  dxl_hw_->set_torque(dxl_ids_, false);
  dxl_hw_->close_port();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type HeadHwInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    
    // 1. 다이나믹셀 읽기 시도
    if (!dxl_hw_->sync_read_radian(dxl_ids_, hw_states_)) {
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

    // 3. (옵션) 성공 시 데이터 모니터링 로그
    RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("HeadHwInterface"),
        *clock_, 1000,
        "Status [J1] Cmd: %.3f, State: %.3f | [J2] Cmd: %.3f, State: %.3f",
        hw_commands_[0], hw_states_[0],
        hw_commands_[1], hw_states_[1]
    );

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
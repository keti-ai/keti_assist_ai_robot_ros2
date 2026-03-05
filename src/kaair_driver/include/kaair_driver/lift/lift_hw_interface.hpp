#ifndef KAAIR_DRIVER__LIFT_HW_INTERFACE_HPP_
#define KAAIR_DRIVER__LIFT_HW_INTERFACE_HPP_

#include <vector>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "kaair_driver/lift/md485_hw.hpp"

namespace kaair_driver {

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  class LiftHwInterface : public hardware_interface::SystemInterface {
  public:
    // 필수 생명주기 함수들
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    
    // 실시간 루프 함수
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  private:
    // md드라이버 통신 객체
    std::unique_ptr<MD485Hw> md485_hw_;

    // ros2_control과 데이터를 주고받을 내부 버퍼 (Radian 단위)
    std::vector<double> hw_commands_;
    std::vector<double> hw_states_;
    
    // 설정값
    MD485HwConfig config;
    int read_error_count_ = 0;
    const int MAX_READ_ERRORS = 5; // 5회 연속 실패 시에만 ERROR 리턴
    rclcpp::Clock::SharedPtr clock_;
  };

} // namespace kaair_driver

#endif
#ifndef KAAIR_DRIVER__LIFT_HW_INTERFACE_HPP_
#define KAAIR_DRIVER__LIFT_HW_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "kaair_driver/lift/md485_hw.hpp"

namespace kaair_driver {

class LiftHwInterface : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(LiftHwInterface)

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    std::unique_ptr<MD485Hw> md485_hw_;
    MD485HwConfig config;

    std::vector<double> hw_commands_;
    std::vector<double> hw_states_;

    std::shared_ptr<rclcpp::Clock> clock_;
    
    int read_error_count_ = 0;
    const int MAX_READ_ERRORS = 10;
    std::mutex com_mutex_;
    bool data_read_success_ = false; // 추가: 읽기 성공 여부 플래그
    double last_hw_command_ = -999.0;

};

}  // namespace kaair_driver

#endif  // KAAIR_DRIVER__LIFT_HW_INTERFACE_HPP_
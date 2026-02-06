#ifndef KAAIR_HARDWARE__HEAD_HW_INTERFACE_HPP_
#define KAAIR_HARDWARE__HEAD_HW_INTERFACE_HPP_

#include <vector>
#include <string>
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

namespace kaair_hardware
{
class HeadDynamixelHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HeadDynamixelHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  dynamixel::PortHandler * port_handler_;
  dynamixel::PacketHandler * packet_handler_;

  // 0: Pan, 1: Tilt
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_pos_;
  std::vector<double> hw_states_vel_;

  std::string usb_port_;
  int baud_rate_;
  std::vector<uint8_t> dxl_ids_;

  // 고정 파라미터
  const int RESOLUTION = 4096;
  const int CENTER_TICK = 2048;
};
} 
#endif
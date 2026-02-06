#include "kaair_hardware/head/head_hw_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>

namespace kaair_hardware
{
hardware_interface::CallbackReturn HeadDynamixelHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  usb_port_ = info_.hardware_parameters.at("usb_port");
  baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
  dxl_ids_ = {static_cast<uint8_t>(std::stoi(info_.hardware_parameters.at("pan_id"))),
              static_cast<uint8_t>(std::stoi(info_.hardware_parameters.at("tilt_id")))};

  hw_commands_.assign(info_.joints.size(), 0.0);
  hw_states_pos_.assign(info_.joints.size(), 0.0);
  hw_states_vel_.assign(info_.joints.size(), 0.0);

  port_handler_ = dynamixel::PortHandler::getPortHandler(usb_port_.c_str());
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HeadDynamixelHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_pos_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_vel_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HeadDynamixelHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn HeadDynamixelHardware::on_activate(const rclcpp_lifecycle::State &)
{
  if (!port_handler_->openPort() || !port_handler_->setBaudRate(baud_rate_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto id : dxl_ids_) {
    packet_handler_->write1ByteTxRx(port_handler_, id, 64, 1);       // Torque On
    packet_handler_->write4ByteTxRx(port_handler_, id, 112, 100);    // Profile Velocity (안전)
    packet_handler_->write4ByteTxRx(port_handler_, id, 116, CENTER_TICK); // 0점 이동
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HeadDynamixelHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto id : dxl_ids_) {
    packet_handler_->write1ByteTxRx(port_handler_, id, 64, 0); // Torque Off
  }
  port_handler_->closePort();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type HeadDynamixelHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < dxl_ids_.size(); ++i) {
    int32_t dxl_pos = 0;
    if (packet_handler_->read4ByteTxRx(port_handler_, dxl_ids_[i], 132, (uint32_t*)&dxl_pos) == COMM_SUCCESS) {
      hw_states_pos_[i] = (static_cast<double>(dxl_pos) - CENTER_TICK) * (2.0 * M_PI / RESOLUTION);
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HeadDynamixelHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < dxl_ids_.size(); ++i) {
    int32_t dxl_pos = static_cast<int32_t>(hw_commands_[i] * (RESOLUTION / (2.0 * M_PI)) + CENTER_TICK);
    dxl_pos = std::max(0, std::min(RESOLUTION - 1, dxl_pos));
    packet_handler_->write4ByteTxRx(port_handler_, dxl_ids_[i], 116, dxl_pos);
  }
  return hardware_interface::return_type::OK;
}
}

PLUGINLIB_EXPORT_CLASS(kaair_hardware::HeadDynamixelHardware, hardware_interface::SystemInterface)
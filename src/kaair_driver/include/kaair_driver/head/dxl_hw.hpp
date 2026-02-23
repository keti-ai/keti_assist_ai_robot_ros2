#ifndef KAAIR_DRIVER__DXL_HW_HPP_
#define KAAIR_DRIVER__DXL_HW_HPP_

#include <string>
#include <vector>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <cmath> // M_PI 사용

namespace kaair_driver {

class DxlHw {
public:
    DxlHw(const std::string & usb_port, int baudrate);
    ~DxlHw();

    bool open_port();
    void close_port();
    
    // 토크 제어 (enable: true이면 ON, false이면 OFF)
    bool set_torque(const std::vector<uint8_t>& ids, bool enable);
    
    // 동기 위치 제어
    bool sync_write_position(const std::vector<uint8_t>& ids, const std::vector<int32_t>& positions);
    bool sync_read_position(const std::vector<uint8_t>& ids, std::vector<int32_t>& positions);

    // 라디안 변환 제어
    bool sync_write_radian(const std::vector<uint8_t>& ids, const std::vector<double>& radians);
    bool sync_read_radian(const std::vector<uint8_t>& ids, std::vector<double>& radians);

private:
    dynamixel::PortHandler * portHandler_;
    dynamixel::PacketHandler * packetHandler_;
    
    std::string device_name_;
    int baudrate_;
    float protocol_version_ = 2.0;

    const int32_t DXL_OFFSET = 2048;
    const double DXL_RESOLUTION = 4096.0;

    // 제어 주소 (시리즈에 따라 다를 수 있지만 보통 X 시리즈 기준)
    const uint16_t ADDR_TORQUE_ENABLE = 64;
    const uint16_t ADDR_GOAL_POSITION = 116;
    const uint16_t LEN_GOAL_POSITION = 4;
    const uint16_t ADDR_PRESENT_POSITION = 132;
    const uint16_t LEN_PRESENT_POSITION = 4;

    // 편의를 위한 변환 헬퍼 함수
    int32_t rad_to_dxl(double rad) {
        return static_cast<int32_t>((rad * DXL_RESOLUTION / (2.0 * M_PI)) + DXL_OFFSET);
    }
    double dxl_to_rad(int32_t dxl) {
        return static_cast<double>(dxl - DXL_OFFSET) * (2.0 * M_PI / DXL_RESOLUTION);
    }

};

} // namespace kaair_driver

#endif
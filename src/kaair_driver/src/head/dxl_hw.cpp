#include "kaair_driver/head/dxl_hw.hpp"
#include <iostream>

namespace kaair_driver {

DxlHw::DxlHw(const std::string & usb_port, int baudrate)
: device_name_(usb_port), baudrate_(baudrate) {
    portHandler_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);
}

DxlHw::~DxlHw() {
    close_port();
}

bool DxlHw::open_port() {
    if (!portHandler_->openPort()) return false;
    if (!portHandler_->setBaudRate(baudrate_)) return false;
    return true;
}

void DxlHw::close_port() {
    if (portHandler_ && portHandler_->is_using_) portHandler_->closePort();
}

// --- 토크 설정 추가 ---
bool DxlHw::set_torque(const std::vector<uint8_t>& ids, bool enable) {
    uint8_t value = enable ? 1 : 0;
    bool all_success = true;

    for (uint8_t id : ids) {
        uint8_t dxl_error = 0;
        int dxl_comm_result = packetHandler_->write1ByteTxRx(
            portHandler_, id, ADDR_TORQUE_ENABLE, value, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            std::cerr << "[DxlHw] ID " << (int)id << " Torque Failed: " 
                      << packetHandler_->getTxRxResult(dxl_comm_result) << std::endl;
            all_success = false;
        } else if (dxl_error != 0) {
            std::cerr << "[DxlHw] ID " << (int)id << " Torque Error: " 
                      << packetHandler_->getRxPacketError(dxl_error) << std::endl;
            all_success = false;
        }
    }
    return all_success;
}

// --- Group Sync Write 적용 ---
bool DxlHw::sync_write_position(const std::vector<uint8_t>& ids, const std::vector<int32_t>& positions) {
    if (ids.size() != positions.size()) return false;

    dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);

    for (size_t i = 0; i < ids.size(); ++i) {
        uint8_t param[4];
        param[0] = DXL_LOBYTE(DXL_LOWORD(positions[i]));
        param[1] = DXL_HIBYTE(DXL_LOWORD(positions[i]));
        param[2] = DXL_LOBYTE(DXL_HIWORD(positions[i]));
        param[3] = DXL_HIBYTE(DXL_HIWORD(positions[i]));

        if (!groupSyncWrite.addParam(ids[i], param)) return false;
    }

    int result = groupSyncWrite.txPacket();
    groupSyncWrite.clearParam();

    return (result == COMM_SUCCESS);
}

bool DxlHw::sync_read_position(const std::vector<uint8_t>& ids, std::vector<int32_t>& positions) {
    if (ids.empty()) return false;
    
    // 1. GroupSyncRead 객체 생성
    dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler_, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

    // 2. 읽을 대상 ID 등록
    for (uint8_t id : ids) {
        if (!groupSyncRead.addParam(id)) {
            std::cerr << "[DxlHw] ID " << (int)id << " groupSyncRead addParam failed" << std::endl;
            return false;
        }
    }

    // 3. 데이터 수신 (txRxPacket 호출)
    int dxl_comm_result = groupSyncRead.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << "[DxlHw] Sync Read Failed: " << packetHandler_->getTxRxResult(dxl_comm_result) << std::endl;
        return false;
    }

    // 4. 수신된 데이터 꺼내기
    positions.clear();
    for (uint8_t id : ids) {
        // 데이터가 사용 가능한지 확인
        if (!groupSyncRead.isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) {
            std::cerr << "[DxlHw] ID " << (int)id << " data not available" << std::endl;
            return false;
        }

        // 4바이트 데이터 읽기
        int32_t pos = groupSyncRead.getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
        positions.push_back(pos);
    }

    groupSyncRead.clearParam();
    return true;
}

// --- 라디안 쓰기 ---
bool DxlHw::sync_write_radian(const std::vector<uint8_t>& ids, const std::vector<double>& radians) {
    std::vector<int32_t> dxl_positions;
    for (double rad : radians) {
        dxl_positions.push_back(rad_to_dxl(rad));
    }
    return sync_write_position(ids, dxl_positions);
}

// --- 라디안 읽기 ---
bool DxlHw::sync_read_radian(const std::vector<uint8_t>& ids, std::vector<double>& radians) {
    std::vector<int32_t> dxl_positions;
    if (!sync_read_position(ids, dxl_positions)) return false;

    radians.clear();
    for (int32_t pos : dxl_positions) {
        radians.push_back(dxl_to_rad(pos));
    }
    return true;
}


} // namespace kaair_driver
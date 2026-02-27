#include "kaair_driver/lift/md485_hw.hpp"
#include <iostream>

namespace kaair_driver {

MD485Hw::MD485Hw(const MD485HwConfig & config)
: cfg_(config)
{
    // serial 객체 초기화 (포트는 아직 열지 않음)
    serial_ = std::make_unique<serial::Serial>(
        "", 
        cfg_.baudrate, 
        serial::Timeout::simpleTimeout(100)
    );
}

MD485Hw::~MD485Hw()
{
    disconnect();
}

bool MD485Hw::connect()
{
    try {
        serial_->setPort(cfg_.port);
        serial_->open();
    } catch (const serial::IOException& e) {
        std::cerr << "[MD485Hw] 시리얼 포트 열기 실패 (" << cfg_.port << "): " << e.what() << std::endl;
        return false;
    }

    if (serial_->isOpen()) {
        std::cout << "[MD485Hw] 모터 드라이버 연결 성공: " << cfg_.port << " (" << cfg_.baudrate << "bps)" << std::endl;
        return true;
    }
    return false;
}

void MD485Hw::disconnect()
{
    if (serial_ && serial_->isOpen()) {
        // 안전을 위해 종료 전 모터 정지 명령(토크 오프 또는 속도 0)을 보낼 수도 있습니다.
        set_velocity_rpm(0); 
        serial_->close();
        std::cout << "[MD485Hw] 모터 드라이버 통신 종료." << std::endl;
    }
}

bool MD485Hw::set_velocity_rpm(int16_t rpm)
{
    if (!serial_->isOpen()) {
        std::cerr << "[MD485Hw] 포트가 닫혀있어 속도 명령을 전송할 수 없습니다." << std::endl;
        return false;
    }

    // 설정값에 따른 모터 방향 반전(motor_inv) 처리
    if (cfg_.motor_inv) {
        rpm = -rpm;
    }

    // RPM 제한(Clamp) 처리
    if (rpm > cfg_.rpm_range[1]) rpm = cfg_.rpm_range[1];
    if (rpm < cfg_.rpm_range[0]) rpm = cfg_.rpm_range[0];

    // 패킷 버퍼 준비: 헤더(5) + 데이터(2) + 체크섬(1) = 8 Bytes
    const size_t packet_size = 8;
    uint8_t tx_buf[packet_size] = {0,};

    // 1. Header 조립
    tx_buf[0] = static_cast<uint8_t>(mdrobot::MID::BLDC_CTR); // RMID (183, 모터 드라이버)
    tx_buf[1] = static_cast<uint8_t>(mdrobot::MID::MMI);      // TMID (172, PC/ROS)
    tx_buf[2] = static_cast<uint8_t>(cfg_.motor_id);          // ID   (슬레이브 ID)
    tx_buf[3] = static_cast<uint8_t>(mdrobot::PID::VEL_CMD);  // PID  (130, 속도 명령)
    tx_buf[4] = 2;                                            // Data_Len (2바이트)

    // 2. Data 조립 (int16_t를 Little Endian으로 분할)
    // 음수 값의 경우에도 2의 보수 형태로 비트 연산이 올바르게 동작합니다.
    uint16_t u_rpm = static_cast<uint16_t>(rpm);
    tx_buf[5] = u_rpm & 0xFF;         // Low Byte
    tx_buf[6] = (u_rpm >> 8) & 0xFF;  // High Byte

    // 3. Checksum 계산 (앞서 md_com.hpp에 만든 inline 함수 사용)
    // 체크섬은 체크섬 바이트 본인을 제외한 배열의 처음부터 마지막 앞까지의 합입니다.
    tx_buf[7] = mdrobot::CalculateChecksum(tx_buf, packet_size - 1);

    // 4. 시리얼 포트로 송신
    size_t written = serial_->write(tx_buf, packet_size);
    
    return (written == packet_size);
}

} // namespace kaair_driver
#include "kaair_driver/lift/md485_hw.hpp"
#include <iostream>
#include <cstring>
#include <chrono>

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
    // 데이터 분할, 시프트 연산, 체크섬 계산을 할 필요가 없습니다!
    // BuildPacket에 rpm 변수만 던져주면 알아서 2바이트로 분할해 줍니다.
    auto packet = kaair_driver::BuildPacket(
        kaair_driver::MID::BLDC_CTR, 
        kaair_driver::MID::MMI, 
        cfg_.motor_id, 
        kaair_driver::PID::VEL_CMD, 
        rpm  // 👈 여기에 데이터 투척
    );

    size_t written = serial_->write(packet);
    return (written == packet.size());
}

bool MD485Hw::read_state(kaair_driver::MainDataPayload& out_main, kaair_driver::IoMonitorPayload& out_io)
{
    if (!serial_ || !serial_->isOpen()) return false;

    // 수신 버퍼에 남아있는 이전 찌꺼기 완벽하게 제거
    serial_->flushInput(); 

    // =========================================================
    // 1. MAIN_DATA (193) 요청 및 수신
    // =========================================================
    // REQ_PID_DATA(4)의 데이터로 '193'을 담아서 보냅니다.
    auto req_main = kaair_driver::BuildPacket(
        kaair_driver::MID::BLDC_CTR, kaair_driver::MID::MMI, cfg_.motor_id, 
        kaair_driver::PID::REQ_PID_DATA, 
        static_cast<uint8_t>(kaair_driver::PID::MAIN_DATA) // 👈 193번 데이터 줘!
    );
    
    serial_->write(req_main);

    std::vector<uint8_t> rx_main;
    if (receive_packet(kaair_driver::PID::MAIN_DATA, rx_main)) {
        // 벡터 배열을 그대로 구조체 메모리에 복사 (리틀 엔디안)
        std::memcpy(&out_main, rx_main.data(), sizeof(kaair_driver::MainDataPayload));
    } else {
        std::cerr << "[MD485Hw] MAIN_DATA 읽기 실패 (Timeout)" << std::endl;
        return false;
    }

    // =========================================================
    // 2. IO_MONITOR (194) 요청 및 수신
    // =========================================================
    // REQ_PID_DATA(4)의 데이터로 '194'를 담아서 보냅니다.
    auto req_io = kaair_driver::BuildPacket(
        kaair_driver::MID::BLDC_CTR, kaair_driver::MID::MMI, cfg_.motor_id, 
        kaair_driver::PID::REQ_PID_DATA, 
        static_cast<uint8_t>(kaair_driver::PID::IO_MONITOR) // 👈 194번 데이터 줘!
    );

    serial_->write(req_io);

    std::vector<uint8_t> rx_io;
    if (receive_packet(kaair_driver::PID::IO_MONITOR, rx_io)) {
        std::memcpy(&out_io, rx_io.data(), sizeof(kaair_driver::IoMonitorPayload));
    } else {
        std::cerr << "[MD485Hw] IO_MONITOR 읽기 실패 (Timeout)" << std::endl;
        return false;
    }

    // 두 데이터 모두 성공적으로 읽었을 때만 true 반환
    return true;
}

// =========================================================
// 2. 메인 데이터만 읽기 (새로 추가한 오버로딩 함수)
// =========================================================
bool MD485Hw::read_state(kaair_driver::MainDataPayload& out_main)
{
    if (!serial_ || !serial_->isOpen()) return false;
    serial_->flushInput(); 

    auto req_main = kaair_driver::BuildPacket(
        kaair_driver::MID::BLDC_CTR, kaair_driver::MID::MMI, cfg_.motor_id, 
        kaair_driver::PID::REQ_PID_DATA, 
        static_cast<uint8_t>(kaair_driver::PID::MAIN_DATA)
    );
    
    serial_->write(req_main);

    std::vector<uint8_t> rx_main;
    if (receive_packet(kaair_driver::PID::MAIN_DATA, rx_main)) {
        std::memcpy(&out_main, rx_main.data(), sizeof(kaair_driver::MainDataPayload));
        return true;
    } 

    std::cerr << "[MD485Hw] MAIN_DATA 읽기 실패 (Timeout)" << std::endl;
    return false;
}


bool MD485Hw::receive_packet(kaair_driver::PID expected_pid, std::vector<uint8_t>& out_data)
{
    if (!serial_ || !serial_->isOpen()) return false;

    uint8_t header[5];
    int header_idx = 0;

    // 1. 헤더 동기화: PC가 수신하는 입장이므로 RMID=172(MMI), TMID=183(BLDC) 순서입니다.
    auto start_time = std::chrono::steady_clock::now();
    while (header_idx < 5) {
        // RS485 응답 지연을 고려하여 최대 100ms 대기 (타임아웃)
        if (std::chrono::steady_clock::now() - start_time > std::chrono::milliseconds(100)) {
            return false; 
        }

        uint8_t byte;
        if (serial_->read(&byte, 1) == 1) {
            // 헤더 패턴 매칭 (172 -> 183 -> 모터ID)
            if (header_idx == 0 && byte != static_cast<uint8_t>(kaair_driver::MID::MMI)) continue;
            if (header_idx == 1 && byte != static_cast<uint8_t>(kaair_driver::MID::BLDC_CTR)) { header_idx = 0; continue; }
            if (header_idx == 2 && byte != cfg_.motor_id) { header_idx = 0; continue; }
            
            header[header_idx++] = byte;
        }
    }

    uint8_t pid = header[3];
    uint8_t data_len = header[4];

    // 2. 남은 데이터(Payload) 및 체크섬(1바이트) 한 번에 읽기
    std::vector<uint8_t> payload(data_len + 1);
    size_t read_bytes = serial_->read(payload.data(), data_len + 1);
    if (read_bytes < static_cast<size_t>(data_len + 1)) return false;

    // 3. 체크섬 검증
    std::vector<uint8_t> full_packet;
    full_packet.insert(full_packet.end(), header, header + 5);
    full_packet.insert(full_packet.end(), payload.begin(), payload.end() - 1); // 체크섬 바이트 제외

    uint8_t calc_chk = kaair_driver::CalculateChecksum(full_packet.data(), full_packet.size());
    if (calc_chk != payload.back()) {
        std::cerr << "[MD485Hw] 수신 데이터 체크섬 에러!" << std::endl;
        return false;
    }

    // 4. 응답 확인
    // Case A: 데이터 읽기(REQ_PID_DATA)를 요청하여 원하는 데이터가 그대로 리턴된 경우
    if (pid == static_cast<uint8_t>(expected_pid)) {
        out_data.assign(payload.begin(), payload.end() - 1); // 체크섬 뺀 순수 데이터만 복사
        return true; 
    }
    return false;
}

} // namespace kaair_driver
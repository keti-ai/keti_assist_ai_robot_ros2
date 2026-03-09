#include "kaair_driver/lift/md485_hw.hpp"
#include <iostream>
#include <cstring>
#include <chrono>
#include <thread>

namespace kaair_driver {

MD485Hw::MD485Hw(const MD485HwConfig & config)
: cfg_(config)
{
    // serial 객체 초기화 (포트는 아직 열지 않음)
    serial_ = std::make_unique<serial::Serial>(
        "",
        cfg_.baud_rate, 
        serial::Timeout::simpleTimeout(3)
    );
}

MD485Hw::~MD485Hw()
{
    disconnect();
}

bool MD485Hw::connect()
{
    try {
        serial_->setPort(cfg_.usb_port);
        serial_->open();
    } catch (const serial::IOException& e) {
        std::cerr << "[MD485Hw] 시리얼 포트 열기 실패 (" << cfg_.usb_port << "): " << e.what() << std::endl;
        return false;
    }

    if (serial_->isOpen()) {
        std::cout << "[MD485Hw] 모터 드라이버 연결 성공: " << cfg_.usb_port << " (" << cfg_.baud_rate << "bps)" << std::endl;
        return true;
    }
    return false;
}

void MD485Hw::disconnect()
{
    if (serial_ && serial_->isOpen()) {
        // 안전을 위해 종료 전 모터 정지 명령(토크 오프 또는 속도 0)을 보낼 수도 있습니다.
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

bool MD485Hw::set_linear_velocity(double velocity)
{
    return set_velocity_rpm(meter_per_s_to_rpm(velocity));
}

bool MD485Hw::set_position_with_rpm(int32_t position, uint16_t rpm)
{
// 1. 메모리 패딩 없이 정확히 6바이트(4+2)만 차지하도록 구조체를 묶습니다.
#pragma pack(push, 1)
    struct PosVelData {
        int32_t pos;
        uint16_t vel;
    } payload = {position, rpm};
#pragma pack(pop)

    // 2. 만들어두신 마법의 템플릿에 데이터 덩어리를 그대로 투척!
    auto packet = kaair_driver::BuildPacket(
        kaair_driver::MID::BLDC_CTR, 
        kaair_driver::MID::MMI, 
        cfg_.motor_id, 
        kaair_driver::PID::POSI_VEL_CMD,
        payload
    );

    size_t written = serial_->write(packet);
    return (written == packet.size());
}

bool MD485Hw::move_abs_pose_meter_with_velocity(double position,double velocity)
{
    return set_position_with_rpm(meter_to_count(position-cfg_.offset_position),meter_per_s_to_rpm(velocity));
}

bool MD485Hw::move_abs_pose_meter_rpm(double position,double rpm)
{
    return set_position_with_rpm(meter_to_count(position-cfg_.offset_position),rpm);
}

bool MD485Hw::set_init_set(uint8_t init_mode)
{
    // 2. 만들어두신 마법의 템플릿에 데이터 덩어리를 그대로 투척!
    auto packet = kaair_driver::BuildPacket(
        kaair_driver::MID::BLDC_CTR, 
        kaair_driver::MID::MMI, 
        cfg_.motor_id, 
        kaair_driver::PID::TQ_RATIO,
        init_mode
    );

    size_t written = serial_->write(packet);
    return (written == packet.size());
}

bool MD485Hw::read_init_set_ok(uint8_t& init_state)
{
    if (!serial_ || !serial_->isOpen()) return false;
    serial_->flushInput(); 

    // 1. MAIN_DATA(193)가 아닌, INIT_SET(35) 데이터를 달라고 요청!
    auto req_packet = kaair_driver::BuildPacket(
        kaair_driver::MID::BLDC_CTR, kaair_driver::MID::MMI, cfg_.motor_id, 
        kaair_driver::PID::REQ_PID_DATA, 
        static_cast<uint8_t>(kaair_driver::PID::TQ_RATIO) // 👈 35번 데이터 요청
    );
    
    serial_->write(req_packet);

    // 2. 응답 수신
    std::vector<uint8_t> rx_data;
    if (receive_packet(kaair_driver::PID::TQ_RATIO, rx_data)) {
        if (!rx_data.empty()) {
            // 수신된 1바이트 값을 참조 변수에 담아줌
            init_state = rx_data[0]; 
            
            // 🌟 핵심 로직: 값이 1 이상이면 true, 0이면 false 리턴
            return (init_state >= 1); 
        }
    } 

    // 통신 실패 시
    std::cerr << "[MD485Hw] INIT_SET 상태 읽기 실패 (Timeout)" << std::endl;
    init_state = 0; // 안전을 위해 0으로 강제 초기화
    return false;
}

bool MD485Hw::reset_position()
{
    // 2. 만들어두신 마법의 템플릿에 데이터 덩어리를 그대로 투척!
    auto packet = kaair_driver::BuildPacket(
        kaair_driver::MID::BLDC_CTR, 
        kaair_driver::MID::MMI, 
        cfg_.motor_id, 
        kaair_driver::PID::COMMAND,
        kaair_driver::Command::POSI_RESET
    );

    size_t written = serial_->write(packet);
    return (written == packet.size());
}

bool MD485Hw::set_max_rpm(uint16_t rpm)
{
    // 2. 만들어두신 마법의 템플릿에 데이터 덩어리를 그대로 투척!
    auto packet = kaair_driver::BuildPacket(
        kaair_driver::MID::BLDC_CTR, 
        kaair_driver::MID::MMI, 
        cfg_.motor_id, 
        kaair_driver::PID::MAX_RPM,
        rpm
    );

    size_t written = serial_->write(packet);
    return (written == packet.size());
}

bool MD485Hw::return_type_ack()
{
    // 2. 만들어두신 마법의 템플릿에 데이터 덩어리를 그대로 투척!
    auto packet = kaair_driver::BuildPacket(
        kaair_driver::MID::BLDC_CTR, 
        kaair_driver::MID::MMI, 
        cfg_.motor_id, 
        kaair_driver::PID::RETURN_TYPE,
        0
    );

    size_t written = serial_->write(packet);
    return (written == packet.size());
}

bool MD485Hw::clear_alarm()
{
    // 2. 만들어두신 마법의 템플릿에 데이터 덩어리를 그대로 투척!
    auto packet = kaair_driver::BuildPacket(
        kaair_driver::MID::BLDC_CTR, 
        kaair_driver::MID::MMI, 
        cfg_.motor_id, 
        kaair_driver::PID::COMMAND,
        kaair_driver::PID::ALARM_RESET
    );

    size_t written = serial_->write(packet);
    return (written == packet.size());
}

bool MD485Hw::stop_brake()
{
    // 2. 만들어두신 마법의 템플릿에 데이터 덩어리를 그대로 투척!
    auto packet = kaair_driver::BuildPacket(
        kaair_driver::MID::BLDC_CTR, 
        kaair_driver::MID::MMI, 
        cfg_.motor_id, 
        kaair_driver::PID::COMMAND,
        kaair_driver::Command::BRAKE
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

// =========================================================
// 3. IO 데이터만 읽기 (새로 추가한 오버로딩 함수)
// =========================================================
bool MD485Hw::read_state(kaair_driver::IoMonitorPayload& out_io)
{
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

        uint8_t byte_val = rx_io[5];
        out_io.dir=(byte_val >> 2) & 0x01;
        out_io.run_brake=(byte_val >> 3) & 0x01;
        out_io.start_stop=(byte_val >> 4) & 0x01;
        
        return true;
    }

    std::cerr << "[MD485Hw] IO 읽기 실패 (Timeout)" << std::endl;
    return false;
}

bool MD485Hw::read_ros_state(kaair_driver::RosDataPayload& out_ros)
{
    MainDataPayload m_data;
    bool success = read_state(m_data);

    out_ros.position=count_to_meter(m_data.position)+cfg_.offset_position;
    out_ros.velocity=rpm_to_meter_per_s(m_data.rpm);
    out_ros.effort=m_data.current*0.1;

    return success;
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

bool MD485Hw::send_command_and_wait_ack(const std::vector<uint8_t>& packet, kaair_driver::PID expected_pid, int timeout_ms)
{
    if (!serial_ || !serial_->isOpen()) return false;

    // 1. 송신 전 수신 버퍼 찌꺼기 완벽히 비우기 (매우 중요)
    serial_->flushInput(); 
    
    // 2. 명령 전송
    serial_->write(packet);

    // 3. ACK 수신 대기
    auto start_time = std::chrono::steady_clock::now();
    std::vector<uint8_t> rx_buffer;
    rx_buffer.reserve(10); // ACK 패킷은 보통 작음

    while (true) {
        // 타임아웃 검사
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() > timeout_ms) {
            std::cerr << "[MD485Hw] 통신 ACK 타임아웃! (PID: " << static_cast<int>(expected_pid) << ")" << std::endl;
            return false;
        }

        // 버퍼에 데이터가 들어오면 읽어들임
        if (serial_->available()) {
            std::vector<uint8_t> temp_buf;
            size_t bytes_to_read = serial_->available();
            temp_buf.resize(bytes_to_read);
            serial_->read(temp_buf.data(), bytes_to_read);
            
            rx_buffer.insert(rx_buffer.end(), temp_buf.begin(), temp_buf.end());

            // 1바이트씩 들어오다가 패킷이 완성되었는지 지속 검사
            // (최소 헤더(5) + 체크섬(1) = 6바이트 이상이어야 함)
            if (rx_buffer.size() >= 6) {
                // 수신된 패킷이 내가 쏜 PID에 대한 정상적인 ACK인지 검증
                if (kaair_driver::IsValidAck(rx_buffer, cfg_.motor_id, expected_pid)) {
                    return true; // 성공! 즉시 탈출!
                }
            }
        }
        // CPU 점유율 하락을 위한 미세 딜레이
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool MD485Hw::read_init_switch_status(bool& init_status)
{
    kaair_driver::IoMonitorPayload io_data;
    bool success = read_state(io_data);

    if (success) {
        // limit_direction 설정에 따라 어느 핀을 볼지 결정
        if (cfg_.limit_direction) {
            // dir 핀도 동일하게 걸리면 0이라고 가정하고 반전(!)
            init_status = !io_data.dir; 
        }
        else {
            // start_stop 핀: 평소 1, 걸리면 0이므로 반전(!)시킴
            // 즉, 걸렸을 때(0일 때) init_status가 true가 됨!
            init_status = !io_data.start_stop;
        }
    } else {
        // 통신 실패 시 안전을 위해 일단 센서가 눌리지 않은 것(false)으로 처리
        // (필요에 따라 에러 처리를 다르게 하실 수 있습니다)
        init_status = false; 
    }

    return success;
}





double MD485Hw::count_to_meter(int count) const
{
    return (count / (cfg_.encoder_ppr * 4.0)) / cfg_.reduction_ratio * cfg_.lead_pitch;
}
int MD485Hw::meter_to_count(double meter) const
{
    return static_cast<int>((meter / cfg_.lead_pitch) * cfg_.reduction_ratio * (cfg_.encoder_ppr * 4.0));
}
double MD485Hw::rpm_to_meter_per_s(int rpm) const
{
    // (rpm / 60초) / 감속비 * 리드피치
    return (rpm / 60.0) / cfg_.reduction_ratio * cfg_.lead_pitch;
}

int MD485Hw::meter_per_s_to_rpm(double meter_per_s) const
{
    // ⭐️ static_cast<int> 적용: (초당 이동거리 / 리드피치) * 감속비 * 60초
    return static_cast<int>((meter_per_s / cfg_.lead_pitch) * cfg_.reduction_ratio * 60.0);
}



} // namespace kaair_driver
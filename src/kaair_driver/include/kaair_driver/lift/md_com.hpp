#include <cstdint>
#include <cstddef>

namespace kaair_driver {

// =================================================================
// 1. MID (Machine ID) 정의
// =================================================================
enum class MID : uint8_t {
    MAIN_CTR   = 128,  // 0x80 (메인 제어기)
    MMI        = 172,  // 0xac (사용자 PC, ROS 제어기)
    BLDC_CTR   = 183,  // 0xb7 (MD 모터 드라이버)
    BROADCAST  = 254   // 0xfe (전체 동시 제어용 ID)
};

// =================================================================
// 2. 주요 PID (Parameter ID) 정의
// =================================================================
enum class PID : uint8_t {
    // 1-Byte Data (명령 및 단일 설정)
    REQ_PID_DATA     = 4,   // 데이터 요청
    TQ_OFF           = 5,   // 자연 정지 (토크 오프)
    BRAKE            = 6,   // 전기적 급제동
    COMMAND          = 10,  // 내부 명령어 실행 (알람 리셋, 위치 초기화 등)
    ALARM_RESET      = 12,  // 알람 리셋
    POSI_RESET       = 13,  // 위치 0으로 초기화
    USE_LIMIT_SW     = 17,  // 리미트 스위치 기능 적용 여부
    CTRL_STATUS      = 34,  // 제어 상태 읽기

    // 2-Byte Data
    VEL_CMD          = 130, // 속도 제어 명령 (rpm)
    INT_RPM_DATA     = 138, // 모터 회전 속도 읽기 (rpm)
    TQ_DATA          = 139, // 모터 전류 읽기 (0.1A 단위)

    // N-Byte Data
    MAIN_DATA        = 193, // 메인 데이터 (상태, 속도, 전류, 위치 등 - 17 bytes)
    IO_MONITOR       = 194, // 제어기 I/O 상태 (17 bytes)
    TAR_POSI         = 195, // 타겟 위치 (4 bytes)
    MONITOR          = 196, // 모니터 데이터 (12 bytes)
    POSI_DATA        = 197, // 위치 데이터 (4 bytes)
    POSI_VEL_CMD     = 219, // 위치 & 속도 제어 명령 (6 bytes)
    INC_POSI_VEL_CMD = 220, // 상대 위치 & 속도 제어 명령 (6 bytes)
};

// =================================================================
// 3. Command (PID가 10(COMMAND)일 때 Data로 들어가는 값)
// =================================================================
enum class Command : uint8_t {
    TQ_OFF          = 2,   // 자연 정지
    BRAKE           = 4,   // 급제동
    MAIN_BC_ON      = 5,   // 메인 데이터 연속 전송(Broadcasting) 시작
    MAIN_BC_OFF     = 6,   // 메인 데이터 연속 전송 종료
    ALARM_RESET     = 8,   // 알람 리셋
    POSI_RESET      = 10,  // 모터 위치 0으로 초기화
};

// =================================================================
// 4. 모터 상태 비트 (PID 34 또는 MAIN_DATA의 status 필드 파싱용)
// =================================================================
enum class StatusBit : uint8_t {
    ALARM           = (1 << 0), // 제어기 알람 유무
    CTRL_FAIL       = (1 << 1), // 제어 실패 (속도 도달 불가 등)
    OVER_VOLT       = (1 << 2), // 과전압/저전압
    OVER_TEMP       = (1 << 3), // 과온도
    OVER_LOAD       = (1 << 4), // 과부하
    HALL_ENC_FAIL   = (1 << 5), // 홀센서/엔코더 신호 실패
    INV_VEL         = (1 << 6), // 회전 방향 역전
    STALL           = (1 << 7)  // 스톨 (출력 대비 모터 구속)
};

// =================================================================
// 5. 프로토콜 버퍼 구조체 (1바이트 정렬 필수)
// =================================================================
#pragma pack(push, 1)

// 패킷 헤더 (고정 5바이트)
struct PacketHeader {
    uint8_t rmid;       // 수신측 ID (모터 제어기로 보낼 땐 183)
    uint8_t tmid;       // 송신측 ID (PC/ROS 땐 172)
    uint8_t id;         // 제어기 로컬 ID (1~253)
    uint8_t pid;        // Parameter ID
    uint8_t data_len;   // 후속 데이터의 길이 (바이트 수)
};

// PID 219 (POSI_VEL_CMD) 송신 페이로드 (6바이트)
struct PosiVelCmdPayload {
    int32_t target_position; // 타겟 위치 
    int16_t max_rpm;         // 위치 제어 시 최대 속도 (rpm)
};

// PID 193 (MAIN_DATA) 수신 페이로드 (17바이트)
struct MainDataPayload {
    int16_t rpm;            // 모터 회전수 (rpm)
    int16_t current;        // 전류 (0.1A 단위)
    uint8_t control_type;   // 제어 타입 (1: 속도, 2: 위치 등)
    int16_t ref_rpm;        // 기준 속도
    int16_t control_out;    // 제어기 출력
    uint8_t status;         // 제어기 상태 표시 (StatusBit 열거형 참고)
    int32_t position;       // 모터 절대 위치
    uint8_t brake_out;      // 브레이크 출력
    uint8_t temperature;    // 온도 (0~100도)
    uint8_t status2;        // 상태 2
};

struct IoMonitorPayload {
    uint16_t input_port;   // 입력 핀 상태 (Start/Stop, Run/Brake 등 비트맵)
    uint16_t output_port;  // 출력 핀 상태
    uint16_t ad_in[4];     // 아날로그 입력 1~4 (8바이트)
    uint8_t reserved[5];   // 나머지 예약 영역 (5바이트)
};


#pragma pack(pop)

// =================================================================
// 6. 체크섬 계산 함수
// 매뉴얼 계산식: byCHK = ~(RMID+TMID+ID+PID+DataNumber+Data..) + 1
// =================================================================
inline uint8_t CalculateChecksum(const uint8_t* buffer, size_t length) {
    uint8_t sum = 0;
    for(size_t i = 0; i < length; i++) {
        sum += buffer[i];
    }
    return static_cast<uint8_t>(~sum + 1);
}

// =================================================================
// 7. PID별 예상 데이터 길이 (데이터 수신/검증 시 유용)
// =================================================================
constexpr uint8_t GetExpectedDataLength(PID pid) {
    switch(pid) {
        case PID::REQ_PID_DATA: return 1;
        case PID::TQ_OFF: return 1;
        case PID::BRAKE: return 1;
        case PID::COMMAND: return 1;
        case PID::ALARM_RESET: return 1;
        case PID::POSI_RESET: return 1;
        
        case PID::VEL_CMD: return 2;
        case PID::INT_RPM_DATA: return 2;
        case PID::TQ_DATA: return 2;
        
        case PID::MAIN_DATA: return 17;
        case PID::TAR_POSI: return 4;
        case PID::POSI_VEL_CMD: return 6;
        default: return 0; // 가변이거나 정의되지 않음
    }
}

// =================================================================
// 8. 🌟 만능 패킷 생성 함수 (C++ Template 활용) 🌟
// =================================================================
// 사용법: BuildPacket(수신ID, 송신ID, 모터ID, PID, 보낼데이터구조체_또는_변수);
template <typename T>
inline std::vector<uint8_t> BuildPacket(MID rmid, MID tmid, uint8_t motor_id, PID pid, const T& data) {
    
    // 템플릿 T의 자료형 크기를 자동으로 계산 (예: int16_t면 2, PosiVelCmdPayload면 6)
    uint8_t data_len = sizeof(T); 
    
    std::vector<uint8_t> packet;
    packet.reserve(5 + data_len + 1); // Header(5) + Data + Checksum(1) 메모리 사전할당

    // 1. Header 조립
    packet.push_back(static_cast<uint8_t>(rmid));
    packet.push_back(static_cast<uint8_t>(tmid));
    packet.push_back(motor_id);
    packet.push_back(static_cast<uint8_t>(pid));
    packet.push_back(data_len);

    // 2. Data 조립 (PC는 리틀 엔디안이므로 메모리를 있는 그대로 복사하면 완벽히 일치함)
    const uint8_t* p_data = reinterpret_cast<const uint8_t*>(&data);
    packet.insert(packet.end(), p_data, p_data + data_len);

    // 3. Checksum 계산 및 추가
    packet.push_back(CalculateChecksum(packet.data(), packet.size()));

    return packet;
}

// 오버로딩: 보낼 데이터가 아예 없는 경우 (Data Length = 0)
inline std::vector<uint8_t> BuildPacket(MID rmid, MID tmid, uint8_t motor_id, PID pid) {
    std::vector<uint8_t> packet = {
        static_cast<uint8_t>(rmid),
        static_cast<uint8_t>(tmid),
        motor_id,
        static_cast<uint8_t>(pid),
        0
    };
    packet.push_back(CalculateChecksum(packet.data(), packet.size()));
    return packet;
}

} // namespace mdrobot
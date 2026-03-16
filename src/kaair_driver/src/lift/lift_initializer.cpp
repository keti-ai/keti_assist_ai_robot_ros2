#include <iostream>
#include <thread>
#include <chrono>
#include "kaair_driver/lift/md485_hw.hpp"

int main() {
    std::cout << "=== KAAIR Lift Homing Initializer ===" << std::endl;

    // 1. 하드웨어 설정
    kaair_driver::MD485HwConfig config;
    config.usb_port = "/dev/ttyLM";  // ⚠️ 실제 사용하는 포트로 변경하세요
    config.baud_rate = 115200;
    config.motor_id = 1;             // ⚠️ 실제 모터 ID로 변경하세요
    config.limit_direction = false;  // 사용하시는 센서 방향에 맞게 설정
    
    // 🌟 핵심: 리미트 센서가 위치한 곳의 절대 높이를 0.1m (10cm)로 설정!
    config.offset_position = 0.1;    
    
    kaair_driver::MD485Hw driver(config);
    if (!driver.connect()) {
        std::cerr << "[ERROR] 모터 드라이버 연결에 실패했습니다: " << config.usb_port << std::endl;
        return -1;
    }
    std::cout << "[INFO] 통신 연결 성공. 알람을 클리어합니다." << std::endl;
    driver.clear_alarm();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // =====================================================================
    // 🧪 [테스트용] 호밍 시퀀스 강제 실행을 위한 상태 초기화
    // 테스트할 때는 아래 주석을 해제하여 드라이버의 INIT_SET을 0으로 만드세요.
    // =====================================================================

    // driver.set_velocity_rpm(600);
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // driver.set_velocity_rpm(0);
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // std::cout << "[DEBUG] 강제로 INIT_SET 상태를 0으로 초기화합니다." << std::endl;
    // driver.set_init_set(0);
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // =====================================================================

    uint8_t init_state = 0;
    // 🌟 read_init_set_ok가 true/false를 리턴하도록 수정된 버전에 맞게 조건문 간소화
    if (driver.read_init_set_ok(init_state)) {
        std::cout << "[INFO] 이미 영점이 잡혀 있습니다. (상태: " << (int)init_state << ")" << std::endl;
        driver.disconnect();
        return 0;
    }

    std::cout << "[INFO] 호밍 시퀀스를 시작합니다. 센서를 향해 이동합니다..." << std::endl;

    // 2. 센서를 향해 이동
    // 현재 위치가 어딘지 모르지만, 일단 센서가 있는 하단(-2.0m 등)을 향해 안전 속도(2cm/s)로 꽂아 넣습니다.
    driver.move_abs_pose_meter_with_velocity(-2.0, 0.02); 

    // 3. 센서 감지 루프
    bool limit_hit = false;
    while (true) {
        if (driver.read_init_switch_status(limit_hit)) {
            if (limit_hit) {
                std::cout << "\n[INFO] 🎯 리미트 센서 감지됨! 정지합니다." << std::endl;
                break;
            }
        } else {
            std::cerr << "\n[ERROR] 센서 상태를 읽는 중 통신 에러 발생!" << std::endl;
            driver.move_abs_pose_meter_with_velocity(config.offset_position, 0.0); // 현재 오프셋 위치에서 정지명령
            driver.disconnect();
            return -1;
        }
        
        std::cout << "." << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // 4. 모터 정지 및 관성 대기
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); 

    // 5. 🌟 영점 잡기 (Offset 동기화의 마법)
    std::cout << "[INFO] 드라이버의 카운트를 0으로 리셋합니다." << std::endl;
    driver.reset_position(); 
    // 설명: 이제 드라이버의 내부 카운트는 0이 되었습니다.
    // 수식: target_count = meter_to_count(target_position - 0.1)
    // 따라서 ROS에서 0.1m를 가라고 명령하면 target_count는 0이 됩니다. 
    // 즉, "현재 리미트를 친 위치 = ROS 기준 0.1m" 로 완벽하게 동기화가 완료되었습니다!
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // 호밍 성공 기록 남기기 (사용자 추가 로직)
    driver.set_init_set(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));


    // 6. 상태 확인 및 회피 기동
    if (driver.read_init_set_ok(init_state)) {
        std::cout << "[INFO] ✅ 영점 초기화 완벽 성공! (현재 위치 = " << config.offset_position << "m)" << std::endl;
        
        std::cout << "[INFO] 센서 보호를 위해 안전 대기 위치로 1cm 상승합니다..." << std::endl;
        driver.move_abs_pose_meter_with_velocity(config.offset_position + 0.01, 0.05); 
        
        // ==========================================================
        // 🌟 7. 위치 모니터링 (10Hz 주기로 2초간 출력)
        // ==========================================================
        std::cout << "[INFO] 현재 위치를 2초간 모니터링합니다 (10Hz)..." << std::endl;
        for (int i = 0; i < 20; ++i) {
            kaair_driver::RosDataPayload ros_main;
            if (driver.read_ros_state(ros_main)) {
                std::cout << "[MONITOR] 위치: " << ros_main.position 
                          << " m | 속도: " << ros_main.velocity << " m/s" << std::endl;
            } else {
                std::cerr << "[MONITOR] 위치 데이터 읽기 실패!" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 100ms = 10Hz
        }
        
        std::cout << "[INFO] 호밍 완료. ROS 2 컨트롤러를 실행할 준비가 되었습니다." << std::endl;
    } else {
        std::cerr << "[ERROR] 영점 설정이 드라이버에 정상 반영되지 않았습니다. (읽어온 상태값: " << (int)init_state << ")" << std::endl;
    }

    driver.disconnect();
    return 0;
}
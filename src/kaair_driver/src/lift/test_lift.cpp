#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include "kaair_driver/lift/md485_hw.hpp"

// Ctrl+C 감지를 위한 플래그
std::atomic<bool> g_running(true);

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\n[INFO] 종료 신호(Ctrl+C) 감지. 루프를 중단합니다..." << std::endl;
        g_running = false;
    }
}

int main() {
    // 1. 시그널 핸들러 등록
    std::signal(SIGINT, signal_handler);

    std::cout << "=== KAAIR Lift Test Loop ===" << std::endl;

    // 2. 하드웨어 설정
    kaair_driver::MD485HwConfig config;
    config.usb_port = "/dev/ttyLM"; 
    config.baud_rate = 115200;
    config.motor_id = 1;            
    config.limit_direction = false; 
    config.offset_position = 0.1;    
    config.global_velocity = 0.05; // 테스트용 안전 속도 (5cm/s)
    
    kaair_driver::MD485Hw driver(config);
    if (!driver.connect()) {
        std::cerr << "[ERROR] 모터 드라이버 연결 실패: " << config.usb_port << std::endl;
        return -1;
    }

    std::cout << "[INFO] 통신 연결 성공. 초기 설정을 수행합니다." << std::endl;
    driver.clear_alarm();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 3. 테스트 변수
    double target_pos = 0.15; // 15cm 지점으로 이동 테스트
    int loop_count = 0;

    // 🌟 메인 테스트 루프
    while (g_running) {
        loop_count++;

        
        // 실제 이동 명령 송신
        driver.move_abs_pose_meter_with_velocity(target_pos, config.global_velocity);
        // [A] 데이터 읽기 테스트 (ROS 상태)
        kaair_driver::RosDataPayload ros_data;
        if (driver.read_ros_state(ros_data)) {
            std::printf("[READ] #%d | Pos: %.4f m | Vel: %.4f m/s | Effort: %.2f A\n", 
                        loop_count, ros_data.position, ros_data.velocity, ros_data.effort);
        } else {
            std::cerr << "[WARN] ROS 상태 읽기 실패 (Timeout)" << std::endl;
        }

        // [B] 초기화 세팅 확인 테스트
        uint8_t init_ok = 0;
        if (driver.read_init_set_ok(init_ok)) {
            std::cout << "[READ] Init OK Status: " << (int)init_ok << std::endl;
        }

        // [C] 이동 명령 테스트 (약 2초마다 목표 위치 변경 - 왕복 테스트 예시)
        if (loop_count % 20 == 0) { // 100ms * 20 = 2초
            target_pos = (target_pos == 0.15) ? 0.25 : 0.15;
            std::cout << "[WRITE] 목표 위치 변경 -> " << target_pos << " m" << std::endl;
        }
        
        // [D] 루프 주기 조절 (너무 빠르면 RS485 부하 발생)
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 10Hz
    }

    // 4. 안전한 종료
    std::cout << "[INFO] 모터를 정지하고 연결을 해제합니다." << std::endl;
    driver.stop_brake();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    driver.disconnect();

    std::cout << "=== 테스트 종료 ===" << std::endl;
    return 0;
}
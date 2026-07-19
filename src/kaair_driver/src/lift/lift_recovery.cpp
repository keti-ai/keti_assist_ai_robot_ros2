#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include "kaair_driver/lift/md485_hw.hpp"

// Ctrl+C 감지를 위한 플래그 (비상 정지용)
std::atomic<bool> g_running(true);

void signal_handler(int signal)
{
    if (signal == SIGINT) {
        std::cout << "\n[INFO] 종료 신호(Ctrl+C) 감지. 즉시 정지합니다..." << std::endl;
        g_running = false;
    }
}

void print_usage(const char * prog_name)
{
    std::cout
        << "=== KAAIR Lift Limit Recovery Node ===\n"
        << "리미트 스위치 지점을 넘어서(오버트래블) 정지한 리프트를 위한 비상 회피 조치용 노드입니다.\n"
        << "지정한 방향으로 지정한 속도만큼만, 지정한 시간 동안 리프트를 움직인 뒤 즉시 정지합니다.\n"
        << "(lift_initializer 와 달리 리미트 센서 감지/영점 설정을 하지 않는 단순 수동 조그 동작입니다)\n\n"
        << "사용법: " << prog_name
        << " <direction: up|down> [duration_sec=3.0] [velocity_mm_s=10.0] [usb_port=/dev/ttyLM] [motor_id=1]\n\n"
        << "  direction     : up(상승) 또는 down(하강). +1 / -1 로도 입력 가능\n"
        << "  duration_sec  : 이동 시간(초). 기본값 3.0초\n"
        << "  velocity_mm_s : 이동 속도(mm/s). 기본값 10.0mm/s\n\n"
        << "예시: " << prog_name << " up 3 10\n";
}

int main(int argc, char ** argv)
{
    std::signal(SIGINT, signal_handler);

    if (argc < 2) {
        print_usage(argv[0]);
        return -1;
    }

    // 1. 방향 파싱
    std::string dir_arg = argv[1];
    int direction = 0;
    if (dir_arg == "up" || dir_arg == "+1" || dir_arg == "1") {
        direction = 1;
    } else if (dir_arg == "down" || dir_arg == "-1") {
        direction = -1;
    } else {
        std::cerr << "[ERROR] 잘못된 방향입니다: " << dir_arg << " (up/down 또는 +1/-1 사용)" << std::endl;
        print_usage(argv[0]);
        return -1;
    }

    // 2. 나머지 파라미터 파싱 (옵션, 미입력 시 기본값 사용)
    double duration_sec = (argc >= 3) ? std::atof(argv[2]) : 3.0;
    double velocity_mm_s = (argc >= 4) ? std::atof(argv[3]) : 10.0;

    if (duration_sec <= 0.0 || velocity_mm_s <= 0.0) {
        std::cerr << "[ERROR] duration_sec, velocity_mm_s는 0보다 큰 값이어야 합니다." << std::endl;
        return -1;
    }

    // 3. 하드웨어 설정
    kaair_driver::MD485HwConfig config;
    config.usb_port = (argc >= 5) ? argv[4] : "/dev/ttyLM";  // ⚠️ 실제 사용하는 포트로 변경하세요
    config.baud_rate = 115200;
    config.motor_id = (argc >= 6) ? std::atoi(argv[5]) : 1;  // ⚠️ 실제 모터 ID로 변경하세요

    const double velocity_m_s = velocity_mm_s / 1000.0;
    const double signed_velocity = velocity_m_s * static_cast<double>(direction);

    std::cout
        << "=== KAAIR Lift Limit Recovery Node ===\n"
        << "[INFO] 방향: " << (direction > 0 ? "상승(UP)" : "하강(DOWN)") << "\n"
        << "[INFO] 속도: " << velocity_mm_s << " mm/s\n"
        << "[INFO] 시간: " << duration_sec << " sec\n"
        << "[INFO] 포트: " << config.usb_port << " (motor_id: " << config.motor_id << ")\n"
        << std::endl;

    kaair_driver::MD485Hw driver(config);
    if (!driver.connect()) {
        std::cerr << "[ERROR] 모터 드라이버 연결에 실패했습니다: " << config.usb_port << std::endl;
        return -1;
    }

    std::cout << "[INFO] 통신 연결 성공. 알람을 클리어합니다." << std::endl;
    driver.clear_alarm();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 4. 시작 전 안전 카운트다운 (Ctrl+C로 취소 가능)
    std::cout << "[WARN] 잠시 후 리프트가 " << (direction > 0 ? "상승" : "하강")
               << " 방향으로 " << velocity_mm_s << "mm/s 속도로 " << duration_sec
               << "초간 움직입니다. 주변에 위험 요소가 없는지 확인하세요!" << std::endl;
    for (int i = 3; i > 0 && g_running; --i) {
        std::cout << "[INFO] " << i << "..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    if (!g_running) {
        std::cout << "[INFO] 시작 전 취소되었습니다." << std::endl;
        driver.stop_brake();
        driver.disconnect();
        return 0;
    }

    // 5. 지정된 속도로 지정된 시간만큼 이동
    std::cout << "[INFO] 이동을 시작합니다..." << std::endl;
    driver.set_linear_velocity(signed_velocity);

    const auto start_time = std::chrono::steady_clock::now();
    const auto move_duration = std::chrono::milliseconds(static_cast<int>(duration_sec * 1000));

    while (g_running) {
        if (std::chrono::steady_clock::now() - start_time >= move_duration) {
            break;
        }

        kaair_driver::RosDataPayload ros_main;
        if (driver.read_ros_state(ros_main)) {
            std::printf(
                "[MONITOR] 위치: %.4f m | 속도: %.4f m/s | 전류: %.2f A\n",
                ros_main.position, ros_main.velocity, ros_main.effort);
        } else {
            std::cerr << "[MONITOR] 상태 읽기 실패!" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // 6. 정지 (시간 만료 또는 Ctrl+C 모두 동일하게 즉시 정지)
    std::cout << "[INFO] 정지 명령을 전송합니다." << std::endl;
    driver.set_linear_velocity(0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    driver.stop_brake();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    driver.disconnect();
    std::cout << "=== 완료. 필요 시 lift_initializer로 정식 영점 초기화를 진행하세요. ===" << std::endl;
    return 0;
}

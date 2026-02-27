
#include "kaair_driver/lift/md485_hw.hpp"
#include <unistd.h>
#include <iostream>
#include <chrono>

int main() {

    kaair_driver::MD485HwConfig config;
    kaair_driver::MainDataPayload main_data;

    //실제 로봇에 연결된 시리얼 포트 이름으로 반드시 변경하세요. (예: /dev/ttyUSB0, /dev/ttyLM 등)
    config.port = "/dev/ttyLM"; 
    config.baudrate = 115200;      // MD로봇 기본 통신 속도
    config.motor_id = 1;

    kaair_driver::MD485Hw lift_motor(config);

    lift_motor.connect();

    lift_motor.set_velocity_rpm(150);

    for(int i=0;i<10;i++)
    {
        usleep(500000);

        auto start_time = std::chrono::steady_clock::now();
        lift_motor.set_velocity_rpm(50+i*10);
        bool success = lift_motor.read_state(main_data);
        auto end_time = std::chrono::steady_clock::now();
        auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        double duration_ms = duration_us / 1000.0; // 밀리초(ms)로 변환
        if(success)
        {
            std::cout << "⏱️ 통신 소요 시간: " << duration_ms << " ms" << std::endl; // 소요 시간 출력
            std::cout << "Position: " << main_data.position << std::endl;
            std::cout << "RPM: " << main_data.rpm << std::endl;
            std::cout << "Current: " << main_data.current << std::endl;
        }
    }

    return 0;
}



#include "kaair_driver/lift/md485_hw.hpp"
#include <unistd.h>
#include <iostream>
#include <chrono>

int main() {

    kaair_driver::MD485HwConfig config;
    kaair_driver::MainDataPayload main_data;
    kaair_driver::IoMonitorPayload main_io;

    kaair_driver::MD485Hw lift_motor(config);

    lift_motor.connect();

    lift_motor.set_position_with_rpm(0,300);



    for(int i=0;i<10;i++)
    {
        usleep(1000000);

        auto start_time = std::chrono::steady_clock::now();
        bool success = lift_motor.read_state(main_data);
        //bool success = lift_motor.read_state(main_io);
        auto end_time = std::chrono::steady_clock::now();
        auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        double duration_ms = duration_us / 1000.0; // 밀리초(ms)로 변환
        if(success)
        {
            std::cout << "⏱️ 통신 소요 시간: " << duration_ms << " ms" << std::endl; // 소요 시간 출력
            std::cout << "Position: " << main_data.position << std::endl;
            std::cout << "RPM: " << main_data.rpm << std::endl;
            std::cout << "Current: " << main_data.current << std::endl;



            // std::cout << main_io.dir << " " << main_io.run_brake << main_io.start_stop;
            // std::cout << std::endl;
            
        }
    }

    return 0;
}



#include "kaair_driver/lift/md485_hw.hpp"
#include <unistd.h>
#include <iostream>
#include <chrono>

int main() {

    kaair_driver::MD485HwConfig config;
    kaair_driver::IoMonitorPayload main_io;
    kaair_driver::RosDataPayload ros_main;

    kaair_driver::MD485Hw lift_motor(config);

    lift_motor.connect();
    
    // std::cout << "연결" << std::endl;
     usleep(100000);
    //lift_motor.clear_alarm();
    // std::cout << "알람제거" << std::endl;
    //usleep(100000);
    //lift_motor.reset_position();
    //std::cout << "위치초기화" << std::endl;

    lift_motor.set_max_rpm(config.max_rpm);
    usleep(100000);


    for(int i=0;i<10;i++)
    {
        usleep(1000000);

        auto start_time = std::chrono::steady_clock::now();
        bool success = lift_motor.read_ros_state(ros_main);
        //bool cmd_suc = lift_motor.set_linear_velocity(-0.01);
        lift_motor.move_abs_pose_meter_with_velocity(0.11,0.03);
        //bool success = lift_motor.read_state(main_io);
        auto end_time = std::chrono::steady_clock::now();
        auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        double duration_ms = duration_us / 1000.0; // 밀리초(ms)로 변환
        if(success)
        {
            std::cout << "⏱️ 통신 소요 시간: " << duration_ms << " ms" << std::endl; // 소요 시간 출력
            std::cout << "Position: " << ros_main.position << std::endl;
            std::cout << "Velocity: " << ros_main.velocity << std::endl;
            std::cout << "Current: " << ros_main.effort << std::endl;



            // std::cout << main_io.dir << " " << main_io.run_brake << main_io.start_stop;
            // std::cout << std::endl;
            
        }
    }

    lift_motor.stop_brake();

    usleep(100000);

    return 0;
}


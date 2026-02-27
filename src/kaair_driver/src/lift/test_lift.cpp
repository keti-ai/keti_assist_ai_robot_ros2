
#include "kaair_driver/lift/md485_hw.hpp"

int main() {

    kaair_driver::MD485HwConfig config;
    //실제 로봇에 연결된 시리얼 포트 이름으로 반드시 변경하세요. (예: /dev/ttyUSB0, /dev/ttyLM 등)
    config.port = "/dev/ttyLM"; 
    config.baudrate = 19200;      // MD로봇 기본 통신 속도
    config.motor_id = 1;

    kaair_driver::MD485Hw lift_motor(config);

    lift_motor.connect();

    bool success = lift_motor.set_velocity_rpm(300);

    return 0;
}


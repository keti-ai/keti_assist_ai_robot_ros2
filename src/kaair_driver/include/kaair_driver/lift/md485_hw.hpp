#ifndef KAAIR_DRIVER__MD485_HW_HPP_
#define KAAIR_DRIVER__MD485_HW_HPP_

#include <string>
#include <vector>
#include <memory>
#include <cmath> // M_PI 사용
#include "serial/serial.h"
#include "kaair_driver/lift/md_com.hpp"

namespace kaair_driver {

    // 1. 설정값을 담는 구조체 (기본값 세팅 완료)
    struct MD485HwConfig {
        std::string port = "/dev/ttyLM";
        int baudrate = 19200;
        std::vector<double> elevation_range = {0.0, 600.0};
        double offset_position = 0.0;
        int encoder_ppr = 1000;
        int reduction_ratio = 14;
        int lead_pitch = 10;
        std::vector<int> rpm_range = {-5000, 5000};
        bool limit_inverted = false;
        bool motor_inv = false;
        int motor_id = 1;
    };
        
    class MD485Hw {
        public:
            // 2. 개별 변수 대신 Config 구조체를 통째로 받도록 생성자 변경
            explicit MD485Hw(const MD485HwConfig & config);
            ~MD485Hw();

            // 통신 포트 열기/닫기
            bool connect();
            void disconnect();

            // 모터에 RPM 속도 명령 전송
            bool set_velocity_rpm(int16_t rpm);

        private:
            MD485HwConfig cfg_;
            std::unique_ptr<serial::Serial> serial_; // 시리얼 통신 객체 포인터
    };

} // namespace kaair_driver
#endif // KAAIR_DRIVER__MD485_HW_HPP_
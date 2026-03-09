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
        std::string usb_port = "/dev/ttyLM";
        int baud_rate = 115200;
        double offset_position = 0.1;
        int encoder_ppr = 16384;
        double lead_pitch = 0.01;
        double reduction_ratio = 1.4736842;
        double global_velocity = 0.1;
        uint16_t max_rpm = 3000;
        bool limit_direction = false;
        bool motor_direction = true;
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
            bool set_linear_velocity(double velocity);
            // 속도기반 모터 위치제어 명령 전송
            bool set_position_with_rpm(int32_t position,uint16_t rpm);
            // 선속도기반 모터 미터단위 제어 명령 전송
            bool move_abs_pose_meter_with_velocity(double position,double velocity);
            bool move_abs_pose_meter_rpm(double position,double rpm);
            // 포지션 0으로 초기화
            bool reset_position();
            // 최대 속도 설정
            bool set_max_rpm(uint16_t rpm);

            bool set_init_set(uint8_t init_mode);
            bool read_init_set_ok(uint8_t& init_state);
            // ACK 활성화
            bool return_type_ack();
            // 에러시 알람 초기화
            bool clear_alarm();
            // 정지 및 브레이크 명령
            bool stop_brake();

            // 메인 데이터와 I/O 데이터를 동시에 읽어오는 함수
            bool read_state(kaair_driver::MainDataPayload& out_main, kaair_driver::IoMonitorPayload& out_io);
            // 메인 데이터만 읽어오는 함수
            bool read_state(kaair_driver::MainDataPayload& out_main);
            // IO 데이터만 읽어오는 함수
            bool read_state(kaair_driver::IoMonitorPayload& out_io);
            // ROS 관련 데이터로 읽기 및 변환 함수 
            bool read_ros_state(kaair_driver::RosDataPayload& out_ros);
            // IO 모니터 초기화 역할 핀 읽기 함수
            bool read_init_switch_status(bool& init_staus);

        private:
            MD485HwConfig cfg_;
            std::unique_ptr<serial::Serial> serial_; // 시리얼 통신 객체 포인터
            // 수신 패킷 동기화 및 파싱 함수
            bool receive_packet(kaair_driver::PID expected_pid, std::vector<uint8_t>& out_data);
            bool send_command_and_wait_ack(const std::vector<uint8_t>& packet, kaair_driver::PID expected_pid, int timeout_ms = 100);

            double count_to_meter(int count) const;
            int meter_to_count(double meter) const;
            double rpm_to_meter_per_s(int rpm) const;
            int meter_per_s_to_rpm(double meter) const;

    };

} // namespace kaair_driver
#endif // KAAIR_DRIVER__MD485_HW_HPP_
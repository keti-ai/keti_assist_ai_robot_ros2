#ifndef KAAIR_DRIVER__KORAS_HW_HPP_
#define KAAIR_DRIVER__KORAS_HW_HPP_


#include <modbus/modbus-rtu.h>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>

namespace kaair_driver {

    class KorasHw {
    public:
        enum class Cmd : uint16_t {
            INIT = 101,
            OPEN = 102,
            CLOSE = 103,
            FINGER_POS = 104,
            VACUUM_GRP_ON = 106,
            VACUUM_GRP_OFF = 107,
            IMPEDANCE_ON = 108,
            IMPEDANCE_OFF = 109,
            SET_IMPEDANCE_PARAMS = 110,
            MOTOR_TORQUE = 212,
            MOTOR_SPEED = 213,
        };
        struct Status {
            uint16_t status = 0;            // 10
            uint16_t motor_position_deg = 0; // 11 (absolute degree, 16bit)
            uint16_t motor_current_mA = 0;   // 12
            uint16_t motor_velocity_rpm = 0; // 13
            uint16_t finger_position = 0;    // 14 (0~1000)

            // 미터(m) 단위 보간 값
            double finger_position_m = 0.0;

            uint16_t bus_voltage_V = 0;      // 17 (단위 스케일은 매뉴얼 확인)
            bool motor_enable = false;
            bool gripper_initialize = false;
            bool motor_position_control = false;
            bool motor_velocity_control = false;
            bool motor_current_control = false;
            bool gripper_open = false;
            bool gripper_close = false;
            bool motor_fault = false;

        };


        KorasHw(const std::string & usb_port, int baudrate);
        ~KorasHw();

        bool open_port();
        void close_port();
        bool isOpen() const;

        std::string lastError() const;

        // High-level API
        bool gripperInit();
        bool gripperOpen();
        bool gripperClose();

        bool impedanceOn();
        bool impedanceOff();
        bool setImpedanceParams(int gripper_type, int stiffness_level);

        bool setMotorSpeedPercent(int speed_percent);     // 1~100
        bool setMotorTorquePercent(int torque_percent);   // 50~100
        bool setFingerPosition(int pos);                  // 0~1000 (원본 동작)
        bool setFingerPositionMeter(double pos_m);

        bool readStatus(Status& out);

    private:
        bool sendOrder(Cmd cmd, uint16_t value);
        bool sendOrder(Cmd cmd, uint16_t value1, uint16_t value2);
        void close_nolock();
        static int clamp(int v, int lo, int hi);

        std::string port_name;
        int baudrate_;
        mutable std::mutex mtx_;
        modbus_t* ctx_ = nullptr;
        std::atomic<bool> is_open_{false};
        std::string last_error_;


    };
};



#endif // KAAIR_DRIVER__KORAS_HW_HPP_
#ifndef KAAIR_DRIVER__KORAS_HW_HPP_
#define KAAIR_DRIVER__KORAS_HW_HPP_


#include <modbus/modbus-rtu.h>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>


namespace kaair_driver {
    class KORASHw {
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
                uint16_t rsv15 = 0;              // 15 (-)
                uint16_t rsv16 = 0;              // 16 (-)
                uint16_t bus_voltage_V = 0;      // 17 (단위 스케일은 매뉴얼 확인)
            };


    }
}



#endif // KAAIR_DRIVER__KORAS_HW_HPP_
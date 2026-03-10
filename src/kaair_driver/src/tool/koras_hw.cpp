#include "kaair_driver/tool/koras_hw.hpp"


namespace kaair_driver {

namespace {
    static const std::vector<std::pair<double, double>> calibration_table_ = {
        {0.000, 0},
        {0.0021, 20},
        {0.0049, 40},
        {0.01228, 60},
        {0.0186, 80},
        {0.02337, 100},
        {0.0409, 200},
        {0.05365, 300},
        {0.06308, 400},
        {0.07103, 500},
        {0.07895, 600},
        {0.08385, 700},
        {0.08899, 800},
        {0.0933, 900},
        {0.09576, 1000}
    };

    static double interpolate(double input, const std::vector<std::pair<double, double>>& table, bool input_is_meter) {
        auto get_x = [&](const std::pair<double, double>& p) { return input_is_meter ? p.first : p.second; };
        auto get_y = [&](const std::pair<double, double>& p) { return input_is_meter ? p.second : p.first; };

        if (input <= get_x(table.front())) return get_y(table.front());
        if (input >= get_x(table.back())) return get_y(table.back());

        for (size_t i = 0; i < table.size() - 1; ++i) {
            double x0 = get_x(table[i]);
            double x1 = get_x(table[i+1]);
            
            if (input >= x0 && input <= x1) {
                double y0 = get_y(table[i]);
                double y1 = get_y(table[i+1]);
                return y0 + (input - x0) * (y1 - y0) / (x1 - x0);
            }
        }
        return 0.0;
    }
}


KorasHw::KorasHw(const std::string & usb_port, int baudrate)
: port_name(usb_port),baudrate_(baudrate){}


KorasHw::~KorasHw()
{
    close_port();
}

bool KorasHw::open_port()
{
    std::lock_guard<std::mutex> lk(mtx_);

    if (ctx_) close_nolock();

    ctx_ = modbus_new_rtu(port_name.c_str(), baudrate_, 'N', 8, 1);
    if (!ctx_) {
        last_error_ = "modbus_new_rtu failed";
        return false;
    }

    modbus_rtu_set_serial_mode(ctx_, MODBUS_RTU_RS485);
    modbus_rtu_set_rts_delay(ctx_, 3000);

    modbus_set_debug(ctx_, false ? 1 : 0);

    if (modbus_set_slave(ctx_, 1) == -1) {
        last_error_ = std::string("modbus_set_slave failed: ") + modbus_strerror(errno);
        close_nolock();
        return false;
    }

    if (modbus_connect(ctx_) == -1) {
        last_error_ = std::string("modbus_connect failed: ") + modbus_strerror(errno);
        close_nolock();
        return false;
    }

    is_open_.store(true);
    return true;
}

void KorasHw::close_port()
{
    std::lock_guard<std::mutex> lk(mtx_);
    close_nolock();
}

void KorasHw::close_nolock() {
    if (ctx_) {
        modbus_close(ctx_);
        modbus_free(ctx_);
        ctx_ = nullptr;
    }
    is_open_.store(false);
}

bool KorasHw::isOpen() const {
    return is_open_.load();
}

std::string KorasHw::lastError() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return last_error_;
}

bool KorasHw::gripperInit() { return sendOrder(Cmd::INIT, 0); }
bool KorasHw::gripperOpen() { return sendOrder(Cmd::OPEN, 0); }
bool KorasHw::gripperClose() { return sendOrder(Cmd::CLOSE, 0); }

bool KorasHw::impedanceOn() { return sendOrder(Cmd::IMPEDANCE_ON, 0); }
bool KorasHw::impedanceOff() { return sendOrder(Cmd::IMPEDANCE_OFF, 0); }

bool KorasHw::setMotorSpeedPercent(int speed_percent) {
    int v = clamp(speed_percent, 1, 100);
    return sendOrder(Cmd::MOTOR_SPEED, static_cast<uint16_t>(v));
}

bool KorasHw::setMotorTorquePercent(int torque_percent) {
    int v = clamp(torque_percent, 50, 100);
    return sendOrder(Cmd::MOTOR_TORQUE, static_cast<uint16_t>(v));
}

bool KorasHw::setFingerPosition(int pos) {
    int v = clamp(pos, 0, 1000);  // manual: 0(closed) ~ 1000(open)
    return sendOrder(Cmd::FINGER_POS, static_cast<uint16_t>(v));
}

bool KorasHw::setFingerPositionMeter(double pos_m) {
    // 1. 미터(m) 입력을 비선형 스트로크(0.0 ~ 1000.0)로 변환
    double target_stroke_double = interpolate(pos_m, calibration_table_, true);
    
    // 2. 소수점을 버리고 정수로 캐스팅
    int target_stroke = static_cast<int>(target_stroke_double);
    
    // 3. 안전을 위해 0~1000 사이로 한 번 더 클램핑
    int v = clamp(target_stroke, 0, 1000);
    
    // 4. 기존 통신 로직을 태워 전송
    return sendOrder(Cmd::FINGER_POS, static_cast<uint16_t>(v));
}

bool KorasHw::setImpedanceParams(int gripper_type, int stiffness_level) {
    int t = clamp(gripper_type, 1, 20);
    int s = clamp(stiffness_level, 1, 10);
    return sendOrder(Cmd::SET_IMPEDANCE_PARAMS,
                     static_cast<uint16_t>(t),
                     static_cast<uint16_t>(s));
}

// --------- Read status (Holding Registers 10~17) ----------
bool KorasHw::readStatus(Status& out) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!ctx_) {
        last_error_ = "readStatus called but ctx is null";
        return false;
    }

    constexpr int start_address = 10;
    constexpr int data_num = 8;          // ✅ 10~17 = 8개
    uint16_t regs[8] = {0,};

    int rc = modbus_read_registers(ctx_, start_address, data_num, regs);
    if (rc == -1) {
        last_error_ = std::string("modbus_read_registers failed: ") + modbus_strerror(errno);
        return false;
    }

    // manual mapping
    out.status             = regs[0]; // 10 Status
    out.motor_position_deg = regs[1]; // 11 Motor Position (absolute degree, 16bit)
    out.motor_current_mA   = regs[2]; // 12 Motor Current (mA)
    out.motor_velocity_rpm = regs[3]; // 13 Motor Velocity (rpm)
    out.finger_position    = regs[4]; // 14 Gripper Finger Position (0~1000)

    out.finger_position_m  = interpolate(static_cast<double>(out.finger_position), calibration_table_, false);

    out.bus_voltage_V      = regs[7]; // 17 Bus Voltage (V) (스케일은 매뉴얼 추가 확인)

    // 🌟 비트 마스킹을 통한 bool 상태 할당
    out.motor_enable           = (regs[0] & (1 << 0)) != 0; // Bit 0
    out.gripper_initialize     = (regs[0] & (1 << 1)) != 0; // Bit 1
    out.motor_position_control = (regs[0] & (1 << 2)) != 0; // Bit 2
    out.motor_velocity_control = (regs[0] & (1 << 3)) != 0; // Bit 3
    out.motor_current_control  = (regs[0] & (1 << 4)) != 0; // Bit 4
    out.gripper_open           = (regs[0] & (1 << 5)) != 0; // Bit 5
    out.gripper_close          = (regs[0] & (1 << 6)) != 0; // Bit 6
    out.motor_fault            = (regs[0] & (1 << 9)) != 0; // Bit 9

    return true;
}


// --------- low-level helpers ----------
bool KorasHw::sendOrder(Cmd cmd, uint16_t value) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!ctx_) {
        last_error_ = "sendOrder called but ctx is null";
        return false;
    }

    uint16_t holding_register[2];
    constexpr uint16_t start_address = 0;
    constexpr uint16_t data_num = 2;

    holding_register[0] = static_cast<uint16_t>(cmd);
    holding_register[1] = value;

    int rc = modbus_write_registers(ctx_, start_address, data_num, holding_register);
    if (rc == -1) {
        last_error_ = std::string("modbus_write_registers(2) failed: ") + modbus_strerror(errno);
        return false;
    }
    return true;
}

bool KorasHw::sendOrder(Cmd cmd, uint16_t value1, uint16_t value2) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!ctx_) {
        last_error_ = "sendOrder(3) called but ctx is null";
        return false;
    }

    uint16_t holding_register[3];
    constexpr uint16_t start_address = 0;
    constexpr uint16_t data_num = 3;

    holding_register[0] = static_cast<uint16_t>(cmd);
    holding_register[1] = value1;
    holding_register[2] = value2;

    int rc = modbus_write_registers(ctx_, start_address, data_num, holding_register);
    if (rc == -1) {
        last_error_ = std::string("modbus_write_registers(3) failed: ") + modbus_strerror(errno);
        return false;
    }
    return true;
}

int KorasHw::clamp(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}



}




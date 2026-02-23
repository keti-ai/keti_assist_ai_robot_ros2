#include "kaair_driver/head/dxl_hw.hpp"
#include <vector>
#include <iostream>
#include <unistd.h>

int main() {
    kaair_driver::DxlHw dxl("/dev/ttyUSB0", 57600);
    std::vector<uint8_t> ids = {1, 2};

    if (dxl.open_port()) {
        dxl.set_torque(ids, true);
        // 1. 현재 위치 라디안으로 읽기
        std::vector<double> cur_rads;
        if (dxl.sync_read_radian(ids, cur_rads)) {
            std::cout << "ID 1: " << cur_rads[0] << " rad, ID 2: " << cur_rads[1] << " rad" << std::endl;
        }


        // 2. 0 라디안(홈 위치 2048)으로 이동
        dxl.sync_write_radian(ids, {0.3, 0.3});
        usleep(1000000);

        if (dxl.sync_read_radian(ids, cur_rads)) {
            std::cout << "ID 1: " << cur_rads[0] << " rad, ID 2: " << cur_rads[1] << " rad" << std::endl;
        }

        // 4. 0 라디안(홈 위치 2048)으로 이동
        dxl.sync_write_radian(ids, {0., 0.});
        usleep(1000000);

        dxl.set_torque(ids, false);
    }
    return 0;
}


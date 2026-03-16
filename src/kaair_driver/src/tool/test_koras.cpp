#include "kaair_driver/tool/koras_hw.hpp"

#include <vector>
#include <iostream>
#include <unistd.h>

int main() {
    kaair_driver::KorasHw koras_("/dev/ttyGripper",115200);

    if(koras_.open_port())
    {
        kaair_driver::KorasHw::Status out;
        koras_.readStatus(out);
        std::cout << "Initialized : " << out.gripper_initialize << std::endl;
        bool ret = koras_.gripperInit();
        std::cout << "Gripper Init: " << ret << std::endl;
    }
    else
    {
        return -1;
    }
    kaair_driver::KorasHw::Status out;
    koras_.readStatus(out);
    std::cout << "Initialized : " << out.gripper_initialize << std::endl;
    usleep(5000000);

    koras_.readStatus(out);
    std::cout << "Initialized : " << out.gripper_initialize << std::endl;
    bool suc = koras_.setFingerPositionMeter(0.0);

    


    koras_.readStatus(out);

    std::cout << "Gripper Pose: " << out.finger_position_m << std::endl;
    std::cout << "Initialized : " << out.gripper_initialize << std::endl;

    return 0;
}
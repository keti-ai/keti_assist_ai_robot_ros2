#include "kaair_driver/lift/lift_hw_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace kaair_driver {

    hardware_interface::CallbackReturn LiftHwInterface::on_init(const hardware_interface:: HardwareInfo & info){
        if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) return hardware_interface::CallbackReturn::ERROR;

        if (info.joints.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("LiftHwInterface"), "LiftHwInterface expects exactly 1 joint!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

        // URDF 파라미터 로드
        if (info.hardware_parameters.count("usb_port")) 
            config.usb_port = info.hardware_parameters.at("usb_port");
        if (info.hardware_parameters.count("baud_rate")) 
            config.baud_rate = std::stoi(info.hardware_parameters.at("baud_rate"));
        if (info.hardware_parameters.count("offset_position")) 
            config.offset_position = std::stod(info.hardware_parameters.at("offset_position"));
        if (info.hardware_parameters.count("encoder_ppr")) 
            config.encoder_ppr = std::stoi(info.hardware_parameters.at("encoder_ppr"));
        if (info.hardware_parameters.count("lead_pitch")) 
            config.lead_pitch = std::stod(info.hardware_parameters.at("lead_pitch"));
        if (info.hardware_parameters.count("reduction_ratio")) 
            config.reduction_ratio = std::stod(info.hardware_parameters.at("reduction_ratio"));
        if (info.hardware_parameters.count("max_rpm")) 
            config.max_rpm = std::stoi(info.hardware_parameters.at("max_rpm"));
        if (info.hardware_parameters.count("global_velocity")) 
            config.global_velocity = std::stod(info.hardware_parameters.at("global_velocity"));

        if (info.hardware_parameters.count("limit_direction")) 
            config.limit_direction = (info.hardware_parameters.at("limit_direction") == "true");
        if (info.hardware_parameters.count("motor_direction")) 
            config.motor_direction = (info.hardware_parameters.at("motor_direction") == "true");

        if (info.joints[0].parameters.count("id")) {
            config.motor_id = std::stoi(info.joints[0].parameters.at("id"));
        }

        hw_states_.resize(3, 0.0);
        hw_commands_.resize(1, 0.0);

        md485_hw_ = std::make_unique<MD485Hw>(config);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> LiftHwInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_states_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_states_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[0].name, hardware_interface::HW_IF_EFFORT, &hw_states_[2]));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> LiftHwInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_commands_[0]));
        return command_interfaces;
    }

    hardware_interface::CallbackReturn LiftHwInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        
        if(!md485_hw_->connect()) return hardware_interface::CallbackReturn::ERROR;

        md485_hw_->clear_alarm();
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
        md485_hw_->set_max_rpm(config.max_rpm);
        std::this_thread::sleep_for(std::chrono::milliseconds(800));

        // 🌟 강력한 안전장치: 단독 초기화 프로그램이 선행되었는지 검사
        uint8_t init_state = 0;
        if (md485_hw_->read_init_set_ok(init_state) && init_state >= 1) {
            RCLCPP_INFO(rclcpp::get_logger("LiftHwInterface"), "드라이버 영점 초기화 확인 완료! 정상 제어를 시작합니다.");
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("LiftHwInterface"), 
                         "❌ 로봇 영점이 잡히지 않았습니다! ROS 컨트롤러 실행 전 'lift_initializer'를 먼저 실행하세요.");
            return hardware_interface::CallbackReturn::ERROR; // 영점이 안 잡혔으면 컨트롤러 로드를 거부함!
        }

        // 초기 시작 위치를 현재 하드웨어 위치로 동기화 (갑자기 튀는 현상 방지)
        kaair_driver::RosDataPayload ros_main;
        if(md485_hw_->read_ros_state(ros_main)) {
            hw_commands_[0] = ros_main.position;
            hw_states_[0] = ros_main.position;
            hw_states_[1] = ros_main.velocity;
            hw_states_[2] = ros_main.effort;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(800));

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn LiftHwInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        md485_hw_->stop_brake();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        md485_hw_->disconnect();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type LiftHwInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        RosDataPayload ros_main;
        auto start_time = clock_->now();

        {
            std::lock_guard<std::mutex> lock(com_mutex_);
            
            // 1. 읽기 시도 (내부적으로 flushInput()이 포함되어 있어야 함)
            bool success = md485_hw_->read_ros_state(ros_main);
            auto end_time = clock_->now();
            double duration = (end_time - start_time).seconds() * 1000.0;

            if(!success) {
                read_error_count_++;
                data_read_success_ = false; // 읽기 실패 시 이번 주기 Write 금지
                
                RCLCPP_ERROR(rclcpp::get_logger("LiftHwInterface"), 
                    "[READ FAIL] Duration: %.3f ms | Count: %d/%d", 
                    duration, read_error_count_, MAX_READ_ERRORS);

                return (read_error_count_ < MAX_READ_ERRORS) ? 
                    hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
            }
            
            // 2. 읽기 성공 처리
            data_read_success_ = true; 
            read_error_count_ = 0;
            
            hw_states_[0] = ros_main.position;
            hw_states_[1] = ros_main.velocity;
            hw_states_[2] = ros_main.effort;

            // 디버깅용 로그 (성공 시에는 1초에 한 번만 출력하도록 Throttle 권장)
            RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("LiftHwInterface"), *clock_, 1000,
                "[READ OK] Pos: %.3f m | Dur: %.3f ms", hw_states_[0], duration);
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type LiftHwInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        // 🌟 조건 1: 이번 주기에 Read가 성공했는가? (순차 보장)
        if (!data_read_success_) {
            return hardware_interface::return_type::OK;
        }

        double target_position = hw_commands_[0];
        
        // 🌟 조건 2: 목표 위치가 유의미하게 변했는가? (불필요한 통신 억제)
        // 정지 상태일 때는 Write를 생략하여 시리얼 라인 점유율을 낮춤
        if (std::abs(target_position - last_hw_command_) < 0.0001) {
            data_read_success_ = false; // 플래그 초기화
            return hardware_interface::return_type::OK;
        }

        auto start_time = clock_->now();
        {
            std::lock_guard<std::mutex> lock(com_mutex_);
            
            // 3. 쓰기 수행
            bool success = md485_hw_->move_abs_pose_meter_with_velocity(target_position, config.global_velocity);
            auto end_time = clock_->now();
            double duration = (end_time - start_time).seconds() * 1000.0;

            if(success) {
                last_hw_command_ = target_position;
                // Write 직후 미세 딜레이는 필요 시 md485_hw 내부에 구현 (보통은 불필요)
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("LiftHwInterface"), "[WRITE FAIL] Duration: %.3f ms", duration);
            }
        }

        // 다음 주기 Read를 위해 플래그 초기화
        data_read_success_ = false; 

        return hardware_interface::return_type::OK;
    }



}

PLUGINLIB_EXPORT_CLASS(kaair_driver::LiftHwInterface, hardware_interface::SystemInterface)
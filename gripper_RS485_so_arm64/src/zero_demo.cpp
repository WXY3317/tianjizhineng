#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "gripper.h"

// ──────────────────── 电机生命周期 ────────────────────

static void shutdown_gripper(std::shared_ptr<GripperController> gripper) {
    if (!gripper) return;
    std::cout << "[SHUTDOWN] Disabling motor..." << std::endl;
    gripper->disable();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    gripper->deinit();
    std::cout << "[SHUTDOWN] Motor deinitialized" << std::endl;
}

static MotorDriverConfig make_motor_config(int motor_id) {
    MotorDriverConfig config;
    config.motor_id = static_cast<std::uint16_t>(motor_id);
    config.protocol_type = "smodule_modbus";
    config.bus_type = "Rs485";
    config.bus_interface = "/dev/ttyUSB0";
    config.motor_type = "smodule_r35";
    config.motor_direction = 1;
    config.master_id_offset = 0;
    config.motor_zero_offset = 0.0;
    config.baud_rate = 115200;
    return config;
}

// ──────────────────── Main ────────────────────

int main(int argc, char** argv) {
    int motor_id = 1;
    if (argc == 2) {
        try {
            std::size_t used = 0;
            motor_id = std::stoi(argv[1], &used);
            if (used != std::string(argv[1]).size() || (motor_id != 1 && motor_id != 2))
                throw std::invalid_argument("invalid motor ID");
        } catch (...) {
            std::cerr << "Usage: zero_demo [1|2]\n";
            return 2;
        }
    } else if (argc != 1) {
        std::cerr << "Usage: zero_demo [1|2]\n";
        return 2;
    }
    std::cout << "======================================" << std::endl;
    std::cout << "  Gripper Zero Calibration Demo" << std::endl;
    std::cout << "======================================" << std::endl;

    std::cout << "[TARGET] Motor ID " << motor_id << std::endl;
    auto gripper = GripperController::create(make_motor_config(motor_id));
    if (!gripper) {
        std::cout << "[ERROR] Failed to create gripper controller" << std::endl;
        return 1;
    }

    if (gripper->init() != 0) {
        std::cout << "[ERROR] Motor init failed" << std::endl;
        return 1;
    }
    std::cout << "[INIT] Motor initialized" << std::endl;

    gripper->clear_error();
    gripper->enable();

    if (!gripper->enabled()) {
        std::cout << "[ERROR] Motor not enabled" << std::endl;
        return 1;
    }
    std::cout << "[INIT] Motor enabled" << std::endl;

    // 自动使能、张开到机械极限并执行标零
    std::cout << "\n[ZERO] Opening gripper and calibrating zero position..."
              << std::endl;
    bool ok = gripper->initialize_zero();

    if (ok) {
        std::cout << "[ZERO] Zero calibration succeeded" << std::endl;
    } else {
        std::cout << "[FAIL] Zero calibration failed" << std::endl;
    }

    shutdown_gripper(gripper);
    return ok ? 0 : 1;
}

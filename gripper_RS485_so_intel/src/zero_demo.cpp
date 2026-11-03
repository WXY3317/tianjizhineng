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

static MotorDriverConfig make_motor_config() {
    MotorDriverConfig config;
    config.motor_id = 1;
    config.protocol_type = "";
    config.bus_type = "";
    config.bus_interface = "/dev/ttyUSB0";
    config.motor_type = "smodule_r35";
    config.motor_direction = 1;
    config.master_id_offset = 0;
    config.motor_zero_offset = 0.0;
    config.baud_rate = 115200;
    return config;
}

// ──────────────────── Main ────────────────────

int main() {
    std::cout << "======================================" << std::endl;
    std::cout << "  Gripper Zero Calibration Demo" << std::endl;
    std::cout << "======================================" << std::endl;

    auto gripper = GripperController::create(make_motor_config());
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

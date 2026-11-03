#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gripper.h"
#include "gripper_tools.hpp"

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
    std::cout << "  Gripper Torque Loop Demo" << std::endl;
    std::cout << "======================================" << std::endl;

    auto gripper = GripperController::create(make_motor_config());
    if (!gripper) return 1;

    if (gripper->init() != 0) {
        std::cout << "[ERROR] Motor init failed" << std::endl;
        return 1;
    }
    std::cout << "[INIT] Motor initialized" << std::endl;

    try {
        // set_control_mode() 内部会先失能，因此必须先设置模式再使能。
        gripper->clear_error();
        gripper->set_control_mode(MotorDriver::CUR);
        std::cout << "[MODE] Position mode configured" << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "[ERROR] Failed to configure position mode: " << e.what()
                  << std::endl;
        gripper->deinit();
        return 1;
    }

    gripper->enable();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "[ENABLE] Motor enabled" << std::endl;

    // if (!gripper->initialize_zero()) {
    //     std::cout << "[ERROR] Failed to initialize zero position" << std::endl;
    //     return 1;
    // }
    // std::cout << "[ZERO] Zero position initialized" << std::endl;

    GripperCsvLogger logger(*gripper);
    logger.start_torque_log();

    const std::vector<float> targets_n = {
        25.0f, 27.0f, 30.0f, 32.0f};
    int total = static_cast<int>(targets_n.size());

    for (int i = 0; i < total; ++i) {
        std::cout << "\n--- Step " << (i + 1) << "/" << total
                  << "  target=" << targets_n[i] << " N ---" << std::endl;
        gripper_tools::run_torque_test(*gripper, targets_n[i], 2000, 50,
                                       &logger);
    }

    logger.stop_log();

    std::cout << "\n======================================" << std::endl;
    std::cout << "  All " << total << " torque tests completed" << std::endl;
    std::cout << "======================================" << std::endl;

    shutdown_gripper(gripper);
    return 0;
}

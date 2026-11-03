#include <chrono>
#include <cmath>
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

// ──────────────────── 测试参数 ────────────────────

/// 位置环测试目标序列 (m)
static const std::vector<double> POS_TARGETS = {0.06, 0.04, 0.08, 0.03, 0.06};

/// 每个目标位置的等待到位时间 (ms)
static constexpr int POS_SETTLE_MS = 3000;

/// 到位判断容差 (m)
static constexpr double POS_TOLERANCE_M = 0.003;

/// 默认运动速度 (m/s)
static constexpr float POS_DEFAULT_SPEED = 0.01f;

// ──────────────────── Main ────────────────────

int main() {
    std::cout << "======================================" << std::endl;
    std::cout << "  Gripper Position Loop Demo" << std::endl;
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
        gripper->set_control_mode(MotorDriver::POS);
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
    logger.start_pos_log();

    int pass_count = 0;
    int total = static_cast<int>(POS_TARGETS.size());

    for (int i = 0; i < total; ++i) {
        std::cout << "\n--- Step " << (i + 1) << "/" << total << " ---" << std::endl;
        const double command_deg =
            gripper->set_gripper_distance(POS_TARGETS[i], POS_DEFAULT_SPEED);
        std::cout << "[CMD] target=" << POS_TARGETS[i]
                  << "m, motor_target=" << command_deg
                  << "deg (" << command_deg / 360.0 << " rev)" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(POS_SETTLE_MS));
        logger.log_pos_state(POS_TARGETS[i], command_deg);
    }

    logger.stop_log();

    std::cout << "\n======================================" << std::endl;
    std::cout << "  Position loop test completed" << std::endl;
    std::cout << "======================================" << std::endl;

    shutdown_gripper(gripper);
    return 0;
}

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

/// 位置环目标序列 (m)
static const std::vector<double> POS_TARGETS = {0.06, 0.04, 0.08, 0.03, 0.06};

/// 每个位置目标的等待到位时间 (ms)
static constexpr int POS_SETTLE_MS = 3000;

/// 位置环默认速度 (m/s)
static constexpr float POS_DEFAULT_SPEED = 0.01f;

/// 速度环目标序列 (m/s)
static const std::vector<float> SPD_TARGETS = {0.02f, -0.01f, 0.01f, -0.02f};

/// 速度环每次测试持续时间 (ms)
static constexpr int SPD_DURATION_MS = 1000;

/// 力矩环目标序列 (夹爪末端力 N)
static const std::vector<float> TORQUE_TARGETS = {10.0f, -10.0f, 12.0f, -12.0f};

/// 力矩环每次测试持续时间 (ms)
static constexpr int TORQUE_DURATION_MS = 2000;

// ──────────────────── 阶段打印 ────────────────────

static void print_phase(const std::string &title) {
    std::cout << "\n╔══════════════════════════════════════╗" << std::endl;
    std::cout << "║  " << title << std::endl;
    std::cout << "╚══════════════════════════════════════╝" << std::endl;
}

// ──────────────────── Main ────────────────────

int main() {
    std::cout << "==============================================" << std::endl;
    std::cout << "  Gripper Combined Loop Demo" << std::endl;
    std::cout << "  Position → Velocity → Torque" << std::endl;
    std::cout << "==============================================" << std::endl;

    // ── 创建 & 初始化 ──
    auto gripper = GripperController::create(make_motor_config());
    if (!gripper) return 1;

    if (gripper->init() != 0) {
        std::cout << "[ERROR] Motor init failed" << std::endl;
        return 1;
    }
    std::cout << "[INIT] Motor initialized" << std::endl;

    // ── 上使能 ──
    gripper->enable();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "[ENABLE] Motor enabled" << std::endl;

    // ────────────── 第一阶段：位置环 ──────────────
    print_phase("Phase 1: Position Control Loop");
    GripperCsvLogger logger(*gripper);
    logger.start_pos_log();

    int pos_total = static_cast<int>(POS_TARGETS.size());
    for (int i = 0; i < pos_total; ++i) {
        std::cout << "\n--- Pos Step " << (i + 1) << "/" << pos_total
                  << "  target=" << (POS_TARGETS[i] * 1000) << "mm ---" << std::endl;
        const double command_deg =
            gripper->set_gripper_distance(POS_TARGETS[i], POS_DEFAULT_SPEED);
        std::this_thread::sleep_for(std::chrono::milliseconds(POS_SETTLE_MS));
        logger.log_pos_state(POS_TARGETS[i], command_deg);
    }

    logger.stop_log();
    std::cout << "[POS] Position loop completed" << std::endl;

    // ────────────── 第二阶段：速度环 ──────────────
    print_phase("Phase 2: Velocity Control Loop");
    logger.start_spd_log();

    int spd_total = static_cast<int>(SPD_TARGETS.size());
    for (int i = 0; i < spd_total; ++i) {
        std::cout << "\n--- Spd Step " << (i + 1) << "/" << spd_total
                  << "  target=" << SPD_TARGETS[i] << " m/s ---" << std::endl;
        gripper_tools::run_velocity_test(*gripper, SPD_TARGETS[i],
                                         SPD_DURATION_MS, 10, &logger);
    }

    logger.stop_log();
    std::cout << "[SPD] Velocity loop completed" << std::endl;

    // ────────────── 第三阶段：力矩环 ──────────────
    print_phase("Phase 3: Torque Control Loop");
    logger.start_torque_log();

    int torque_total = static_cast<int>(TORQUE_TARGETS.size());
    for (int i = 0; i < torque_total; ++i) {
        std::cout << "\n--- Torque Step " << (i + 1) << "/" << torque_total
                  << "  target=" << TORQUE_TARGETS[i] << " N ---" << std::endl;
        gripper_tools::run_torque_test(*gripper, TORQUE_TARGETS[i],
                                       TORQUE_DURATION_MS, 50, &logger);
    }

    logger.stop_log();
    std::cout << "[TORQUE] Torque loop completed" << std::endl;

    // ── 总结 ──
    std::cout << "\n==============================================" << std::endl;
    std::cout << "  All 3 control loops completed" << std::endl;
    std::cout << "  Position: " << pos_total << " targets" << std::endl;
    std::cout << "  Velocity: " << spd_total << " targets" << std::endl;
    std::cout << "  Torque:   " << torque_total << " targets" << std::endl;
    std::cout << "==============================================" << std::endl;

    // ── 下使能 ──
    shutdown_gripper(gripper);
    return 0;
}

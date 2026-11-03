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
    std::cout << "  Gripper Velocity Loop Demo" << std::endl;
    std::cout << "======================================" << std::endl;

    auto gripper = GripperController::create(make_motor_config());
    if (!gripper) return 1;

    if (gripper->init() != 0) {
        std::cout << "[ERROR] Motor init failed" << std::endl;
        return 1;
    }
    std::cout << "[INIT] Motor initialized" << std::endl;

    try{
        gripper->clear_error();
        if(gripper->get_error_id() != 0){
            std::cerr << "[ERROR] Motor error detected after init, error code: " << gripper->get_error_id() << std::endl;
            gripper->deinit();
            return 1;
        }
    } catch (const std::exception &e) {
        std::cerr << "[ERROR] Failed to clear motor error: " << e.what()
                  << std::endl;
        gripper->deinit();
        return 1;
    }

    try {
        // 先通过位置环将夹爪闭合
        gripper->set_control_mode(MotorDriver::POS);
        gripper->enable();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        gripper->set_gripper_distance(0.005, 0.05f);
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));

        if(gripper->refresh_state()){
            std::cout << "[POS] Gripper closed to "<< gripper->get_gripper_distance() <<" m" << std::endl;
        }
        gripper->disable();
    }catch (const std::exception &e) {
        std::cerr << "[ERROR] Failed to close gripper: " << e.what()
                  << std::endl;
        gripper->deinit();
        return 1;
    }

    try {
        // set_control_mode() 内部会先失能，因此必须先设置模式再使能。
        gripper->clear_error();
        gripper->set_control_mode(MotorDriver::SPD);
        std::cout << "[MODE] Speed mode configured" << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "[ERROR] Failed to configure speed mode: " << e.what()
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
    logger.start_spd_log();

    const std::vector<float> targets = {-0.3f};
    int total = static_cast<int>(targets.size());

    for (int i = 0; i < total; ++i) {
        std::cout << "\n--- Step " << (i + 1) << "/" << total << " ---" << std::endl;
        gripper_tools::run_velocity_test(*gripper, targets[i], 2000, 10,
                                         &logger);
    }

    logger.stop_log();

    std::cout << "\n======================================" << std::endl;
    std::cout << "  All " << total << " speed tests completed" << std::endl;
    std::cout << "======================================" << std::endl;

    shutdown_gripper(gripper);
    return 0;
}

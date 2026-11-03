// Diagnostic snapshot: never enables, homes, clears errors or commands motion.
#include "gripper.h"
#include <iomanip>
#include <iostream>
#include <thread>

int main(int argc, char** argv) {
    const bool set_open_zero = argc == 2 && std::string(argv[1]) == "--set-open-zero";
    if (argc > 1 && !set_open_zero) return 1;
    MotorDriverConfig config;
    config.motor_id = 1;
    config.protocol_type = "smodule_modbus";
    config.bus_type = "Rs485";
    config.bus_interface = "/dev/ttyUSB0";
    config.motor_type = "smodule_r35";
    config.motor_direction = 1;
    config.master_id_offset = 0;
    config.motor_zero_offset = 0;
    config.baud_rate = 115200;
    auto gripper = GripperController::create(config);
    if (!gripper) return 1;
    int result = 0;
    try {
        if (gripper->init() != 0) throw std::runtime_error("init failed");
        gripper->disable();
        if (set_open_zero) {
            if (!gripper->refresh_state() || gripper->get_error_id() != 0 || gripper->enabled())
                throw std::runtime_error("device state unhealthy; refuse zero");
            const auto before = gripper->get_motor_pos();
            // Guard this one-time operation against an unexpected physical move.
            if (std::abs(before - 70.5596926925) > 1.0)
                throw std::runtime_error("position changed since confirmed mechanical open endpoint");
            if (!gripper->set_zero()) throw std::runtime_error("set_zero failed");
            std::cout << "OPEN_ZERO_SET previous_deg=" << before << std::endl;
        }
        for (int i = 0; i < 10; ++i) {
            if (!gripper->refresh_state()) throw std::runtime_error("state refresh failed");
            const auto raw_deg = gripper->get_motor_pos();
            const auto distance = gripper->get_gripper_distance();
            std::cout << std::setprecision(12) << std::boolalpha
                << "SNAPSHOT {\"motor_position_deg\":" << raw_deg
                << ",\"api_distance_m\":" << distance
                << ",\"motor_torque_percent\":" << gripper->get_motor_torque()
                << ",\"error_code\":" << gripper->get_error_id()
                << ",\"enabled\":" << gripper->enabled() << "}" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; result = 1; }
    try { gripper->disable(); } catch (...) { result = 1; }
    try { gripper->deinit(); } catch (...) { result = 1; }
    return result;
}

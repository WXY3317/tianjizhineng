#include "gripper_tools.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "gripper.h"
#include "utils.hpp"

namespace {

constexpr double POSITION_TOLERANCE_M = 0.003;

void validate_test_args(float command, int duration_ms, int sample_ms) {
    if (!std::isfinite(command) || duration_ms <= 0 || sample_ms <= 0) {
        throw std::invalid_argument(
            "command must be finite and durations must be positive");
    }
}

void stop_velocity_safely(GripperController &gripper) noexcept {
    try {
        gripper.set_gripper_velocity(0.0f);
    } catch (...) {
        try {
            gripper.disable();
        } catch (...) {
        }
    }
}

void stop_torque_safely(GripperController &gripper) noexcept {
    try {
        gripper.set_gripper_torque(0.0f);
    } catch (...) {
        try {
            gripper.disable();
        } catch (...) {
        }
    }
}

}  // namespace

GripperCsvLogger::GripperCsvLogger(GripperController &gripper)
    : gripper_(gripper), start_time_(std::chrono::steady_clock::now()) {}

void GripperCsvLogger::open_log(const std::string &prefix,
                                const std::string &header) {
    if (log_file_.is_open()) log_file_.close();
    log_file_.clear();
    create_log_dir();

    const std::string path = "log/" + prefix + "_" + now_filename() + ".csv";
    log_file_.open(path);
    if (!log_file_.is_open()) {
        throw std::runtime_error("failed to open log file: " + path);
    }

    log_file_ << header << '\n';
    start_time_ = std::chrono::steady_clock::now();
    std::cout << "[LOG] " << path << std::endl;
}

void GripperCsvLogger::start_pos_log() {
    open_log("pos_log", "time,set_m,cmd_deg,actual_m,err_m,result");
}

void GripperCsvLogger::start_spd_log() {
    open_log("spd_log", "time,cmd_mps,actual_mps,pos_m,elapsed_s");
}

void GripperCsvLogger::start_torque_log() {
    open_log("torque_log",
             "time,cmd_force_n,actual_force_n,pos_deg,elapsed_s");
}

void GripperCsvLogger::log_pos_state(double target_m, double command_deg) {
    const double actual_m = gripper_.get_gripper_distance();
    const double error_m = actual_m - target_m;
    const bool reached = std::fabs(error_m) < POSITION_TOLERANCE_M;

    std::cout << (reached ? "[PASS]" : "[FAIL]")
              << " target=" << target_m << "m, actual=" << actual_m
              << "m, err=" << error_m << "m" << std::endl;

    if (log_file_.is_open()) {
        log_file_ << now_str() << ',' << std::fixed << std::setprecision(4)
                  << target_m << ',' << std::setprecision(6) << command_deg
                  << ',' << std::setprecision(4) << actual_m << ',' << error_m
                  << ',' << (reached ? "REACHED" : "TIMEOUT") << '\n';
    }
}

void GripperCsvLogger::log_spd_state(float command_mps) {
    const double elapsed_s = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - start_time_)
                                 .count();
    const float actual_mps = gripper_.get_gripper_velocity();
    const double position_m = gripper_.get_gripper_distance();

    std::cout << "[SPD_LOG] t=" << std::fixed << std::setprecision(2)
              << elapsed_s << "s, cmd=" << command_mps
              << " m/s, actual=" << actual_mps
              << " m/s, pos=" << position_m << " m" << std::endl;

    if (log_file_.is_open()) {
        log_file_ << now_str() << ',' << std::fixed << std::setprecision(4)
                  << command_mps << ',' << actual_mps << ',' << position_m
                  << ',' << std::setprecision(3) << elapsed_s << '\n';
    }
}

void GripperCsvLogger::log_torque_state(float command_force_n) {
    const double elapsed_s = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - start_time_)
                                 .count();
    const float actual_force_n = gripper_.get_gripper_torque();
    const double position_deg = gripper_.get_motor_pos();

    std::cout << "[TORQUE_LOG] t=" << std::fixed << std::setprecision(2)
              << elapsed_s << "s, cmd=" << command_force_n
              << " N, actual=" << actual_force_n << " N" << std::endl;

    if (log_file_.is_open()) {
        log_file_ << now_str() << ',' << std::fixed << std::setprecision(2)
                  << command_force_n << ',' << actual_force_n << ','
                  << std::setprecision(6) << position_deg << ','
                  << std::setprecision(3) << elapsed_s << '\n';
    }
}

void GripperCsvLogger::stop_log() {
    if (!log_file_.is_open()) return;
    log_file_.close();
    std::cout << "[LOG] Log closed" << std::endl;
}

namespace gripper_tools {

int run_velocity_test(GripperController &gripper, float speed_mps,
                      int duration_ms, int sample_ms,
                      GripperCsvLogger *logger) {
    validate_test_args(speed_mps, duration_ms, sample_ms);
    std::cout << "[SPD_CMD] cmd=" << speed_mps << " m/s" << std::endl;

    const auto start = std::chrono::steady_clock::now();
    int samples = 0;
    try {
        gripper.set_gripper_velocity(0.0f);
        if (logger) logger->log_spd_state(0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(sample_ms));

        while (std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - start)
                   .count() < duration_ms) {
            gripper.set_gripper_velocity(speed_mps);
            if (logger) logger->log_spd_state(speed_mps);
            ++samples;
            std::this_thread::sleep_for(std::chrono::milliseconds(sample_ms));
        }
        gripper.set_gripper_velocity(0.0f);
    } catch (...) {
        stop_velocity_safely(gripper);
        throw;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "[SPD_DONE] speed=" << speed_mps
              << " m/s, samples=" << samples << std::endl;
    return samples;
}

int run_torque_test(GripperController &gripper, float force_n,
                    int duration_ms, int sample_ms,
                    GripperCsvLogger *logger) {
    validate_test_args(force_n, duration_ms, sample_ms);
    std::cout << "[TORQUE_CMD] cmd=" << force_n << " N" << std::endl;

    const auto start = std::chrono::steady_clock::now();
    int samples = 0;
    try {
        while (std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - start)
                   .count() < duration_ms) {
            gripper.set_gripper_torque(force_n);
            if (logger) logger->log_torque_state(force_n);
            ++samples;
            std::this_thread::sleep_for(std::chrono::milliseconds(sample_ms));
        }
        gripper.set_gripper_torque(0.0f);
    } catch (...) {
        stop_torque_safely(gripper);
        throw;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "[TORQUE_DONE] force=" << force_n
              << " N, samples=" << samples << std::endl;
    return samples;
}

}  // namespace gripper_tools

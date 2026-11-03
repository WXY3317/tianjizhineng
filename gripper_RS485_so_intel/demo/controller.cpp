#include "controller.hpp"
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <iostream>

namespace gripper_demo {
Controller::Controller(std::unique_ptr<Hardware> hardware, double current_threshold_a)
    : hardware_(std::move(hardware)), current_threshold_a_(current_threshold_a) {
    if (!hardware_) throw std::invalid_argument("hardware is null");
    if (!std::isfinite(current_threshold_a_) || current_threshold_a_ <= 0)
        throw std::invalid_argument("current threshold must be finite and positive");
}
Controller::~Controller() { hardware_->shutdown(); }
void Controller::init(bool use_existing_zero) {
    hardware_->init();
    if (use_existing_zero) {
        const auto f = sample();
        if (!f.valid || f.error_code != 0 || f.enabled)
            throw std::runtime_error("cannot use existing zero: device state unhealthy");
        homed_ = true;
    }
}
void Controller::fail_safe() noexcept {
    target_valid_ = false;
    current_limit_reached_ = false;
    homed_ = false;
    try { hardware_->disable(); } catch (...) {} // Communication loss may prevent disable.
}
Feedback Controller::sample() {
    Feedback f;
    f.command_id = last_command_id_;
    HardwareState state{};
    try {
        if (!hardware_->read(state)) throw std::runtime_error("state refresh failed");
        if (!std::isfinite(state.position_m) || !std::isfinite(state.motor_position_rad) || !std::isfinite(state.motor_current_a) || !std::isfinite(state.motor_torque_percent) ||
            !std::isfinite(state.force_n)) throw std::runtime_error("non-finite feedback");
        f.valid = true;
        f.position_m = state.position_m;
        f.motor_position_rad = state.motor_position_rad;
        f.motor_current_a = state.motor_current_a;
        f.motor_torque_percent = state.motor_torque_percent;
        f.force_n = state.force_n;
        f.error_code = state.error_code;
        f.enabled = state.enabled;
        if (state.error_code != 0) {
            fail_safe();
            // Re-read to avoid publishing the pre-disable enabled flag.
            HardwareState stopped{};
            if (hardware_->read(stopped)) f.enabled = stopped.enabled;
            else f.valid = false;
        } else if (state.enabled && target_valid_ && !current_limit_reached_ &&
                   std::abs(state.motor_current_a) >= current_threshold_a_) {
            hardware_->hold_position(state.motor_position_rad, target_speed_mps_);
            target_m_ = state.position_m;
            current_limit_reached_ = true;
            std::cerr << "[CURRENT HOLD] command_id=" << last_command_id_
                      << " current_a=" << state.motor_current_a
                      << " captured_position_m=" << target_m_ << '\n';
        }
    } catch (const std::exception& e) {
        std::cerr << "[FEEDBACK ERROR] " << e.what() << " raw_motor_rad=" << state.motor_position_rad << '\n';
        fail_safe(); f.valid = false;
    } catch (...) { fail_safe(); f.valid = false; }
    f.homed = homed_;
    f.current_threshold_a = current_threshold_a_;
    f.current_limit_reached = current_limit_reached_;
    f.target_active = f.valid && target_valid_;
    f.target_position_m = target_m_;
    f.holding_position = f.valid && f.enabled && current_limit_reached_ && target_valid_;
    f.at_target = f.valid && f.error_code == 0 && f.enabled && target_valid_ &&
                  !current_limit_reached_ &&
                  std::abs(f.position_m - target_m_) <= 0.001; // Position tolerance only.
    f.stamp_ns = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    return f;
}
Result Controller::command(const Command& cmd) {
    if (cmd.action == Action::Move &&
        (!std::isfinite(cmd.position_m) || cmd.position_m < 0 || cmd.position_m > 0.098 ||
         !std::isfinite(cmd.speed_mps) || cmd.speed_mps <= 0))
        return {false, "position must be 0..0.098 m; speed must be finite and positive"};
    try {
        if (cmd.action == Action::Disable) {
            hardware_->disable(); target_valid_ = false; current_limit_reached_ = false;
        } else if (cmd.action == Action::Clear) {
            hardware_->clear();
            const auto f = sample();
            if (!f.valid || f.error_code != 0) return {false, "fault or communication failure remains"};
        } else {
            const auto f = sample();
            if (!f.valid || f.error_code != 0) return {false, "feedback unavailable or driver fault"};
            switch (cmd.action) {
            case Action::Zero:
                target_valid_ = false; homed_ = false; current_limit_reached_ = false;
                if (!hardware_->zero()) { fail_safe(); return {false, "zero failed"}; }
                // Manual requires disable after calibration; do not depend on library side effects.
                hardware_->disable();
                {
                    const auto zeroed = sample();
                    if (!zeroed.valid || zeroed.error_code != 0 || zeroed.enabled) {
                        fail_safe(); return {false, "zero completion state not confirmed"};
                    }
                }
                homed_ = true;
                break;
            case Action::Enable:
                if (!homed_) return {false, "ZERO required before enable"};
                hardware_->enable_position();
                {
                    const auto enabled = sample();
                    if (!enabled.valid || enabled.error_code != 0 || !enabled.enabled || !enabled.homed) {
                        fail_safe(); return {false, "enable not confirmed"};
                    }
                }
                break;
            case Action::Move:
                if (!homed_ || !f.enabled) return {false, "ZERO and ENABLE required before MOVE"};
                hardware_->move(cmd.position_m, cmd.speed_mps);
                target_m_ = cmd.position_m; target_valid_ = true;
                target_speed_mps_ = cmd.speed_mps; current_limit_reached_ = false;
                break;
            default: return {false, "unknown action"};
            }
        }
        last_command_id_ = cmd.command_id;
        return {true, "accepted"};
    } catch (const std::exception& e) { fail_safe(); return {false, e.what()}; }
}
} // namespace gripper_demo

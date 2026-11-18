#include "controller.hpp"
#include "calibration.hpp"
#include "motor_driver.hpp"
#include "gripper.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace gripper_demo {
// Keep SDK cout logging off the JSON message stream; all SDK access is serialized.
class VendorOutput {
public:
    VendorOutput() : original_(std::cout.rdbuf(std::cerr.rdbuf())) {}
    ~VendorOutput() { std::cout.rdbuf(original_); }
private:
    std::streambuf* original_;
};
class Rs485Hardware final : public Hardware {
public:
    Rs485Hardware(const std::string& port, int id, int baud, const std::string& calibration_path)
        : calibration_(calibration_path.empty() ? DEMO_CALIBRATION_PATH : calibration_path), motor_id_(id) {
        VendorOutput logs;
        if (port.empty() || (id < 1 || id > 247) || baud <= 0)
            throw std::invalid_argument("port must be nonempty, Modbus ID must be 1..247 and baud must be positive");
        MotorDriverConfig config;
        config.motor_id = static_cast<std::uint16_t>(id);
        config.protocol_type = "smodule_modbus";
        config.bus_type = "Rs485";
        config.bus_interface = port;
        config.motor_type = "smodule_r35";
        config.motor_direction = 1;
        config.master_id_offset = 0;
        config.motor_zero_offset = 0;
        config.baud_rate = baud;
        motor_ = MotorDriverFactory::instance().create(config);
        if (!motor_) throw std::runtime_error("driver creation failed");
    }
    void init() override {
        VendorOutput logs;
        auto init_result = motor_->init_motor();
        for (int retry = 0; init_result != 0 && retry < 2; ++retry) {
            std::cerr << "[INIT] Motor ID " << motor_id_ << " init_motor returned "
                      << static_cast<unsigned>(init_result) << "; retrying\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            init_result = motor_->init_motor();
        }
        if (init_result != 0)
            throw std::runtime_error("motor ID " + std::to_string(motor_id_) +
                                     " driver initialization failed: init_motor return code " +
                                     std::to_string(static_cast<unsigned>(init_result)));
        motor_->set_bus_control_enabled(true);
        motor_->disable();
    }
    bool read(HardwareState& state) override {
        VendorOutput logs;
        if (!motor_->refresh_state()) return false;
        const double angle = motor_->get_motor_pos(); // Native SDK radians.
        state.motor_position_rad = angle;
        state.position_m = calibration_.width(angle);
        state.motor_torque_percent = motor_->get_motor_torque();
        state.motor_current_a = motor_->get_motor_current();
        state.force_n = calibration_.estimated_force(angle, state.motor_torque_percent);
        state.error_code = motor_->get_error_id();
        state.enabled = motor_->enabled();
        return true;
    }
    bool zero() override {
        VendorOutput logs;
        // Use the vendor's torque based homing, exactly as zero_demo does.
        // The previous low speed homing could mistake friction mid-stroke for
        // the opening stop and overwrite the zero point there.
        motor_->disable();
        if (!motor_->refresh_state() || !std::isfinite(motor_->get_motor_pos()) ||
            std::abs(motor_->get_motor_spd()) > 0.05)
            throw std::runtime_error("homing initial state invalid");
        GripperController sdk(motor_); // Shares the already initialized bus and motor.
        try {
            sdk.clear_error();
            if (!motor_->refresh_state() || motor_->get_error_id() != 0)
                throw std::runtime_error("homing driver fault remains");
            sdk.enable();
            if (!sdk.enabled())
                throw std::runtime_error("homing enable failed");
            const bool ok = sdk.initialize_zero();
            sdk.disable();
            if (!ok)
                throw std::runtime_error("SDK initialize_zero failed");
            if (!motor_->refresh_state() || motor_->get_error_id() != 0 || motor_->enabled() ||
                !std::isfinite(motor_->get_motor_pos()) ||
                std::abs(motor_->get_motor_pos()) > 0.03)
                throw std::runtime_error("homing zero feedback invalid");
            std::cerr << "[HOMING] SDK initialize_zero succeeded; angle_rad="
                      << motor_->get_motor_pos() << '\n';
            return true;
        } catch (...) {
            try { sdk.disable(); } catch (...) {}
            throw;
        }
    }
    void enable_position() override {
        VendorOutput logs;
        motor_->set_motor_control_mode(MotorDriver::POS);
        motor_->enable();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    void disable() override { VendorOutput logs; motor_->disable(); }
    void clear() override { VendorOutput logs; motor_->clear_motor_error(); }
    void move(double position_m, float speed_mps) override {
        VendorOutput logs;
        const double angle = calibration_.angle(position_m);
        const double speed = calibration_.angular_speed(position_m, speed_mps);
        if (!std::isfinite(speed) || speed <= 0) throw std::invalid_argument("invalid calibrated speed");
        // Native position control avoids the old API's negative model-width rejection.
        motor_->motor_pos_cmd(static_cast<float>(angle), static_cast<float>(speed));
    }
    void shutdown() noexcept override {
        VendorOutput logs;
        if (!motor_) return;
        try { motor_->disable(); } catch (...) {}
        try { motor_->set_bus_control_enabled(false); } catch (...) {}
        try { motor_->deinit_motor(); } catch (...) {}
    }
    void hold_position(double angle, float speed_mps) override {
        VendorOutput logs;
        const double width = calibration_.width(angle);
        const double speed = calibration_.angular_speed(std::clamp(width, 0.0, calibration_.max_width()), speed_mps);
        motor_->motor_pos_cmd(static_cast<float>(angle), static_cast<float>(speed));
    }
    double max_position_m() const override { return calibration_.max_width(); }
    double min_position_m() const override { return 0.0; }
private:
    Calibration calibration_;
    int motor_id_;
    std::shared_ptr<MotorDriver> motor_;
};
std::unique_ptr<Hardware> make_hardware(const std::string& port, int id, int baud, const std::string& calibration_path) {
    return std::make_unique<Rs485Hardware>(port, id, baud, calibration_path);
}
} // namespace gripper_demo

#include "motor_driver.hpp"
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <sys/wait.h>
#include <unistd.h>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void on_signal(int) { interrupted = 1; }
int number(const char* s) {
    std::size_t end = 0;
    const std::string value(s);
    int n = std::stoi(value, &end);
    if (end != value.size()) throw std::invalid_argument("invalid integer");
    return n;
}
double real(const char* s) {
    std::size_t end = 0;
    const std::string value(s);
    double n = std::stod(value, &end);
    if (end != value.size() || !std::isfinite(n)) throw std::invalid_argument("invalid number");
    return n;
}
void idle_port(const char* port) {
    const pid_t child = fork();
    if (child < 0) throw std::runtime_error("cannot check serial port");
    if (child == 0) {
        // fuser may print /proc permission warnings while scanning unrelated
        // processes. Its exit status still reports whether the port is in use.
        if (!std::freopen("/dev/null", "w", stderr)) _exit(127);
        execlp("fuser", "fuser", "-s", port, static_cast<char*>(nullptr));
        _exit(127);
    }
    int status = 0;
    if (waitpid(child, &status, 0) != child || !WIFEXITED(status)) throw std::runtime_error("port check interrupted");
    if (WEXITSTATUS(status) == 0) throw std::runtime_error("serial port occupied; stop ROS node first");
    if (WEXITSTATUS(status) != 1) throw std::runtime_error("fuser is required to check serial port");
}
struct Session {
    std::shared_ptr<MotorDriver> motor;
    bool bus = false;
    bool speed = false;
    bool stop() noexcept {
        bool ok = true;
        try { if (speed) motor->motor_spd_cmd(0); } catch (...) { ok = false; }
        try { motor->disable(); } catch (...) { ok = false; }
        try { if (bus) motor->set_bus_control_enabled(false); } catch (...) { ok = false; }
        try { motor->deinit_motor(); } catch (...) { ok = false; }
        bus = false; speed = false;
        return ok;
    }
    ~Session() { if (motor && bus) stop(); }
};
void check(MotorDriver& motor, bool enabled) {
    if (interrupted || !motor.refresh_state()) throw std::runtime_error("interrupted or feedback failed");
    if (motor.get_error_id() != 0) throw std::runtime_error("driver fault");
    if (motor.enabled() != enabled) throw std::runtime_error("unexpected enable state");
    const double current = motor.get_motor_current();
    if (!std::isfinite(current) || std::abs(current) >= 0.5) throw std::runtime_error("0.5 A jog current threshold reached");
}
void print_state(MotorDriver& motor) {
    check(motor, false);
    std::cout << std::setprecision(12) << "angle_rad=" << motor.get_motor_pos()
              << " current_a=" << motor.get_motor_current()
              << " speed_rad_s=" << motor.get_motor_spd() << '\n';
}
} // namespace

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "Usage: gripper_calibrate PORT ID status|open|close|set-open-zero [duration_ms|expected_angle_rad]\n"
                  << "open/close: 100..5000 ms at -/+0.5 motor rad/s, then stop and disable.\n"
                  << "set-open-zero: requires expected angle within 0.03 rad of current angle.\n";
        return 0;
    }
    try {
        if (argc < 4 || argc > 5) throw std::invalid_argument("use --help for syntax");
        const std::string port(argv[1]), action(argv[3]);
        const int id = number(argv[2]);
        if (port.empty() || port.front() != '/' || id < 1 || id > 127 || access(port.c_str(), R_OK | W_OK) != 0)
            throw std::invalid_argument("accessible absolute serial path and ID 1..127 required");
        if (action != "status" && action != "open" && action != "close" && action != "set-open-zero")
            throw std::invalid_argument("unknown action");
        if ((action == "status" && argc != 4) || (action == "set-open-zero" && argc != 5))
            throw std::invalid_argument("wrong number of arguments");
        const int duration = (action == "open" || action == "close") ? (argc == 5 ? number(argv[4]) : 1000) : 0;
        if (duration && (duration < 100 || duration > 5000)) throw std::invalid_argument("duration must be 100..5000 ms");
        const double expected = action == "set-open-zero" ? real(argv[4]) : 0;
        std::signal(SIGINT, on_signal); std::signal(SIGTERM, on_signal);
        idle_port(port.c_str());
        MotorDriverConfig config;
        config.motor_id = static_cast<std::uint16_t>(id);
        config.protocol_type = "smodule_modbus"; config.bus_type = "Rs485";
        config.bus_interface = port; config.motor_type = "smodule_r35";
        config.motor_direction = 1; config.baud_rate = 115200;
        Session session{MotorDriverFactory::instance().create(config)};
        if (!session.motor || session.motor->init_motor() != 0) throw std::runtime_error("motor init failed");
        auto& motor = *session.motor;
        session.bus = true;
        motor.disable(); check(motor, false);
        if (std::abs(motor.get_motor_spd()) > 0.05) throw std::runtime_error("motor still moving");
        if (action == "status") { print_state(motor); return session.stop() ? 0 : 1; }
        if (action == "set-open-zero") {
            const double before = motor.get_motor_pos();
            if (!std::isfinite(before) || std::abs(before - expected) > 0.03)
                throw std::runtime_error("angle differs from confirmed open endpoint; refusing zero");
            session.bus = true; motor.set_bus_control_enabled(true);
            if (!motor.set_motor_zero()) throw std::runtime_error("set_motor_zero failed");
            check(motor, false);
            const double after = motor.get_motor_pos();
            if (!std::isfinite(after) || std::abs(after) > 0.03)
                throw std::runtime_error("zero write not confirmed by feedback");
            std::cout << std::setprecision(12) << "open_zero_set previous_angle_rad=" << before << " angle_rad=" << after << '\n';
            return session.stop() ? 0 : 1;
        }
        session.bus = true; motor.set_bus_control_enabled(true);
        motor.set_motor_control_mode(MotorDriver::SPD); session.speed = true;
        motor.motor_spd_cmd(0); check(motor, false);
        const double before = motor.get_motor_pos();
        motor.enable(); std::this_thread::sleep_for(std::chrono::milliseconds(500)); check(motor, true);
        const float command = action == "open" ? -0.5f : 0.5f;
        motor.motor_spd_cmd(command);
        const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(duration);
        double peak_current = 0;
        while (std::chrono::steady_clock::now() < end) {
            check(motor, true);
            peak_current = std::max(peak_current, std::abs(static_cast<double>(motor.get_motor_current())));
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        motor.motor_spd_cmd(0); motor.disable(); std::this_thread::sleep_for(std::chrono::milliseconds(100));
        check(motor, false);
        const double after = motor.get_motor_pos();
        std::cout << std::setprecision(12) << "before_rad=" << before << " after_rad=" << after
                  << " delta_rad=" << after-before << " sampled_peak_current_a=" << peak_current << '\n';
        return session.stop() ? 0 : 1;
    } catch (const std::exception& e) { std::cerr << "Calibration tool error: " << e.what() << '\n'; return 1; }
}

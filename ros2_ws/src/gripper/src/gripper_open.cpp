#include "motor_driver.hpp"

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <sys/wait.h>
#include <unistd.h>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void interrupt_handler(int) { interrupted = 1; }

int integer(const char* value) {
  std::size_t end = 0;
  const std::string text(value);
  const int result = std::stoi(text, &end);
  if (end != text.size()) throw std::invalid_argument("invalid integer: " + text);
  return result;
}

// Check before the SDK opens its own descriptor. Other serial applications must
// remain stopped for the entire jog; the vendor SDK does not lock the device.
void require_idle_port(const char* port) {
  const pid_t child = fork();
  if (child < 0) throw std::runtime_error("cannot check serial port ownership");
  if (child == 0) {
    execlp("fuser", "fuser", "-s", port, static_cast<char*>(nullptr));
    _exit(127);
  }
  int status = 0;
  if (waitpid(child, &status, 0) != child || !WIFEXITED(status))
    throw std::runtime_error("serial ownership check interrupted");
  if (WEXITSTATUS(status) == 0)
    throw std::runtime_error("serial port is occupied; stop the gripper node first");
  if (WEXITSTATUS(status) != 1)
    throw std::runtime_error("cannot check serial ownership (fuser is required)");
}

struct Session {
  std::shared_ptr<MotorDriver> motor;
  bool speed_mode = false;
  bool bus_control = false;
  bool cleaned = false;

  bool stop() noexcept {
    if (cleaned || !motor) return true;
    cleaned = true;
    bool ok = true;
    const auto attempt = [&ok](auto operation) {
      try { operation(); }
      catch (const std::exception& error) {
        std::cerr << "Cleanup failed: " << error.what() << '\n';
        ok = false;
      }
      catch (...) { ok = false; }
    };
    if (speed_mode) attempt([&] { motor->motor_spd_cmd(0.0f); });
    attempt([&] { motor->disable(); });
    if (bus_control) attempt([&] { motor->set_bus_control_enabled(false); });
    attempt([&] { motor->deinit_motor(); });
    return ok;
  }
  ~Session() { stop(); }
};

void check_state(MotorDriver& motor, bool expect_enabled) {
  if (interrupted) throw std::runtime_error("jog interrupted");
  if (!motor.refresh_state()) throw std::runtime_error("state refresh failed");
  const auto error = motor.get_error_id();
  if (error != 0) {
    std::cerr << "Driver error bitmap: 0x" << std::hex << error << std::dec << '\n';
    throw std::runtime_error("driver fault; no automatic error clearing");
  }
  const double current = motor.get_motor_current();
  if (!std::isfinite(current) || std::abs(current) >= 0.5)
    throw std::runtime_error("invalid current or 0.5 A jog current limit reached");
  if (motor.enabled() != expect_enabled)
    throw std::runtime_error("unexpected motor enable state");
  if (interrupted) throw std::runtime_error("jog interrupted");
}
}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "Usage: gripper_open PORT MOTOR_ID [DURATION_MS]\n"
                 "Opening jog only: -0.5 motor rad/s, 115200 baud, direction +1.\n"
                 "Duration defaults to 1000 ms; allowed range 100..2000 ms.\n"
                 "Stops and disables afterwards; never clears faults or writes zero.\n";
    return 0;
  }
  try {
    if (argc < 3 || argc > 4)
      throw std::invalid_argument("usage: gripper_open PORT MOTOR_ID [DURATION_MS]");
    const std::string port(argv[1]);
    const int id = integer(argv[2]);
    const int duration = argc == 4 ? integer(argv[3]) : 1000;
    if (port.empty() || port.front() != '/' || id < 1 || id > 127)
      throw std::invalid_argument("absolute serial path and motor ID 1..127 required");
    if (duration < 100 || duration > 2000)
      throw std::invalid_argument("duration must be 100..2000 ms");
    if (access(port.c_str(), R_OK | W_OK) != 0)
      throw std::runtime_error("serial device missing or not readable/writable");
    std::signal(SIGINT, interrupt_handler);
    std::signal(SIGTERM, interrupt_handler);
    require_idle_port(port.c_str());

    MotorDriverConfig config;
    config.motor_id = static_cast<std::uint16_t>(id);
    config.protocol_type = "smodule_modbus";
    config.bus_type = "Rs485";
    config.bus_interface = port;
    config.motor_type = "smodule_r35";
    config.motor_direction = 1;
    config.baud_rate = 115200;
    Session session{MotorDriverFactory::instance().create(config)};
    if (!session.motor) throw std::runtime_error("driver creation failed");
    auto& motor = *session.motor;
    if (motor.init_motor() != 0) throw std::runtime_error("driver initialization failed");
    motor.disable();
    check_state(motor, false);
    if (!std::isfinite(motor.get_motor_spd()) || std::abs(motor.get_motor_spd()) > 0.05f)
      throw std::runtime_error("motor is still moving; wait for it to stop");
    // Mark ownership before the operation so cleanup also handles partial writes.
    session.bus_control = true;
    motor.set_bus_control_enabled(true);
    motor.set_motor_control_mode(MotorDriver::SPD);
    session.speed_mode = true;
    motor.motor_spd_cmd(0.0f);
    check_state(motor, false);
    const double initial_angle = motor.get_motor_pos();
    if (!std::isfinite(initial_angle))
      throw std::runtime_error("invalid encoder position before jog");
    double peak_current = std::abs(motor.get_motor_current());
    double peak_speed = 0.0;
    std::cout << std::setprecision(9)
              << "Encoder before: " << initial_angle << " rad\n"
              << "Jogging ID " << id << " at -0.5 motor rad/s for " << duration
              << " ms (expected opening direction; verify mechanically); no zero-point changes\n"
              << std::flush;
    motor.enable();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    check_state(motor, true);
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(duration);
    motor.motor_spd_cmd(-0.5f);
    while (std::chrono::steady_clock::now() < end) {
      check_state(motor, true);
      const double speed = motor.get_motor_spd();
      if (!std::isfinite(speed)) throw std::runtime_error("invalid motor speed feedback");
      peak_speed = std::max(peak_speed, std::abs(speed));
      peak_current = std::max(peak_current, static_cast<double>(std::abs(motor.get_motor_current())));
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    motor.motor_spd_cmd(0.0f);
    motor.disable();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    check_state(motor, false);
    const double final_angle = motor.get_motor_pos();
    if (!std::isfinite(final_angle)) throw std::runtime_error("invalid encoder position after jog");
    std::cout << "Encoder after: " << final_angle << " rad; delta: "
              << final_angle - initial_angle << " rad\n"
              << "Sampled peak speed: " << peak_speed << " rad/s; peak current: "
              << peak_current << " A\n";
    if (!session.stop()) return 1;
    std::cout << "Jog complete; stop and disable commands sent\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Opening jog failed: " << error.what() << '\n';
    return 1;
  }
}

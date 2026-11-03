#pragma once
#include "messages.hpp"
#include <memory>
#include <string>

namespace gripper_demo {
struct HardwareState {
    double position_m, motor_torque_percent, force_n;
    std::uint32_t error_code;
    bool enabled;
    double motor_position_rad{0};
    double motor_current_a{0};
};
// All methods called by one serial owner. No ROS or terminal dependencies.
class Hardware {
public:
    virtual ~Hardware() = default;
    virtual void init() = 0;
    virtual bool read(HardwareState& state) = 0;
    virtual bool zero() = 0;
    virtual void enable_position() = 0;
    virtual void disable() = 0;
    virtual void clear() = 0;
    virtual void move(double position_m, float speed_mps) = 0;
    virtual void hold_position(double motor_position_rad, float speed_mps) = 0;
    virtual double max_position_m() const { return 0.098; }
    virtual void shutdown() noexcept = 0;
};
std::unique_ptr<Hardware> make_hardware(const std::string& port, int id, int baud, const std::string& calibration_path = "");

class Controller {
public:
    explicit Controller(std::unique_ptr<Hardware> hardware, double current_threshold_a = 0.5);
    ~Controller();
    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;
    // Caller explicitly declares that the existing device zero is still valid.
    void init(bool use_existing_zero = false);
    Result command(const Command& command);
    Feedback sample();
private:
    void fail_safe() noexcept;
    std::unique_ptr<Hardware> hardware_;
    bool homed_{false};
    bool target_valid_{false};
    double target_m_{0.0};
    float target_speed_mps_{0.01f};
    double current_threshold_a_{0.5};
    bool current_limit_reached_{false};
};
} // namespace gripper_demo

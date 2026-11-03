#pragma once
#include <cstdint>
#include <string>

namespace gripper_demo {
enum class Action : std::uint8_t { Move = 0, Zero = 1, Enable = 2, Disable = 3, Clear = 4 };
struct Command {
    std::uint64_t command_id{0};
    Action action{Action::Move};
    double position_m{0.0};
    float speed_mps{0.01f};
};
struct Feedback {
    std::uint64_t stamp_ns{0};  // Unix time; ROS adapter can replace with node clock.
    std::uint64_t command_id{0}; // Last accepted command, not proof of completion.
    double position_m{0.0};
    double motor_position_rad{0.0}; // Raw SDK angle for calibration checks.
    double motor_current_a{0.0}; // Driver current feedback in amperes.
    double motor_torque_percent{0.0}; // 100 means 100% rated torque/current, not Nm.
    double force_n{0.0}; // Model estimate, not a force sensor reading.
    std::uint32_t error_code{0}; // Driver bitmask, meaningful when valid=true.
    bool valid{false}; // All measured fields invalid when false.
    bool enabled{false};
    bool homed{false}; // Established here or explicitly confirmed valid by caller.
    bool at_target{false};
    double current_threshold_a{0.5};
    bool current_limit_reached{false}; // Latched until new MOVE or disable/zero/fault.
    bool holding_position{false};
    bool target_active{false};
    double target_position_m{0}; // Effective target, captured position after threshold trip.
};
struct Result { bool accepted; std::string detail; };
} // namespace gripper_demo

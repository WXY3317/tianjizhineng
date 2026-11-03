#pragma once

#include <chrono>
#include <fstream>
#include <string>

class GripperController;

/** @brief 面向客户示例和调试的 CSV 日志工具。 */
class GripperCsvLogger {
   public:
    explicit GripperCsvLogger(GripperController &gripper);

    void start_pos_log();
    void start_spd_log();
    void start_torque_log();

    void log_pos_state(double target_m, double command_deg);
    void log_spd_state(float command_mps);
    void log_torque_state(float command_force_n);

    void stop_log();

   private:
    void open_log(const std::string &prefix, const std::string &header);

    GripperController &gripper_;
    std::ofstream log_file_;
    std::chrono::steady_clock::time_point start_time_;
};

namespace gripper_tools {

int run_velocity_test(GripperController &gripper, float speed_mps,
                      int duration_ms = 2000, int sample_ms = 10,
                      GripperCsvLogger *logger = nullptr);

int run_torque_test(GripperController &gripper, float force_n,
                    int duration_ms = 2000, int sample_ms = 50,
                    GripperCsvLogger *logger = nullptr);

}  // namespace gripper_tools

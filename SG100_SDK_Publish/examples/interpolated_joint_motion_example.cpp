/**
 * @file interpolated_joint_motion_example.cpp
 * @brief 交互式单关节定时插值运动示例
 *
 * 输入格式: <joint_id> <target_deg> <duration_s>
 * 例如:    4 35.0 1.250
 * 输入 q/quit/exit 退出。每段完成后自动提示下一段。
 */

#include "sg100_device_manager.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#ifdef _WIN32
#include <windows.h>
#endif

namespace {

constexpr std::chrono::milliseconds kInterpolationPeriod(10); // 与 SDK 100 Hz 调度一致
constexpr auto kStatusWait = std::chrono::milliseconds(1500);
constexpr float kPositionKp = 0.5f;
constexpr float kPositionKi = 0.0f;
constexpr float kPositionKd = 0.05f;
constexpr float kOutputLimit = 1.0f;

bool FileExists(const std::string& path)
{
    std::ifstream file(path);
    return file.good();
}

std::string FindSerialConfigPath()
{
    const char* candidates[] = {
        "serial_config.json", "../serial_config.json", "../examples/serial_config.json",
        "../../serial_config.json", "../../examples/serial_config.json"
    };
    for (const char* candidate : candidates) {
        if (FileExists(candidate)) return candidate;
    }
    return {};
}

bool AddDevice(SG100DeviceManager& manager)
{
    const std::string config_path = FindSerialConfigPath();
    if (!config_path.empty() && manager.AddDevice("HAND", config_path, 1)) return true;

    const auto ports = SG100DeviceManager::ListSerialPorts();
    if (ports.empty()) {
        std::cerr << "[错误] 未找到可用串口。\n";
        return false;
    }

    std::cout << "可用串口:\n";
    for (std::size_t i = 0; i < ports.size(); ++i) {
        std::cout << "  [" << i << "] " << ports[i].port << " - " << ports[i].description << '\n';
    }
    std::cout << "选择串口编号: " << std::flush;
    std::string line;
    if (!std::getline(std::cin, line)) return false;
    std::istringstream input(line);
    std::size_t index = 0;
    char extra = 0;
    if (!(input >> index) || (input >> extra) || index >= ports.size()) {
        std::cerr << "[错误] 串口编号无效。\n";
        return false;
    }
    return manager.AddDeviceByParams("HAND", ports[index].port, 1000000);
}

bool WaitForStatus(SG100Device* hand)
{
    const uint32_t initial_count = hand->GetStatusRecvCount();
    const auto deadline = std::chrono::steady_clock::now() + kStatusWait;
    while (std::chrono::steady_clock::now() < deadline) {
        if (hand->GetStatusRecvCount() != initial_count) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

// 五次 smoothstep: 起点和终点的速度、加速度均为零，减少位置阶跃冲击。
double SmoothStep5(double t)
{
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
}

bool RunInterpolation(SG100Device* hand, Hand_Control_t& control, uint8_t joint_id,
                      double start_deg, double target_deg, double duration_s)
{
    using Clock = std::chrono::steady_clock;
    const auto start_time = Clock::now();
    const auto duration = std::chrono::duration<double>(duration_s);
    auto next_tick = start_time;

    while (true) {
        const auto now = Clock::now();
        const double elapsed = std::chrono::duration<double>(now - start_time).count();
        if (elapsed >= duration_s) break;

        const double raw_phase = elapsed / duration_s;
        const double phase = raw_phase < 0.0 ? 0.0 : (raw_phase > 1.0 ? 1.0 : raw_phase);
        const double command_deg = start_deg + (target_deg - start_deg) * SmoothStep5(phase);
        Joint_Control_t& joint = control.joints[joint_id - 1];
        joint.id = joint_id;
        joint.mode = MODE_JOINT_POSITION;
        joint.position = static_cast<float>(command_deg); // SDK API 缓存单位为 degree
        if (!hand->SetHandControl(control)) return false;

        next_tick += kInterpolationPeriod;
        const auto end_time = start_time + std::chrono::duration_cast<Clock::duration>(duration);
        if (next_tick > end_time) next_tick = end_time;
        std::this_thread::sleep_until(next_tick);
    }

    // 避免周期量化导致最后一个采样点未恰好等于目标值。
    Joint_Control_t& joint = control.joints[joint_id - 1];
    joint.id = joint_id;
    joint.mode = MODE_JOINT_POSITION;
    joint.position = static_cast<float>(target_deg);
    return hand->SetHandControl(control);
}

void PrintJointFeedback(SG100Device* hand, uint8_t joint_id)
{
    const Hand_Status_t status = hand->GetCachedHandStatus();
    const Joint_Status_t& joint = status.joints[joint_id - 1];
    std::cout << "[反馈] J" << static_cast<int>(joint_id)
              << " mode=" << static_cast<int>(joint.mode)
              << ", position=" << joint.position << " deg"
              << ", velocity=" << SG100_Quant::decodeVel(joint.velocity) << " deg/s"
              << ", cur_q=" << SG100_Quant::decodeCurQ(joint.cur_q) << " A"
              << ", cur_d=" << SG100_Quant::decodeCurD(joint.cur_d) << " A\n";
}

} // namespace

int main()
{
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif

    std::cout << "SG100 交互式关节插值运动示例\n"
              << "输入: <关节ID 1~" << JOINT_COUNT << "> <目标角度deg> <时间s>\n"
              << "示例: 4 35.0 1.250    退出: q\n\n";

    auto& manager = SG100DeviceManager::Instance();
    if (!AddDevice(manager)) return 1;
    SG100Device* hand = manager.GetDevice("HAND");
    if (!hand || !manager.StartAll()) {
        std::cerr << "[错误] 设备启动失败。\n";
        manager.Clear();
        return 1;
    }

    if (!hand->SwitchToRealtimeMode()) {
        std::cerr << "[错误] 无法进入掌板实时控制模式。\n";
        manager.StopAll();
        manager.Clear();
        return 1;
    }

    if (!WaitForStatus(hand)) {
        std::cerr << "[错误] 1.5 s 内没有收到关节状态，拒绝以未知起点运动。\n";
        manager.StopAll();
        manager.Clear();
        return 1;
    }

    // 用真实反馈和完整的位置环参数初始化全部关节：
    // 1. 未选择的关节保持当前位置，避免被默认 0 度目标拉动；
    // 2. 后续 SetJointPosition() 从 SDK 缓存复制控制帧时会保留 PID 和输出限制。
    float targets[JOINT_COUNT] = {};
    hand->GetCachedPositions(targets);
    Hand_Control_t initial_control{};
    for (int i = 0; i < JOINT_COUNT; ++i) {
        initial_control.joints[i].id = static_cast<uint8_t>(i + 1);
        initial_control.joints[i].mode = MODE_JOINT_POSITION;
        initial_control.joints[i].position = targets[i];
        initial_control.joints[i].p = SG100_Quant::encodePid(kPositionKp);
        initial_control.joints[i].i = SG100_Quant::encodePid(kPositionKi);
        initial_control.joints[i].d = SG100_Quant::encodePid(kPositionKd);
        initial_control.joints[i].output_limit =
            SG100_Quant::encodeOutLim(kOutputLimit);
    }
    if (!hand->SetHandControl(initial_control)) {
        std::cerr << "[错误] 初始化当前位置目标失败。\n";
        manager.StopAll();
        manager.Clear();
        return 1;
    }
    std::cout << "[控制参数] 关节位置模式(mode=" << static_cast<int>(MODE_JOINT_POSITION)
              << "), Kp=" << kPositionKp
              << ", Ki=" << kPositionKi << ", Kd=" << kPositionKd
              << ", output_limit=" << kOutputLimit << "\n";

    std::cout << std::fixed << std::setprecision(3);
    std::string line;
    while (true) {
        std::cout << "\n请输入 id 目标角度(deg) 插值时间(s): " << std::flush;
        if (!std::getline(std::cin, line)) break;
        if (line == "q" || line == "quit" || line == "exit") break;

        int id = 0;
        double target_deg = 0.0;
        double duration_s = 0.0;
        char extra = 0;
        std::istringstream input(line);
        if (!(input >> id >> target_deg >> duration_s) || (input >> extra)) {
            std::cout << "[提示] 格式错误，例如: 4 35.0 1.250\n";
            continue;
        }
        if (id < 1 || id > JOINT_COUNT || !std::isfinite(target_deg) ||
            !std::isfinite(duration_s) || duration_s < 0.001) {
            std::cout << "[提示] ID 范围为 1~" << JOINT_COUNT
                      << "，角度须为有限数，时间须 >= 0.001 s。\n";
            continue;
        }

        const double start_deg = hand->GetCachedPosition(static_cast<uint8_t>(id));
        std::cout << "[运行] J" << id << ": " << start_deg << " -> " << target_deg
                  << " deg, " << duration_s << " s\n";
        const auto wall_start = std::chrono::steady_clock::now();
        if (!RunInterpolation(hand, initial_control, static_cast<uint8_t>(id),
                              start_deg, target_deg, duration_s)) {
            std::cerr << "[错误] 插值过程中下发目标失败。\n";
            break;
        }
        const double actual_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - wall_start).count();
        std::cout << "[完成] J" << id << " 插值目标下发完成，实际调度用时 "
                  << actual_s << " s。\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PrintJointFeedback(hand, static_cast<uint8_t>(id));
    }

    manager.StopAll();
    manager.Clear();
    return 0;
}

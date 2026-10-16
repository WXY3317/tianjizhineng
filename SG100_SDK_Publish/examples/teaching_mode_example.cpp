/**
 * @file teaching_mode_example.cpp
 * @brief SG100 灵巧手示教/动作帧回放示例
 *
 * 示教模式不发送位置、速度或力矩目标，仅发送 MODE_NULL，确保不会沿用
 * 上一次的驱动输出；状态回调持续采集当前关节角度。
 *
 * 控制台命令（可作为“按钮”触发）：
 *   record                 记录当前 11 个关节角度为一帧
 *   list                   列出已记录动作帧
 *   play <index>           回放指定动作帧
 *   playall                按顺序回放全部动作帧
 *   stop                   立即停止回放并回到示教模式
 *   status                 显示当前关节角度
 *   help                   显示帮助
 *   quit                   退出
 *
 * 注意：本示例使用 SDK 已解码的用户角度（degree），角度映射由 SDK 处理。
 */

#include "sg100_device_manager.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

namespace {

constexpr float kPositionKp = 0.5f;
constexpr float kPositionKi = 0.0f;
constexpr float kPositionKd = 0.05f;
constexpr float kOutputLimit = 1.0f;
constexpr float kPositionToleranceDeg = 2.0f;
constexpr std::chrono::milliseconds kPlaybackTimeout(8000);

struct TeachingFrame {
    std::vector<float> positions_deg;
};

std::mutex g_status_mutex;
std::condition_variable g_status_cv;
std::array<float, JOINT_COUNT> g_positions{};
bool g_have_status = false;
std::atomic<bool> g_stop_playback{false};

bool FileExists(const std::string& path)
{
    std::ifstream file(path);
    return file.good();
}

std::string FindSerialConfigPath()
{
    const char* candidates[] = {
        "serial_config.json", "../serial_config.json",
        "../examples/serial_config.json", "../../serial_config.json",
        "../../examples/serial_config.json"
    };
    for (const char* candidate : candidates) {
        if (FileExists(candidate)) return candidate;
    }
    return {};
}

bool AddDevice(SG100DeviceManager& manager)
{
    const std::string config_path = FindSerialConfigPath();
    if (!config_path.empty() && manager.AddDevice("HAND", config_path, 1)) {
        return true;
    }

    const auto ports = SG100DeviceManager::ListSerialPorts();
    if (ports.empty()) {
        std::cerr << "[错误] 未找到可用串口\n";
        return false;
    }

    std::cout << "可用串口:\n";
    for (std::size_t i = 0; i < ports.size(); ++i) {
        std::cout << "  [" << i << "] " << ports[i].port
                  << " - " << ports[i].description << '\n';
    }
    std::cout << "选择串口编号: " << std::flush;
    std::string line;
    if (!std::getline(std::cin, line)) return false;
    std::istringstream input(line);
    std::size_t index = 0;
    char extra = 0;
    if (!(input >> index) || (input >> extra) || index >= ports.size()) {
        std::cerr << "[错误] 串口编号无效\n";
        return false;
    }
    return manager.AddDeviceByParams("HAND", ports[index].port, 1000000);
}

void OnStatus(const Hand_Status_t& status)
{
    std::lock_guard<std::mutex> lock(g_status_mutex);
    for (int i = 0; i < JOINT_COUNT; ++i) {
        g_positions[static_cast<std::size_t>(i)] = status.joints[i].position;
    }
    g_have_status = true;
    g_status_cv.notify_all();
}

std::vector<float> CurrentPositions()
{
    std::lock_guard<std::mutex> lock(g_status_mutex);
    return std::vector<float>(g_positions.begin(), g_positions.end());
}

void PrintPositions(const std::vector<float>& positions)
{
    std::cout << std::fixed << std::setprecision(2);
    for (std::size_t i = 0; i < positions.size(); ++i) {
        std::cout << "J" << (i + 1) << "=" << positions[i] << " deg";
        if (i + 1 != positions.size()) std::cout << "  ";
    }
    std::cout << '\n';
}

Hand_Control_t MakeNullControl()
{
    Hand_Control_t control{};
    for (int i = 0; i < JOINT_COUNT; ++i) {
        control.joints[i].id = static_cast<uint8_t>(i + 1);
        control.joints[i].mode = MODE_NULL;
    }
    return control;
}

Hand_Control_t MakePositionControl(const std::vector<float>& positions)
{
    Hand_Control_t control{};
    for (int i = 0; i < JOINT_COUNT; ++i) {
        control.joints[i].id = static_cast<uint8_t>(i + 1);
        control.joints[i].mode = MODE_JOINT_POSITION;
        control.joints[i].position = positions[static_cast<std::size_t>(i)];
        control.joints[i].p = SG100_Quant::encodePid(kPositionKp);
        control.joints[i].i = SG100_Quant::encodePid(kPositionKi);
        control.joints[i].d = SG100_Quant::encodePid(kPositionKd);
        control.joints[i].output_limit = SG100_Quant::encodeOutLim(kOutputLimit);
    }
    return control;
}

void PrintHelp()
{
    std::cout << "\n命令:\n"
              << "  record       记录当前关节角度为动作帧\n"
              << "  list         列出动作帧\n"
              << "  play <n>     回放第 n 帧（从 0 开始）\n"
              << "  playall      顺序回放全部动作帧\n"
              << "  stop         停止回放，回到示教模式\n"
              << "  status       显示当前角度\n"
              << "  help         显示帮助\n"
              << "  quit         退出\n\n";
}

void EnterTeachingMode(SG100Device* hand)
{
    // SDK 会缓存并周期性发送控制帧；显式发送 MODE_NULL，避免保留旧目标。
    const Hand_Control_t null_control = MakeNullControl();
    if (!hand->SetHandControl(null_control)) {
        std::cerr << "[警告] MODE_NULL 未能入队\n";
    }
}

bool PlayFrame(SG100Device* hand, const TeachingFrame& frame)
{
    if (frame.positions_deg.size() != JOINT_COUNT) return false;

    g_stop_playback.store(false);
    const Hand_Control_t control = MakePositionControl(frame.positions_deg);
    const auto deadline = std::chrono::steady_clock::now() + kPlaybackTimeout;

    while (!g_stop_playback.load() && std::chrono::steady_clock::now() < deadline) {
        if (!hand->SetHandControl(control)) return false;

        const std::vector<float> actual = CurrentPositions();
        bool reached = true;
        for (int i = 0; i < JOINT_COUNT; ++i) {
            if (std::fabs(actual[static_cast<std::size_t>(i)] - frame.positions_deg[static_cast<std::size_t>(i)]) >
                kPositionToleranceDeg) {
                reached = false;
                break;
            }
        }
        if (reached) {
            EnterTeachingMode(hand);
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    EnterTeachingMode(hand);
    return !g_stop_playback.load();
}

} // namespace

int main()
{
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif

    std::cout << "============================================\n"
              << "  SG100 灵巧手示教模式示例\n"
              << "============================================\n"
              << "启动后只监控角度，不主动控制关节输出。\n";

    auto& manager = SG100DeviceManager::Instance();
    if (!AddDevice(manager)) return 1;

    SG100Device* hand = manager.GetDevice("HAND");
    if (!hand) return 1;
    hand->RegisterHandStatusCallback(OnStatus);
    hand->RegisterErrorCallback([](uint8_t joint_id, uint8_t error_code) {
        std::cerr << "[关节错误] J" << static_cast<int>(joint_id)
                  << " code=" << static_cast<int>(error_code) << '\n';
    });

    if (!manager.StartAll() || !hand->SwitchToRealtimeMode()) {
        std::cerr << "[错误] 无法启动或进入实时模式\n";
        manager.StopAll();
        manager.Clear();
        return 1;
    }

    EnterTeachingMode(hand);
    {
        std::unique_lock<std::mutex> lock(g_status_mutex);
        if (!g_status_cv.wait_for(lock, std::chrono::seconds(2), [] { return g_have_status; })) {
            std::cerr << "[警告] 尚未收到完整状态，仍可继续等待\n";
        }
    }

    std::vector<TeachingFrame> frames;
    PrintHelp();
    std::string line;
    while (std::cout << "teach> " && std::getline(std::cin, line)) {
        std::istringstream input(line);
        std::string command;
        input >> command;
        if (command == "record" || command == "r") {
            const std::vector<float> positions = CurrentPositions();
            if (positions.size() == JOINT_COUNT) {
                frames.push_back({positions});
                std::cout << "已记录动作帧 #" << (frames.size() - 1) << ": ";
                PrintPositions(positions);
            }
        } else if (command == "list" || command == "l") {
            for (std::size_t i = 0; i < frames.size(); ++i) {
                std::cout << "#" << i << ": ";
                PrintPositions(frames[i].positions_deg);
            }
            if (frames.empty()) std::cout << "（暂无动作帧）\n";
        } else if (command == "status" || command == "s") {
            PrintPositions(CurrentPositions());
        } else if (command == "stop") {
            g_stop_playback.store(true);
            EnterTeachingMode(hand);
            std::cout << "已停止回放，回到示教模式。\n";
        } else if (command == "playall") {
            for (const TeachingFrame& frame : frames) {
                if (!PlayFrame(hand, frame)) break;
            }
            std::cout << "回放结束，已回到示教模式。\n";
        } else if (command == "play") {
            std::size_t index = 0;
            if (!(input >> index) || index >= frames.size()) {
                std::cout << "用法：play <帧编号>\n";
                continue;
            }
            std::cout << "正在回放动作帧 #" << index << "\n";
            std::cout << (PlayFrame(hand, frames[index]) ? "回放完成\n" : "回放失败或超时\n");
        } else if (command == "help" || command == "h") {
            PrintHelp();
        } else if (command == "quit" || command == "q" || command == "exit") {
            break;
        } else if (!command.empty()) {
            std::cout << "未知命令，输入 help 查看帮助。\n";
        }
    }

    EnterTeachingMode(hand);
    manager.StopAll();
    manager.Clear();
    return 0;
}

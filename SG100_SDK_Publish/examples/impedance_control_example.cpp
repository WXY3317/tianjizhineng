/**
 * @file impedance_control_example.cpp
 * @brief SG100 单关节阻抗控制命令行示例
 *
 * 命令:
 *   set <id> <position_deg> [p] [d] [feedforward] [limit]
 *   status <id>
 *   help
 *   quit
 *
 * 可选参数默认值: p=0.001, d=0.001, feedforward=0, limit=1。
 * position 使用用户角度 degree；SDK 发帧时自动执行角度映射并转换为 radian。
 */

#include "sg100_device_manager.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#ifdef _WIN32
#include <windows.h>
#endif

namespace {

constexpr float kDefaultP = 0.001f;
constexpr float kDefaultD = 0.001f;
constexpr float kDefaultFeedforward = 0.0f;
constexpr float kDefaultLimit = 1.0f;

std::mutex g_status_mutex;
Hand_Status_t g_hand_status{};
std::atomic<uint32_t> g_status_count{0};

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
        std::cerr << "[错误] 串口编号无效。\n";
        return false;
    }
    return manager.AddDeviceByParams("HAND", ports[index].port, 1000000);
}

void OnHandStatus(const Hand_Status_t& status)
{
    {
        std::lock_guard<std::mutex> lock(g_status_mutex);
        g_hand_status = status;
    }
    g_status_count.fetch_add(1);
}

bool WaitForStatus(uint32_t timeout_ms)
{
    const uint32_t initial = g_status_count.load();
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (g_status_count.load() != initial) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

void PrintJointStatus(uint8_t joint_id)
{
    Hand_Status_t status{};
    {
        std::lock_guard<std::mutex> lock(g_status_mutex);
        status = g_hand_status;
    }
    const Joint_Status_t& joint = status.joints[joint_id - 1];
    std::cout << std::fixed << std::setprecision(4)
              << "[反馈] J" << static_cast<int>(joint_id)
              << " mode=" << static_cast<int>(joint.mode)
              << ", position=" << joint.position << " deg"
              << ", velocity=" << SG100_Quant::decodeVel(joint.velocity) << " deg/s"
              << ", cur_q=" << SG100_Quant::decodeCurQ(joint.cur_q) << " A"
              << ", cur_d=" << SG100_Quant::decodeCurD(joint.cur_d) << " A\n";
}

void PrintHelp()
{
    std::cout
        << "\n命令:\n"
        << "  set <id> <position_deg> [p] [d] [feedforward] [limit]\n"
        << "      示例: set 8 30\n"
        << "      示例: set 8 30 0.002 0.001 0 1\n"
        << "      默认: p=" << kDefaultP << ", d=" << kDefaultD
        << ", feedforward=" << kDefaultFeedforward
        << ", limit=" << kDefaultLimit << "\n"
        << "  status <id>   打印指定关节反馈\n"
        << "  help          显示帮助\n"
        << "  quit          退出并释放全部关节\n";
}

bool ParseOptionalFloat(std::istringstream& input, float& value)
{
    input >> std::ws;
    if (input.eof()) return true;
    return static_cast<bool>(input >> value);
}

bool SendSingleJointCommand(SG100Device* hand, uint8_t joint_id,
                            float position_deg, float p, float d,
                            float feedforward, float limit)
{
    Hand_Control_t control{};
    for (int i = 0; i < JOINT_COUNT; ++i) {
        control.joints[i].id = static_cast<uint8_t>(i + 1);
        control.joints[i].mode = MODE_NULL;
    }

    Joint_Control_t& joint = control.joints[joint_id - 1];
    joint.mode = MODE_JOINT_IMPEDANCE;
    joint.imp_position = position_deg; // SDK 缓存层单位为 degree
    joint.imp_velocity = SG100_Quant::encodeVel(0.0f);
    joint.imp_kp = SG100_Quant::encodePid(p);
    joint.imp_kd = SG100_Quant::encodePid(d);
    joint.imp_torque_ff = SG100_Quant::encodeOutLim(feedforward);
    joint.imp_output_limit = SG100_Quant::encodeOutLim(limit);

    return hand->SetHandControl(control);
}

void ReleaseAllJoints(SG100Device* hand)
{
    Hand_Control_t control{};
    for (int i = 0; i < JOINT_COUNT; ++i) {
        control.joints[i].id = static_cast<uint8_t>(i + 1);
        control.joints[i].mode = MODE_NULL;
    }
    hand->SetHandControl(control);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

} // namespace

int main()
{
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif

    std::cout << "SG100 单关节阻抗控制示例\n";
    auto& manager = SG100DeviceManager::Instance();
    if (!AddDevice(manager)) return 1;

    SG100Device* hand = manager.GetDevice("HAND");
    if (!hand) return 1;

    hand->RegisterHandStatusCallback(OnHandStatus);
    hand->RegisterErrorCallback([](uint8_t source, uint8_t error) {
        std::cerr << "[设备错误] source=" << static_cast<int>(source)
                  << ", code=0x" << std::hex << static_cast<int>(error)
                  << std::dec << '\n';
    });

    if (!manager.StartAll()) {
        manager.Clear();
        return 1;
    }
    if (!hand->SwitchToRealtimeMode()) {
        std::cerr << "[错误] 无法进入实时控制模式。\n";
        manager.StopAll();
        manager.Clear();
        return 1;
    }
    if (!WaitForStatus(1500)) {
        std::cerr << "[错误] 1.5 s 内没有收到状态反馈。\n";
        manager.StopAll();
        manager.Clear();
        return 1;
    }

    PrintHelp();
    std::string line;
    while (true) {
        std::cout << "\nimpedance> " << std::flush;
        if (!std::getline(std::cin, line)) break;

        std::istringstream input(line);
        std::string command;
        input >> command;
        if (command.empty()) continue;
        if (command == "quit" || command == "q" || command == "exit") break;
        if (command == "help" || command == "h") {
            PrintHelp();
            continue;
        }
        if (command == "status") {
            int id = 0;
            char extra = 0;
            if (!(input >> id) || (input >> extra) || id < 1 || id > JOINT_COUNT) {
                std::cout << "用法: status <id 1~" << JOINT_COUNT << ">\n";
                continue;
            }
            PrintJointStatus(static_cast<uint8_t>(id));
            continue;
        }
        if (command != "set") {
            std::cout << "未知命令，输入 help 查看帮助。\n";
            continue;
        }

        int id = 0;
        float position = 0.0f;
        float p = kDefaultP;
        float d = kDefaultD;
        float feedforward = kDefaultFeedforward;
        float limit = kDefaultLimit;
        if (!(input >> id >> position) ||
            !ParseOptionalFloat(input, p) ||
            !ParseOptionalFloat(input, d) ||
            !ParseOptionalFloat(input, feedforward) ||
            !ParseOptionalFloat(input, limit)) {
            std::cout << "用法: set <id> <position_deg> [p] [d] [feedforward] [limit]\n";
            continue;
        }
        input >> std::ws;
        if (!input.eof() || id < 1 || id > JOINT_COUNT ||
            !std::isfinite(position) || !std::isfinite(p) || !std::isfinite(d) ||
            !std::isfinite(feedforward) || !std::isfinite(limit) ||
            p < 0.0f || p > 10.0f || d < 0.0f || d > 10.0f ||
            feedforward < -10.0f || feedforward > 10.0f ||
            limit < 0.0f || limit > 10.0f) {
            std::cout << "参数错误：id=1~" << JOINT_COUNT
                      << "，p/d/limit=0~10，feedforward=-10~10。\n";
            continue;
        }

        if (!SendSingleJointCommand(hand, static_cast<uint8_t>(id), position,
                                    p, d, feedforward, limit)) {
            std::cerr << "[错误] 控制命令写入失败。\n";
            continue;
        }

        std::cout << std::fixed << std::setprecision(4)
                  << "[下发] J" << id << " position=" << position << " deg"
                  << ", p=" << p << ", d=" << d
                  << ", feedforward=" << feedforward
                  << ", limit=" << limit << '\n';
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PrintJointStatus(static_cast<uint8_t>(id));
    }

    ReleaseAllJoints(hand);
    manager.StopAll();
    manager.Clear();
    return 0;
}

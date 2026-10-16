/**
 * @file realtime_joint_control_example.cpp
 * @brief SG100 SDK 实时关节控制示例
 *
 * 流程:
 *   1. 连接设备
 *   2. 启用位置字段内部变换, 直接按用户角度(度)发送/显示 position
 *   3. 显式切换到实时模式
 *   4. 初始化所有关节为位置模式, 目标位置=0 deg
 *   4. 命令行交互:
 *        set <id> <pos_deg>               — 设置指定关节目标位置 (deg)
 *        pid <id> <kp> <ki> <kd>          — 修改关节 PID 参数
 *        limit <id> <val>                  — 修改关节输出限制 (0~1)
 *        mode <id> <mode>                  — 修改关节控制模式 (1=位置 2=速度 3=力矩 4=开环)
 *        status                            — 打印所有关节当前状态
 *        help                              — 显示帮助
 *        quit                              — 退出
 */

#include "sg100_device_manager.h"

#include <atomic>
#include <chrono>
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

// 角度映射配置目录（留空表示使用默认搜索路径）
const std::string JOINT_MAP_CONFIG_DIR = "";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// 全局控制状态
// ============================================================================
static std::mutex        g_ctrl_mutex;
static Hand_Control_t    g_hand_ctrl;
static Hand_Status_t     g_hand_status;
static std::mutex        g_status_mutex;
static std::atomic<bool> g_status_updated{false};
static std::atomic<bool> g_running{true};
static std::atomic<bool> g_control_paused{false};
static std::atomic<bool> g_position_transform_enabled{false};

static const char* PositionValueLabel()
{
    return g_position_transform_enabled.load() ? "deg" : "raw";
}

static bool FileExists(const std::string& path)
{
    std::ifstream file(path);
    return file.good();
}

static std::string FindSerialConfigPath()
{
    const char* candidates[] = {
        "serial_config.json",
        "../serial_config.json",
        "../examples/serial_config.json",
        "../../serial_config.json",
        "../../examples/serial_config.json"
    };

    for (const char* candidate : candidates) {
        if (FileExists(candidate)) {
            return candidate;
        }
    }

    return "";
}

static const char* RuntimeModeName(uint8_t mode)
{
    switch (mode) {
        case PALM_RUNTIME_MODE_REALTIME:
            return "realtime";
        case PALM_RUNTIME_MODE_NONRT_PAUSED:
            return "nonrt-paused";
        case PALM_RUNTIME_MODE_UART_LINK_PC:
            return "uart-link-pc";
        case PALM_RUNTIME_MODE_UART_LINK_ESP32:
            return "uart-link-esp32";
        case PALM_RUNTIME_MODE_FAKE_PC:
            return "fake-pc";
        default:
            return "unknown";
    }
}

static bool QueryAndReadRuntimeMode(SG100Device* hand,
                                    Palm_Runtime_Mode_Status_t& status,
                                    uint32_t wait_ms = 250,
                                    uint32_t retries = 3)
{
    for (uint32_t attempt = 0; attempt < retries; ++attempt) {
        if (!hand->PalmRuntimeModeGetStatus()) {
            continue;
        }

        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms);
        while (std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            if (hand->GetPalmRuntimeModeStatus(status)) {
                return true;
            }
        }

        if (hand->GetPalmRuntimeModeStatus(status)) {
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return false;
}

static bool WaitForRealtimeTraffic(SG100Device* hand, uint32_t timeout_ms = 1000)
{
    const uint32_t baseline = hand->GetStatusRecvCount();
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (hand->GetStatusRecvCount() != baseline) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return hand->GetStatusRecvCount() != baseline;
}

static void PrintRuntimeMode(const char* stage, const Palm_Runtime_Mode_Status_t& status)
{
    std::cout << "[模式查询] " << stage
              << " -> mode=" << RuntimeModeName(status.mode)
              << " (" << static_cast<int>(status.mode) << ")"
              << ", realtime_enabled=" << static_cast<int>(status.realtime_enabled)
              << ", uart_link_enabled=" << static_cast<int>(status.uart_link_enabled)
              << std::endl;
}

// ============================================================================
// 初始化整手控制帧 (位置模式, 所有关节目标=0)
// ============================================================================
static void InitHandCtrl(float kp = 0.1f, float ki = 0.0f, float kd = 0.0f,
                         float output_limit = 0.5f)
{
    std::lock_guard<std::mutex> lock(g_ctrl_mutex);
    std::memset(&g_hand_ctrl, 0, sizeof(g_hand_ctrl));
    for (int i = 0; i < JOINT_COUNT; i++) {
        g_hand_ctrl.joints[i].id           = static_cast<uint8_t>(i + 1);
        g_hand_ctrl.joints[i].mode         = MODE_POSITION;
        g_hand_ctrl.joints[i].position     = 0.0f;
        g_hand_ctrl.joints[i].p            = SG100_Quant::encodePid(kp);
        g_hand_ctrl.joints[i].i            = SG100_Quant::encodePid(ki);
        g_hand_ctrl.joints[i].d            = SG100_Quant::encodePid(kd);
        g_hand_ctrl.joints[i].output_limit = SG100_Quant::encodeOutLim(output_limit);
    }
}

// ============================================================================
// 实时控制线程: 20 Hz 周期下发整手控制帧
// ============================================================================
static void RealtimeControlThread(SG100Device* hand)
{
    while (g_running.load()) {
        if (!g_control_paused.load()) {
            std::lock_guard<std::mutex> lock(g_ctrl_mutex);
            hand->SetHandControl(g_hand_ctrl);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

// ============================================================================
// 打印整手状态
// ============================================================================
static void PrintStatus()
{
    Hand_Status_t snap;
    {
        std::lock_guard<std::mutex> lock(g_status_mutex);
        snap = g_hand_status;
    }
    std::cout << "\n--- 整手状态 ---\n";
    std::cout << std::fixed << std::setprecision(4);
    for (int i = 0; i < JOINT_COUNT; i++) {
          const auto& j = snap.joints[i];
          float pos_value = j.position;
        float vel     = SG100_Quant::decodeVel(j.velocity);
        float cur_q   = SG100_Quant::decodeCurQ(j.cur_q);
        std::cout << "  J" << std::setw(2) << (i + 1)
              << "  pos=" << std::setw(12) << pos_value << " " << PositionValueLabel()
                  << "  vel=" << std::setw(8) << vel    << " dps"
                  << "  cur_q=" << std::setw(6) << cur_q << " A"
                  << "  mode=" << (int)j.mode
                  << "\n";
    }
    std::cout << "----------------\n";
}

// ============================================================================
// 打印帮助
// ============================================================================
static void PrintHelp()
{
    std::cout <<
        "\n命令:\n"
        "  set <id> <pos>             — 设置关节目标位置 (当前单位由 transform 决定, 1~" << JOINT_COUNT << ")\n"
        "  cali <id>                  — 对指定关节执行零点校准并自动恢复实时模式\n"
        "  transform <on|off>         — on=启用内部角度变换(deg), off=关闭并直通 raw\n"
        "  pid <id> <kp> <ki> <kd>   — 修改关节 PID 参数\n"
        "  limit <id> <val>           — 修改关节输出限制 (0~1)\n"
        "  mode <id> <m>              — 设置控制模式 (1=位置 2=速度 3=力矩 4=开环)\n"
        "  status                     — 显示所有关节当前状态\n"
        "  help                       — 显示本帮助\n"
        "  quit                       — 退出程序\n\n";
}

// ============================================================================
// main
// ============================================================================
int main()
{
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif

    std::cout << "============================================\n"
              << "  SG100 实时关节控制示例\n"
              << "============================================\n\n";

    auto& manager = SG100DeviceManager::Instance();

    // ---------- 连接设备 ----------
    const std::string config_path = FindSerialConfigPath();
    if (!config_path.empty()) {
        std::cout << "使用串口配置文件: " << config_path << "\n";
    }

    if (config_path.empty() || !manager.AddDevice("HAND", config_path, 1)) {
        std::cout << "配置文件串口无效，枚举可用串口...\n";
        auto ports = SG100DeviceManager::ListSerialPorts();
        if (ports.empty()) {
            std::cerr << "[错误] 未发现任何串口\n";
            return 1;
        }
        std::cout << "可用串口:\n";
        for (int i = 0; i < (int)ports.size(); i++) {
            std::cout << "  [" << i << "] " << ports[i].port
                      << " - " << ports[i].description << "\n";
        }
        std::cout << "请输入串口编号: " << std::flush;
        int idx = 0;
        std::cin >> idx;
        std::cin.ignore();
        if (idx < 0 || idx >= (int)ports.size()) {
            std::cerr << "[错误] 无效编号\n";
            return 1;
        }
        if (!manager.AddDeviceByParams("HAND", ports[idx].port, 1000000)) {
            std::cerr << "[错误] 添加设备失败\n";
            return 1;
        }
    }

    auto* hand = manager.GetDevice("HAND");
    if (!hand) {
        std::cerr << "[错误] 获取设备句柄失败\n";
        return 1;
    }

    if (!JOINT_MAP_CONFIG_DIR.empty()) {
        hand->SetConfigSearchDir(JOINT_MAP_CONFIG_DIR);
        std::cout << "使用角度映射目录: " << hand->GetConfigSearchDir() << "\n";
    }

    // ---------- 注册状态回调 ----------
    hand->RegisterHandStatusCallback([](const Hand_Status_t& status) {
        std::lock_guard<std::mutex> lock(g_status_mutex);
        g_hand_status = status;
        g_status_updated.store(true);
    });

    // ---------- 启动 ----------
    if (!manager.StartAll()) {
        std::cerr << "[错误] 启动设备失败\n";
        manager.Clear();
        return 1;
    }
    std::cout << "[OK] 设备已连接并启动\n";

    if (!WaitForRealtimeTraffic(hand)) {
        std::cout << "[WARN] 启动后 1s 内未观察到新的实时状态，继续尝试运行模式查询..." << std::endl;
    }

    Palm_Runtime_Mode_Status_t mode_status{};
    if (!QueryAndReadRuntimeMode(hand, mode_status)) {
        std::cerr << "[错误] 切换前查询掌板运行模式失败\n";
        manager.StopAll();
        manager.Clear();
        return 1;
    }
    PrintRuntimeMode("切换前", mode_status);

    // ---------- 位置映射已固定启用 ----------
    g_position_transform_enabled.store(true);
    std::cout << "[OK] 位置映射固定启用，示例将发送/显示用户角度 (deg)\n";

    // ---------- 按需切换到实时模式 ----------
    if (mode_status.realtime_enabled && mode_status.mode == PALM_RUNTIME_MODE_REALTIME) {
        std::cout << "[OK] 掌板当前已处于实时模式，跳过重复切换\n";
    } else {
        std::cout << "切换到实时模式...\n";
        if (!hand->SwitchToRealtimeMode()) {
            std::cerr << "[错误] 切换实时模式失败\n";
            manager.StopAll();
            manager.Clear();
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        if (!QueryAndReadRuntimeMode(hand, mode_status)) {
            std::cerr << "[错误] 切换后查询掌板运行模式失败\n";
            manager.StopAll();
            manager.Clear();
            return 1;
        }
        PrintRuntimeMode("切换后", mode_status);
    }
    std::cout << "[OK] 已进入实时模式\n\n";

    // ---------- 初始化控制帧 ----------
    InitHandCtrl(0.1f, 0.0f, 0.0f, 0.5f);
    std::cout << "所有关节已初始化: 位置模式, 目标=0 deg, kp=0.1, limit=0.5\n";

    // ---------- 启动实时控制线程 ----------
    std::thread ctrl_thread(RealtimeControlThread, hand);

    PrintHelp();

    // ---------- 命令行交互 ----------
    std::string line;
    while (g_running.load()) {
        std::cout << "> " << std::flush;
        if (!std::getline(std::cin, line)) break;

        std::istringstream ss(line);
        std::string cmd;
        ss >> cmd;

        if (cmd == "quit" || cmd == "q") {
            g_running.store(false);

        } else if (cmd == "help" || cmd == "h") {
            PrintHelp();

        } else if (cmd == "status") {
            PrintStatus();

        } else if (cmd == "cali") {
            int id;
            if (!(ss >> id)) {
                std::cout << "用法: cali <id>\n";
                continue;
            }
            if (id < 1 || id > JOINT_COUNT) {
                std::cout << "关节ID 范围: 1~" << JOINT_COUNT << "\n";
                continue;
            }

            std::cout << "开始校准 J" << id << "，校准期间暂停实时控制下发...\n";
            g_control_paused.store(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(60));

            const bool ok = hand->CalibrateJointZero(static_cast<uint8_t>(id));

            g_control_paused.store(false);
            std::cout << (ok ? "[OK] " : "[错误] ")
                      << "J" << id << (ok ? " 校准完成" : " 校准失败") << "\n";

        } else if (cmd == "transform") {
            std::cout << "位置映射开关已移除，当前固定为 ON (deg 模式)\n";

        } else if (cmd == "set") {
            int id;
            float pos;
            if (!(ss >> id >> pos)) {
                std::cout << "用法: set <id> <pos>\n";
                continue;
            }
            if (id < 1 || id > JOINT_COUNT) {
                std::cout << "关节ID 范围: 1~" << JOINT_COUNT << "\n";
                continue;
            }
            {
                std::lock_guard<std::mutex> lock(g_ctrl_mutex);
                g_hand_ctrl.joints[id - 1].position = pos;
                g_hand_ctrl.joints[id - 1].mode     = MODE_POSITION;
            }
            std::cout << "J" << id << " 目标位置 -> " << pos << " " << PositionValueLabel() << "\n";

        } else if (cmd == "pid") {
            int id;
            float kp, ki, kd;
            if (!(ss >> id >> kp >> ki >> kd)) {
                std::cout << "用法: pid <id> <kp> <ki> <kd>\n";
                continue;
            }
            if (id < 1 || id > JOINT_COUNT) {
                std::cout << "关节ID 范围: 1~" << JOINT_COUNT << "\n";
                continue;
            }
            {
                std::lock_guard<std::mutex> lock(g_ctrl_mutex);
                g_hand_ctrl.joints[id - 1].p = SG100_Quant::encodePid(kp);
                g_hand_ctrl.joints[id - 1].i = SG100_Quant::encodePid(ki);
                g_hand_ctrl.joints[id - 1].d = SG100_Quant::encodePid(kd);
            }
            std::cout << "J" << id << " PID -> kp=" << kp
                      << " ki=" << ki << " kd=" << kd << "\n";

        } else if (cmd == "limit") {
            int id;
            float val;
            if (!(ss >> id >> val)) {
                std::cout << "用法: limit <id> <val>\n";
                continue;
            }
            if (id < 1 || id > JOINT_COUNT) {
                std::cout << "关节ID 范围: 1~" << JOINT_COUNT << "\n";
                continue;
            }
            val = (std::max)(0.0f, (std::min)(1.0f, val));
            {
                std::lock_guard<std::mutex> lock(g_ctrl_mutex);
                g_hand_ctrl.joints[id - 1].output_limit = SG100_Quant::encodeOutLim(val);
            }
            std::cout << "J" << id << " output_limit -> " << val << "\n";

        } else if (cmd == "mode") {
            int id, m;
            if (!(ss >> id >> m)) {
                std::cout << "用法: mode <id> <m>  (1=位置 2=速度 3=力矩 4=开环)\n";
                continue;
            }
            if (id < 1 || id > JOINT_COUNT) {
                std::cout << "关节ID 范围: 1~" << JOINT_COUNT << "\n";
                continue;
            }
            if (m < 1 || m > 4) {
                std::cout << "模式范围: 1~4\n";
                continue;
            }
            {
                std::lock_guard<std::mutex> lock(g_ctrl_mutex);
                g_hand_ctrl.joints[id - 1].mode = static_cast<uint8_t>(m);
            }
            const char* mode_names[] = {"", "位置", "速度", "力矩", "开环"};
            std::cout << "J" << id << " 控制模式 -> " << mode_names[m] << "\n";

        } else if (!cmd.empty()) {
            std::cout << "未知命令: " << cmd << "，输入 help 查看帮助\n";
        }
    }

    // ---------- 退出清理 ----------
    if (ctrl_thread.joinable()) ctrl_thread.join();
    manager.StopAll();
    manager.Clear();
    std::cout << "\n[已退出]\n";
    return 0;
}

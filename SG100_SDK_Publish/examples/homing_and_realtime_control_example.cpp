/**
 * @file homing_and_realtime_control_example.cpp
 * @brief SG100 SDK 归零校准 + 实时位置控制示例
 *
 * 流程:
 *   1. 连接设备
 *   2. 循环从命令行读取十六进制掩码 (如 0x0003 表示关节1+2) 执行归零校准, 输入 quit 结束校准
 *   3. 初始化整手控制帧
 *   4. 整手位置环实时控制 (默认: kp=0.1, ki=0.0, kd=0.0, output_limit=1.0)
 *   5. 命令行交互:
 *        set <id> <pos>            — 设置关节目标位置 (角度，单位: 度)
 *        pid <id> <kp> <ki> <kd>  — 修改关节 PID 参数
 *        limit <id> <val>          — 修改关节输出限制 (0~1)
 *        status                    — 打印所有关节当前缓存状态
 *        help                      — 显示帮助
 *        quit                      — 退出程序
 */

#include "sg100_device_manager.h"

#include <atomic>
#include <chrono>
#include <cstring>
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

// ============================================================================
// 全局共享控制状态
// ============================================================================
static std::mutex       g_ctrl_mutex;
static Hand_Control_t   g_hand_ctrl;
static std::atomic<bool> g_running{true};

static constexpr uint32_t kHomingIntervalMs = 10000;

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
// 辅助: 初始化整手控制帧为位置环默认参数 (所有关节目标位置=0 rad)
// ============================================================================
static void InitHandCtrl(float kp, float ki, float kd, float output_limit)
{
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
// 控制线程: 以 10 ms 周期持续下发整手控制帧
// ============================================================================
static void ControlThreadFunc(SG100Device* hand)
{
    while (g_running.load()) {
        {
            std::lock_guard<std::mutex> lock(g_ctrl_mutex);
            hand->SetHandControl(g_hand_ctrl);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// ============================================================================
// 打印帮助菜单
// ============================================================================
static void PrintHelp()
{
    std::cout
        << "\n命令帮助:\n"
        << "  set <id> <pos>            设置关节 id (1-" << JOINT_COUNT << ") 目标位置 (角度, 单位: 度)\n"
        << "  pid <id> <kp> <ki> <kd>  修改关节 id 的 PID 参数\n"
        << "  limit <id> <val>          修改关节 id 的输出限制 (0~1)\n"
        << "  status                    打印所有关节当前缓存位置/速度/电流\n"
        << "  help                      显示此帮助\n"
        << "  quit                      退出程序\n"
        << std::endl;
}

// ============================================================================
// 命令行解析与执行
// ============================================================================
static void ParseAndExecute(const std::string& line, SG100Device* hand)
{
    std::istringstream ss(line);
    std::string cmd;
    ss >> cmd;

    if (cmd == "set") {
        int id;
        float pos;
        if (!(ss >> id >> pos)) {
            std::cout << "[错误] 用法: set <id> <pos>" << std::endl;
            return;
        }
        if (id < 1 || id > JOINT_COUNT) {
            std::cout << "[错误] id 范围: 1~" << JOINT_COUNT << std::endl;
            return;
        }
        {
            std::lock_guard<std::mutex> lock(g_ctrl_mutex);
            g_hand_ctrl.joints[id - 1].position = pos;
        }
        std::cout << "[OK] 关节 " << id << " 目标位置 -> " << pos << std::endl;

    } else if (cmd == "pid") {
        int id;
        float kp, ki, kd;
        if (!(ss >> id >> kp >> ki >> kd)) {
            std::cout << "[错误] 用法: pid <id> <kp> <ki> <kd>" << std::endl;
            return;
        }
        if (id < 1 || id > JOINT_COUNT) {
            std::cout << "[错误] id 范围: 1~" << JOINT_COUNT << std::endl;
            return;
        }
        {
            std::lock_guard<std::mutex> lock(g_ctrl_mutex);
            g_hand_ctrl.joints[id - 1].p = SG100_Quant::encodePid(kp);
            g_hand_ctrl.joints[id - 1].i = SG100_Quant::encodePid(ki);
            g_hand_ctrl.joints[id - 1].d = SG100_Quant::encodePid(kd);
        }
        std::cout << "[OK] 关节 " << id << " PID -> kp=" << kp
                  << " ki=" << ki << " kd=" << kd << std::endl;

    } else if (cmd == "limit") {
        int id;
        float val;
        if (!(ss >> id >> val)) {
            std::cout << "[错误] 用法: limit <id> <val>" << std::endl;
            return;
        }
        if (id < 1 || id > JOINT_COUNT) {
            std::cout << "[错误] id 范围: 1~" << JOINT_COUNT << std::endl;
            return;
        }
        {
            std::lock_guard<std::mutex> lock(g_ctrl_mutex);
            g_hand_ctrl.joints[id - 1].output_limit = SG100_Quant::encodeOutLim(val);
        }
        std::cout << "[OK] 关节 " << id << " output_limit -> " << val << std::endl;

    } else if (cmd == "status") {
        auto status = hand->GetCachedHandStatus();
        std::cout << "\n当前关节缓存状态:\n";
        for (int i = 0; i < JOINT_COUNT; i++) {
            const auto& js = status.joints[i];
            std::cout << "  J" << (i + 1)
                      << ": pos="    << js.position
                      << " vel="     << SG100_Quant::decodeVel(js.velocity)    << "\xb0/s"
                      << " cur_q="   << SG100_Quant::decodeCurQ(js.cur_q)     << "A"
                      << " cur_d="   << SG100_Quant::decodeCurD(js.cur_d)     << "A"
                      << " mode="    << static_cast<int>(js.mode)
                      << "\n";
        }
        std::cout << std::endl;

    } else if (cmd == "help") {
        PrintHelp();

    } else if (cmd == "quit" || cmd == "exit") {
        g_running.store(false);

    } else if (!cmd.empty()) {
        std::cout << "[未知命令] 输入 help 查看帮助" << std::endl;
    }
}

// ============================================================================
// main
// ============================================================================
int main()
{
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif
    std::cout << "============================================\n"
              << "  SG100 归零校准 + 实时位置控制示例\n"
              << "============================================\n" << std::endl;

    auto& manager = SG100DeviceManager::Instance();

    // ------------------------------------------------------------------
    // 1. 连接设备 (优先读取配置文件, 否则枚举串口供用户选择)
    // ------------------------------------------------------------------
    if (!manager.AddDevice("HAND", "serial_config.json", 1)) {
        auto ports = SG100DeviceManager::ListSerialPorts();
        if (ports.empty()) {
            std::cerr << "[错误] 未找到串口设备，请检查连接" << std::endl;
            return 1;
        }
        std::cout << "可用串口:\n";
        for (size_t i = 0; i < ports.size(); i++) {
            std::cout << "  [" << i << "] " << ports[i].port
                      << " - " << ports[i].description << "\n";
        }
        std::cout << "请输入串口编号: " << std::flush;
        int idx = 0;
        std::cin >> idx;
        std::cin.ignore();
        if (idx < 0 || idx >= static_cast<int>(ports.size())) {
            std::cerr << "[错误] 无效编号" << std::endl;
            return 1;
        }
        if (!manager.AddDeviceByParams("HAND", ports[idx].port, 1000000)) {
            std::cerr << "[错误] 添加设备失败" << std::endl;
            return 1;
        }
    }

    auto* hand = manager.GetDevice("HAND");
    if (!hand) {
        std::cerr << "[错误] 获取设备句柄失败" << std::endl;
        return 1;
    }

    if (!JOINT_MAP_CONFIG_DIR.empty()) {
        hand->SetConfigSearchDir(JOINT_MAP_CONFIG_DIR);
        std::cout << "使用角度映射目录: " << hand->GetConfigSearchDir() << std::endl;
    }

    hand->RegisterErrorCallback([](uint8_t joint_id, uint8_t error_code) {
        std::cerr << "[ERROR] 关节 " << static_cast<int>(joint_id)
                  << " 错误码=" << static_cast<int>(error_code) << std::endl;
    });

    if (!manager.StartAll()) {
        std::cerr << "[错误] 启动设备失败" << std::endl;
        manager.Clear();
        return 1;
    }
    std::cout << "[OK] 设备已连接并启动\n" << std::endl;

    if (!WaitForRealtimeTraffic(hand)) {
        std::cout << "[WARN] 启动后 1s 内未观察到新的实时状态，继续尝试运行模式查询..." << std::endl;
    }

    Palm_Runtime_Mode_Status_t mode_status{};
    if (!QueryAndReadRuntimeMode(hand, mode_status)) {
        std::cerr << "[错误] 归零前查询掌板运行模式失败" << std::endl;
        manager.StopAll();
        manager.Clear();
        return 1;
    }
    PrintRuntimeMode("归零前", mode_status);
    std::cout << "[INFO] 归零接口将按当前掌板模式自动管理 pause/resume，并在结束后恢复到原模式" << std::endl;

    // ------------------------------------------------------------------
    // 2. 使用掩码接口执行归零校准 (循环, 输入 quit 退出)
    // ------------------------------------------------------------------
    std::cout << "进入归零校准模式\n"
              << "  掩码格式: 每一位对应一个关节, bit0=关节1, bit1=关节2, ..., bit11=关节12\n"
              << "  例: 0x0001=关节1  0x0002=关节2  0x0003=关节1+2  0x0FFF=全部关节\n"
              << "  输入 quit 完成校准并进入控制模式\n"
              << "  interval_ms = " << kHomingIntervalMs << "\n"
              << std::endl;

    {
        std::string homing_line;
        while (true) {
            std::cout << "[归零] 请输入掩码 (quit 退出): " << std::flush;
            if (!std::getline(std::cin, homing_line)) {
                break;  // EOF
            }
            // 去除首尾空白
            size_t s = homing_line.find_first_not_of(" \t\r\n");
            if (s == std::string::npos) continue;
            homing_line = homing_line.substr(s);
            size_t e = homing_line.find_last_not_of(" \t\r\n");
            if (e != std::string::npos) homing_line = homing_line.substr(0, e + 1);

            if (homing_line == "quit" || homing_line == "exit") {
                std::cout << "[归零] 退出校准，进入控制模式\n" << std::endl;
                break;
            }

            // 解析十六进制掩码 (支持带/不带 0x 前缀)
            unsigned long mask_val = 0;
            try {
                size_t parsed = 0;
                mask_val = std::stoul(homing_line, &parsed, 16);
                if (parsed != homing_line.size()) {
                    throw std::invalid_argument("partial parse");
                }
            } catch (...) {
                std::cout << "[错误] 无效掩码格式，请输入十六进制数 (如 0x0003 表示关节1+2)" << std::endl;
                continue;
            }

            if (mask_val == 0 || mask_val > 0x0FFF) {
                std::cout << "[错误] 掩码范围应为 0x0001 ~ 0x0FFF" << std::endl;
                continue;
            }

            SG100JointMask joint_mask = static_cast<SG100JointMask>(mask_val);
            std::cout << "[归零] 执行归零校准, joint_mask = 0x"
                      << std::hex << mask_val << std::dec << " ..." << std::endl;

            if (!hand->HomingZeroMasked(joint_mask, kHomingIntervalMs, true)) {
                std::cerr << "[错误] 归零校准失败" << std::endl;
            } else {
                std::cout << "[OK] 关节归零校准完成，已自动恢复实时模式\n" << std::endl;
            }
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    if (!QueryAndReadRuntimeMode(hand, mode_status)) {
        std::cerr << "[错误] 归零后查询掌板运行模式失败" << std::endl;
        manager.StopAll();
        manager.Clear();
        return 1;
    }
    PrintRuntimeMode("归零后", mode_status);
    std::cout << "[OK] 归零校准完成，已自动恢复实时模式\n" << std::endl;

    // ------------------------------------------------------------------
    // 3. 初始化整手控制帧 (位置环, 默认 PID 参数)
    // ------------------------------------------------------------------
    constexpr float DEFAULT_KP           = 0.1f;
    constexpr float DEFAULT_KI           = 0.0f;
    constexpr float DEFAULT_KD           = 0.0f;
    constexpr float DEFAULT_OUTPUT_LIMIT = 1.0f;

    InitHandCtrl(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_OUTPUT_LIMIT);

    // ------------------------------------------------------------------
    // 4. 启动控制线程 (10 ms 周期持续下发 Hand_Control_t)
    // ------------------------------------------------------------------
    std::thread ctrl_thread(ControlThreadFunc, hand);

    // ------------------------------------------------------------------
    // 5. 命令行交互
    // ------------------------------------------------------------------
    std::cout << "进入实时位置控制模式\n"
              << "  kp=" << DEFAULT_KP
              << "  ki=" << DEFAULT_KI
              << "  kd=" << DEFAULT_KD
              << "  output_limit=" << DEFAULT_OUTPUT_LIMIT << "\n";
    PrintHelp();

    std::string line;
    while (g_running.load()) {
        std::cout << "> " << std::flush;
        if (!std::getline(std::cin, line)) {
            break;  // EOF (Ctrl+Z / Ctrl+D)
        }
        ParseAndExecute(line, hand);
    }

    // ------------------------------------------------------------------
    // 6. 清理退出
    // ------------------------------------------------------------------
    g_running.store(false);
    if (ctrl_thread.joinable()) {
        ctrl_thread.join();
    }

    std::cout << "\n停止设备..." << std::endl;
    manager.StopAll();
    manager.Clear();
    std::cout << "已退出。" << std::endl;
    return 0;
}

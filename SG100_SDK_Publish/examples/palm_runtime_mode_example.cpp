/**
 * @file palm_runtime_mode_example.cpp
 * @brief SG100 SDK 掌板运行模式查询示例
 *
 * 演示:
 *   1. 连接掌板设备并启动 SDK
 *   2. 在实时模式下查询当前运行模式
 *   3. 切换到透传模式后再次查询
 *   4. 切回实时模式并再次查询
 */

#include "sg100_device_manager.h"

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#ifdef _WIN32
#include <windows.h>
#endif

// 角度映射配置目录（留空表示使用默认搜索路径）
const std::string JOINT_MAP_CONFIG_DIR = "";

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

static const char* UartPeerName(uint8_t peer)
{
    switch (peer) {
        case UART_LINK_PEER_PC:
            return "pc";
        case UART_LINK_PEER_ESP32:
            return "esp32";
        default:
            return "unknown";
    }
}

static bool QueryAndReadRuntimeMode(SG100Device* hand,
                                    Palm_Runtime_Mode_Status_t& status,
                                    uint32_t wait_ms = 200)
{
    if (!hand->PalmRuntimeModeGetStatus()) {
        return false;
    }

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (hand->GetPalmRuntimeModeStatus(status)) {
            return true;
        }
    }

    return hand->GetPalmRuntimeModeStatus(status);
}

static void PrintRuntimeMode(const Palm_Runtime_Mode_Status_t& status, const char* stage)
{
    std::cout << "[" << stage << "]\n"
              << "  mode                 = " << RuntimeModeName(status.mode)
              << " (" << static_cast<int>(status.mode) << ")\n"
              << "  realtime_enabled     = " << static_cast<int>(status.realtime_enabled) << "\n"
              << "  fake_pc_enabled      = " << static_cast<int>(status.fake_pc_enabled) << "\n"
              << "  uart_link_enabled    = " << static_cast<int>(status.uart_link_enabled) << "\n"
              << "  uart_link_peer       = " << UartPeerName(status.uart_link_peer)
              << " (" << static_cast<int>(status.uart_link_peer) << ")\n"
              << "  uart_link_target_uart= " << static_cast<int>(status.uart_link_target_uart_id) << "\n"
              << std::endl;
}

int main()
{
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif

    std::cout << "============================================\n"
              << "  SG100 掌板运行模式查询示例\n"
              << "============================================\n"
              << std::endl;

    auto& manager = SG100DeviceManager::Instance();
    const std::string config_path = FindSerialConfigPath();

    if (!config_path.empty()) {
        std::cout << "使用串口配置文件: " << config_path << std::endl;
    }

    if (config_path.empty() || !manager.AddDevice("HAND", config_path, 1)) {
        std::cout << "配置文件不可用，枚举串口..." << std::endl;
        auto ports = SG100DeviceManager::ListSerialPorts();
        if (ports.empty()) {
            std::cerr << "[错误] 未发现任何串口" << std::endl;
            return 1;
        }

        for (size_t index = 0; index < ports.size(); ++index) {
            std::cout << "  [" << index << "] " << ports[index].port
                      << " - " << ports[index].description << std::endl;
        }

        std::cout << "请输入串口编号: " << std::flush;
        int selected = 0;
        std::cin >> selected;
        std::cin.ignore();
        if (selected < 0 || selected >= static_cast<int>(ports.size())) {
            std::cerr << "[错误] 无效编号" << std::endl;
            return 1;
        }

        if (!manager.AddDeviceByParams("HAND", ports[selected].port, 1000000)) {
            std::cerr << "[错误] 添加设备失败" << std::endl;
            return 1;
        }
    }

    SG100Device* hand = manager.GetDevice("HAND");
    if (!hand) {
        std::cerr << "[错误] 获取设备句柄失败" << std::endl;
        return 1;
    }

    if (!JOINT_MAP_CONFIG_DIR.empty()) {
        hand->SetConfigSearchDir(JOINT_MAP_CONFIG_DIR);
        std::cout << "使用角度映射目录: " << hand->GetConfigSearchDir() << std::endl;
    }

    hand->RegisterErrorCallback([](uint8_t joint_id, uint8_t error_code) {
        std::cerr << "[ERROR] src=" << static_cast<int>(joint_id)
                  << " error=" << static_cast<int>(error_code) << std::endl;
    });

    if (!manager.StartAll()) {
        std::cerr << "[错误] 启动设备失败" << std::endl;
        manager.Clear();
        return 1;
    }

    Palm_Runtime_Mode_Status_t status{};

    if (!QueryAndReadRuntimeMode(hand, status)) {
        std::cerr << "[错误] 实时模式下查询运行模式失败" << std::endl;
        manager.StopAll();
        manager.Clear();
        return 1;
    }
    PrintRuntimeMode(status, "startup/realtime");

    if (!hand->SwitchToPassthroughMode()) {
        std::cerr << "[错误] 切换到透传模式失败" << std::endl;
        manager.StopAll();
        manager.Clear();
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (!QueryAndReadRuntimeMode(hand, status)) {
        std::cerr << "[错误] 透传模式下查询运行模式失败" << std::endl;
        manager.StopAll();
        manager.Clear();
        return 1;
    }
    PrintRuntimeMode(status, "passthrough");

    if (!hand->SwitchToRealtimeMode()) {
        std::cerr << "[错误] 切回实时模式失败" << std::endl;
        manager.StopAll();
        manager.Clear();
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (!QueryAndReadRuntimeMode(hand, status)) {
        std::cerr << "[错误] 切回实时模式后查询运行模式失败" << std::endl;
        manager.StopAll();
        manager.Clear();
        return 1;
    }
    PrintRuntimeMode(status, "back-to-realtime");

    manager.StopAll();
    manager.Clear();

    std::cout << "示例执行结束" << std::endl;
    return 0;
}
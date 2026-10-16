/**
 * @file comm_status_test.cpp
 * @brief SG100 通讯状态监控测试
 * 
 * 功能:
 * 1. 在实时调度模式下持续发送控制数据
 * 2. 监控掌板回传状态计数位 (GetStatusRecvCount)
 * 3. 若超过1s未更新则判定通讯超时，打印带时间戳的异常信息
 * 4. 检查串口状态是否异常
 */

#include "sg100_device_manager.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <atomic>
#include <csignal>
#ifdef _WIN32
#include <windows.h>
#endif

// 角度映射配置目录（留空表示使用默认搜索路径）
const std::string JOINT_MAP_CONFIG_DIR = "";

// ============================================================================
// 全局控制
// ============================================================================
static std::atomic<bool> g_running(true);

static void SignalHandler(int sig) {
    (void)sig;
    g_running = false;
}

// ============================================================================
// 获取当前时间戳字符串 (yyyy-MM-dd HH:mm:ss.mmm)
// ============================================================================
static std::string GetTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::tm tm_buf;
#ifdef _WIN32
    localtime_s(&tm_buf, &time_t_now);
#else
    localtime_r(&time_t_now, &tm_buf);
#endif

    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S")
        << "." << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

// ============================================================================
// 状态更新回调 (用于验证数据接收)
// ============================================================================
static void OnHandStatusUpdate(const Hand_Status_t& status) {
    // 静默接收, 不打印 (由主循环监控计数位)
    (void)status;
}

// ============================================================================
// 错误回调
// ============================================================================
static void OnError(uint8_t joint_id, uint8_t error_code) {
    std::cerr << "[" << GetTimestamp() << "] ERROR: Joint " << (int)joint_id 
              << " error_code=" << (int)error_code << std::endl;
}

// ============================================================================
// 检查串口状态
// ============================================================================
static void CheckSerialStatus(SG100Device* device) {
    if (!device->IsSerialOpen()) {
        std::cerr << "[" << GetTimestamp() << "] 串口异常: 串口未打开或已断开 ("
                  << device->GetPort() << ")" << std::endl;
    } else {
        std::cout << "[" << GetTimestamp() << "] 串口正常: " 
                  << device->GetPort() << std::endl;
    }
}

// ============================================================================
// 主函数
// ============================================================================
int main() {
#ifdef _WIN32
    SetConsoleOutputCP(65001);
    SetConsoleCP(65001);
#endif
    std::signal(SIGINT, SignalHandler);

    std::cout << "========================================" << std::endl;
    std::cout << "  SG100 通讯状态监控测试" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "[" << GetTimestamp() << "] 测试启动" << std::endl;

    // ---- 初始化设备 ----
    auto& manager = SG100DeviceManager::Instance();

    if (!manager.AddDevice("HAND1", "serial_config.json", 1)) {
        auto ports = SG100DeviceManager::ListSerialPorts();
        if (ports.empty()) {
            std::cerr << "[" << GetTimestamp() << "] 未找到可用串口" << std::endl;
            return 1;
        }
        std::cout << "[" << GetTimestamp() << "] 使用串口: " << ports[0].port << std::endl;
        if (!manager.AddDeviceByParams("HAND1", ports[0].port, 1000000)) {
            std::cerr << "[" << GetTimestamp() << "] 添加设备失败" << std::endl;
            return 1;
        }
    }

    auto* hand = manager.GetDevice("HAND1");
    if (!hand) {
        std::cerr << "[" << GetTimestamp() << "] 获取设备失败" << std::endl;
        return 1;
    }

    if (!JOINT_MAP_CONFIG_DIR.empty()) {
        hand->SetConfigSearchDir(JOINT_MAP_CONFIG_DIR);
        std::cout << "[" << GetTimestamp() << "] 使用角度映射目录: "
                  << hand->GetConfigSearchDir() << std::endl;
    }

    // 注册回调
    hand->RegisterHandStatusCallback(OnHandStatusUpdate);
    hand->RegisterErrorCallback(OnError);

    // 启动设备 (默认实时调度模式)
    if (!manager.StartAll()) {
        std::cerr << "[" << GetTimestamp() << "] 启动设备失败" << std::endl;
        manager.Clear();
        return 1;
    }

    std::cout << "[" << GetTimestamp() << "] 设备启动成功, 工作模式: 实时调度" << std::endl;
    std::cout << "[" << GetTimestamp() << "] 开始通讯状态监控 (Ctrl+C 退出)..." << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    // ---- 初始化控制数据 (所有关节位置0度) ----
    float positions[JOINT_COUNT] = {0};

    // ---- 通讯状态监控 ----
    uint32_t last_count = hand->GetStatusRecvCount();
    auto last_update_time = std::chrono::steady_clock::now();
    bool was_timeout = false;
    const auto COMM_TIMEOUT = std::chrono::milliseconds(1000);  // 1s超时阈值

    while (g_running) {
        // 1. 持续发送位置控制数据 (保持通讯)
        hand->SetJointPositions(positions, JOINT_COUNT);

        // 2. 读取当前通讯状态计数
        uint32_t current_count = hand->GetStatusRecvCount();

        // 3. 判断计数是否更新
        if (current_count != last_count) {
            // 计数已更新 - 通讯正常
            last_count = current_count;
            last_update_time = std::chrono::steady_clock::now();

            if (was_timeout) {
                // 从超时状态恢复
                std::cout << "[" << GetTimestamp() << "] 通讯恢复正常, 计数=" 
                          << current_count << std::endl;
                CheckSerialStatus(hand);
                was_timeout = false;
            }
        } else {
            // 计数未更新 - 检查是否超时
            auto elapsed = std::chrono::steady_clock::now() - last_update_time;
            if (elapsed >= COMM_TIMEOUT && !was_timeout) {
                // 首次检测到超时
                auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
                std::cerr << "[" << GetTimestamp() << "] 通讯连接异常: 超过 " 
                          << elapsed_ms << "ms 未收到掌板回传状态, 最后计数=" 
                          << last_count << std::endl;

                // 检查串口状态
                CheckSerialStatus(hand);
                was_timeout = true;
            }
        }

        // 每100ms检测一次
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // ---- 退出清理 ----
    std::cout << "\n----------------------------------------" << std::endl;
    std::cout << "[" << GetTimestamp() << "] 测试结束" << std::endl;
    std::cout << "[" << GetTimestamp() << "] 最终通讯状态计数: " 
              << hand->GetStatusRecvCount() << std::endl;
    CheckSerialStatus(hand);

    manager.StopAll();
    manager.Clear();

    return 0;
}

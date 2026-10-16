/**
 * @file impedance_position_sin_test.cpp
 * @brief SG100 阻抗模式与位置控制模式的正弦运动测试 Demo
 *
 * 功能描述:
 * 1. 启动并连接 SG100 灵巧手，切换为实时调度模式。
 * 2. 交互式命令行菜单：
 *    - 输入需要控制的关节编号 (1~11)。
 *    - 选择控制模式 (1-位置模式，2-阻抗模式)。
 *    - 输入正弦运动的频率 (Hz)。
 *    - 输入正弦测试持续时间 (秒)。
 *    - 如果是阻抗模式，可进一步输入期望的刚度 Kp 与阻尼 Kd。
 * 3. 执行 -1 ~ +1 弧度 (约 -57.3 ~ +57.3 度) 的正弦目标跟踪运动。
 * 4. 实时在终端中以 5Hz 频率输出目标位置与传感器反馈位置（弧度/度数对照）、运行速度、电流等。
 * 5. 提示关节的软限幅极限并处理越界保护。
 */

#include "sg100_device_manager.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstring>
#include <mutex>
#include <atomic>
#include <string>
#include <fstream>
#include <iomanip>

#ifdef _WIN32
#include <windows.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 角度映射配置目录（留空表示使用默认搜索路径）
const std::string JOINT_MAP_CONFIG_DIR = "";

// ============================================================================
// 全局状态和互斥锁
// ============================================================================
static std::mutex g_status_mutex;
static Hand_Status_t g_hand_status;
static std::atomic<bool> g_status_received{false};

// 状态反馈回调
static void OnHandStatusUpdate(const Hand_Status_t& status) {
    std::lock_guard<std::mutex> lock(g_status_mutex);
    g_hand_status = status;
    g_status_received.store(true);
}

// 错误反馈回调
static void OnJointError(uint8_t joint_id, uint8_t error_code) {
    std::cerr << "\n[ERROR] 关节 " << (int)joint_id 
              << " 报错, 错误码: 0x" << std::hex << (int)error_code << std::dec << std::endl;
}

// 辅助方法：判断配置文件是否存在
static bool FileExists(const std::string& path) {
    std::ifstream file(path);
    return file.good();
}

// 寻找配置文件路径
static std::string FindSerialConfigPath() {
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

// 主函数
int main() {
#ifdef _WIN32
    // Windows下防止中文输出乱码
    SetConsoleOutputCP(CP_UTF8);
#endif

    std::cout << "================================================================\n";
    std::cout << "       SG100 阻抗控制与位置控制模式正弦运动测试程序 (Radian)\n";
    std::cout << "================================================================\n\n";

    auto& manager = SG100DeviceManager::Instance();
    const std::string config_path = FindSerialConfigPath();

    // 1. 初始化并连接灵巧手设备
    if (!config_path.empty()) {
        std::cout << "[INFO] 加载串口配置文件: " << config_path << "\n";
        if (!manager.AddDevice("HAND", config_path, 1)) {
            std::cerr << "[ERROR] 从配置文件添加设备失败！\n";
            return 1;
        }
    } else {
        std::cout << "[INFO] 未发现配置文件，自动枚举系统可用串口...\n";
        auto ports = SG100DeviceManager::ListSerialPorts();
        if (ports.empty()) {
            std::cerr << "[ERROR] 系统中未检测到串口，请确认串口线是否插好！\n";
            return 1;
        }
        std::cout << "[INFO] 自动选择串口: " << ports[0].port << " (" << ports[0].description << ")\n";
        if (!manager.AddDeviceByParams("HAND", ports[0].port, 1000000)) {
            std::cerr << "[ERROR] 添加串口设备失败！\n";
            return 1;
        }
    }

    auto* hand = manager.GetDevice("HAND");
    if (!hand) {
        std::cerr << "[ERROR] 获取设备实例失败！\n";
        return 1;
    }

    if (!JOINT_MAP_CONFIG_DIR.empty()) {
        hand->SetConfigSearchDir(JOINT_MAP_CONFIG_DIR);
        std::cout << "[INFO] 使用角度映射目录: " << hand->GetConfigSearchDir() << "\n";
    }

    // 2. 注册回调
    hand->RegisterHandStatusCallback(OnHandStatusUpdate);
    hand->RegisterErrorCallback(OnJointError);

    // 3. 启动后台线程
    if (!manager.StartAll()) {
        std::cerr << "[ERROR] 启动后台通信调度失败！\n";
        manager.Clear();
        return 1;
    }
    std::cout << "[OK] 通信已启动。\n";

    // 等待接收到第一帧状态数据
    std::cout << "[INFO] 等待掌板状态反馈...";
    for (int i = 0; i < 20; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (g_status_received.load()) break;
    }
    if (!g_status_received.load()) {
        std::cerr << "\n[WARNING] 未能及时获得关节状态反馈，请检查电路供电！\n";
    } else {
        std::cout << " [OK]\n";
    }

    // 4. 强制切换到实时闭环调度模式 (阻抗和位控需要实时周期任务)
    if (hand->GetWorkMode() != WORK_MODE_REALTIME) {
        std::cout << "[INFO] 切换工作模式至: 实时调度模式...\n";
        if (hand->SwitchToRealtimeMode()) {
            std::cout << "[OK] 模式切换成功。\n";
        } else {
            std::cerr << "[ERROR] 切换至实时调度模式失败！\n";
            manager.StopAll();
            manager.Clear();
            return 1;
        }
    } else {
        std::cout << "[OK] 设备当前已处于实时调度模式。\n";
    }

    // 5. 交互式主菜单循环
    std::string input;
    while (true) {
        std::cout << "\n----------------------------------------------------------------\n";
        std::cout << "                        控制测试菜单\n";
        std::cout << "----------------------------------------------------------------\n";
        std::cout << "请输入要控制的关节编号 (1~11) [输入 q 退出]: ";
        if (!std::getline(std::cin, input)) break;
        if (input == "q" || input == "quit") {
            break;
        }
        if (input.empty()) continue;

        int joint_id = 0;
        try {
            joint_id = std::stoi(input);
        } catch (...) {
            std::cout << "[输入错误] 请输入有效的数字编号！\n";
            continue;
        }

        if (joint_id < 1 || joint_id > 11) {
            std::cout << "[输入错误] 关节编号范围必须是 1 ~ 11！\n";
            continue;
        }

        // 选择模式
        int control_mode = 1;
        std::cout << "请选择控制模式 (1-关节位置模式, 2-阻抗模式, 3-关节空间阻抗模式) [默认 1]: ";
        std::getline(std::cin, input);
        if (!input.empty()) {
            try {
                control_mode = std::stoi(input);
            } catch (...) {
                std::cout << "输入无效, 使用默认关节位置模式。\n";
                control_mode = 1;
            }
        }
        if (control_mode != 1 && control_mode != 2 && control_mode != 3) {
            std::cout << "输入无效, 默认设为关节位置模式。\n";
            control_mode = 1;
        }

        // 设置正弦运动频率
        float frequency = 0.5f;
        std::cout << "请输入正弦运动频率 (Hz) [默认 0.5Hz, 建议 0.1~2.0Hz]: ";
        std::getline(std::cin, input);
        if (!input.empty()) {
            try {
                frequency = std::stof(input);
            } catch (...) {
                std::cout << "输入无效, 使用默认频率 0.5Hz。\n";
                frequency = 0.5f;
            }
        }

        // 设置运动持续时间
        float duration = 10.0f;
        std::cout << "请输入测试运动时间 (秒) [默认 10秒]: ";
        std::getline(std::cin, input);
        if (!input.empty()) {
            try {
                duration = std::stof(input);
            } catch (...) {
                std::cout << "输入无效, 使用默认时长 10秒。\n";
                duration = 10.0f;
            }
        }

        // 阻抗特有参数 Kp, Kd
        float kp = 0.04f;
        float kd = 0.01f;
        if (control_mode == 2 || control_mode == 3) {
            std::cout << "请输入阻抗刚度 Kp (0~10) [默认 0.04]: ";
            std::getline(std::cin, input);
            if (!input.empty()) {
                try { kp = std::stof(input); } catch (...) {}
            }
            std::cout << "请输入阻抗阻尼 Kd (0~10) [默认 0.01]: ";
            std::getline(std::cin, input);
            if (!input.empty()) {
                try { kd = std::stof(input); } catch (...) {}
            }
        }

        // 关节软限幅参考 (源自 joint_angle_map.json 角度度数)
        float min_angle = 0.0f;
        float max_angle = 90.0f;
        switch (joint_id) {
            case 1:  min_angle = 0.0f;   max_angle = 90.0f;  break;
            case 2:  min_angle = -90.0f;  max_angle = 90.0f;  break;
            case 3:  min_angle = -90.0f;  max_angle = 90.0f;  break;
            case 4:  min_angle = -90.0f;  max_angle = 0.0f;   break;
            case 5:  min_angle = 0.0f;   max_angle = 90.0f;  break;
            case 6:  min_angle = -90.0f;  max_angle = 90.0f;  break;
            case 7:  min_angle = 0.0f;   max_angle = 90.0f;  break;
            case 8:  min_angle = 0.0f;   max_angle = 90.0f;  break;
            case 9:  min_angle = 0.0f;   max_angle = 180.0f; break;
            case 10: min_angle = 0.0f;   max_angle = 180.0f; break;
            case 11: min_angle = 0.0f;   max_angle = 90.0f;  break;
            default: break;
        }

        std::cout << "\n[配置确认] 控制 J" << joint_id << " 关节进行正弦测试:\n";
        std::cout << "  - 控制模式: " << (control_mode == 1 ? "关节位置模式" : (control_mode == 2 ? "阻抗模式" : "关节空间阻抗模式")) << "\n";
        std::cout << "  - 正弦频率: " << frequency << " Hz, 信号幅值: -1 ~ +1 弧度\n";
        std::cout << "  - 持续时间: " << duration << " 秒\n";
        if (control_mode == 2 || control_mode == 3) {
            std::cout << "  - 阻抗参数: Kp=" << kp << ", Kd=" << kd << "\n";
        }
        std::cout << "  - 关节软极限: [" << min_angle << ", " << max_angle << "] 度"
                  << " (约 [" << (min_angle * M_PI / 180.0f) << ", " << (max_angle * M_PI / 180.0f) << "] 弧度)\n";
        std::cout << "  * 超出极限位置的目标弧度会被 SDK 自动安全裁剪限幅。\n";
        std::cout << "\n测试将在 1.5 秒后自动启动，请注意安全防护，准备观察...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));

        // 6. 执行正弦跟踪主循环
        auto start_time = std::chrono::steady_clock::now();
        int frame_idx = 0;
        uint8_t idx = joint_id - 1;

        std::cout << "\n---- 正弦跟踪控制测试开始 ----\n";
        while (true) {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - start_time).count();
            if (elapsed >= duration) break;

            // 计算目标弧度 (幅值 -1 到 1 rad)
            float target_rad = std::sin(2.0f * static_cast<float>(M_PI) * frequency * static_cast<float>(elapsed));
            // 弧度转为度数 (SDK 内部会对角度进行映射限幅并转回弧度下发给掌板)
            float target_deg = target_rad * 180.0f / static_cast<float>(M_PI);

            // 目标角速度
            float target_vel_rad_s = 2.0f * static_cast<float>(M_PI) * frequency * std::cos(2.0f * static_cast<float>(M_PI) * frequency * static_cast<float>(elapsed));
            float target_vel_deg_s = target_vel_rad_s * 180.0f / static_cast<float>(M_PI);

            // 组装整手控制帧 (其余关节设为空闲/ unpowered)
            Hand_Control_t ctrl;
            std::memset(&ctrl, 0, sizeof(ctrl));
            for (int i = 0; i < JOINT_COUNT; i++) {
                ctrl.joints[i].id = static_cast<uint8_t>(i + 1);
                ctrl.joints[i].mode = MODE_NULL;
                ctrl.joints[i].position = 0.0f;
            }

            // 装载控制关节参数
            ctrl.joints[idx].id = joint_id;
            if (control_mode == 1) {
                // 关节位置模式 (绝对角度闭环控制)
                ctrl.joints[idx].mode = MODE_JOINT_POSITION;
                ctrl.joints[idx].position = target_deg;
                ctrl.joints[idx].p = SG100_Quant::encodePid(0.5f); // 适中的位置刚度
                ctrl.joints[idx].i = SG100_Quant::encodePid(0.0f);
                ctrl.joints[idx].d = SG100_Quant::encodePid(0.05f); // 适中微分阻尼
                ctrl.joints[idx].output_limit = SG100_Quant::encodeOutLim(1.0f); // 全力限幅
            } else if (control_mode == 2) {
                // 阻抗控制模式
                ctrl.joints[idx].mode = MODE_IMPEDANCE;
                ctrl.joints[idx].imp_position = target_deg;
                ctrl.joints[idx].imp_velocity = SG100_Quant::encodeVel(target_vel_deg_s);
                ctrl.joints[idx].imp_kp = SG100_Quant::encodePid(kp);
                ctrl.joints[idx].imp_kd = SG100_Quant::encodePid(kd);
                ctrl.joints[idx].imp_torque_ff = SG100_Quant::encodeOutLim(0.0f);
                ctrl.joints[idx].imp_output_limit = SG100_Quant::encodeOutLim(1.0f);
            } else {
                // 关节空间阻抗控制模式
                ctrl.joints[idx].mode = MODE_JOINT_IMPEDANCE;
                ctrl.joints[idx].imp_position = target_deg;
                ctrl.joints[idx].imp_velocity = SG100_Quant::encodeVel(target_vel_deg_s);
                ctrl.joints[idx].imp_kp = SG100_Quant::encodePid(kp);
                ctrl.joints[idx].imp_kd = SG100_Quant::encodePid(kd);
                ctrl.joints[idx].imp_torque_ff = SG100_Quant::encodeOutLim(0.0f);
                ctrl.joints[idx].imp_output_limit = SG100_Quant::encodeOutLim(1.0f);
            }

            // 发送控制帧
            hand->SetHandControl(ctrl);

            // 5Hz 频率打印反馈数据到控制台 (每10个发送周期打印一次, 周期=20ms)
            if (frame_idx % 10 == 0) {
                Hand_Status_t status_snap;
                {
                    std::lock_guard<std::mutex> lock(g_status_mutex);
                    status_snap = g_hand_status;
                }
                const auto& feedback = status_snap.joints[idx];
                float actual_rad = feedback.position;
                float actual_deg = actual_rad * 180.0f / static_cast<float>(M_PI);
                float actual_vel = SG100_Quant::decodeVel(feedback.velocity);
                float actual_cur = SG100_Quant::decodeCurQ(feedback.cur_q);

                std::cout << "T: " << std::fixed << std::setprecision(2) << elapsed << "s"
                          << " | 目标: " << std::setprecision(3) << std::setw(6) << target_rad << " rad (" << std::setw(5) << (int)target_deg << " deg)"
                          << " | 反馈: " << std::setprecision(3) << std::setw(6) << actual_rad << " rad (" << std::setw(5) << (int)actual_deg << " deg)"
                          << " | 目标速度: " << std::setw(5) << (int)target_vel_deg_s << " dps"
                          << " | 实际速度: " << std::setw(5) << (int)actual_vel << " dps"
                          << " | 电流: " << std::setprecision(2) << std::setw(5) << actual_cur << " A"
                          << std::endl;
            }

            frame_idx++;
            std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50Hz 发送率
        }

        // 7. 单次正弦运动结束后软化/停止该关节
        std::cout << "[INFO] 正弦运动执行完毕。平稳软化/卸载该关节...\n";
        Hand_Control_t stop_ctrl;
        std::memset(&stop_ctrl, 0, sizeof(stop_ctrl));
        for (int i = 0; i < JOINT_COUNT; i++) {
            stop_ctrl.joints[i].id = static_cast<uint8_t>(i + 1);
            stop_ctrl.joints[i].mode = MODE_NULL; // 模式 NULL 表示停机/释放
            stop_ctrl.joints[i].position = 0.0f;
        }
        hand->SetHandControl(stop_ctrl);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        std::cout << "[OK] 关节已卸力。\n";
    }

    // 8. 退出并清理
    std::cout << "\n[INFO] 正在关闭测试程序...\n";
    manager.StopAll();
    manager.Clear();
    std::cout << "[OK] 成功关闭，程序退出。\n";

    return 0;
}

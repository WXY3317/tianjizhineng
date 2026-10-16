/**
 * @file hand_control_example.cpp
 * @brief SG100 SDK 整手控制高级示例
 * 
 * 演示:
 * 1. 使用Hand_Control_t进行精细控制 (带PID参数)
 * 2. 单关节控制
 * 3. 状态回调
 * 4. 逐指控制
 */

#include "sg100_device_manager.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstring>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 角度映射配置目录（留空表示使用默认搜索路径）
// 左右手可分别使用不同目录，例如: "./config/left" / "./config/right"
const std::string JOINT_MAP_CONFIG_DIR = "";

// 状态更新回调
void OnHandStatusUpdate(const Hand_Status_t& status) {
    // 每次收到掌板状态更新时触发
    // 打印 11 个关节的状态信息
    std::cout << "Hand status update:" << std::endl;
    for (int i = 0; i < JOINT_COUNT; i++) {
        const auto& js = status.joints[i];
        std::cout << "  J" << (i + 1)
                  << ": pos=" << (js.position * 180.0f / M_PI)
                  << "\u00b0 vel=" << SG100_Quant::decodeVel(js.velocity)
                  << " cur_q=" << SG100_Quant::decodeCurQ(js.cur_q)
                  << " cur_d=" << SG100_Quant::decodeCurD(js.cur_d)
                  << " mode=" << (int)js.mode
                  << std::endl;
    }
}

// 错误回调
void OnError(uint8_t joint_id, uint8_t error_code) {
    std::cerr << "ERROR: Joint " << (int)joint_id 
              << " error code=" << (int)error_code << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  SG100 Hand Control Example" << std::endl;
    std::cout << "========================================" << std::endl;

    auto& manager = SG100DeviceManager::Instance();

    // 通过配置文件添加设备
    if (!manager.AddDevice("HAND1", "serial_config.json", 1)) {
        // 如果配置文件不存在, 尝试直接参数添加
        auto ports = SG100DeviceManager::ListSerialPorts();
        if (ports.empty()) {
            std::cout << "No serial ports found!" << std::endl;
            return 1;
        }
        if (!manager.AddDeviceByParams("HAND1", ports[0].port, 1000000)) {
            std::cout << "Failed to add device" << std::endl;
            return 1;
        }
    }

    auto* hand = manager.GetDevice("HAND1");
    if (!hand) return 1;

    if (!JOINT_MAP_CONFIG_DIR.empty()) {
        hand->SetConfigSearchDir(JOINT_MAP_CONFIG_DIR);
        std::cout << "Use joint map config dir: " << hand->GetConfigSearchDir() << std::endl;
    }

    // 注册回调
    hand->RegisterHandStatusCallback(OnHandStatusUpdate);
    hand->RegisterErrorCallback(OnError);

    // 启动
    if (!manager.StartAll()) {
        manager.Clear();
        return 1;
    }

    std::cout << "\n--- Demo 1: Full hand control with Hand_Control_t ---" << std::endl;
    {
        Hand_Control_t ctrl;
        std::memset(&ctrl, 0, sizeof(ctrl));

        // 设置所有11个关节为位置模式, 目标0度
        for (int i = 0; i < JOINT_COUNT; i++) {
            ctrl.joints[i].id = i + 1;
            ctrl.joints[i].mode = MODE_POSITION;
            ctrl.joints[i].position = 0.0f;
            ctrl.joints[i].p = SG100_Quant::encodePid(0.5f);
            ctrl.joints[i].i = SG100_Quant::encodePid(0.0f);
            ctrl.joints[i].d = SG100_Quant::encodePid(0.05f);
            ctrl.joints[i].output_limit = SG100_Quant::encodeOutLim(1.0f);
        }

        hand->SetHandControl(ctrl);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // 正弦波运动
        std::cout << "Sine wave motion for 5 seconds..." << std::endl;
        auto start = std::chrono::steady_clock::now();
        while (true) {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - start).count();
            if (elapsed > 5.0) break;

            float angle = 10.0f * static_cast<float>(std::sin(2.0 * M_PI * 0.5 * elapsed));

            for (int i = 0; i < JOINT_COUNT; i++) {
                ctrl.joints[i].position = angle * (float)M_PI / 180.0f;
            }
            hand->SetHandControl(ctrl);

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    std::cout << "\n--- Demo 2: Per-finger control ---" << std::endl;
    {
        using namespace SG100_FingerMap;

        // 逐个手指弯曲
        const char* finger_names[] = {"Thumb", "Index", "Middle", "Ring"};

        for (int f = 0; f < FINGER_COUNT; f++) {
            std::cout << "  Bending " << finger_names[f] << "..." << std::endl;

            for (int j = 0; j < FINGER_JOINT_COUNT[f]; j++) {
                uint8_t jid = FINGER_JOINTS[f][j];
                if (jid > 0) {
                    hand->SetJointPosition(jid, 5.0f);
                }
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // 全部回零
        std::cout << "  All fingers returning to zero..." << std::endl;
        float zeros[JOINT_COUNT] = {0};
        hand->SetJointPositions(zeros, JOINT_COUNT);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    std::cout << "\n--- Demo 3: Read status ---" << std::endl;
    {
       // hand->RequestHandStatus();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        auto status = hand->GetCachedHandStatus();
        {
            std::cout << "  Joint Status:" << std::endl;
            for (int i = 0; i < JOINT_COUNT; i++) {
                const auto& js = status.joints[i];
                std::cout << "    J" << (i + 1) 
                          << ": pos=" << (js.position * 180.0f / M_PI)
                          << "\u00b0 vel=" << SG100_Quant::decodeVel(js.velocity)
                          << " cur_q=" << SG100_Quant::decodeCurQ(js.cur_q)
                          << " mode=" << (int)js.mode
                          << std::endl;
            }
        }
    }

    // 清理
    std::cout << "\nStopping..." << std::endl;
    manager.StopAll();
    manager.Clear();

    std::cout << "Done!" << std::endl;
    return 0;
}

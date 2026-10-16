/**
 * @file ota_upgrade_example.cpp
 * @brief SG100 SDK OTA 固件升级示例 (支持关节板和掌板)
 *
 * 演示:
 * 1. 从命令行输入目标 ID (0=掌板, 1~JOINT_COUNT=关节板, 当前 JOINT_COUNT=11, 4根手指)
 * 2. 从命令行输入 BIN 固件文件路径
 * 3. 将原始 BIN 固件打包为 PKG 文件
 * 4. 执行一键 OTA 升级 (OtaUpgradePalm / OtaUpgradeJoint)
 * 5. 升级完成后查询固件版本验证
 *
 * 升级流程 (SDK 内部自动执行):
 *   CMD_REALTIME_PAUSE(0x24→掌板) → 跳转BL(0xBB) → 等待重启 → 查询状态(0x86)
 *   → 握手(0x7F) → 启动下载(0x82) → 发送分片(0x83, 128B/片)
 *   → 结束下载(0x84) → 等待重启 → CMD_REALTIME_RESUME(0x25→掌板)
 *
 * 注意: 0x24/0x25 在关节板协议中另有用途 (目标地址不同, 互不干扰):
 *   CMD_KTH78_SET_CURRENT_ZERO = 0x24 (关节板 KTH78 电流零点标定)
 *   CMD_SET_JOINT_ZERO         = 0x25 (关节板 关节机械零点设置)
 */

#include "sg100_device_manager.h"
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <cstring>
#ifdef _WIN32
#include <windows.h>
#endif

// ============================================================================
// 用户配置区 - 请根据实际情况修改以下参数
// ============================================================================
const std::string SERIAL_PORT = "";   // 串口名称, 留空""则从配置文件或自动探测
const std::string JOINT_MAP_CONFIG_DIR = ""; // 角度映射配置目录，左右手可设置不同路径

static const char* OtaStageName(OTA_ProgressStage_e stage) {
    switch (stage) {
    case OTA_STAGE_PREPARING:                 return "preparing";
    case OTA_STAGE_SWITCHING_TO_PASSTHROUGH: return "switching_to_passthrough";
    case OTA_STAGE_PAUSING_MOTOR:             return "pausing_motor";
    case OTA_STAGE_ENTERING_BOOTLOADER:       return "entering_bootloader";
    case OTA_STAGE_WAITING_FOR_BOOTLOADER:    return "waiting_for_bootloader";
    case OTA_STAGE_HANDSHAKING:               return "handshaking";
    case OTA_STAGE_ERASING_FLASH:             return "erasing_flash";
    case OTA_STAGE_TRANSFERRING:              return "transferring";
    case OTA_STAGE_VERIFYING:                 return "verifying";
    case OTA_STAGE_REBOOTING:                 return "rebooting";
    case OTA_STAGE_RECONNECTING:              return "reconnecting";
    case OTA_STAGE_RESTORING_REALTIME:        return "restoring_realtime";
    case OTA_STAGE_COMPLETED:                 return "completed";
    case OTA_STAGE_FAILED:                    return "failed";
    default:                                  return "unknown";
    }
}

// ============================================================================
// main
// ============================================================================
int main() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif
    std::cout << "========================================" << std::endl;
    std::cout << "  SG100 OTA Upgrade Example" << std::endl;
    std::cout << "  (Joint Board & Palm Board)" << std::endl;
    std::cout << "========================================" << std::endl;

    // ------- 从命令行输入目标ID -------
    int target_id = -1;
    std::cout << "Please enter target ID:" << std::endl;
    std::cout << "  0       = Palm Board  (掌板)" << std::endl;
    std::cout << "  1 ~ " << JOINT_COUNT << "  = Joint Board (关节板)" << std::endl;
    std::cout << "ID: ";
    std::cin >> target_id;

    bool is_palm = (target_id == 0);
    if (!is_palm && (target_id < 1 || target_id > JOINT_COUNT)) {
        std::cerr << "[ERROR] Invalid ID: " << target_id
                  << ". Must be 0 (palm) or 1-" << JOINT_COUNT << " (joint)." << std::endl;
        return 1;
    }

    if (is_palm) {
        std::cout << "Target: Palm Board (掌板)" << std::endl;
    } else {
        std::cout << "Target: Joint Board (关节板), ID = " << target_id << std::endl;
    }

    // ------- 从命令行输入BIN文件路径 -------
    std::string bin_filepath;
    std::cout << "Please enter BIN firmware file path: ";
    std::cin.ignore();
    std::getline(std::cin, bin_filepath);

    if (bin_filepath.empty()) {
        std::cerr << "[ERROR] BIN file path is empty." << std::endl;
        return 1;
    }

    // ------- 从命令行输入固件版本号 -------
    uint32_t firmware_version = 1;
    std::cout << "Please enter firmware version (hex like 0x00020001 or decimal, default=1): ";
    std::string ver_str;
    std::getline(std::cin, ver_str);
    if (!ver_str.empty()) {
        try {
            firmware_version = static_cast<uint32_t>(
                std::stoul(ver_str, nullptr, ver_str.find("0x") == 0 ? 16 : 10));
        } catch (...) {
            std::cout << "  Invalid version input, using default: 1" << std::endl;
            firmware_version = 1;
        }
    }
    std::cout << "  Firmware version: 0x" << std::hex << std::uppercase << firmware_version
              << std::dec << std::endl;

    std::string port_override = SERIAL_PORT;

    // 生成 .pkg 文件路径 (same dir, replace .bin → .pkg)
    std::string pkg_filepath = bin_filepath;
    {
        auto dot_pos = pkg_filepath.rfind('.');
        if (dot_pos != std::string::npos) {
            pkg_filepath = pkg_filepath.substr(0, dot_pos);
        }
        pkg_filepath += ".pkg";
    }

    // ------- Step 1: 打包固件 (BIN → PKG) -------
    std::cout << "\n[Step 1] Packing firmware..." << std::endl;
    std::cout << "  BIN: " << bin_filepath << std::endl;
    std::cout << "  PKG: " << pkg_filepath << std::endl;

    bool pack_ok = false;
    if (is_palm) {
        pack_ok = SG100Device::PackPalmFirmware(bin_filepath, pkg_filepath, firmware_version);
    } else {
        pack_ok = SG100Device::PackFirmware(bin_filepath, pkg_filepath, firmware_version);
    }
    if (!pack_ok) {
        std::cerr << "[ERROR] Failed to pack firmware: " << bin_filepath << std::endl;
        return 1;
    }
    std::cout << "  Pack OK." << std::endl;

    // ------- Step 2: 初始化设备 -------
    std::cout << "\n[Step 2] Initializing device..." << std::endl;

    auto& manager = SG100DeviceManager::Instance();

    bool added = false;
    if (!port_override.empty()) {
        // 使用指定的串口
        added = manager.AddDeviceByParams("HAND1", port_override, 1000000);
    } else {
        // 尝试从配置文件读取
        added = manager.AddDevice("HAND1", "serial_config.json", 1);
        if (!added) {
            // 枚举所有可用串口，让用户选择
            auto ports = SG100DeviceManager::ListSerialPorts();
            if (ports.empty()) {
                std::cerr << "[ERROR] No serial ports found!" << std::endl;
                return 1;
            }
            std::cout << "Available serial ports:" << std::endl;
            for (int i = 0; i < (int)ports.size(); ++i) {
                std::cout << "  [" << i << "] " << ports[i].port;
                if (!ports[i].description.empty()) {
                    std::cout << "  -- " << ports[i].description;
                }
                std::cout << std::endl;
            }
            int port_idx = 0;
            std::cout << "Select port index [0-" << (int)ports.size() - 1 << "] (baud 1000000): ";
            std::string idx_str;
            std::getline(std::cin, idx_str);
            if (!idx_str.empty()) {
                try { port_idx = std::stoi(idx_str); } catch (...) { port_idx = 0; }
            }
            if (port_idx < 0 || port_idx >= (int)ports.size()) {
                std::cerr << "[ERROR] Invalid port index: " << port_idx << std::endl;
                return 1;
            }
            std::cout << "  Selected: " << ports[port_idx].port << " @ 1000000" << std::endl;
            added = manager.AddDeviceByParams("HAND1", ports[port_idx].port, 1000000);
        }
    }

    if (!added) {
        std::cerr << "[ERROR] Failed to add device." << std::endl;
        return 1;
    }

    auto* hand = manager.GetDevice("HAND1");
    if (!hand) {
        std::cerr << "[ERROR] Device not found." << std::endl;
        manager.Clear();
        return 1;
    }

    if (!JOINT_MAP_CONFIG_DIR.empty()) {
        hand->SetConfigSearchDir(JOINT_MAP_CONFIG_DIR);
        std::cout << "  Joint map config dir: " << hand->GetConfigSearchDir() << std::endl;
    }

    // 注册错误回调 (方便观察升级过程中的异常)
    hand->RegisterErrorCallback([](uint8_t jid, uint8_t err) {
        std::cerr << "  [CB] Joint " << (int)jid 
                  << " error: 0x" << std::hex << (int)err << std::dec << std::endl;
    });

    if (!manager.StartAll()) {
        std::cerr << "[ERROR] Failed to start device." << std::endl;
        manager.Clear();
        return 1;
    }

    // 等待设备通信稳定
    std::cout << "  Device started, waiting for communication to stabilize..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // ------- Step 3: 执行 OTA 升级 -------
    if (is_palm) {
        std::cout << "\n[Step 3] Starting OTA upgrade for Palm Board (掌板) ..." << std::endl;
    } else {
        std::cout << "\n[Step 3] Starting OTA upgrade for Joint " << target_id << " ..." << std::endl;
    }
    std::cout << "  PKG file: " << pkg_filepath << std::endl;
    std::cout << "  This may take a while, please do not disconnect the device.\n" << std::endl;

    auto t_start = std::chrono::steady_clock::now();

    OTA_ProgressStage_e last_stage = OTA_STAGE_FAILED;
    OTAProgressCallback progress_callback = [&](const OTA_Progress_t& progress) {
        const bool stage_changed = progress.stage != last_stage;
        const bool chunk_checkpoint = progress.stage == OTA_STAGE_TRANSFERRING &&
            (progress.current_chunk % 10 == 0 ||
             progress.current_chunk == progress.total_chunks);
        if (stage_changed || chunk_checkpoint) {
            std::cout << "  [PROGRESS] " << OtaStageName(progress.stage)
                      << " " << progress.percent << "%"
                      << " (" << progress.bytes_sent << "/"
                      << progress.total_bytes << " bytes)" << std::endl;
        }
        last_stage = progress.stage;
    };

    bool success = false;
    if (is_palm) {
        success = hand->OtaUpgradePalm(pkg_filepath, progress_callback);
    } else {
        success = hand->OtaUpgradeJoint(static_cast<uint8_t>(target_id), pkg_filepath,
                                        progress_callback);
    }

    auto t_end = std::chrono::steady_clock::now();
    double elapsed_s = std::chrono::duration<double>(t_end - t_start).count();

    if (success) {
        std::cout << "\n[RESULT] OTA upgrade SUCCEEDED!" << std::endl;
        if (is_palm) {
            std::cout << "  Target: Palm Board" << std::endl;
        } else {
            std::cout << "  Joint : " << target_id << std::endl;
        }
        std::cout << "  Time  : " << elapsed_s << " s" << std::endl;
    } else {
        std::cerr << "\n[RESULT] OTA upgrade FAILED!" << std::endl;
        if (is_palm) {
            std::cerr << "  Target: Palm Board" << std::endl;
        } else {
            std::cerr << "  Joint : " << target_id << std::endl;
        }
        std::cerr << "  Time  : " << elapsed_s << " s" << std::endl;
        std::cerr << "  Please check the connection and firmware file, then retry." << std::endl;
    }

    // ------- Step 4: 验证 (可选) -------
    std::cout << "\n[Step 4] Post-upgrade verification..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if (!is_palm) {
        // 查询新固件版本
        if (hand->GetFwInfo(static_cast<uint8_t>(target_id))) {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            FirmwareVersionInfo_t fw_info{};
            if (hand->GetJointFwInfo(static_cast<uint8_t>(target_id), fw_info)) {
                char ver[33] = {0};
                std::memcpy(ver, fw_info.version_string, 32);
                std::cout << "  Joint " << target_id << " firmware: "
                          << ver
                          << " (v" << (int)fw_info.major << "."
                          << (int)fw_info.minor << "."
                          << (int)fw_info.patch << "+" << fw_info.build << ")"
                          << std::endl;
            } else {
                std::cout << "  Joint " << target_id << " firmware info not yet available." << std::endl;
            }
        }
        // 关节运行状态
        auto status = hand->GetCachedHandStatus();
        const auto& js = status.joints[target_id - 1];
        std::cout << "  Joint " << target_id << " status after upgrade:" << std::endl;
        std::cout << "    Position: " << js.position << " (raw float)" << std::endl;
        std::cout << "    Velocity: " << SG100_Quant::decodeVel(js.velocity) << " deg/s" << std::endl;
        std::cout << "    Cur_Q   : " << SG100_Quant::decodeCurQ(js.cur_q) << " A" << std::endl;
        std::cout << "    Mode    : " << (int)js.mode << std::endl;
    }

    // ------- 清理 -------
    std::cout << "\nStopping..." << std::endl;
    manager.StopAll();
    manager.Clear();

    std::cout << "Done!" << std::endl;
    return success ? 0 : 1;
}

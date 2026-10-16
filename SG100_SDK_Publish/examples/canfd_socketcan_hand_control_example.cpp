#include "sg100_device_manager.h"
#include "json.hpp"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#ifdef __linux__
#include <arpa/inet.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#ifndef __linux__
int main() {
    std::cerr << "This example only supports Linux with SocketCAN" << std::endl;
    return 1;
}
#else

namespace {

struct AppConfig {
    std::string interface_name = "can0";
    uint32_t tx_start_id = 0x200;
    uint32_t rx_filter_min_id = 0x200;
    uint32_t rx_filter_max_id = 0x2FF;
    bool use_extended_id = false;
    int recv_timeout_ms = 20;
    int tx_poll_interval_ms = 2;

    float kp = 0.1f;
    float ki = 0.0f;
    float kd = 0.0f;
    float output_limit = 0.5f;
    float default_position_rad = 0.0f;
};

std::atomic<bool> g_running(true);

void SignalHandler(int) {
    g_running.store(false);
}

std::string ResolveConfigPath(const char* argv0, const std::string& user_path) {
    // 用户显式传入的路径直接使用
    if (!user_path.empty()) {
        std::ifstream test(user_path);
        if (test.is_open()) return user_path;
        std::cerr << "Config not found at: " << user_path << std::endl;
    }

    // 尝试可执行文件所在目录的兄弟 examples/ 目录
    if (argv0 && argv0[0] != '\0') {
        std::string exe_dir(argv0);
        exe_dir = exe_dir.substr(0, exe_dir.find_last_of("/\\"));
        if (!exe_dir.empty()) {
            std::vector<std::string> candidates = {
                exe_dir + "/../examples/canfd_socketcan_config.json",
                exe_dir + "/../../examples/canfd_socketcan_config.json",
                exe_dir + "/../../../examples/canfd_socketcan_config.json",
            };
            for (const auto& p : candidates) {
                std::ifstream test(p);
                if (test.is_open()) return p;
            }
        }
    }

    // 兜底: 尝试当前工作目录
    std::ifstream test("examples/canfd_socketcan_config.json");
    if (test.is_open()) return "examples/canfd_socketcan_config.json";

    return "";
}

bool LoadConfig(const std::string& path, AppConfig& cfg) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open config file: " << path << std::endl;
        return false;
    }

    nlohmann::json j;
    try {
        ifs >> j;
    } catch (const std::exception& e) {
        std::cerr << "Failed to parse JSON config: " << e.what() << std::endl;
        return false;
    }

    const auto& can = j.value("can", nlohmann::json::object());
    const auto& control = j.value("control", nlohmann::json::object());

    cfg.interface_name = can.value("interface", cfg.interface_name);
    cfg.tx_start_id = can.value("tx_start_id", can.value("tx_can_id", cfg.tx_start_id));
    cfg.rx_filter_min_id = can.value("rx_filter_min_id", can.value("rx_can_id", cfg.rx_filter_min_id));
    cfg.rx_filter_max_id = can.value("rx_filter_max_id", cfg.rx_filter_max_id);
    cfg.use_extended_id = can.value("use_extended_id", cfg.use_extended_id);
    cfg.recv_timeout_ms = can.value("recv_timeout_ms", cfg.recv_timeout_ms);
    cfg.tx_poll_interval_ms = can.value("tx_poll_interval_ms", cfg.tx_poll_interval_ms);

    cfg.kp = control.value("kp", cfg.kp);
    cfg.ki = control.value("ki", cfg.ki);
    cfg.kd = control.value("kd", cfg.kd);
    cfg.output_limit = control.value("output_limit", cfg.output_limit);
    cfg.default_position_rad = control.value("default_position_rad", cfg.default_position_rad);

    return true;
}

int OpenSocketCan(const AppConfig& cfg) {
    int fd = socket(AF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        std::perror("socket(AF_CAN)");
        return -1;
    }

    const int enable_canfd = 1;
    if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0) {
        std::perror("setsockopt(CAN_RAW_FD_FRAMES)");
        close(fd);
        return -1;
    }

    // RX 过滤统一由 SDK 的 min/max 配置执行，SocketCAN 层不过滤。

    struct timeval tv {};
    tv.tv_sec = cfg.recv_timeout_ms / 1000;
    tv.tv_usec = (cfg.recv_timeout_ms % 1000) * 1000;
    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        std::perror("setsockopt(SO_RCVTIMEO)");
        close(fd);
        return -1;
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, cfg.interface_name.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        std::perror("ioctl(SIOCGIFINDEX)");
        close(fd);
        return -1;
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::perror("bind(AF_CAN)");
        close(fd);
        return -1;
    }

    return fd;
}

void PrintStatus(const Hand_Status_t& status) {
    std::cout << "Joint status:" << std::endl;
    for (int i = 0; i < JOINT_COUNT; ++i) {
        const auto& js = status.joints[i];
        std::cout << "  J" << (i + 1)
                  << " mode=" << static_cast<int>(js.mode)
                  << " pos_rad=" << js.position
                  << " vel_deg_s=" << SG100_Quant::decodeVel(js.velocity)
                  << " cur_q=" << SG100_Quant::decodeCurQ(js.cur_q)
                  << " cur_d=" << SG100_Quant::decodeCurD(js.cur_d)
                  << std::endl;
    }
}

bool ParseIdPosition(const std::string& line, int& id, float& position) {
    std::istringstream iss(line);
    if (!(iss >> id >> position)) {
        return false;
    }
    char c = '\0';
    if (iss >> c) {
        return false;
    }
    return true;
}

} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    std::string config_path = ResolveConfigPath(argv[0],
        (argc >= 2) ? std::string(argv[1]) : std::string());
    if (config_path.empty()) {
        std::cerr << "Cannot find canfd_socketcan_config.json" << std::endl;
        return 1;
    }

    AppConfig cfg;
    if (!LoadConfig(config_path, cfg)) {
        return 1;
    }

    int can_fd = OpenSocketCan(cfg);
    if (can_fd < 0) {
        return 1;
    }

    auto& mgr = SG100DeviceManager::Instance();
    if (!mgr.AddCANFDDevice("HAND_CANFD")) {
        std::cerr << "AddCANFDDevice failed" << std::endl;
        close(can_fd);
        return 1;
    }

    SG100Device* dev = mgr.GetDevice("HAND_CANFD");
    if (dev == nullptr) {
        std::cerr << "GetDevice failed" << std::endl;
        mgr.Clear();
        close(can_fd);
        return 1;
    }

    CANFD_Config_t canfd_cfg{};
    canfd_cfg.tx_start_id = cfg.tx_start_id;
    canfd_cfg.rx_filter_min_id = cfg.rx_filter_min_id;
    canfd_cfg.rx_filter_max_id = cfg.rx_filter_max_id;
    canfd_cfg.use_extended_id = cfg.use_extended_id ? 1U : 0U;
    canfd_cfg.enable_sequential_tx_id = 1U;
    canfd_cfg.enable_rx_filter = (cfg.rx_filter_min_id <= cfg.rx_filter_max_id) ? 1U : 0U;
    if (!dev->SetCANFDConfig(canfd_cfg)) {
        std::cerr << "SetCANFDConfig failed" << std::endl;
        mgr.Clear();
        close(can_fd);
        return 1;
    }

    if (!mgr.StartAll()) {
        std::cerr << "StartAll failed" << std::endl;
        mgr.Clear();
        close(can_fd);
        return 1;
    }

    Hand_Control_t ctrl;
    std::memset(&ctrl, 0, sizeof(ctrl));

    const uint16_t kp_q = SG100_Quant::encodePid(cfg.kp);
    const uint16_t ki_q = SG100_Quant::encodePid(cfg.ki);
    const uint16_t kd_q = SG100_Quant::encodePid(cfg.kd);
    const uint16_t out_q = SG100_Quant::encodeOutLim(cfg.output_limit);

    for (int i = 0; i < JOINT_COUNT; ++i) {
        ctrl.joints[i].id = static_cast<uint8_t>(i + 1);
        ctrl.joints[i].mode = MODE_JOINT_POSITION;
        ctrl.joints[i].position = cfg.default_position_rad;
        ctrl.joints[i].p = kp_q;
        ctrl.joints[i].i = ki_q;
        ctrl.joints[i].d = kd_q;
        ctrl.joints[i].output_limit = out_q;
    }

    std::thread tx_thread([&]() {
        CANFD_Frame_t pending_frame;
        bool has_pending = false;

        while (g_running.load()) {
            // 1) 优先重试上次发送失败的帧 (仅 ENOBUFS 可恢复)
            if (has_pending) {
                struct canfd_frame cfd {};
                cfd.len    = pending_frame.data_len;
                cfd.flags |= CANFD_BRS;
                cfd.can_id = cfg.use_extended_id
                    ? ((pending_frame.can_id & CAN_EFF_MASK) | CAN_EFF_FLAG)
                    : (pending_frame.can_id & CAN_SFF_MASK);
                std::memcpy(cfd.data, pending_frame.data, pending_frame.data_len);

                ssize_t n = write(can_fd, &cfd, sizeof(cfd));
                if (n >= 0) {
                    has_pending = false;
                } else if (errno == ENOBUFS) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                } else {
                    std::perror("write(canfd_frame)");
                    has_pending = false;
                }
            }

            // 2) 从 SDK 获取新的待发帧
            bool has_frame = false;
            while (dev->CANFDGetTxFrame(pending_frame)) {
                has_frame = true;
                struct canfd_frame cfd {};
                cfd.len    = pending_frame.data_len;
                cfd.flags |= CANFD_BRS;
                cfd.can_id = cfg.use_extended_id
                    ? ((pending_frame.can_id & CAN_EFF_MASK) | CAN_EFF_FLAG)
                    : (pending_frame.can_id & CAN_SFF_MASK);
                std::memcpy(cfd.data, pending_frame.data, pending_frame.data_len);

                ssize_t n = write(can_fd, &cfd, sizeof(cfd));
                if (n >= 0) {
                    has_pending = false;
                } else if (errno == ENOBUFS) {
                    has_pending = true;
                    break;
                } else {
                    std::perror("write(canfd_frame)");
                    has_pending = false;
                }
            }

            if (!has_frame && !has_pending) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(cfg.tx_poll_interval_ms));
            }
        }
    });

    std::thread rx_thread([&]() {
        while (g_running.load()) {
            struct canfd_frame cfd {};
            ssize_t n = read(can_fd, &cfd, sizeof(cfd));
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                    continue;
                }
                std::perror("read(canfd_frame)");
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }
            if (n < static_cast<ssize_t>(CAN_MTU)) {
                continue;
            }
            dev->CANFDProcessRxFrame(cfd.can_id, cfd.data, cfd.len);
        }
    });

    std::cout << "SocketCAN example started on " << cfg.interface_name << std::endl;
    std::cout << "Input format: id position_rad  (example: 3 0.25)" << std::endl;
    std::cout << "Type 'getstatus' to request and print all joint status" << std::endl;
    std::cout << "Type 'quit' to exit" << std::endl;

    while (g_running.load()) {
        std::cout << "> " << std::flush;
        std::string line;
        if (!std::getline(std::cin, line)) {
            g_running.store(false);
            break;
        }

        if (line.empty()) {
            continue;
        }

        if (line == "quit" || line == "exit") {
            g_running.store(false);
            break;
        }

        if (line == "getstatus") {
            if (!dev->RequestHandStatus()) {
                std::cerr << "RequestHandStatus failed" << std::endl;
                continue;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(80));
            PrintStatus(dev->GetCachedHandStatus());
            continue;
        }

        int id = 0;
        float position_rad = 0.0f;
        if (!ParseIdPosition(line, id, position_rad)) {
            std::cout << "Invalid input. Use: id position_rad, getstatus, quit" << std::endl;
            continue;
        }

        if (id < 1 || id > JOINT_COUNT) {
            std::cout << "Invalid id. Valid range is 1-" << JOINT_COUNT << std::endl;
            continue;
        }

        ctrl.joints[id - 1].position = position_rad;

        if (!dev->SetHandControl(ctrl)) {
            std::cerr << "SetHandControl failed" << std::endl;
            continue;
        }

        std::cout << "Sent MODE_JOINT_POSITION: id=" << id
                  << " position_rad=" << position_rad
                  << " kp=" << cfg.kp
                  << " ki=" << cfg.ki
                  << " kd=" << cfg.kd
                  << " output_limit=" << cfg.output_limit
                  << std::endl;
    }

    g_running.store(false);

    if (tx_thread.joinable()) {
        tx_thread.join();
    }
    if (rx_thread.joinable()) {
        rx_thread.join();
    }

    mgr.StopAll();
    mgr.Clear();

    close(can_fd);
    return 0;
}

#endif

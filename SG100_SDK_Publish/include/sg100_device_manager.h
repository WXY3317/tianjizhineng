/**
 * @file sg100_device_manager.h
 * @brief SG100设备管理器 - 单例模式，统一管理多个SG100设备
 */

#ifndef SG100_DEVICE_MANAGER_H
#define SG100_DEVICE_MANAGER_H

#include "sg100_device.h"
#include <map>
#include <memory>
#include <mutex>
#include <vector>
#include <string>

#ifdef _WIN32
    #ifdef SG100_SDK_EXPORTS
        #define SG100_SDK_API __declspec(dllexport)
    #else
        #define SG100_SDK_API __declspec(dllimport)
    #endif
#else
    #define SG100_SDK_API
#endif

/**
 * @brief 串口设备信息
 */
struct SG100_SDK_API SerialPortInfo {
    std::string port;           // 串口名称 (如 "COM1", "/dev/ttyUSB0")
    std::string description;    // 设备描述
    std::string hardware_id;    // 硬件ID (如 USB VID:PID)
};

/**
 * @brief SG100设备管理器 - 单例模式
 * 
 * 统一管理多个SG100灵巧手设备(左手/右手等)，提供:
 * - 设备创建和销毁
 * - 设备查找
 * - 批量操作 (启动/停止所有设备)
 * - 串口枚举
 * 
 * 使用示例:
 * @code
 *   auto& mgr = SG100DeviceManager::Instance();
 *   mgr.AddDeviceByParams("LEFT_HAND", "COM3", 1000000);
 *   mgr.AddDeviceByParams("RIGHT_HAND", "COM4", 1000000);
 *   mgr.StartAll();
 *   
 *   auto* hand = mgr.GetDevice("LEFT_HAND");
 *   hand->SetJointPositions(positions, 11);
 *   
 *   mgr.StopAll();
 *   mgr.Clear();
 * @endcode
 */
class SG100_SDK_API SG100DeviceManager {
public:
    /**
     * @brief 获取管理器单例
     */
    static SG100DeviceManager& Instance();
    
    /**
     * @brief 列出系统中所有可用的串口设备
     * @return 串口设备信息列表
     */
    static std::vector<SerialPortInfo> ListSerialPorts();
    
    /**
     * @brief 通过JSON配置文件添加设备
     * @param name 设备名称 (唯一标识, 如 "LEFT_HAND")
     * @param filename JSON配置文件路径
     * @param flag 配置文件中的设备索引号 (1-based)
     * @return true-成功, false-失败
     */
    bool AddDevice(const std::string& name, const std::string& filename, uint8_t flag);
    
    /**
     * @brief 通过参数直接添加设备
     * @param name 设备名称 (唯一标识)
     * @param port 串口名称
    * @param baudrate 波特率 (默认1000000)
     * @return true-成功, false-失败
     */
    bool AddDeviceByParams(const std::string& name, 
                           const std::string& port,
                      unsigned long baudrate = 1000000);

    /**
     * @brief 添加 CANFD 模式设备
     *
     * 创建一个工作在 CANFD 模式的 SG100 设备实例。
     * CANFD 模式下无需串口, SDK 仅处理协议数据的封包/解包:
     *   - 发送: 通过 device->CANFDGetTxFrame() 获取待发 CANFD 帧
     *   - 接收: 通过 device->CANFDProcessRxFrame() 将收到的帧注入 SDK
     *
     * @param name 设备名称 (唯一标识, 如 "LEFT_HAND_CAN")
     * @return true-成功, false-失败
     */
    bool AddCANFDDevice(const std::string& name);
    
    /**
     * @brief 移除设备
     * @param name 设备名称
     * @return true-成功
     */
    bool RemoveDevice(const std::string& name);
    
    /**
     * @brief 获取设备指针
     * @param name 设备名称
     * @return 设备指针, 不存在返回nullptr
     */
    SG100Device* GetDevice(const std::string& name);
    
    /**
     * @brief 检查设备是否存在
     */
    bool HasDevice(const std::string& name) const;
    
    /**
     * @brief 启动所有设备
     * @return true-全部成功
     */
    bool StartAll();
    
    /**
     * @brief 停止所有设备
     */
    void StopAll();
    
    /**
     * @brief 获取设备数量
     */
    size_t GetDeviceCount() const;
    
    /**
     * @brief 获取所有设备名称
     */
    std::vector<std::string> GetDeviceNames() const;
    
    /**
     * @brief 清除所有设备
     */
    void Clear();

private:
    SG100DeviceManager() = default;
    ~SG100DeviceManager();
    
    SG100DeviceManager(const SG100DeviceManager&) = delete;
    SG100DeviceManager& operator=(const SG100DeviceManager&) = delete;
    
    std::map<std::string, std::unique_ptr<SG100Device>> devices_;
    mutable std::mutex mutex_;
};

#endif // SG100_DEVICE_MANAGER_H

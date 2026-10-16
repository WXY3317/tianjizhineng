/**
 * @file sg100_device.h
 * @brief SG100设备类 - 管理与SG100灵巧手掌板的通信
 * 
 * 基于WA100 SDK架构，适配SG100协议:
 * - CRC-32/BZIP2 校验
 * - 地址寻址 (PC=0xFE → Palm=0xFD → Joint=0x01~0x0B)
 * - 量化浮点传输
 * - 整手/单关节控制
 */

#ifndef SG100_DEVICE_H
#define SG100_DEVICE_H

#include "sg100_types.h"
#include "sg100_angle_map.h"
#include <string>
#include <thread>
#include <memory>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <vector>

#ifdef _WIN32
    #ifdef SG100_SDK_EXPORTS
        #define SG100_SDK_API __declspec(dllexport)
    #else
        #define SG100_SDK_API __declspec(dllimport)
    #endif
#else
    #define SG100_SDK_API
#endif

// ============================================================================
// 设备配置结构
// ============================================================================
struct SG100_SDK_API SG100Config {
    std::string port;              // 串口名称 (如 "COM1", "/dev/ttyUSB0")
    unsigned long baudrate;        // 波特率 (默认1000000，与掌板通信)
    int databyte;                  // 数据位 (默认8)
    int parity;                    // 校验位 (0=none, 1=odd, 2=even)
    int stopbyte;                  // 停止位 (默认1)
    int flowcontrol;               // 流控制 (0=none, 1=software, 2=hardware)
    uint32_t timeout_ms;           // 超时时间(ms)
    
    SG100Config() : 
        port("COM1"),
        baudrate(1000000),
        databyte(8),
        parity(0),
        stopbyte(1),
        flowcontrol(0),
        timeout_ms(100) {}
};

// ============================================================================
// 回调类型
// ============================================================================

// HandStatusCallback 和 ErrorCallback 已在 sg100_types.h 中定义

/**
 * @brief 响应回调 (用于异步操作)
 * @param success 是否成功
 * @param data 响应数据
 * @param size 数据大小
 */
using ResponseCallback = std::function<void(bool success, const uint8_t* data, uint16_t size)>;


// ============================================================================
// SG100 设备类
// ============================================================================

/**
 * @brief SG100灵巧手设备类
 * 
 * 每个实例管理:
 * - 一条串口连接 (PC ↔ Palm)
 * - 独立的通信线程
 * - 11个关节的状态缓存
 * - 命令队列与优先级调度
 * 
 * 通信架构:
 *   PC ←(UART)→ Palm Board ←(4x RS485)→ 11个关节
 */
class SG100_SDK_API SG100Device {
public:
    /**
     * @brief 构造函数
     * @param device_name 设备名称 (用于标识和日志)
     */
    explicit SG100Device(const std::string& device_name);
    
    /**
     * @brief 析构函数 - 自动停止并清理资源
     */
    ~SG100Device();
    
    // 禁止拷贝
    SG100Device(const SG100Device&) = delete;
    SG100Device& operator=(const SG100Device&) = delete;

    // ========================================================================
    // 设备生命周期
    // ========================================================================
    
    /**
     * @brief 从JSON配置文件读取串口参数
     * @param filename JSON配置文件路径
     * @param flag 串口配置数组索引 (1-based)
     * @return true-成功, false-失败
     */
    bool GetConfig(const std::string& filename, uint8_t flag);

    /**
     * @brief 直接设置串口配置
     * @param config 配置对象
     * @return true-成功
     */
    bool SetConfig(const SG100Config& config);

    /**
     * @brief 设置配置文件搜索目录
     *
     * 加载 joint_angle_map.json 等配置文件时，优先在此目录下查找。
     * 未设置（空字符串）时仅使用默认相对路径候选。
     * 必须在 Start() 前设置；运行中修改会被忽略，以避免控制映射发生跳变。
     * @param dir 目录路径，如 "/home/user/config" 或 "C:\\myapp\\config"
     */
    void SetConfigSearchDir(const std::string& dir);

    /**
     * @brief 获取当前配置文件搜索目录
     * @return 已设置的目录路径，未设置时返回空字符串
     */
    std::string GetConfigSearchDir() const;

    /**
     * @brief 设置新版 L/R 角度映射所使用的手型
     *
     * 必须在 Start() 前调用。运行中切换会被拒绝，避免控制目标和状态解码发生跳变。
     */
    bool SetAngleMapHand(SG100AngleMapHand hand);

    /** @brief 获取当前角度映射手型。 */
    SG100AngleMapHand GetAngleMapHand() const;

    /**
     * @brief 加载并校验角度映射配置
     * @return 配置有效或没有配置文件而使用内置兼容默认值时返回 true
     */
    bool ValidateAngleMap();

    /** @brief 获取最近一次角度映射加载状态和错误信息。 */
    SG100AngleMapStatus GetAngleMapStatus() const;

    /**
     * @brief 初始化设备 (打开串口)
     * @return true-成功
     */
    bool Init();
    
    /**
     * @brief 启动通信线程
     * @return true-成功
     */
    bool Start();
    
    /**
     * @brief 停止通信线程并关闭串口
     */
    void Stop();
    
    /**
     * @brief 检查设备是否正在运行
     */
    bool IsRunning() const;

    /**
     * @brief 检查串口是否打开
     * @return true-串口正常打开, false-串口异常
     */
    bool IsSerialOpen() const;

    // ========================================================================
    // 工作模式切换
    // ========================================================================
    
    /**
     * @brief 获取当前工作模式
     * @return 当前工作模式 (WORK_MODE_REALTIME 或 WORK_MODE_PASSTHROUGH)
     */
    SG100_WorkMode_e GetWorkMode() const;

    /**
     * @brief 切换到透传模式
     * 
     * 流程:
     * 1. 将实时调度队列中的指令全部发出 (至少保留实时控制指令)
     * 2. 发送 0x24 暂停掌板实时闭环任务
     * 3. 等待掌板确认响应 (CMD=0xA4, payload=[0x00])
     * 4. 切换标识符为透传模式
     * 
     * @return true-切换成功, false-切换失败
     */
    bool SwitchToPassthroughMode();


    /**
     * @brief 切换到实时调度模式
     * 
     * 流程:
     * 1. 将透传队列中的指令全部发出并清除
     * 2. 发送 0x25 恢复掌板实时闭环任务
     * 3. 等待掌板确认响应 (CMD=0xA5, payload=[0x00])
     * 4. 切换标识符为实时调度模式
     * 
     * @return true-切换成功, false-切换失败
     */
    bool SwitchToRealtimeMode();

    // ========================================================================
    // 透传模式操作 (需先调用 SwitchToPassthroughMode)
    // ========================================================================
    
    /**
     * @brief 透传模式下发送自定义命令
     * 
     * 将用户指定的命令帧放入透传队列，目的地址可以是掌板或关节板。
     * 
     * @param dst_addr 目标地址 (ADDR_PALM=0xFD 或 关节ID 1~14)
     * @param cmd 命令码
     * @param data 数据指针 (可为nullptr)
     * @param data_len 数据长度
     * @return true-命令已入队
     */
    bool SendPassthroughCommand(uint8_t dst_addr, uint8_t cmd, 
                                const uint8_t* data, uint16_t data_len);

    /** @brief 工厂工具直连设备发送，不依赖掌板透传模式。 */
    bool SendDirectCommand(uint8_t dst_addr, uint8_t cmd,
                           const uint8_t* data, uint16_t data_len);


    // ========================================================================
    // 整手控制 (PC ↔ Palm, 实时调度模式下使用)
    // ========================================================================
    
    /**
     * @brief 发送整手实时控制命令 (CMD_REALTIME_CTRL)
     * 
     * 将11个关节的控制数据打包发送到掌板，掌板返回整手状态。
     * 
     * @param ctrl 整手控制数据 (11×MotorDown_t = 154字节)
     * @return true-命令已入队
     */
    bool SetHandControl(const Hand_Control_t& ctrl);

    /**
     * @brief Enable or disable realtime TX/RX control logging.
     *
     * Logs are disabled by default. When enabled, logs are written to
     * sg100_control.log under the config search directory. If no config search
     * directory is set, logs are written to the console logger only.
     */
    void SetControlLogEnabled(bool enabled);

    /**
     * @brief Return whether realtime TX/RX control logging is enabled.
     */
    bool IsControlLogEnabled() const;

    /**
     * @brief 便捷整手位置控制
     * 
     * 以位置模式 (MODE_POSITION) 设置所有11个关节的角度。
     * 
     * @param positions_deg 11个关节的目标角度数组 (度, 范围±180)
     * @param count 数组元素数量 (应为JOINT_COUNT=11)
     * @return true-命令已入队
     */
    bool SetJointPositions(const float* positions_deg, uint8_t count);

    /**
     * @brief 便捷单关节位置控制 (通过整手帧)
     * 
     * 修改缓存中指定关节的目标位置，其余关节保持不变。
     * 
     * @param joint_id 关节ID (1-11)
     * @param position_deg 目标角度 (度)
     * @return true-成功
     */
    bool SetJointPosition(uint8_t joint_id, float position_deg);

    /**
     * @brief 便捷单关节阻抗控制 (通过整手帧)
     *
     * 以阻抗模式 (MODE_IMPEDANCE) 控制指定关节，其余关节保持原模式不变。
     * 内部自动完成角度→弧度转换、量化编码和缓存更新。
     *
     * @param joint_id       关节ID (1-11)
     * @param position_deg   目标角度 (度), SDK 内部将按极性映射并转为弧度下发
     * @param velocity_deg_s 目标角速度 (deg/s, 范围 ±3200)
     * @param kp             阻抗 Kp 增益 (0~10)
     * @param kd             阻抗 Kd 增益 (0~10)
     * @param torque_ff      力矩前馈系数 (0~10)
     * @param output_limit   输出限制系数 (0~10, 默认 1.0)
     * @return true-成功, false-参数非法或设备未运行
     */
    bool SetJointImpedance(uint8_t joint_id, float position_deg, float velocity_deg_s,
                           float kp, float kd, float torque_ff, float output_limit = 1.0f);

    /**
     * @brief 关节空间下的单关节阻抗控制 (基于关节物理反馈，不经过增量规划器)
     *
     * 以关节阻抗模式 (MODE_JOINT_IMPEDANCE) 控制指定关节，其余关节保持原模式不变。
     * 内部自动完成角度→弧度转换、量化编码和缓存更新。
     *
     * @param joint_id       关节ID (1-11)
     * @param position_deg   目标关节角度 (度), SDK 内部将按极性映射并转为弧度下发
     * @param velocity_deg_s 目标关节角速度 (deg/s, 范围 ±3200)
     * @param kp             刚度系数 Kp (0~10)
     * @param kd             阻尼系数 Kd (0~10)
     * @param torque_ff      力矩前馈系数 (0~10)
     * @param output_limit   输出限制系数 (0~10, 默认 1.0)
     * @return true-成功, false-参数非法或设备未运行
     */
    bool SetJointJointImpedance(uint8_t joint_id, float position_deg, float velocity_deg_s,
                                float kp, float kd, float torque_ff, float output_limit = 1.0f);

    /**
     * @brief 请求获取整手状态 (CMD_REALTIME_GET_STATUS)
     * @return true-命令已入队
     */
    bool RequestHandStatus();

    // ========================================================================
    // 掌板Flash参数操作 (需先切换到透传模式)
    // ========================================================================

    /**
     * @brief 读取掌板Flash参数 (CMD_FLASH_READ, 0x10)
     */
    bool ReadPalmFlash();

    /**
     * @brief 写入掌板Flash参数 (CMD_FLASH_WRITE, 0x11)
     * @param data 参数数据指针
     * @param size 数据大小 (字节)
     */
    bool WritePalmFlash(const uint8_t* data, uint16_t size);

    // ========================================================================
    // 掌板串口互联模式 (PC ↔ Palm, 可在实时模式下使用)
    // 功能: 将掌板上行口与指定手指UART直通, 实现原始帧透传调试
    // ========================================================================

    /**
     * @brief 开启串口互联模式 (CMD_UART_LINK_OPEN, 0x30)
     *
     * 掌板将上行串口 (PC口或ESP32口) 与指定手指UART直通。
     * 开启期间实时闭环自动暂停，关闭后自动恢复。
     *
     * @param peer          上行口对端类型 (UartLink_Peer_e)
     * @param target_uart_id 目标手指UART ID (1~5)
     * @return true-命令已入队
     */
    bool UartLinkOpen(uint8_t peer, uint8_t target_uart_id);

    /**
     * @brief 关闭串口互联模式 (CMD_UART_LINK_CLOSE, 0x31)
     * @return true-命令已入队
     */
    bool UartLinkClose();

    /**
     * @brief 查询串口互联模式状态 (CMD_UART_LINK_STATUS, 0x32)
     *
     * 响应载荷为 UartLink_Status_t (10字节)。
     * 可通过 RegisterHandStatusCallback 或 SendPassthroughCommand 读取原始响应。
     *
     * @return true-命令已入队
     */
    bool UartLinkGetStatus();

    /**
     * @brief 查询掌板当前运行模式 (CMD_RUNTIME_GET_MODE, 0x27)
     *
     * 响应载荷为 Palm_Runtime_Mode_Status_t (6字节)。
     * 查询结果会缓存，可通过 GetPalmRuntimeModeStatus() 读取。
     *
     * @return true-命令已入队
     */
    bool PalmRuntimeModeGetStatus();

    // ========================================================================
    // 单关节操作 (透传: PC → Palm → Joint, 需先切换到透传模式)
    // ========================================================================

    /**
     * @brief 获取单关节状态 (CMD_GET_STATUS, 0x02), 响应 MotorUp_t
     * @param joint_id 关节ID (1-11)
     */
    bool GetJointStatus(uint8_t joint_id);

    /**
     * @brief 设置单关节控制命令 (CMD_SET_COMMAND, 0x05), 载荷 MotorDown_t
     * @param joint_id 关节ID (1-11)
     * @param ctrl 控制帧
     */
    bool SetJointCommand(uint8_t joint_id, const MotorDown_t& ctrl);

    /**
     * @brief 关节归零 (CMD_HOMING_ZERO, 0x07)
     * @param joint_id 关节ID (1-11)
     */
    bool HomingZero(uint8_t joint_id);

    /**
    * @brief 按位掩码批量归零校准
    *
    * 支持类似 STM32 GPIO 的按位或写法，例如:
    * `SG100_JOINT_1 | SG100_JOINT_3 | SG100_JOINT_12`
    *
    * @param joint_mask 需要归零的关节掩码，可组合多个 `SG100_JOINT_x`
    * @param interval_ms 相邻关节指令下发间隔，默认 10000ms
    * @param manage_mode true=自动切换透传/实时模式并恢复原模式，false=要求调用前已在透传模式
    * @return true-所有选定关节命令均下发成功，false-存在失败或输入无效
    */
    bool HomingZeroMasked(SG100JointMask joint_mask,
                     uint32_t interval_ms = 10000,
                     bool manage_mode = true);

    /**
    * @brief 对指定关节执行归零校准 (CMD_HOMING_ZERO, 0x07)
    *
    * 完整流程:
    * 1. 若当前为实时模式，自动切换到透传模式（暂停掌板实时调度）
    * 2. 发送归零指令，等待固件完成归零（固件内部最长 12秒）
    * 3. 完成后自动切回原工作模式
    *
    * @param joint_id   关节ID (1-12)
    * @param timeout_ms 等待超时 (毫秒)，默认 15000（应大于固件超时 12s）
    * @return true-校准成功, false-校准失败或超时
    */
    bool CalibrateJointZero(uint8_t joint_id, uint32_t timeout_ms = 15000);

    /**
     * @brief 使能FOC (CMD_FOC_ENABLE, 0x1A)
     * @param joint_id 关节ID (1-11)
     */
    bool FocEnable(uint8_t joint_id);

    /**
     * @brief 禁用FOC (CMD_FOC_DISABLE, 0x1B)
     * @param joint_id 关节ID (1-11)
     */
    bool FocDisable(uint8_t joint_id);

    /**
     * @brief 获取运行时状态 (CMD_GET_RUNTIME_STATUS, 0x17), 响应 RuntimeStatusFrame_t
     * @param joint_id 关节ID (1-11)
     */
    bool GetRuntimeStatus(uint8_t joint_id);

    /**
     * @brief 获取原始数据快照 (CMD_GET_RAW_SNAPSHOT, 0x18), 响应 RawSnapshotFrame_t
     * @param joint_id 关节ID (1-11)
     */
    bool GetRawSnapshot(uint8_t joint_id);

    /**
     * @brief 获取传感器状态 (CMD_GET_SENSOR_STATUS, 0x19), 响应 SensorStatusFrame_t
     * @param joint_id 关节ID (1-11)
     */
    bool GetSensorStatus(uint8_t joint_id);

    /**
     * @brief 重新校准ADC偏移 (CMD_RECALIB_ADC_OFFSET, 0x1D)
     * @param joint_id 关节ID (1-11)
     */
    bool RecalibAdcOffset(uint8_t joint_id);

    /**
     * @brief 使零点失效 (CMD_INVALIDATE_ZERO, 0x1C)
     * @param joint_id 关节ID (1-11)
     */
    bool InvalidateZero(uint8_t joint_id);

    /**
     * @brief 配置遥测 (CMD_TELEM_CONFIG, 0x1E)
     * @param joint_id 关节ID (1-11)
     * @param stream_type 0=禁用 1=RuntimeStatus 2=RawSnapshot 3=SensorStatus
     * @param period_ms 发送周期 (ms)
     */
    bool TelemetryConfig(uint8_t joint_id, uint8_t stream_type, uint16_t period_ms);

    /**
     * @brief 启动遥测 (CMD_TELEM_START, 0x1F)
     * @param joint_id 关节ID (1-11)
     */
    bool TelemetryStart(uint8_t joint_id);

    /**
     * @brief 停止遥测 (CMD_TELEM_STOP, 0x20)
     * @param joint_id 关节ID (1-11)
     */
    bool TelemetryStop(uint8_t joint_id);

    // ========================================================================
    // 关节 Flash 分组参数操作 (透传, 需先PAUSE)
    // ========================================================================

    /** @brief 读取设备信息参数 (CMD_PARAM_READ_DEVICE, 0x10), 响应 ParamDeviceGroup_t */
    bool ReadParamDevice(uint8_t joint_id);
    /** @brief 写入设备信息参数 (CMD_PARAM_WRITE_DEVICE, 0x11) */
    bool WriteParamDevice(uint8_t joint_id, const ParamDeviceGroup_t& group);

    /** @brief 读取控制参数 (CMD_PARAM_READ_CONTROL, 0x12), 响应 ParamControlGroup_t */
    bool ReadParamControl(uint8_t joint_id);
    /** @brief 写入控制参数 (CMD_PARAM_WRITE_CONTROL, 0x13) */
    bool WriteParamControl(uint8_t joint_id, const ParamControlGroup_t& group);

    /** @brief 读取标定参数 (CMD_PARAM_READ_CALIB, 0x14), 响应 ParamCalibrationGroup_t */
    bool ReadParamCalib(uint8_t joint_id);
    /** @brief 写入标定参数 (CMD_PARAM_WRITE_CALIB, 0x15) */
    bool WriteParamCalib(uint8_t joint_id, const ParamCalibrationGroup_t& group);

    /** @brief 读取高级控制参数 (CMD_PARAM_READ_ADV_CONTROL, 0x21), 响应 ParamAdvancedControlGroup_t */
    bool ReadParamAdvControl(uint8_t joint_id);
    /** @brief 写入高级控制参数 (CMD_PARAM_WRITE_ADV_CONTROL, 0x22) */
    bool WriteParamAdvControl(uint8_t joint_id, const ParamAdvancedControlGroup_t& group);

    /** @brief 获取固件版本信息 (CMD_GET_FW_INFO, 0x23), 响应 FirmwareVersionInfo_t */
    bool GetFwInfo(uint8_t joint_id);

    /** @brief FOC零点标定 (CMD_FOC_CALIBRATE_ZERO, 0x16), 响应 FocZeroCalibrationResult_t */
    bool FocCalibrateZero(uint8_t joint_id);

    /** @brief KTH78 编码器电流零点标定 (CMD_KTH78_SET_CURRENT_ZERO, 0x24), 无响应数据 */
    bool Kth78SetCurrentZero(uint8_t joint_id);

    /** @brief 设置关节机械零点 (CMD_SET_JOINT_ZERO, 0x25), 响应 JointZeroResult_t */
    bool SetJointZero(uint8_t joint_id);

    /** @brief 读取关节零点设置结果缓存 (CMD_SET_JOINT_ZERO 响应) */
    bool GetJointZeroResult(uint8_t joint_id, JointZeroResult_t& out) const;
    /** @brief 内部出厂工具读取最近一次 MT6826S 校准状态。 */
    bool GetMt6826sFactoryCalStatus(uint8_t joint_id, Mt6826sFactoryCalStatus_t& out) const;


    // ========================================================================
    // OTA 固件升级
    // ========================================================================

    /**
     * @brief 打包固件包 (BIN → PKG)
     * @param bin_filepath BIN固件文件路径
     * @param pkg_filepath 输出的PKG文件路径
     * @param version 固件版本号
     */
    static bool PackFirmware(const std::string& bin_filepath, 
                             const std::string& pkg_filepath,
                             uint32_t version);

    /**
     * @brief 关节板OTA一键升级
     * 
     * 完整升级流程:
     * 1. 暂停实时闭环任务 (0x24)
     * 2. 跳转到 Bootloader (0xBB)
     * 3. 等待MCU重启 (2s)
     * 4. 查询升级状态 (0x86)
     * 5. 握手 (0x7F)
     * 6. 启动下载 (0x82): 发送版本/大小/CRC
     * 7. 发送固件分片 (0x83): 128B/片, 带序号, 失败重试3次
     * 8. 结束下载 (0x84): 设备做 CRC/SHA256 校验
     * 9. 等待MCU重启 (2s)
     * 10. 查询关节信息 (0x24)
     * 11. 恢复实时闭环任务 (0x25)
     * 
     * @param joint_id 目标关节ID (1-11)
     * @param pkg_filepath 固件包文件路径 (.pkg)
     * @return true-升级成功, false-失败
     */
    bool OtaUpgradeJoint(uint8_t joint_id, const std::string& pkg_filepath);

    /**
     * @brief Joint OTA upgrade with structured progress notifications.
     * @param joint_id Target joint ID (1-JOINT_COUNT)
     * @param pkg_filepath Firmware package path
     * @param progress_callback Synchronous progress callback; may be empty
     * @return true on success, false on failure
     * @note The callback must return quickly and should not call control APIs.
     */
    bool OtaUpgradeJoint(uint8_t joint_id,
                         const std::string& pkg_filepath,
                         const OTAProgressCallback& progress_callback);

    /**
     * @brief 打包掌板固件包 (BIN → PKG, magic=0x55AA55AA)
     * @param bin_filepath BIN固件文件路径
     * @param pkg_filepath 输出的PKG文件路径
     * @param version 固件版本号
     */
    static bool PackPalmFirmware(const std::string& bin_filepath,
                                 const std::string& pkg_filepath,
                                 uint32_t version);

    /**
     * @brief 掌板OTA一键升级
     *
     * 升级流程与关节板完全一致, 目标地址为 ADDR_PALM (0xFD),
     * 固件包魔数为 OTA_PALM_PKG_MAGIC (0x55AA55AA)。
     *
     * 完整升级流程:
     * 1. 暂停实时闭环任务 (0x24)
     * 2. 跳转到 Bootloader (0xBB)
     * 3. 等待MCU重启 (2s)
     * 4. 查询升级状态 (0x86)
     * 5. 握手 (0x7F)
     * 6. 启动下载 (0x82): 发送版本/大小/CRC
     * 7. 发送固件分片 (0x83): 128B/片, 带序号, 失败重试3次
     * 8. 结束下载 (0x84): 设备做 CRC/SHA256 校验
     * 9. 等待MCU重启 (2s)
     * 10. 查询掌板Flash信息确认启动 (0x10)
     * 11. 恢复实时闭环任务 (0x25)
     *
     * @param pkg_filepath 固件包文件路径 (.pkg, 须用 PackPalmFirmware 打包)
     * @return true-升级成功, false-失败
     */
    bool OtaUpgradePalm(const std::string& pkg_filepath);

    /**
     * @brief Palm OTA upgrade with structured progress notifications.
     * @param pkg_filepath Palm firmware package path
     * @param progress_callback Synchronous progress callback; may be empty
     * @return true on success, false on failure
     * @note The callback must return quickly and should not call control APIs.
     */
    bool OtaUpgradePalm(const std::string& pkg_filepath,
                        const OTAProgressCallback& progress_callback);

    // ========================================================================
    // 状态查询 (从缓存获取, 不发送命令)
    // ========================================================================
    
    /**
     * @brief 获取缓存的整手状态 (返回拷贝, 线程安全)
     * @return 整手状态副本
     */
    Hand_Status_t GetCachedHandStatus() const;

    /**
    * @brief 获取指定关节的缓存位置
     * @param joint_id 关节ID (1-11)
    * @return 开关开启时为角度值(度); 关闭时为协议原始 float 值; 无效返回0
     */
    float GetCachedPosition(uint8_t joint_id) const;

    /**
    * @brief 获取所有关节的缓存位置
    * @param positions 输出数组 (至少JOINT_COUNT=11个元素)
    *               开关开启时为角度值(度); 关闭时为协议原始 float 值
     */
    void GetCachedPositions(float* positions) const;

    /**
     * @brief 获取指定关节的缓存电流 (已解码)
     * @param joint_id 关节ID (1-11)
     * @return 电流值
     */
    float GetCachedCurrent(uint8_t joint_id) const;

    /**
     * @brief 获取指定关节的缓存速度 (已解码)
     * @param joint_id 关节ID (1-11)
     * @return 速度值
     */
    float GetCachedVelocity(uint8_t joint_id) const;

    /**
     * @brief 获取通讯状态计数值
     * 
     * 每收到一次掌板回传的状态数据时自增1，到达UINT32_MAX时置0。
     * 可用于监控通讯是否正常 (若长时间未更新则判定通讯超时)。
     * 
     * @return 当前计数值
     */
    uint32_t GetStatusRecvCount() const;

    // ========================================================================
    // 关节参数缓存读取 (透传模式, 需先发送对应读取命令后等待响应)
    // 返回 true 表示缓存有效并已写入 out; false 表示缓存尚未收到响应
    // ========================================================================

    bool GetJointMotorStatus(uint8_t joint_id, MotorUp_t& out) const;
    bool GetJointParamDevice(uint8_t joint_id, ParamDeviceGroup_t& out) const;
    bool GetJointParamControl(uint8_t joint_id, ParamControlGroup_t& out) const;
    bool GetJointParamCalib(uint8_t joint_id, ParamCalibrationGroup_t& out) const;
    bool GetJointFocCalib(uint8_t joint_id, FocZeroCalibrationResult_t& out) const;
    bool GetJointRuntimeStatus(uint8_t joint_id, RuntimeStatusFrame_t& out) const;
    bool GetJointRawSnapshot(uint8_t joint_id, RawSnapshotFrame_t& out) const;
    bool GetJointSensorStatus(uint8_t joint_id, SensorStatusFrame_t& out) const;
    bool GetJointParamAdvControl(uint8_t joint_id, ParamAdvancedControlGroup_t& out) const;
    bool GetJointFwInfo(uint8_t joint_id, FirmwareVersionInfo_t& out) const;
    bool GetPalmRuntimeModeStatus(Palm_Runtime_Mode_Status_t& out) const;


    
    /**
     * @brief 注册整手状态更新回调
     * 每次收到掌板的状态响应时触发
     */
    void RegisterHandStatusCallback(HandStatusCallback callback);

    /**
     * @brief 注册错误回调
     * 收到错误响应时触发
     */
    void RegisterErrorCallback(ErrorCallback callback);

    // ========================================================================
    // 通信模式查询与切换
    // ========================================================================

    /**
     * @brief 获取当前通信模式
     * @return COMM_MODE_RS485 或 COMM_MODE_CANFD
     */
    SG100_CommMode_e GetCommMode() const;

    /**
     * @brief 获取当前驱动模式
     *
     * @return DRIVE_MODE_ACTIVE  — SDK 内部调度线程驱动收发
     *         DRIVE_MODE_PASSIVE — 外部主动调用, SDK 仅做协议层处理
     */
    SG100_DriveMode_e GetDriveMode() const;

    /**
     * @brief 切换到 CANFD 模式
     *
     * 若当前为 RS485 模式, 将停止通信线程并关闭串口, 然后切换到 CANFD 被动模式。
     * 被动模式下 SDK 仅处理协议数据帧的封包/解包, 物理层收发由外部驱动负责:
     *   - 发送: 调用 CANFDGetTxFrame() 获取待发的 CANFD 帧
     *   - 接收: 调用 CANFDProcessRxFrame() 将收到的 CANFD 帧注入 SDK
     *
     * @return true-切换成功, false-失败
     */
    bool SwitchToCANFD();

    /**
     * @brief 切换到 RS485 模式 (通过参数)
     *
     * 若当前为 CANFD 模式则关闭 CANFD, 根据参数打开串口并切换为 RS485 主动模式。
     * 若当前已是 RS485 模式, 将重新配置串口。
     *
     * @param port      串口名称 (如 "COM3", "/dev/ttyUSB0")
     * @param baudrate  波特率 (默认 1000000)
     * @return true-切换成功, false-失败
     */
    bool SwitchToRS485ByParams(const std::string& port, unsigned long baudrate = 1000000);

    /**
     * @brief 切换到 RS485 模式 (通过 JSON 配置文件)
     *
     * 从指定配置文件中读取串口参数, 然后执行与 SwitchToRS485ByParams 相同的切换流程。
     *
     * @param filename  JSON 配置文件路径
     * @param flag      配置文件中的串口索引 (1-based)
     * @return true-切换成功, false-失败
     */
    bool SwitchToRS485ByConfig(const std::string& filename, uint8_t flag);

    /**
     * @brief 初始化为 CANFD 被动模式 (无需串口, 供 AddCANFDDevice 调用)
     *
     * 创建设备后直接设置为 CANFD 被动模式, 不进行串口初始化。
     * 设备创建后需调用 Start() 启动处理。
     *
     * @return true-成功
     */
    bool InitForCANFD();

    /**
     * @brief 设置 CANFD 传输配置
     *
     * 可在 InitForCANFD/SwitchToCANFD 前调用以覆盖默认配置；
     * 若当前已在 CANFD 模式，会立即下发到适配器生效。
     */
    bool SetCANFDConfig(const CANFD_Config_t& cfg);

    /**
     * @brief 获取当前 CANFD 传输配置
     *
     * 若当前未进入 CANFD 模式，返回最近一次设置值或默认值。
     */
    CANFD_Config_t GetCANFDConfig() const;

    // ========================================================================
    // 被动模式通用接口 (DRIVE_MODE_PASSIVE)
    // ========================================================================

    /**
     * @brief 注入外部收到的原始字节流 (被动模式通用入口)
     *
     * 适配器内部完成帧同步与完整性验证，提取完整帧后更新状态缓存。
     *
     * RS485 被动模式: 传入串口原始字节，内部完成帧同步 + CRC-32 验证后调用 rx_cb_
     * CANFD 被动模式: 传入任意长度的原始字节流，内部直接完成流重组后投递完整帧
     *
     * @param data  外部接收到的原始字节
     * @param len   字节数
     * @return true=成功; false=未处于被动模式或参数非法
     */
    bool PassiveInjectRxBytes(const uint8_t* data, uint16_t len);

    /**
     * @brief 取出下一条待发字节序列 (被动模式, 仅 RS485 有效)
     *
     * 每次调用返回一帧完整发送字节（含帧封装与 CRC-32），外部取到后直接写入总线。
     * 队列为空时返回 false。
     * CANFD 被动模式请使用 CANFDGetTxFrame()。
     *
     * @param out 输出: 待发字节序列
     * @return true=有数据; false=队列为空或非 RS485 被动模式
     */
    bool PassiveGetTxBytes(std::vector<uint8_t>& out);

    /**
     * @brief 检查被动模式 TX 队列是否有待发帧 (仅 RS485 被动模式有效)
     * @return true=有待发帧
     */
    bool PassiveHasTxBytes() const;

    // ========================================================================
    // CANFD 数据处理 (物理层由外部驱动提供)
    // ========================================================================

    /**
     * @brief 将外部驱动收到的 CANFD 帧注入 SDK 处理 (接收输入)
     *
     * 数据格式: CANFD 帧载荷 = RS485 帧去掉尾部 4 字节 CRC 后的数据流
     *   [0x48][0x4D][Len][Dst][Src][Cmd][Data...]  共 6+Len 字节
     * 多帧拼接: 当一条 RS485 消息超过 64 字节时, 会被拆成多个连续 CANFD 帧发送。
     *   SDK 内部维护流式缓冲区, 根据 Len 字段推算总长度, 自动完成多帧拼接后解析。
     * CRC 校验: 由 CANFD 物理层硬件保证, SDK 层不再验证 RS485 CRC。
     * CAN ID: 不用于地址过滤, 地址信息在 RS485 帧的 Dst/Src 字段中。
     *
     * @param can_id    接收帧的 CAN 扩展帧 ID
     * @param data      帧载荷数据指针
     * @param data_len  帧载荷有效长度 (1~64)
     * @return true-帧已处理, false-模式非CANFD或参数非法
     */
    bool CANFDProcessRxFrame(uint32_t can_id, const uint8_t* data, uint8_t data_len);

    /**
     * @brief 从 SDK 发送队列中取出一个待发的 CANFD 帧 (发送输出)
     *
     * 外部驱动应周期性调用本函数, 将取得的帧通过物理层发送。
     * 仅在 COMM_MODE_CANFD 被动模式下有效。
     *
     * @param out_frame 输出帧结构 (can_id / data / data_len)
     * @return true-成功取得待发帧, false-发送队列为空或模式非CANFD
     */
    bool CANFDGetTxFrame(CANFD_Frame_t& out_frame);

    /**
     * @brief 查询 CANFD 发送队列是否有待发帧
     * @return true-有待发帧
     */
    bool CANFDHasTxFrame() const;

    // ========================================================================
    // 设备信息
    // ========================================================================
    
    std::string GetDeviceName() const { return device_name_; }
    std::string GetPort() const { return config_.port; }

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    
    std::string device_name_;
    SG100Config config_;
    bool is_running_;
};

#endif // SG100_DEVICE_H

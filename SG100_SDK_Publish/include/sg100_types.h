/**
 * @file sg100_types.h
 * @brief SG100 SDK 类型定义 - 协议常量、数据结构、量化工具
 * 
 * SG100灵巧手由11块关节板 + 1块掌板组成 (4个手指: 拇指3+食指3+中指2+无名指3)
 * 帧格式: [0x48][0x4D][Len][Dst][Src][Cmd][Data...][CRC32-LE]
 * CRC算法: CRC-32/BZIP2 (Poly=0x04C11DB7, Init=0xFFFFFFFF, RefIn=false, RefOut=false, XorOut=0xFFFFFFFF)
 */

#ifndef SG100_TYPES_H
#define SG100_TYPES_H

#include <cstdint>
#include <cmath>
#include <algorithm>
#include <functional>

// ============================================================================
// DLL 导出宏
// ============================================================================
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
// 协议帧常量
// ============================================================================
#define PROTOCOL_HEADER_0           0x48        // 帧头第1字节 'H'
#define PROTOCOL_HEADER_1           0x4D        // 帧头第2字节 'M'
#define PROTOCOL_HEADER_SIZE        2           // 帧头大小
#define PROTOCOL_MIN_FRAME_SIZE     10          // 最小帧长 (header(2)+len(1)+dst(1)+src(1)+cmd(1)+crc(4))
#define PROTOCOL_OVERHEAD           10          // 帧固定开销
#define SERIAL_MAX_DATA_SIZE        256         // 最大数据段大小
#define SERIAL_BUFFER_SIZE          1024         // 接收缓冲区大小

// ============================================================================
// 地址定义
// ============================================================================
#define ADDR_PC                     0xFE        // 上位机(PC)地址
#define ADDR_PALM                   0xFD        // 掌板地址
#define ADDR_BROADCAST              0xFF        // 广播地址
#define ADDR_INVALID                0x00        // 无效/保留地址
#define ADDR_JOINT_MIN              0x01        // 关节最小地址
#define ADDR_JOINT_MAX              0x0B        // 关节最大地址 (11个关节)

// ============================================================================
// 设备数量
// ============================================================================
#define JOINT_COUNT                 11          // 关节数量 (1~11)

using SG100JointMask = uint16_t;

enum SG100JointMaskBits : SG100JointMask {
    SG100_JOINT_NONE = 0x0000,
    SG100_JOINT_1    = 0x0001,
    SG100_JOINT_2    = 0x0002,
    SG100_JOINT_3    = 0x0004,
    SG100_JOINT_4    = 0x0008,
    SG100_JOINT_5    = 0x0010,
    SG100_JOINT_6    = 0x0020,
    SG100_JOINT_7    = 0x0040,
    SG100_JOINT_8    = 0x0080,
    SG100_JOINT_9    = 0x0100,
    SG100_JOINT_10   = 0x0200,
    SG100_JOINT_11   = 0x0400,
    SG100_JOINT_ALL  = 0x07FF,
};

inline constexpr SG100JointMask SG100JointToMask(uint8_t joint_id) {
    return (joint_id >= 1 && joint_id <= JOINT_COUNT)
        ? static_cast<SG100JointMask>(1u << (joint_id - 1))
    : static_cast<SG100JointMask>(SG100_JOINT_NONE);
}

// ============================================================================
// 命令定义 - 掌板命令 (PC ↔ Palm)
// ============================================================================
#define CMD_REALTIME_CTRL           0x05        // 实时运动控制 / 状态反馈
#define CMD_FLASH_READ              0x10        // Flash读取 (掌板级)
#define CMD_FLASH_WRITE             0x11        // Flash写入 (掌板级)
#define CMD_REALTIME_PAUSE          0x24        // 暂停掌板实时闭环
#define CMD_REALTIME_RESUME         0x25        // 恢复掌板实时闭环
#define CMD_REALTIME_GET_STATUS     0x26        // 获取整手状态缓存
#define CMD_RUNTIME_GET_MODE        0x27        // 获取掌板当前运行模式

// 串口互联控制 (PC <-> 掌板上行口, 掌板内部透传到手指UART)
#define CMD_UART_LINK_OPEN          0x30        // 开启互联模式: payload UartLink_Open_t (2B)
#define CMD_UART_LINK_CLOSE         0x31        // 关闭互联模式: 无payload
#define CMD_UART_LINK_STATUS        0x32        // 查询互联模式状态: 响应 UartLink_Status_t (10B)

// ============================================================================
// 命令定义 - 关节命令 (透传: PC → Palm → Joint, 需先 PAUSE)
// 对应 SG100_Joint_Brushless firmware device_config.h
// CMD_SET_COMMAND = CMD_REALTIME_CTRL = 0x05 (同一命令字, 区分靠地址)
// ============================================================================
#define CMD_PING                    0x01        // 心跳/测试
#define CMD_GET_STATUS              0x02        // 获取关节状态 (MotorUp_t)
// CMD_SET_COMMAND = CMD_REALTIME_CTRL = 0x05
#define CMD_SET_COMMAND_V2          0x06        // 按模式载荷的控制命令
#define CMD_HOMING_ZERO             0x07        // 关节归零指令
#define CMD_PARAM_READ_DEVICE       0x10        // 读取设备信息参数 (ParamDeviceGroup_t)
#define CMD_PARAM_WRITE_DEVICE      0x11        // 写入设备信息参数
#define CMD_PARAM_READ_CONTROL      0x12        // 读取控制参数 (ParamControlGroup_t)
#define CMD_PARAM_WRITE_CONTROL     0x13        // 写入控制参数
#define CMD_PARAM_READ_CALIB        0x14        // 读取标定参数 (ParamCalibrationGroup_t)
#define CMD_PARAM_WRITE_CALIB       0x15        // 写入标定参数
#define CMD_FOC_CALIBRATE_ZERO      0x16        // FOC零点标定
#define CMD_GET_RUNTIME_STATUS      0x17        // 获取运行时状态 (RuntimeStatusFrame_t)
#define CMD_GET_RAW_SNAPSHOT        0x18        // 获取原始数据快照 (RawSnapshotFrame_t)
#define CMD_GET_SENSOR_STATUS       0x19        // 获取传感器状态 (SensorStatusFrame_t)
#define CMD_FOC_ENABLE              0x1A        // 使能FOC
#define CMD_FOC_DISABLE             0x1B        // 禁用FOC
#define CMD_INVALIDATE_ZERO         0x1C        // 使零点失效
#define CMD_RECALIB_ADC_OFFSET      0x1D        // 重新校准ADC偏移
#define CMD_TELEM_CONFIG            0x1E        // 配置遥测 (TelemetryConfig_t)
#define CMD_TELEM_START             0x1F        // 启动遥测
#define CMD_TELEM_STOP              0x20        // 停止遥测
#define CMD_PARAM_READ_ADV_CONTROL  0x21        // 读取高级控制参数 (ParamAdvancedControlGroup_t)
#define CMD_PARAM_WRITE_ADV_CONTROL 0x22        // 写入高级控制参数
#define CMD_GET_FW_INFO             0x23        // 获取固件版本信息 (FirmwareVersionInfo_t)
#define CMD_KTH78_SET_CURRENT_ZERO  0x24        // KTH78 编码器电流零点标定（关节板，无 payload）
#define CMD_SET_JOINT_ZERO          0x25        // 设置关节机械零点（关节板），响应 JointZeroResult_t
#define CMD_MT6826S_FACTORY_CAL     0x30        // MT6826S 出厂自校准（内部工具）

// ============================================================================
// OTA 命令 - 关节板 Bootloader 升级
// ============================================================================
#define CMD_OTA_JUMPTO_BL           0xBB        // 跳转到 Bootloader
#define CMD_OTA_HANDSHAKE           0x7F        // Bootloader 握手
#define CMD_OTA_DOWNLOAD_START      0x82        // 启动下载 (含固件元信息, 触发Flash擦除)
#define CMD_OTA_DOWNLOAD_DATA       0x83        // 发送固件数据分片
#define CMD_OTA_DOWNLOAD_END        0x84        // 结束下载 (触发校验)
#define CMD_OTA_GET_STATUS          0x86        // 查询升级状态

// ============================================================================
// 响应标志
// ============================================================================
#define CMD_RESPONSE_FLAG           0x80        // 响应标志位 (CMD | 0x80 表示错误响应)

// ============================================================================
// 命令结果码
// ============================================================================
enum Cmd_Result_t : uint8_t {
    CMD_OK              = 0,
    CMD_ERROR           = 1,
    CMD_INVALID_PARAM   = 2,
    CMD_NOT_SUPPORTED   = 3,
    CMD_CRC_ERROR       = 4,
    CMD_SEQ_ERROR       = 5,
    CMD_TIMEOUT         = 6
};

// ============================================================================
// 控制模式 (对应 SG100_Joint_V1.5 foc.h motor_control_type)
// ============================================================================
enum CtrlMode_e : uint8_t {
    MODE_NULL               = 0,    // 空控制 (停机)
    MODE_POSITION           = 1,    // 位置环控制
    MODE_SPEED              = 2,    // 速度环控制
    MODE_TORQUE             = 3,    // 电流/力矩环控制
    MODE_SPEED_TORQUE       = 4,    // 速度-力矩串级控制
    MODE_POS_VEL_TRQ        = 5,    // 三环串级 (位置→速度→力矩)
    MODE_OPENLOOP           = 6,    // 开环控制
    MODE_JOINT_POSITION     = 7,    // 关节位置控制 (基于 joint_zero_rad 的绝对位置)
    MODE_IMPEDANCE          = 8,    // 阻抗控制 (SG100_Joint_V1.5 新增)
    MODE_JOINT_IMPEDANCE    = 9     // 关节空间阻抗控制 (新增)
};

// ============================================================================
// 关节运行状态枚举 (对应 RuntimeState_t)
// ============================================================================
enum JointState_e : uint8_t {
    JOINT_STATE_BOOT_INIT       = 1,
    JOINT_STATE_SAFE_IDLE       = 2,
    JOINT_STATE_ACTIVE_LOOP     = 3,
    JOINT_STATE_CALIBRATING     = 4,
    JOINT_STATE_FLASH_BUSY      = 5,
    JOINT_STATE_ADC_CALIB       = 6,
    JOINT_STATE_HOMING          = 7
};

// ============================================================================
// 数据结构 - 紧凑对齐 (对应 device_config.h)
// ============================================================================

#pragma pack(push, 1)

/**
 * @brief 关节控制帧 - 位置模式 (PC → Joint, 14字节)
 * 对应 SG100_Joint_Brushless MotorDown_t
 */
typedef struct {
    uint8_t  id;            // [0]    关节ID (1-11)
    uint8_t  mode;          // [1]    控制模式 (CtrlMode_e)
    float    position;      // [2-5]  目标位置: 弧度 (rad), 原始float
    uint16_t p;             // [6-7]  位置环 Kp (量化 ±2)
    uint16_t i;             // [8-9]  位置环 Ki (量化 ±2)
    uint16_t d;             // [10-11] 位置环 Kd (量化 ±2)
    uint16_t output_limit;  // [12-13] 输出限制 (量化 ±1)
} MotorDown_t;              // Total: 14 bytes

/**
 * @brief 关节状态帧 (Joint → PC, 12字节)
 * 对应 SG100_Joint_Brushless MotorUp_t
 */
typedef struct {
    uint8_t  id;            // [0]    关节ID
    uint8_t  mode;          // [1]    当前控制模式
    float    position;      // [2-5]  当前累积位置: 弧度 (rad), 原始float
    uint16_t velocity;      // [6-7]  速度 (量化 ±3200 °/s)
    uint16_t cur_q;         // [8-9]  Q轴电流 (量化 ±5 A)
    uint16_t cur_d;         // [10-11] D轴电流 (量化 ±5 A)
} MotorUp_t;                // Total: 12 bytes

/**
 * @brief 位置控制命令 (CMD_SET_COMMAND_V2 + mode=1, 14字节)
 */
typedef struct {
    uint8_t  id;
    uint8_t  mode;
    float    position;      // 目标位置 (rad)
    uint16_t p;             // 量化 ±2
    uint16_t i;             // 量化 ±2
    uint16_t d;             // 量化 ±2
    uint16_t output_limit;  // 量化 ±1
} PositionControlCommand_t; // 14 bytes

/**
 * @brief 速度控制命令 (CMD_SET_COMMAND_V2 + mode=2, 6字节)
 */
typedef struct {
    uint8_t  id;
    uint8_t  mode;
    uint16_t speed;         // 量化 ±3200 °/s
    uint16_t output_limit;  // 量化 ±1
} SpeedControlCommand_t;    // 6 bytes

/**
 * @brief 电流/力矩控制命令 (CMD_SET_COMMAND_V2 + mode=3, 6字节)
 */
typedef struct {
    uint8_t  id;
    uint8_t  mode;
    uint16_t current_d;     // D轴电流 量化 ±5 A
    uint16_t current_q;     // Q轴电流 量化 ±5 A
} CurrentControlCommand_t;  // 6 bytes

/**
 * @brief 开环控制命令 (CMD_SET_COMMAND_V2 + mode=4, 8字节)
 */
typedef struct {
    uint8_t  id;
    uint8_t  mode;
    uint16_t voltage_d;     // D轴电压 量化 ±1
    uint16_t voltage_q;     // Q轴电压 量化 ±1
    uint16_t speed;         // 电角速度 量化 ±3200 °/s
} OpenloopControlCommand_t; // 8 bytes

/**
 * @brief 设备信息参数组 (CMD_PARAM_READ/WRITE_DEVICE)
 */
typedef struct {
    uint32_t device_id;
    uint8_t  device_name[32];
    uint8_t  UID[12];
    uint8_t  hand_sn[32];
    uint32_t baudrate;
    uint8_t  can_addr;
    uint8_t  joint_id;
} ParamDeviceGroup_t;

/**
 * @brief 控制参数组 (CMD_PARAM_READ/WRITE_CONTROL)
 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float Ikp;
    float Iki;
    float Ikd;
    float speed_kp;
    float speed_ki;
    float speed_kd;
    float pll_kp;
    float pll_ki;
} ParamControlGroup_t;

/**
 * @brief 高级控制参数组 (CMD_PARAM_READ/WRITE_ADV_CONTROL)
 */
typedef struct {
    float pos_integral_limit;
    float pos_derivative_limit;
    float pos_output_limit;
    float speed_integral_limit;
    float speed_derivative_limit;
    float speed_output_limit;
    float current_integral_limit;
    float current_derivative_limit;
    float current_output_limit;
    float default_max_speed;
    float default_max_torque_norm;
    uint8_t motor_pole_pairs;
} ParamAdvancedControlGroup_t;

/**
 * @brief 标定参数组 (CMD_PARAM_READ/WRITE_CALIB)
 */
typedef struct {
    float    electrical_angle_zero;
    uint8_t  foc_mode;
    uint32_t calibration_flags;
    float    align_voltage_d;
    uint32_t align_time_ms;
    float    joint_zero_rad;        // 关节机械零点角度 (rad)
} ParamCalibrationGroup_t;

/**
 * @brief FOC零点标定结果 (CMD_FOC_CALIBRATE_ZERO 响应)
 */
typedef struct {
    float    electrical_angle_zero;
    uint32_t calibration_flags;
} FocZeroCalibrationResult_t;

/**
 * @brief 关节零点设置结果 (CMD_SET_JOINT_ZERO 响应)
 */
typedef struct {
    float joint_zero_rad;   // 保存后的关节零点角度 (rad)
} JointZeroResult_t;        // 4 bytes

typedef struct {
    uint8_t state, chip_status, error;
    uint16_t target_rpm;
    int16_t actual_rpm;
    uint16_t revolution_count;
    uint32_t elapsed_ms;
    uint8_t power_cycle_required, reserved;
} Mt6826sFactoryCalStatus_t;

/**
 * @brief 遥测配置 (CMD_TELEM_CONFIG)
 */
typedef struct {
    uint8_t  stream_type;   // 0=禁用 1=RuntimeStatus 2=RawSnapshot 3=SensorStatus
    uint16_t period_ms;     // 发送周期 (ms)
} TelemetryConfig_t;

/**
 * @brief 运行时状态帧 (CMD_GET_RUNTIME_STATUS 响应)
 */
typedef struct {
    uint8_t  id;
    uint8_t  foc_ready;
    uint8_t  runtime_fault;
    uint8_t  runtime_state;
    uint8_t  control_mode;
    uint8_t  foc_mode;
    uint8_t  driver_fault_active;
    uint8_t  adc_offset_ready;
    uint8_t  telemetry_running;
    uint32_t calibration_flags;
    float    target_position;
    float    target_speed;
    float    target_torque_d;
    float    target_torque_q;
    float    max_speed;
    float    max_torque_norm;
    float    rotor_zero_angle;
} RuntimeStatusFrame_t;

/**
 * @brief 原始数据快照帧 (CMD_GET_RAW_SNAPSHOT 响应)
 */
typedef struct {
    uint16_t adc_in1;
    uint16_t adc_in2;
    uint16_t adc_in3;
    uint16_t hall_value;
    uint16_t ia_offset;
    uint16_t ib_offset;
    uint16_t ic_offset;
    float    ia;
    float    ib;
    float    ic;
    float    id;
    float    iq;
    float    encoder_angle;
    float    joint_angle;
    float    elec_angle;
    float    logic_angle;
    float    motor_speed;
} RawSnapshotFrame_t;

/**
 * @brief 传感器状态帧 (CMD_GET_SENSOR_STATUS 响应)
 */
typedef struct {
    uint16_t motor_raw_count;
    uint16_t joint_raw_count;
    uint16_t hall_value;
    uint8_t  driver_fault_active;
    uint8_t  clearzero_status;
    uint8_t  foc_ready;
    uint8_t  runtime_fault;
} SensorStatusFrame_t;

/**
 * @brief 固件版本信息 (CMD_GET_FW_INFO 响应, 54字节)
 * 对应 SG100_Joint_V1.5 fw_version.h FirmwareVersionInfo_t
 */
typedef struct {
    uint32_t magic;             // [0-3]   魔数 0x46575652UL "FWVR"
    uint16_t struct_size;       // [4-5]   结构体大小 (字节)
    uint8_t  major;             // [6]     主版本号
    uint8_t  minor;             // [7]     次版本号
    uint8_t  patch;             // [8]     补丁版本号
    uint8_t  reserved0;         // [9]     保留
    uint32_t build;             // [10-13] 构建号
    uint32_t flags;             // [14-17] 功能标志位
    uint32_t version_packed;    // [18-21] 打包版本号 (major<<16|minor<<8|patch)
    uint8_t  version_string[32];// [22-53] 版本字符串 e.g. "SG100_Joint 1.0.0+1"
} FirmwareVersionInfo_t;        // Total: 54 bytes

/**
 * @brief 关节控制帧 - 掌板级 (PC → Palm → Joint, 16字节/关节)
 * 与 SG100_Plam serial_protocol.h Joint_Control_t/Joint_Impedance_Control_t 兼容
 *
 * 匿名联合体布局 (共 16 字节):
 *   位置模式字段 (mode=1/7): position(f32) + p/i/d(u16) + output_limit(u16) + reserved[2]
 *   阻抗模式字段 (mode=8):   imp_position(f32) + imp_velocity/kp/kd/torque_ff/output_limit(u16×5)
 *
 * 注意: 两组字段的 [0-5] 字节布局完全一致 (id/mode/float position)，源码级向前兼容。
 */
typedef struct {
    uint8_t  id;            // [0]    关节ID (1-11)
    uint8_t  mode;          // [1]    控制模式 (CtrlMode_e)
    union {
        // --- 位置/关节位置模式字段 (mode=1/7, 原 14 字节布局 + 2 字节填充) ---
        struct {
            float    position;      // [2-5]  目标位置: 弧度 (rad), 原始float
            uint16_t p;             // [6-7]  位置环 Kp (量化 ±2)
            uint16_t i;             // [8-9]  位置环 Ki (量化 ±2)
            uint16_t d;             // [10-11] 位置环 Kd (量化 ±2)
            uint16_t output_limit;  // [12-13] 输出限制 (量化 ±10)
            uint8_t  reserved[2];   // [14-15] 填充到 16 字节
        };
        // --- 阻抗控制模式字段 (mode=8, 对应 ImpedanceControlCommand_t) ---
        struct {
            float    imp_position;     // [2-5]  目标关节角度 (rad)
            uint16_t imp_velocity;     // [6-7]  目标角速度 (量化 ±3200 deg/s)
            uint16_t imp_kp;           // [8-9]  阻抗 Kp (量化 ±10)
            uint16_t imp_kd;           // [10-11] 阻抗 Kd (量化 ±10)
            uint16_t imp_torque_ff;    // [12-13] 力矩前馈 (量化 ±10)
            uint16_t imp_output_limit; // [14-15] 阻抗输出限制 (量化 ±10)
        };
        uint8_t  raw_data[14];      // 用于 raw 字节访问 (兼容旧 14 字节字段)
    };
} Joint_Control_t;          // Total: 16 bytes

/**
 * @brief 关节状态帧 - 掌板级 (Joint → Palm → PC, 12字节/关节)
 * 与 SG100_Plam serial_protocol.h Joint_Status_t 完全一致
 * position: float 弧度 (无量化); velocity: ±3200°/s; cur_q/cur_d: ±5A
 */
typedef struct {
    uint8_t  id;            // [0]    关节ID
    uint8_t  mode;          // [1]    当前控制模式
    float    position;      // [2-5]  当前累积位置: 弧度 (rad), 原始float
    uint16_t velocity;      // [6-7]  速度 (量化 ±3200 °/s)
    uint16_t cur_q;         // [8-9]  Q轴电流 (量化 ±5 A)
    uint16_t cur_d;         // [10-11] D轴电流 (量化 ±5 A)
} Joint_Status_t;           // Total: 12 bytes

// ============================================================================
// 掌板互联模式 (CMD_UART_LINK_OPEN/CLOSE/STATUS, 0x30~0x32)
// 功能: PC 命令掌板将上行串口与指定手指UART直通，实现原始帧透传调试
// ============================================================================

/**
 * @brief UART互联的对端类型
 * 对应 SG100_Plam firmware uart_link.h UART_Link_Peer_t
 */
enum UartLink_Peer_e : uint8_t {
    UART_LINK_PEER_PC    = 0,   // 上行口对端为 PC
    UART_LINK_PEER_ESP32 = 1    // 上行口对端为 ESP32
};

/**
 * @brief CMD_UART_LINK_OPEN 载荷 (2字节)
 * 对应 firmware: data[0]=peer, data[1]=target_uart_id
 */
#pragma pack(push, 1)
typedef struct {
    uint8_t peer;           // UartLink_Peer_e: 上行口对端类型
    uint8_t target_uart_id; // 目标手指UART ID (1~5)
} UartLink_Open_t;          // 2 bytes
#pragma pack(pop)

/**
 * @brief CMD_UART_LINK_STATUS 响应载荷 (10字节)
 * 对应 SG100_Plam firmware uart_link.h UART_Link_Status_t
 */
#pragma pack(push, 1)
typedef struct {
    uint8_t  enabled;                   // 互联是否开启
    uint8_t  peer;                      // UartLink_Peer_e: 当前对端类型
    uint8_t  target_uart_id;            // 当前透传目标UART ID
    uint8_t  saved_realtime_enabled;    // 开启互联前的实时闭环状态（用于恢复）
    uint32_t rx_frames;                 // 已透传接收帧数
    uint32_t tx_frames;                 // 已透传发送帧数
    uint32_t drop_frames;               // 丢弃帧数（缓冲区满等原因）
} UartLink_Status_t;        // 10 bytes
#pragma pack(pop)

/**
 * @brief 掌板运行模式枚举
 * 对应 SG100_Plam firmware serial_protocol.h
 */
enum Palm_Runtime_Mode_e : uint8_t {
    PALM_RUNTIME_MODE_REALTIME        = 0x00,
    PALM_RUNTIME_MODE_NONRT_PAUSED    = 0x01,
    PALM_RUNTIME_MODE_UART_LINK_PC    = 0x02,
    PALM_RUNTIME_MODE_UART_LINK_ESP32 = 0x03,
    PALM_RUNTIME_MODE_FAKE_PC         = 0x04,
};

/**
 * @brief CMD_RUNTIME_GET_MODE 响应载荷 (6字节)
 * 对应 SG100_Plam firmware Palm_Runtime_Mode_Status_t
 */
#pragma pack(push, 1)
typedef struct {
    uint8_t mode;                      // Palm_Runtime_Mode_e
    uint8_t realtime_enabled;          // 实时闭环使能状态
    uint8_t fake_pc_enabled;           // Fake PC 输入使能状态
    uint8_t uart_link_enabled;         // 串口互联是否使能
    uint8_t uart_link_peer;            // UartLink_Peer_e
    uint8_t uart_link_target_uart_id;  // 互联目标手指UART ID
} Palm_Runtime_Mode_Status_t; // 6 bytes
#pragma pack(pop)

/**
 * @brief 整手控制结构 = 11 × Joint_Control_t = 176字节
 * CMD_REALTIME_CTRL (0x05) 载荷, PC → Palm
 * 新版 (Joint_Control_t=16B): 11 × 16 = 176 字节
 * 旧版 (Joint_Control_t=14B): 11 × 14 = 154 字节 (掌板固件向下兼容)
 */
typedef struct {
    Joint_Control_t joints[JOINT_COUNT];
} Hand_Control_t;           // Total: 176 bytes

#ifdef SG100_ENABLE_TACTILE
/**
 * @brief 三维力传感器基础数据 (来自 capRead_C 库 finger.h)
 */
typedef struct {
    float    nf;       // 法向力 (N)
    float    tf;       // 切向力 (N)
    uint16_t tfDir;    // 切向力方向 (°)
} ydds;                // Total: 10 bytes

/** @brief 触觉传感器数量 (拇指/食指/中指/无名指) */
#define FINGERTIP_COUNT 4

/**
 * @brief 指尖触觉传感器数据 (38字节/手指)
 * tactile[0]=拇指 tactile[1]=食指 tactile[2]=中指 tactile[3]=无名指
 */
typedef struct {
    uint32_t capChannelData[6]; // [0-23]  6通道原始电容数据
    ydds     yddsData;          // [24-33] 三维力: 法向力、切向力、切向角度
    uint32_t sProxCapData;      // [34-37] 接近觉电容值
} FingerTactile_t;              // Total: 38 bytes

static_assert(sizeof(FingerTactile_t) == 38, "FingerTactile_t must be 38 bytes");
#endif // SG100_ENABLE_TACTILE

/**
 * @brief 整手状态结构
 * 无触觉 (ENABLE_TACTILE=OFF): 12 × Joint_Status_t = 144字节
 * 有触觉 (ENABLE_TACTILE=ON):  + 4 × FingerTactile_t = 296字节
 * CMD_REALTIME_CTRL (0x05) 响应, Palm → PC
 */
typedef struct {
    Joint_Status_t  joints[JOINT_COUNT];        // [0-143]   144字节
#ifdef SG100_ENABLE_TACTILE
    FingerTactile_t tactile[FINGERTIP_COUNT];   // [144-295] 4×38=152字节
#endif
} Hand_Status_t;

/**
 * @brief 掌板用户参数 (CMD_FLASH_READ/WRITE 载荷)
 * 对应 SG100_Plam user_param.h UserParam_Data_t
 */
typedef struct {
    uint32_t magic;             // 魔数 0x504DAA55
    uint32_t version;           // 参数版本号
    uint32_t save_count;        // 保存计数
    uint32_t device_id;         // 设备唯一ID (生产追踪)
    uint8_t  device_name[16];   // 设备名称 e.g. "SG100-Palm"
    uint8_t  serial_number[16]; // 设备序列号
    uint8_t  palm_id;           // 掌板总线地址 (1~254)
    uint8_t  finger_count;      // 手指数量
    uint8_t  joint_count;       // 关节数
    uint8_t  hand_type;         // 左右手 (0=右手, 1=左手)
    uint8_t  reserved[200];     // 预留扩展
} UserParam_Data_t;             // Total: 252 bytes

#pragma pack(pop)

// ============================================================================
// 量化参数 - 对应 SG100_Joint_V1.5 quantization_profiles.h
// 规则: 对称 int16 编码 [-32767, +32767] ↔ [-range, +range]
// position 字段在协议中为 float 直传，无需量化
// ============================================================================
namespace SG100_Quant {

constexpr int16_t QUANT_MAX         = 32767;

// 控制帧量化范围 (MotorDown_t / PositionControlCommand_t)
constexpr float CTRL_PID_RANGE      = 10.0f;        // P/I/D: ±2
constexpr float CTRL_VEL_RANGE      = 3200.0f;     // 速度: ±3200 °/s
constexpr float CTRL_CUR_RANGE      = 5.0f;        // DQ轴电流: ±5 A
constexpr float CTRL_NORM_RANGE     = 1.0f;        // 归一化输出: ±1
constexpr float CTRL_OUTLIM_RANGE   = 10.0f;        // output_limit: ±1

// 状态帧量化范围 (MotorUp_t)
constexpr float STAT_VEL_RANGE      = 3200.0f;     // 速度: ±3200 °/s
constexpr float STAT_CUR_RANGE      = 5.0f;        // DQ轴电流: ±5 A

/**
 * @brief 浮点值 → 量化uint16 (编码)
 */
inline uint16_t encode(float value, float range) {
    float clamped = std::max(-range, std::min(range, value));
    int16_t q = static_cast<int16_t>(std::round(clamped * QUANT_MAX / range));
    return static_cast<uint16_t>(q);
}

/**
 * @brief 量化uint16 → 浮点值 (解码)
 */
inline float decode(uint16_t raw, float range) {
    int16_t q = static_cast<int16_t>(raw);
    return q * range / static_cast<float>(QUANT_MAX);
}

// --- 控制帧编码 ---
inline uint16_t encodePid(float v)      { return encode(v, CTRL_PID_RANGE); }
inline uint16_t encodeVel(float vel)    { return encode(vel, CTRL_VEL_RANGE); }
inline uint16_t encodeCur(float c)      { return encode(c, CTRL_CUR_RANGE); }
inline uint16_t encodeNorm(float v)     { return encode(v, CTRL_NORM_RANGE); }
inline uint16_t encodeOutLim(float v)   { return encode(v, CTRL_OUTLIM_RANGE); }

// --- 状态帧解码 ---
inline float decodeVel(uint16_t raw)    { return decode(raw, STAT_VEL_RANGE); }
inline float decodeCurQ(uint16_t raw)   { return decode(raw, STAT_CUR_RANGE); }
inline float decodeCurD(uint16_t raw)   { return decode(raw, STAT_CUR_RANGE); }

} // namespace SG100_Quant

// ============================================================================
// 手指-关节映射 (对应 SG100_Plam finger_map.h)
// ============================================================================
namespace SG100_FingerMap {

enum Finger {
    FINGER_THUMB    = 0,    // 拇指:   Joint 1,2,3
    FINGER_INDEX    = 1,    // 食指:   Joint 4,5,6
    FINGER_MIDDLE   = 2,    // 中指:   Joint 7,8
    FINGER_RING     = 3,    // 无名指: Joint 9,10,11
    FINGER_COUNT    = 4
};

// 每个手指包含的关节ID (1-based, 0表示无效)
constexpr uint8_t FINGER_JOINTS[FINGER_COUNT][3] = {
    {1,  2,  3},    // 拇指
    {4,  5,  6},    // 食指
    {7,  8,  0},    // 中指 (2个关节)
    {9,  10, 11},   // 无名指
};

// 每个手指的关节数量
constexpr uint8_t FINGER_JOINT_COUNT[FINGER_COUNT] = {3, 3, 2, 3};

} // namespace SG100_FingerMap

// ============================================================================
// 工作模式枚举
// ============================================================================
enum SG100_WorkMode_e : uint8_t {
    WORK_MODE_REALTIME      = 0,    // 实时调度模式 (默认)
    WORK_MODE_PASSTHROUGH   = 1     // 透传模式
};

// ============================================================================
// 命令队列结构
// ============================================================================
#define MAX_QUEUE_DATA_SIZE     256

#pragma pack(push, 1)
typedef struct {
    uint8_t  cmd;                           // 命令字
    uint8_t  dst_addr;                      // 目标地址
    uint8_t  priority;                      // 优先级 (数值越大越优先)
    uint16_t data_size;                     // 数据长度
    uint8_t  data[MAX_QUEUE_DATA_SIZE];     // 数据内容
    bool     wait_response;                 // 是否等待响应
} SG100_QueueCmd_t;
#pragma pack(pop)

static_assert(sizeof(SG100_QueueCmd_t) == 262, "SG100_QueueCmd_t must be 1-byte packed");

// ============================================================================
// OTA 固件包相关
// ============================================================================
#define OTA_JOINT_PKG_MAGIC         0x55AA55AAU // 关节板固件包魔数 (uint32 LE)
#define OTA_PALM_PKG_MAGIC          0x504DAA55U // 掌板固件包魔数 (uint32 LE)
#define OTA_PKG_HEADER_SIZE         20      // 固件包头大小
#define OTA_CHUNK_SIZE              128     // 分片大小
#define OTA_MAX_RETRY               3       // 单片最大重试次数
#define OTA_REBOOT_WAIT_MS          2000    // 等待MCU重启时间(ms)
#define OTA_DOWNLOAD_TIMEOUT_MS     5000    // 下载启动超时(ms)
#define OTA_CHUNK_TIMEOUT_MS        2500    // 分片传输超时(ms)

#pragma pack(push, 1)

/**
 * @brief OTA固件包头结构 (20字节)
 */
typedef struct {
    uint8_t  magic[4];          // [0-3]   魔数 "FWPK"
    uint32_t version;           // [4-7]   固件版本号
    uint32_t size;              // [8-11]  固件大小 (字节)
    uint32_t crc;               // [12-15] 固件CRC32校验值
    uint32_t reserved;          // [16-19] 保留字段
} OTA_PkgHeader_t;              // Total: 20 bytes

/**
 * @brief CMD_OTA_GET_STATUS (0x86) 响应载荷 (6字节)
 */
typedef struct {
    uint8_t  status;            // 0x00=无升级, 0x01=升级中, 0x02=存在应用
    uint32_t total_received;    // 已接收字节数
    uint8_t  progress;          // 升级进度 (%)
} OTA_UpgradeStatus_t;          // Total: 6 bytes

#pragma pack(pop)

// ============================================================================
// OTA 结束下载结果码
// ============================================================================
enum OTA_EndResult_e : uint8_t {
    OTA_END_OK                  = 0x00,     // 升级成功
    OTA_END_CRC_FAIL            = 0x04,     // CRC校验失败
    OTA_END_LENGTH_MISMATCH     = 0x07,     // 接收长度与元数据不匹配
    OTA_END_SHA256_FAIL         = 0x08,     // SHA256校验失败
    OTA_END_META_WRITE_FAIL     = 0x09,     // 元数据区写入失败
    OTA_END_META_CRC_MISMATCH   = 0x0A      // 元数据区CRC不匹配
};

// ============================================================================
// OTA upgrade progress
// ============================================================================

/**
 * @brief OTA upgrade stage reported by OTAProgressCallback.
 *
 * The numeric values are part of the public SDK contract and are mirrored by
 * the C wrapper used by language bindings. Keep existing values stable.
 */
enum OTA_ProgressStage_e : uint8_t {
    OTA_STAGE_PREPARING                 = 0,
    OTA_STAGE_SWITCHING_TO_PASSTHROUGH = 1,
    OTA_STAGE_PAUSING_MOTOR             = 2,
    OTA_STAGE_ENTERING_BOOTLOADER       = 3,
    OTA_STAGE_WAITING_FOR_BOOTLOADER    = 4,
    OTA_STAGE_HANDSHAKING               = 5,
    OTA_STAGE_ERASING_FLASH             = 6,
    OTA_STAGE_TRANSFERRING              = 7,
    OTA_STAGE_VERIFYING                 = 8,
    OTA_STAGE_REBOOTING                 = 9,
    OTA_STAGE_RECONNECTING              = 10,
    OTA_STAGE_RESTORING_REALTIME        = 11,
    OTA_STAGE_COMPLETED                 = 12,
    OTA_STAGE_FAILED                    = 13
};

/**
 * @brief Snapshot delivered while an OTA upgrade is running.
 *
 * target_id uses the public API convention: 0 means the palm board and
 * 1..JOINT_COUNT means a joint board. percent is transfer progress. It remains
 * at 100 while verification, reboot and communication restoration complete.
 */
struct OTA_Progress_t {
    OTA_ProgressStage_e stage;
    uint8_t target_id;
    uint32_t current_chunk;
    uint32_t total_chunks;
    uint32_t bytes_sent;
    uint32_t total_bytes;
    float percent;
};

/** @brief OTA progress callback. Called synchronously from the OTA worker. */
using OTAProgressCallback = std::function<void(const OTA_Progress_t&)>;

// ============================================================================
// 通信模式枚举
// ============================================================================
enum SG100_CommMode_e : uint8_t {
    COMM_MODE_RS485  = 0,   // RS485 串口通信 (默认)
    COMM_MODE_CANFD  = 1    // CANFD 总线通信 (仅数据处理, 物理层由外部驱动负责)
};

// ============================================================================
// 驱动模式枚举
// ============================================================================
/**
 * @brief 适配器驱动模式
 *
 * DRIVE_MODE_ACTIVE  (主动模式):
 *   SDK 内部启动 CommScheduler 后台线程，自动完成帧发送和接收轮询。
 *   适用于 SDK 直接持有物理总线访问权（如打开串口）的场景。
 *
 * DRIVE_MODE_PASSIVE (被动模式):
 *   SDK 不启动后台线程，仅完成协议层解析。
 *   发送: SDK 将待发帧缓存，外部通过 PassiveGetTxBytes() 取走后自行写入总线。
 *   接收: 外部将收到的原始字节通过 PassiveInjectRx() 注入，SDK 完成帧同步与解析。
 *   适用于外部已有总线驱动（CANFD、自定义协议等）的集成场景。
 */
enum SG100_DriveMode_e : uint8_t {
    DRIVE_MODE_ACTIVE  = 0,   // 主动模式: SDK 内部调度线程驱动收发 (默认)
    DRIVE_MODE_PASSIVE = 1    // 被动模式: 外部主动调用, SDK 仅做协议层处理
};

// ============================================================================
// CANFD 协议常量
// ============================================================================

// CANFD 单帧最大载荷字节数
#define CANFD_MAX_PAYLOAD       64

// CANFD 默认 ID 配置 (对齐 SG100_Plam 当前固件)
// - 默认使用 11-bit 标准帧
// - TX 分片起始 ID: 0x200, 后续分片按 +1 递增
// - RX 过滤默认接收 0x200~0x2FF (min=0x200, max=0x2FF)
#define CANFD_BASE_ID             0x00000200U   // 兼容旧代码: 等价于默认 TX 起始 ID
#define CANFD_DEFAULT_TX_START_ID 0x00000200U
#define CANFD_DEFAULT_RX_MIN_ID   0x00000200U
#define CANFD_DEFAULT_RX_MAX_ID   0x000002FFU

// CANFD 帧内容格式:
//   数据 = RS485 完整帧, 即 [0x48][0x4D][Len][Dst][Src][Cmd][Data...][CRC32-LE]
//   帧头固定占 6 字节 (PROTOCOL_HEADER_0, PROTOCOL_HEADER_1, Len, Dst, Src, Cmd)
//   总字节数 = 6 + Len + 4; 单帧 ≤ 64 字节; 超过则按 64 字节顺序分片
//   其中 CRC32-LE 使用 CRC-32/BZIP2, 以兼容掌板串口协议解析
#define CANFD_FRAME_HEADER_SIZE 6       // RS485 帧头大小 (不含 CRC)

// CANFD 接收流缓冲区大小 (RS485 最大帧体 = 6+255 = 261 字节, 取 512 留余量)
#define CANFD_RX_STREAM_BUF_SIZE  512

// CANFD 发送队列最大深度
#define CANFD_TX_QUEUE_MAX      32

/**
 * @brief CANFD 帧结构
 *
 * SDK 输出到外部驱动 (通过 CANFDGetTxFrame 获取) 或
 * 外部驱动输入到 SDK  (通过 CANFDProcessRxFrame 注入)。
 *
 * 多帧说明:
 *   当 RS485 帧体 (6+Len 字节) 超过 64 字节时, SDK 会自动拆分为多个 CANFD 帧
 *   依次发出; 接收侧将连续到达的帧载荷流式拼接, 通过 Len 字段判断一条完整
 *   RS485 消息的边界。
 */
#pragma pack(push, 1)
typedef struct {
    uint32_t can_id;                    // CAN ID 数值 (标准11位或扩展29位，不含平台标志位)
    uint8_t  data[CANFD_MAX_PAYLOAD];   // 帧载荷
    uint8_t  data_len;                  // 有效数据长度 (1~64)
} CANFD_Frame_t;
#pragma pack(pop)

/**
 * @brief CANFD 传输配置 (用于 SDK CANFD 适配器)
 *
 * use_extended_id=0: 按 11-bit 标准 ID 处理 (默认, 对齐掌板)
 * use_extended_id=1: 按 29-bit 扩展 ID 处理
 *
 * enable_sequential_tx_id=1: 一条协议消息被分片后，CAN ID 从 tx_start_id 连续递增
 * enable_rx_filter=1: InjectRxFrame 时按 rx_filter_min_id <= can_id <= rx_filter_max_id 过滤
 */
#pragma pack(push, 1)
typedef struct {
    uint32_t tx_start_id;               // 发送分片起始 ID
    uint32_t rx_filter_min_id;          // 接收过滤最小 ID (含)
    uint32_t rx_filter_max_id;          // 接收过滤最大 ID (含)
    uint8_t  use_extended_id;           // 0=标准帧, 1=扩展帧
    uint8_t  enable_sequential_tx_id;   // 0=固定 tx_start_id, 1=分片递增
    uint8_t  enable_rx_filter;          // 0=不过滤, 1=按 min/max 过滤
} CANFD_Config_t;
#pragma pack(pop)

// ============================================================================
// 设备回调类型 (协议处理层与设备层共用)
// ============================================================================

/** @brief 整手状态更新回调 */
using HandStatusCallback = std::function<void(const Hand_Status_t&)>;

/** @brief 错误回调 (src_addr, error_code) */
using ErrorCallback = std::function<void(uint8_t, uint8_t)>;

#endif // SG100_TYPES_H

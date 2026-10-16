# SG100 SDK

SG100 灵巧手 C++ SDK —— 面向 SG100 11 关节机器人灵巧手的跨平台软件开发工具包，提供完整的实时控制、参数配置、状态监控与 OTA 固件升级能力。

## 特性

- **整手实时控制** —— 11 关节同步位置/速度/力矩/阻抗控制，100Hz 闭环刷新
- **单关节透传操作** —— FOC 使能/禁用、归零标定、Flash 参数读写、遥测配置
- **双工作模式** —— 实时调度模式 (整手控制) 与透传模式 (单关节直访)，可动态切换
- **多设备管理** —— 单例管理器同时管控多只手 (左手+右手等)
- **OTA 固件升级** —— 关节板与掌板一键远程升级 (.bin → .pkg 打包 + 128B 分片传输 + CRC/SHA256 校验)
- **双传输层** —— RS485 (串口 + CRC-32/BZIP2) 与 CANFD (64B 帧分片重组)，支持主动/被动驱动模式
- **跨平台** —— Windows (MSVC/MinGW)、Linux (x86_64/ARM64)、macOS (x86_64/ARM64)
- **状态缓存 + 回调** —— 内置线程安全的状态缓存，支持 HandStatusCallback / ErrorCallback 异步通知
- **版本化故障诊断** —— 支持故障快照、活动/锁存状态、严重级别、事件序号和扩展命令错误回调
- **C++14** —— 标准 C++；nlohmann/json、跨平台串口库和 spdlog 均随源码/发布包内置

## 系统架构

```
┌──────────────────────────────────────────────────┐
│  应用层 (examples/)                                │
├──────────────────────────────────────────────────┤
│  SG100DeviceManager (单例) — 多设备编排             │
├──────────────────────────────────────────────────┤
│  SG100Device (Pimpl)    — 单设备控制入口            │
│  ├── CommScheduler      — 实时/透传双队列 + 10ms 线程│
│  └── ProtocolHandler    — 帧解析 + 状态缓存 + 回调   │
├──────────────────────────────────────────────────┤
│  ITransportAdapter (策略接口)                      │
│  ├── RS485Adapter       — 串口 + CRC-32/BZIP2     │
│  └── CANFDAdapter       — CANFD 碎片化/重组        │
├──────────────────────────────────────────────────┤
│  SG100Protocol (静态工具) — 帧打包/解包             │
└──────────────────────────────────────────────────┘
```

**物理拓扑**

```
PC (0xFE) ←→ UART/RS485 (460800bps) ←→ Palm Board (0xFD) ←→ 4×RS485 ←→ 11 Joints (0x01–0x0B)
```

## 目录结构

```
SG100_SDK_Publish/
├── CMakeLists.txt               # CMake 构建配置 (≥3.16)
├── serial_config.json           # 默认串口配置 (端口/波特率/校验等)
├── examples/serial_config.macos.example.json  # macOS 串口配置模板
├── joint_angle_map_v3.0.json   # 新版 L/R 角度映射 (a*angle+b) 与限位
├── joint_angle_map.json         # 旧版单表格式（兼容）
├── include/                     # 公共头文件
│   ├── sg100_device.h           # SG100Device - 核心设备 API
│   ├── sg100_device_manager.h   # SG100DeviceManager - 多设备管理器
│   ├── sg100_types.h            # 协议常量、数据结构、量化工具
│   ├── sg100_angle_map.h        # 角度映射类型、L/R 选择与校验状态
│   ├── crc.hpp                  # CRC-32/BZIP2 校验
│   ├── json.hpp                 # nlohmann JSON 解析器
│   ├── serial/                  # 跨平台串口库
│   ├── protocol/                # 协议层
│   │   └── sg100_protocol.h     # 帧打包/解包工具
│   ├── comm/                    # 通信调度层
│   │   ├── comm_scheduler.h     # 命令队列 + 10ms 定时线程
│   │   └── protocol_handler.h   # 帧解析 + 状态缓存 + 回调
│   └── transport/               # 传输层适配器
│       ├── i_transport_adapter.h
│       ├── rs485_adapter.h
│       └── canfd_adapter.h
├── lib/<platform>/              # 预编译 SDK 动态库
├── examples/                    # 示例程序源码与配置
├── docs/                        # 详细文档
└── .gitignore                   # 忽略本地构建目录和 macOS 元数据
```

发布仓库不包含 SDK `src/` 实现源码或固件文件；需要从源码编译时请使用
`SG100_SDK` 仓库，发布仓库仅用于集成预编译库和构建示例程序。

## 快速开始

### 环境要求

| 项目 | 要求 |
|------|------|
| C++ 标准 | C++14 |
| CMake | ≥ 3.16 |
| 编译器 | MSVC 2019+ / GCC 7+ / MinGW-w64 / Apple Clang (Xcode Command Line Tools) |
| 操作系统 | Windows / Linux (x86_64, ARM64) / macOS (x86_64, ARM64) |

### 编译

```bash
# Windows (MSVC)
cd SG100_SDK
mkdir build && cd build
cmake .. -G "Visual Studio 16 2019" -A x64
cmake --build . --config Release

# Linux / macOS
cd SG100_SDK
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

### CMake 选项

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `BUILD_EXAMPLES` | ON | 编译示例程序 |
| `BUILD_SHARED_LIBS` | ON | 构建动态库 |
| `ENABLE_TACTILE` | OFF | 启用手部触觉传感器数据 (`FingerTactile_t`) |

### 最小示例

```cpp
#include "sg100_device.h"

int main() {
    SG100Device hand("MY_HAND");

    // 1. 配置串口
    SG100Config cfg;
    cfg.port = "COM3";          // Linux: "/dev/ttyUSB0"; macOS: "/dev/cu.*"
    cfg.baudrate = 460800;
    hand.SetConfig(cfg);

    // 2. 启动设备
    hand.Init();
    hand.Start();

    // 3. 设置所有关节位置 (度)
    float positions[11] = {30, 45, 60, 0, 30, 60, 0, 30, 90, 60, 30};
    hand.SetJointPositions(positions, 11);

    // 4. 读取缓存状态
    float pos1 = hand.GetCachedPosition(1);
    float cur1 = hand.GetCachedCurrent(1);

    // 5. 停止
    hand.Stop();
    return 0;
}
```

### 多设备管理

```cpp
#include "sg100_device_manager.h"

auto& mgr = SG100DeviceManager::Instance();
mgr.AddDeviceByParams("LEFT_HAND",  "COM3", 460800);
mgr.AddDeviceByParams("RIGHT_HAND", "COM4", 460800);
mgr.StartAll();

auto* left = mgr.GetDevice("LEFT_HAND");
left->SetJointPositions(positions, 11);

mgr.StopAll();
```

## 核心 API

### SG100Device — 设备控制

#### 生命周期
| 方法 | 说明 |
|------|------|
| `GetConfig(filename, flag)` | 从 JSON 配置文件读取串口参数 |
| `SetConfig(config)` | 直接设置串口配置 |
| `Init()` | 打开串口连接 |
| `Start()` | 启动通信线程 (10ms 周期) |
| `Stop()` | 停止通信并关闭串口 |
| `IsRunning()` | 查询运行状态 |

#### 角度映射配置
| 方法 | 说明 |
|------|------|
| `SetConfigSearchDir(dir)` | 设置角度映射配置目录 |
| `SetAngleMapHand(RIGHT/LEFT)` | 选择新版 JSON 的 R/L 映射（必须在 Start 前调用） |
| `ValidateAngleMap()` | 加载并校验映射配置 |
| `GetAngleMapStatus()` | 获取配置来源、版本和错误信息 |

SDK 优先加载 `joint_angle_map_v3.0.json`，不存在时兼容旧版
`joint_angle_map.json`。新版文件存在但校验失败时不会静默回退。

#### 整手控制 (实时模式)
| 方法 | 说明 |
|------|------|
| `SetHandControl(ctrl)` | 发送整手实时控制命令 (11×MotorDown_t) |
| `SetJointPositions(positions, count)` | 便捷位置控制 (11 个角度, 度) |
| `SetJointPosition(joint_id, position_deg)` | 单关节位置控制 |
| `SetJointImpedance(...)` | 单关节阻抗控制 (位置+速度+Kp+Kd+前馈) |
| `SetJointJointImpedance(...)` | 关节空间阻抗控制 (不经过增量规划器) |
| `RequestHandStatus()` | 请求整手状态反馈 |

#### 单关节操作 (透传模式)
| 方法 | 说明 |
|------|------|
| `HomingZero(joint_id)` | 单关节归零 |
| `HomingZeroMasked(mask, ...)` | 批量按位掩码归零 |
| `CalibrateJointZero(joint_id)` | 归零校准 (自动切换模式, 含 15s 超时) |
| `FocEnable(joint_id)` / `FocDisable(joint_id)` | FOC 使能/禁用 |
| `GetJointStatus(joint_id)` | 获取关节 MotorUp_t 状态 |
| `GetRuntimeStatus(joint_id)` | 获取运行时状态 |
| `GetRawSnapshot(joint_id)` | 获取原始数据快照 |
| `GetSensorStatus(joint_id)` | 获取传感器状态 |
| `TelemetryConfig/Start/Stop(joint_id, ...)` | 遥测数据流配置 |
| `ReadParamDevice/Control/Calib/AdvControl(...)` | 读取 Flash 参数组 |
| `WriteParamDevice/Control/Calib/AdvControl(...)` | 写入 Flash 参数组 |
| `GetFwInfo(joint_id)` | 获取固件版本信息 |
| `FocCalibrateZero(joint_id)` | FOC 零点标定 |
| `Kth78SetCurrentZero(joint_id)` | KTH78 编码器电流零点 |
| `SetJointZero(joint_id)` | 设置机械零点 |

#### 工作模式切换
| 方法 | 说明 |
|------|------|
| `SwitchToRealtimeMode()` | 切换到实时调度模式 |
| `SwitchToPassthroughMode()` | 切换到透传模式 |
| `GetWorkMode()` | 获取当前模式 |
| `SendPassthroughCommand(dst, cmd, data, len)` | 透传自定义命令 |

#### OTA 升级
| 方法 | 说明 |
|------|------|
| `PackFirmware(bin_path, pkg_path, version)` | 打包关节固件 (.bin → .pkg) |
| `PackPalmFirmware(bin_path, pkg_path, version)` | 打包掌板固件 |
| `OtaUpgradeJoint(joint_id, pkg_path)` | 关节板 OTA 一键升级 |
| `OtaUpgradePalm(pkg_path)` | 掌板 OTA 一键升级 |
| `OtaUpgradeJoint(joint_id, pkg_path, callback)` | 关节板 OTA，并回调阶段、分片、字节数与百分比 |
| `OtaUpgradePalm(pkg_path, callback)` | 掌板 OTA，并回调结构化进度 |

带进度回调的重载不会改变原有接口。回调在 OTA 工作线程中同步执行，
应快速返回，且不应在回调中调用控制接口：

```cpp
OTAProgressCallback progress = [](const OTA_Progress_t& p) {
    std::cout << "stage=" << static_cast<int>(p.stage)
              << " progress=" << p.percent << "%"
              << " bytes=" << p.bytes_sent << "/" << p.total_bytes
              << std::endl;
};

bool ok = hand->OtaUpgradeJoint(7, "joint.pkg", progress);
```

#### 状态缓存
| 方法 | 说明 |
|------|------|
| `GetCachedHandStatus()` | 获取整手状态副本 |
| `GetCachedPosition(joint_id)` | 获取指定关节位置 |
| `GetCachedPositions(positions)` | 获取所有关节位置 |
| `GetCachedCurrent(joint_id)` | 获取指定关节电流 |
| `GetCachedVelocity(joint_id)` | 获取指定关节速度 |
| `GetStatusRecvCount()` | 获取通讯状态计数 |

#### 回调
| 方法 | 说明 |
|------|------|
| `RegisterHandStatusCallback(cb)` | 注册整手状态更新回调 |
| `RegisterErrorCallback(cb)` | 注册兼容旧版本的错误回调 `(src_addr, result)` |
| `RegisterExtendedErrorCallback(cb)` | 注册版本化错误回调，接收 `CmdErrorResponseV1_t` |

#### 故障诊断
| 方法/类型 | 说明 |
|------|------|
| `GetFaultStatus(joint_id)` | 请求关节故障快照，命令 `0x28`；结果异步写入 SDK 缓存 |
| `GetJointFaultStatus(joint_id, out)` | 读取最近一次有效的 `FaultStatusFrameV1_t` 缓存 |
| `ClearFaults(joint_id, clear_mask, expected_event_seq)` | 清除非活动的锁存故障，命令 `0x29` |
| `GetJointLastCommandError(joint_id, out)` | 读取最近一次版本化命令错误缓存 |
| `FaultSeverity_e` / `SG100FaultCode` | 故障严重级别和完整错误码常量 |

故障码是 32 位值 `0xSSMMRRRR`：`SS` 为严重级别，`MM` 为子系统，`RRRR` 为具体原因。
`0x28` 返回 `FaultStatusFrameV1_t`，其中包含 `active_flags`、`latched_flags`、
`last_error_code`、`last_error_detail`、`event_seq` 和各类计数器；`0x29` 使用
`FaultClearRequestV1_t` 携带清除掩码和事件序号。完整字段、兼容性约束、错误码目录和推荐处理流程见
[`docs/ERROR_CODE_GUIDE.md`](docs/ERROR_CODE_GUIDE.md)。

建议的基本流程是：切换到透传模式后请求快照，按 `highest_severity` 和 `active_flags`
决定是否停止控制；仅对已恢复的锁存故障执行清除，并携带读取快照时的 `event_seq`，避免把读取期间新产生的故障误清除。

#### 传输模式
| 方法 | 说明 |
|------|------|
| `GetCommMode()` | RS485 / CANFD |
| `GetDriveMode()` | Active (SDK 驱动收发) / Passive (外部驱动) |
| `SwitchToCANFD()` | 切换到 CANFD 被动模式 |
| `SwitchToRS485ByParams(port, baud)` | 切换到 RS485 |
| `CANFDProcessRxFrame(can_id, data, len)` | 注入 CANFD 接收帧 |
| `CANFDGetTxFrame(out)` | 取出 CANFD 待发帧 |
| `PassiveInjectRxBytes(data, len)` | 注入被动模式接收字节 |
| `PassiveGetTxBytes(out)` | 取出被动模式待发字节 |

### SG100DeviceManager — 多设备管理

| 方法 | 说明 |
|------|------|
| `Instance()` | 获取单例 |
| `AddDevice(name, filename, flag)` | 通过 JSON 配置添加设备 |
| `AddDeviceByParams(name, port, baud)` | 通过参数添加设备 |
| `AddCANFDDevice(name)` | 添加 CANFD 模式设备 |
| `GetDevice(name)` | 获取设备指针 |
| `RemoveDevice(name)` | 移除设备 |
| `HasDevice(name)` | 检查设备是否存在 |
| `StartAll()` / `StopAll()` | 启动/停止全部设备 |
| `Clear()` | 清除所有设备 |
| `ListSerialPorts()` | 枚举系统串口 |

## 通信协议

### 帧格式

```
[H=0x48][M=0x4D][Len][Dst][Src][Cmd][Data...][CRC32-LE (4B)]
```

- 帧头: `0x48 0x4D` ("HM")
- 最小帧长: 10 字节 (header 2 + len 1 + dst 1 + src 1 + cmd 1 + crc 4)
- 最大数据段: 255 字节
- CRC: CRC-32/BZIP2 (Poly=0x04C11DB7, 无反射, 最终异或 0xFFFFFFFF)

### 地址分配

| 地址 | 设备 |
|------|------|
| `0xFE` | PC (上位机) |
| `0xFD` | Palm Board (掌板) |
| `0xFF` | Broadcast (广播) |
| `0x01–0x0B` | Joint 1–11 (关节板) |

### 双工作模式

| 模式 | 说明 | 掌板闭环 | 队列 | 空闲行为 |
|------|------|:---:|------|------|
| 实时调度 | 掌板运行 100Hz 闭环控制 | ✅ | `cmd_queue_` (FIFO) | 自动发送缓存的位置控制帧 |
| 透传 | 掌板暂停闭环，PC 直访关节 | ❌ | `passthrough_queue_` (FIFO) | 无操作 |

## 示例程序

| 示例 | 文件 | 说明 |
|------|------|------|
| 整手控制 | `hand_control_example.cpp` | 演示 11 关节同步位置控制与状态读取 |
| 实时关节控制 | `realtime_joint_control_example.cpp` | 单关节实时控制 |
| 定时插值运动 | `interpolated_joint_motion_example.cpp` | 单关节按指定时间插值到目标角度 |
| 归零与实时控制 | `homing_and_realtime_control_example.cpp` | 归零校准 + 实时位置控制完整流程 |
| 阻抗控制 | `impedance_control_example.cpp` | 演示阻抗控制模式 (Kp/Kd/前馈) |
| 阻抗正弦测试 | `impedance_position_sin_test.cpp` | 阻抗控制与位置正弦波叠加测试 |
| OTA 升级 | `ota_upgrade_example.cpp` | 演示关节/掌板固件一键升级 |
| 通讯状态监控 | `comm_status_test.cpp` | 监控接收计数与通讯健康状态 |
| 掌板运行模式 | `palm_runtime_mode_example.cpp` | 查询掌板当前运行模式 |
| CANFD 手部控制 | `canfd_socketcan_hand_control_example.cpp` | Linux SocketCAN + CANFD 控制 (仅 Linux) |

## 文档

| 文件 | 说明 |
|------|------|
| `docs/SG100_SDK_User_Guide.md` | 完整使用指南 (API 参考、快速上手、FAQ) |
| `docs/ERROR_CODE_GUIDE.md` | 故障快照、错误码、扩展错误回调与清除流程 |
| `docs/Architecture_Analysis.md` | 架构设计与重构记录 |
| `docs/Homing_Calibration_Flow.md` | 关节归零标定流程说明 |
| `docs/Position_Transform_Calculation.md` | 位置变换计算说明 |
| `joint_angle_map_v3.0.json` | 新版 L/R 11 关节角度映射、版本与限位配置 |
| `joint_angle_map.json` | 旧版单表角度映射（兼容格式） |

## 平台支持

| 平台 | 架构 | 标识 | 传输方式 |
|------|------|------|------|
| Windows (MSVC) | x86_64 | `win-msvc` | RS485 |
| Windows (MinGW) | x86_64 | `win-mingw32` | RS485 |
| Linux | x86_64 | `linux-x86_64` | RS485 / CANFD (SocketCAN) |
| Linux | ARM64 | `linux-arm64` | RS485 / CANFD (SocketCAN) |
| macOS | x86_64 | `macos-x86_64` | RS485 |
| macOS | ARM64 | `macos-arm64` | RS485 |

## 依赖

| 依赖 | 说明 |
|------|------|
| [nlohmann/json](https://github.com/nlohmann/json) | JSON 解析 (内嵌单头文件 `include/json.hpp`) |
| [wjwwood/serial](https://github.com/wjwwood/serial) | 跨平台串口库 (内嵌于 `include/serial/`) |
| pthread (Unix) / setupapi (Windows) | 系统级平台链接库 |
| IOKit + CoreFoundation (macOS) | macOS 串口枚举所需系统框架 |

## 发布包

构建项目后，`share/` 目录自动生成 SDK 发布包：

```
share/
├── include/          # 完整头文件
├── lib/<platform>/   # 动态库 + 导入库
├── examples/         # 示例源码
├── docs/             # 使用指南
├── joint_angle_map_v3.0.json
└── joint_angle_map.json
```

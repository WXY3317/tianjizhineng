# SG100 SDK 使用指南

## 目录

- [1. 概述](#1-概述)
- [2. 系统架构](#2-系统架构)
- [3. 环境要求与编译](#3-环境要求与编译)
- [4. 快速上手](#4-快速上手)
- [5. 核心概念](#5-核心概念)
- [6. API 参考](#6-api-参考)
- [7. 数据结构与类型](#7-数据结构与类型)
- [8. 协议说明](#8-协议说明)
- [9. OTA 固件升级](#9-ota-固件升级)
- [10. 示例程序](#10-示例程序)
- [11. 常见问题](#11-常见问题)

---

## 1. 概述

SG100 SDK 是用于控制 **SG100 灵巧手** 的 C++ 开发库，提供：

- **整手实时控制** — 11 个关节同时位置/速度/力矩控制 (100Hz)
- **单关节透传操作** — 直接访问关节板参数、标定、使能等
- **OTA 固件升级** — 一键远程更新关节板固件
- **多设备管理** — 同时管理多只灵巧手

通信链路：

```
PC (0xFE) ── UART ──→ 掌板 (0xFD) ── 4×RS485 ──→ 11个关节 (0x01~0x0B)
```

## 2. 系统架构

### 2.1 项目结构

```
SG100_SDK/
├── CMakeLists.txt          # 构建配置
├── serial_config.json      # 串口配置文件
├── joint_angle_map_v3.0.json  # 新版 L/R 运行时角度映射/限位配置
├── joint_angle_map.json        # 旧版单表格式（兼容）
├── include/                # 公共头文件
│   ├── sg100_device.h          # SG100Device 类
│   ├── sg100_device_manager.h  # SG100DeviceManager 单例
│   ├── sg100_types.h           # 协议常量、数据结构、量化工具
│   ├── crc.hpp                 # CRC-32 计算 (Boost)
│   ├── json.hpp                # JSON 解析 (nlohmann)
│   └── serial/                 # 跨平台串口库
├── src/                    # 实现源码
├── examples/               # 示例程序
│   ├── hand_control_example.cpp
│   ├── ota_upgrade_example.cpp
│   └── serial_config.json
├── firmware/               # 固件文件 (.bin/.pkg)
├── share/                  # SDK 发布包 (构建时自动生成)
│   ├── include/                # 公共头文件
│   ├── lib/<platform>/         # 编译好的动态库
│   ├── examples/               # 示例与配置
│   ├── sg100_angle_map.h       # 角度映射类型与校验状态
│   └── joint_angle_map_v3.0.json # 新版 L/R 角度映射配置
└── docs/                   # 文档
```

### 2.2 类层次

```
SG100DeviceManager (单例)
  └─ SG100Device ("HAND1")          ── 管理一条串口连接
       ├─ SerialTask (通信线程)      ── 10ms 周期收发
       ├─ cmd_queue_ (实时队列)      ── FIFO
       ├─ passthrough_queue_ (透传队列) ── FIFO
       └─ Hand_Status_t (状态缓存)   ── 11关节位置/速度/电流
```

### 2.3 双工作模式

| 模式 | 说明 | 队列 | 空闲行为 |
|------|------|------|----------|
| **实时调度** (`WORK_MODE_REALTIME`) | 掌板闭环运行，PC 发整手控制帧 | `cmd_queue_` (FIFO) | 队列为空时自动发送位置控制指令 (0x05)，数据来自 `cached_control_` 缓存 (初始为全关节 0° 位置模式) |
| **透传模式** (`WORK_MODE_PASSTHROUGH`) | 掌板闭环暂停，PC 直接访问关节板 | `passthrough_queue_` (FIFO) | 不发送 |

---

## 3. 环境要求与编译

### 3.1 环境要求

| 项目 | 要求 |
|------|------|
| **C++ 标准** | C++14 或更高 |
| **CMake** | ≥ 3.16 |
| **编译器** | MSVC 2019+, GCC 7+, MinGW-w64 |
| **操作系统** | Windows, Linux (x86_64 / ARM64), macOS |

### 3.2 编译步骤

**Windows (MSVC)**
```bash
cd SG100_SDK
mkdir build && cd build
cmake .. -G "Visual Studio 17 2022"
cmake --build . --config Release
```

**Windows (MinGW)**
```bash
cd SG100_SDK
mkdir build && cd build
cmake .. -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build .
```

**Linux / macOS**
```bash
cd SG100_SDK
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### 3.2.1 按需编译示例程序

示例程序不是发布 SDK 的必需产物，但在联调或回归测试时可以单独编译。

推荐方式：先完成一次正常配置，再显式构建 `build_sdk_examples` 目标。

**Windows (MSVC)**
```bash
cd SG100_SDK
cmake -S . -B build -G "Visual Studio 17 2022"
cmake --build build --config Release --target build_sdk_examples
```

**Windows (MinGW)**
```bash
cd SG100_SDK
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build --target build_sdk_examples
```

**Linux / macOS**
```bash
cd SG100_SDK
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc) --target build_sdk_examples
```

构建完成后，示例程序会输出到 `build/bin/` 目录，例如：

- `build/bin/hand_control_example`
- `build/bin/ota_upgrade_example`
- `build/bin/comm_status_test`
- `build/bin/homing_and_realtime_control_example`
- `build/bin/realtime_joint_control_example`

### 3.3 编译产物

| 产物 | 路径 |
|------|------|
| 动态库 | `build/Release/sg100_sdk.dll` (Windows) 或 `build/libsg100_sdk.so` (Linux) |
| 导入库 | `build/Release/sg100_sdk.lib` (MSVC) |
| 示例程序 | Windows: `build/bin/Release/*.exe`；Linux / macOS: `build/bin/*` |
| 发布包 | `share/` 目录 (自动打包) |

### 3.4 CMake 选项

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `BUILD_EXAMPLES` | `ON` | 生成示例目标；需要时可通过 `build_sdk_examples` 显式编译 |
| `BUILD_SHARED_LIBS` | `ON` | 构建动态库 |

### 3.5 使用发布包 (share)

如果不从源码编译，可直接使用 `share/` 目录中的预编译库：

```cmake
# 你的 CMakeLists.txt
cmake_minimum_required(VERSION 3.16)
project(my_app CXX)
set(CMAKE_CXX_STANDARD 14)

# 指向 SDK 发布包路径
set(SG100_SDK_DIR "/path/to/SG100_SDK/share")

include_directories(${SG100_SDK_DIR}/include)
link_directories(${SG100_SDK_DIR}/lib/win-msvc)  # 按平台选择

add_executable(my_app main.cpp)
target_link_libraries(my_app sg100_sdk)
```

运行时请确保动态库位于可执行文件同级目录或系统搜索路径中；若启用了角度映射配置化，还需同步部署 `joint_angle_map_v3.0.json`。启动前调用 `SetAngleMapHand(SG100AngleMapHand::LEFT/RIGHT)`，再调用 `ValidateAngleMap()`；新版文件存在但校验失败时 SDK 不会静默回退。没有新版文件时仍兼容旧版 `joint_angle_map.json`。

### 3.6 导出 Publish 发布目录

当前项目约定：**`SG100_SDK/` 是唯一源码基线**，`SG100_SDK_Publish_V1/` 这类目录仅作为从当前 SDK 编译并整理出来的发布包，不反向修改源码，也不以发布目录作为比对基准。

推荐导出流程如下：

1. 在当前 SDK 工程内完成目标平台编译。
2. 使用 `package_sdk` 自动生成 `share/` 中间目录。
3. 将 `share/` 中的头文件、示例、文档以及根目录下的 `joint_angle_map.json` 同步到发布目录。
4. 将对应平台的库文件同步到发布目录的 `lib/<platform>/`。
5. 不要把 `build/`、`build-mingw/`、日志文件等本地构建产物拷贝进发布目录。

> 说明：`package_sdk` 会打包 `share/` 所需内容，但不会强制编译测试用示例可执行文件；如果当前需要运行示例，请先执行 `build_sdk_examples`。

**Windows (MinGW) 导出示例**

```powershell
cd SG100_SDK
cmake -S . -B build-mingw -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build-mingw --config Release
```

执行完成后会自动生成：

- `share/include/`：完整头文件目录。
- `share/examples/`：完整示例与 `serial_config.json`。
- `share/docs/`：发布所需文档。
- `share/joint_angle_map_v3.0.json`：新版 L/R 运行时角度映射/限位配置。
- `share/joint_angle_map.json`：旧版兼容映射配置。
- `share/lib/win-mingw32/`：MinGW 平台动态库。

如果需要导出到 `SG100_SDK_Publish_V1/`，建议按下面的结构同步：

```text
SG100_SDK_Publish_V1/
├── CMakeLists.txt
├── README.md
├── joint_angle_map.json
├── docs/
├── examples/
├── include/
└── lib/
    ├── win-mingw32/
    └── win-msvc/
```

其中：

- `include/`、`examples/`、`docs/` 直接以 `share/` 中内容为准。
- 根目录的 `joint_angle_map.json` 也应直接使用 `share/joint_angle_map.json` 的导出结果。
- `lib/win-mingw32/` 可直接使用 `share/lib/win-mingw32/` 中的导出结果。
- `lib/win-msvc/` 需要在 MSVC 工具链下单独编译后补充对应 `dll/lib`。

**Windows (MSVC) 导出示例**

```powershell
cd SG100_SDK
cmake -S . -B build-msvc -G "Visual Studio 17 2022"
cmake --build build-msvc --config Release
```

MSVC 构建完成后，将对应的 `sg100_sdk.dll` 与 `sg100_sdk.lib` 同步到发布目录的 `lib/win-msvc/`。

> 说明：在部分 Windows + MinGW 环境下，首次 `cmake -S ... -B ...` 可能出现编译器探测异常；直接重新执行同一条配置命令通常即可恢复正常。

---

## 4. 快速上手

### 4.1 最简示例

```cpp
#include "sg100_device_manager.h"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    // 1. 获取设备管理器
    auto& manager = SG100DeviceManager::Instance();

    // 2. 添加设备 (方式一: 配置文件)
    manager.AddDevice("HAND1", "serial_config.json", 1);
    // 或 (方式二: 直接指定串口)
    // manager.AddDeviceByParams("HAND1", "COM3", 1000000);

    // 3. 获取设备指针
    auto* hand = manager.GetDevice("HAND1");

    // 4. 启动通信
    manager.StartAll();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 5. 控制 — 设置关节1到30度
    hand->SetJointPosition(1, 30.0f);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 6. 读取缓存状态
    float pos = hand->GetCachedPosition(1);
    std::cout << "Joint 1 position: " << pos << " deg" << std::endl;

    // 7. 清理
    manager.StopAll();
    manager.Clear();
    return 0;
}
```

### 4.2 串口配置文件

`serial_config.json` 格式：

```json
{
    "serial_port": [
        {
            "port": "COM3",
            "baudrate": 1000000,
            "databyte": 8,
            "parity": 0,
            "stopbyte": 1,
            "flowcontrol": 0,
            "timeout_ms": 100
        }
    ]
}
```

`AddDevice("HAND1", "serial_config.json", 1)` 中第 3 个参数 `flag` 表示使用配置数组的第几个条目 (1-based)。

### 4.3 自动探测串口

```cpp
auto ports = SG100DeviceManager::ListSerialPorts();
for (auto& p : ports) {
    std::cout << p.port << " - " << p.description << std::endl;
}
// 选择第一个可用串口
manager.AddDeviceByParams("HAND1", ports[0].port, 1000000);
```

---

## 5. 核心概念

### 5.1 关节编号与手指映射

SG100 灵巧手有 **11 个关节** (ID 1~11)，分布在 4 根手指上：

| 手指 | 关节 ID | 关节数 |
|------|---------|--------|
| 拇指 (Thumb) | 1, 2, 3 | 3 |
| 食指 (Index) | 4, 5, 6 | 3 |
| 中指 (Middle) | 7, 8 | 2 |
| 无名指 (Ring) | 9, 10, 11 | 3 |

代码中可使用 `SG100_FingerMap` 命名空间的映射表：

```cpp
using namespace SG100_FingerMap;
// FINGER_JOINTS[THUMB] = {1, 2, 3}
// FINGER_JOINT_COUNT[THUMB] = 3
```

### 5.2 控制模式

```cpp
enum CtrlMode_e : uint8_t {
    MODE_DUTY       = 0,    // 占空比 (开环)
    MODE_CURRENT    = 1,    // 电流闭环
    MODE_VELOCITY   = 2,    // 速度闭环
    MODE_POSITION   = 3,    // 位置闭环 (最常用)
    MODE_VEL_CUR    = 4,    // 速度+电流
    MODE_POS_CUR    = 5,    // 位置+电流
    MODE_POS_VEL_CUR= 6,    // 位置+速度+电流
};
```

### 5.3 量化编解码

所有控制量和状态量在协议中以 `uint16_t` 传输，SDK 提供量化工具自动转换：

| 物理量 | 范围 | 编码函数 | 解码函数 |
|--------|------|----------|----------|
| 位置 (°) | ±180 | `SG100_Quant::encodePos(deg)` | `SG100_Quant::decodePos(raw)` |
| 速度 | ±1000 | `SG100_Quant::encodeVel(vel)` | `SG100_Quant::decodeVel(raw)` |
| 力矩 | ±25.0 | `SG100_Quant::encodeTorque(t)` | `SG100_Quant::decodeTorque(raw)` |
| 电流 | ±1000 | `SG100_Quant::encodeCurrent(c)` | `SG100_Quant::decodeCurrent(raw)` |
| Kp / Ki / Kd | ±1.0 | `encodeKp()` / `encodeKi()` / `encodeKd()` | - |

### 5.4 关节角度限位

SDK 内部对各关节有硬编码限位保护：

| 关节 ID | 角度范围 |
|---------|----------|
| 1 | -90° ~ +90° |
| 4 | 0° ~ +90° |
| 9 | -90° ~ 0° |
| 其他 | -10° ~ +100° |

### 5.5 位置字段内部变换 (`transform on/off`)

SDK 现在支持一个**位置字段内部变换开关**：

- `transform on` 或 `SetPositionTransformEnabled(true)`：启用内部位置变换
- `transform off` 或 `SetPositionTransformEnabled(false)`：关闭内部位置变换，直接收发协议中的原始 `float position`

#### 5.5.1 `transform on` 时发送链路做了什么

当调用 `SetJointPosition()`、`SetJointPositions()`、`SetHandControl()`，或者实时线程从 `cached_control_` 自动重发位置控制帧时，SDK 会对每个关节的 `position` 执行下面两步变换：

1. **用户角度限幅 + 关节映射**

    先对用户输入角度做限幅，然后按下式转换为“手侧实际角度”：

    $$
    actual\_deg = a_i \cdot clamp(user\_deg) + b_i
    $$

2. **实际角度 -> 协议原始 `float position`（弧度）**

    再将手侧实际角度从度转换为协议里的 `position` 字段（rad）：

    $$
    wire\_position = actual\_deg \cdot \frac{\pi}{180}
    $$

#### 5.5.2 `transform on` 时接收链路做了什么

收到掌板整手状态、单关节透传状态后，SDK 会把协议中的原始 `position` 反向解码回调用方看到的值：

1. **协议原始值（弧度） -> 手侧实际角度**

    $$
    actual\_deg = wire\_position \cdot \frac{180}{\pi}
    $$

2. **手侧实际角度 -> 用户角度**

    $$
    user\_deg = \frac{actual\_deg - b_i}{a_i}
    $$

所以在 `transform on` 时：

- `SetJointPosition(1, 30.0f)` 里的 `30.0f` 是**用户角度（度）**
- `GetCachedPosition()`、`GetCachedPositions()`、`RegisterHandStatusCallback()` 看到的 `position` 也是**用户角度（度）**

#### 5.5.3 每个关节的映射参数

| 关节 | 用户角度限幅 | 实际映射公式 |
|------|--------------|--------------|
| J1 | `[-30, 90]` | `actual = user - 90` |
| J2 | `[-10, 100]` | `actual = -user - 10` |
| J3 | `[-16, 110]` | `actual = -user - 16` |
| J4 | `[-90, 10]` | `actual = user + 90` |
| J5 | `[-10, 100]` | `actual = -user - 10` |
| J6 | `[-16, 110]` | `actual = -user - 16` |
| J7 | `[-10, 100]` | `actual = -user - 10` |
| J8 | `[-16, 110]` | `actual = -user - 16` |
| J9 | `[0, 180]` | `actual = -user` |
| J10 | `[-10, 100]` | `actual = -user - 10` |
| J11 | `[-16, 110]` | `actual = -user - 16` |
| J12 | `[-90, 90]` | `actual = user` |

#### 5.5.4 `transform off` 时的行为

关闭内部位置变换后，SDK 不再对 `position` 做上述编码/解码：

- 发送时：调用方传入什么 `float`，协议里就发什么 `float`
- 接收时：协议里收到什么 `float`，缓存和状态回调里就看到什么 `float`

这对调试底层协议、直接下发 raw 值、或对接已有上位机标定逻辑更方便。

---

## 6. API 参考

### 6.1 SG100DeviceManager (设备管理器)

全局单例，负责设备生命周期管理。

```cpp
#include "sg100_device_manager.h"

auto& manager = SG100DeviceManager::Instance();
```

| 方法 | 说明 |
|------|------|
| `static Instance()` | 获取单例引用 |
| `static ListSerialPorts()` | 列举系统可用串口 → `vector<SerialPortInfo>` |
| `AddDevice(name, config_file, flag)` | 从 JSON 配置文件添加设备 |
| `AddDeviceByParams(name, port, baudrate)` | 直接参数添加设备 |
| `RemoveDevice(name)` | 移除设备 |
| `GetDevice(name)` | 获取设备指针 → `SG100Device*` |
| `HasDevice(name)` | 检查设备是否存在 |
| `StartAll()` | 启动所有设备通信线程 |
| `StopAll()` | 停止所有设备 |
| `GetDeviceCount()` | 获取设备数量 |
| `GetDeviceNames()` | 获取所有设备名称列表 |
| `Clear()` | 停止并移除所有设备 |

### 6.2 SG100Device (设备类)

每个实例代表一只灵巧手的串口连接。通过 Pimpl 模式实现，线程安全。

#### 6.2.1 生命周期

| 方法 | 说明 |
|------|------|
| `GetConfig(filename, flag)` | 从 JSON 读取串口配置 |
| `SetConfig(config)` | 直接设置 `SG100Config` |
| `Init()` | 打开串口 |
| `Start()` | 启动通信线程 |
| `Stop()` | 停止线程并关闭串口 |
| `IsRunning()` | 检查是否运行中 |

> 通常不需要手动调用这些方法，`SG100DeviceManager::StartAll()` 会自动完成 `Init()` + `Start()`。

#### 6.2.2 工作模式切换

| 方法 | 说明 |
|------|------|
| `GetWorkMode()` | 返回 `WORK_MODE_REALTIME` 或 `WORK_MODE_PASSTHROUGH` |
| `SwitchToPassthroughMode()` | 暂停掌板闭环 → 进入透传模式 |
| `SwitchToRealtimeMode()` | 恢复掌板闭环 → 回到实时模式 |
| `SetPositionTransformEnabled(bool)` | 开启/关闭位置字段内部变换 |
| `IsPositionTransformEnabled()` | 查询位置字段内部变换是否开启 |

```cpp
// 切换到透传模式执行参数操作
hand->SwitchToPassthroughMode();
hand->GetPIDParam(1, 0);      // 读取关节1位置PID
hand->EnableJoint(1);          // 使能关节1
hand->GetJointLimits(1);       // 读取关节1限位
hand->SetDeviceInfo(1, 100, "motor_1");  // 设置设备信息
hand->SwitchToRealtimeMode();  // 恢复实时控制
```

#### 6.2.3 整手控制 (实时模式)

| 方法 | 说明 |
|------|------|
| `SetHandControl(ctrl)` | 发送整手控制数据 (`Hand_Control_t`, 11 关节) |
| `SetJointPositions(positions_deg, count)` | 批量设置关节位置 (度) |
| `SetJointPosition(joint_id, deg)` | 设置单个关节位置 |
| `RequestHandStatus()` | 主动请求整手状态 |

> 默认情况下，位置接口按“用户角度（度）”工作。
> 如果调用 `SetPositionTransformEnabled(false)`，则这些接口中的 `position` 会改为直接使用协议原始 `float` 值。


```cpp
// 简单位置控制
hand->SetJointPosition(1, 30.0f);

// 批量设置
float positions[11] = {0, 10, 20, 30, 40, 50, 0, 0, 0, 0, 0};
hand->SetJointPositions(positions, 11);

// 精细控制 (含PID参数)
Hand_Control_t ctrl = {};
for (int i = 0; i < JOINT_COUNT; i++) {
    ctrl.joints[i].id = i + 1;
    ctrl.joints[i].mode = MODE_POSITION;
    ctrl.joints[i].position = 0.0f;
    ctrl.joints[i].p = SG100_Quant::encodePid(0.5f);
    ctrl.joints[i].i = SG100_Quant::encodePid(0.01f);
    ctrl.joints[i].d = SG100_Quant::encodePid(0.05f);
    ctrl.joints[i].output_limit = SG100_Quant::encodeOutLim(1.0f);
}
hand->SetHandControl(ctrl);

// 说明:
// - transform on (默认) 时，position 按用户角度(度)传入，SDK 内部会换算到协议 raw float。
// - transform off 时，position 直接按协议原始 float 传入。
```

#### 6.2.4 透传模式操作

> **重要**: 以下所有方法均仅在 **透传模式** 下可用。调用前必须先执行 `SwitchToPassthroughMode()`，否则命令会被拒绝并返回 `false`。

**关节控制与状态**

| 方法 | 说明 |
|------|------|
| `SendPassthroughCommand(dst, cmd, data, len)` | 发送自定义透传命令 |
| `GetJointStatus(joint_id)` | 查询单关节状态 |
| `SetJointCommand(joint_id, ctrl)` | 发送单关节控制 |
| `EnableJoint(joint_id)` | 使能关节 |
| `DisableJoint(joint_id)` | 禁用关节 |
| `GetJointState(joint_id)` | 查询关节运行状态 |

**设备与关节配置**

| 方法 | 说明 |
|------|------|
| `SetDeviceInfo(joint_id, id, name)` | 设置设备信息 |
| `GetDeviceInfo(joint_id)` | 读取设备信息 |
| `SetJointLimits(...)` | 设置关节限位 |
| `GetJointLimits(joint_id)` | 读取关节限位 |
| `SetJointInfo(joint_id, new_id, name)` | 设置关节身份 |
| `GetJointInfo(joint_id)` | 读取关节身份 |

**掌板Flash参数**

| 方法 | 说明 |
|------|------|
| `ReadPalmFlash()` | 读取掌板Flash参数 (CMD 0x10) |
| `WritePalmFlash(data, size)` | 写入掌板Flash参数 (CMD 0x11) |

```cpp
// 读写掌板Flash参数示例
hand->SwitchToPassthroughMode();
hand->ReadPalmFlash();  // 读取掌板参数
// ... 修改参数后写回
// hand->WritePalmFlash(param_data, param_size);
hand->SwitchToRealtimeMode();
```

#### 6.2.5 参数管理 (透传模式)

| 方法 | 说明 |
|------|------|
| `GetPIDParam(joint_id, which)` | 读取 PID 参数 (0=位置, 1=电流) |
| `SetPIDParam(joint_id, which, pid)` | 写入 PID 参数 |
| `SaveParam(joint_id)` | 保存参数到 Flash |
| `RestoreParam(joint_id)` | 恢复出厂参数 |
| `WriteDefault(joint_id)` | 写入默认参数 |
| `GetParamAll(joint_id)` | 读取全部参数 |
| `SetParamAll(joint_id, data, size)` | 写入全部参数 |

```cpp
// PID参数调节示例
hand->SwitchToPassthroughMode();

PidParam_t pid;
pid.kp = 0.5f;    pid.ki = 0.01f;   pid.kd = 0.05f;
pid.fc = 100.0f;  pid.out_min = -1.0f; pid.out_max = 1.0f;
hand->SetPIDParam(1, 0, pid);   // 设置关节1位置PID
hand->SaveParam(1);              // 保存到Flash

hand->SwitchToRealtimeMode();
```

#### 6.2.6 OTA 固件升级

| 方法 | 说明 |
|------|------|
| `PackFirmware(bin, pkg, version)` *(static)* | 将 BIN 文件打包为 PKG |
| `OtaUpgradeJoint(joint_id, pkg)` | 一键升级关节板固件 |
| `OtaUpgradeJoint(joint_id, pkg, callback)` | 升级关节板并接收结构化进度回调 |
| `OtaUpgradePalm(pkg, callback)` | 升级掌板并接收结构化进度回调 |

详细说明参见 [第 9 节](#9-ota-固件升级)。

#### 6.2.7 状态缓存

| 方法 | 说明 |
|------|------|
| `GetCachedHandStatus()` | 获取整手状态副本 → `Hand_Status_t` (值拷贝, 线程安全) |
| `GetCachedPosition(joint_id)` | 获取关节位置；`transform on` 时为角度(度)，`transform off` 时为 raw float |
| `GetCachedPositions(out_array)` | 获取全部关节位置；单位随位置字段内部变换开关变化 |
| `GetCachedCurrent(joint_id)` | 获取关节电流 |
| `GetCachedVelocity(joint_id)` | 获取关节速度 |

> 缓存数据由通信线程自动更新 (100Hz)，调用时无需等待。
> `GetCachedHandStatus()` 返回值拷贝 (非指针)，使用时直接用 `.` 访问成员。

```cpp
auto status = hand->GetCachedHandStatus();
float pos = status.joints[0].position;
```

#### 6.2.8 回调注册

```cpp
// 整手状态更新回调 (每次收到掌板响应时触发)
hand->RegisterHandStatusCallback([](const Hand_Status_t& status) {
    for (int i = 0; i < JOINT_COUNT; i++) {
        float pos = SG100_Quant::decodePos(status.joints[i].pos);
        // 处理状态数据...
    }
});

// 错误回调
hand->RegisterErrorCallback([](uint8_t joint_id, uint8_t error_code) {
    std::cerr << "Joint " << (int)joint_id 
              << " error: " << (int)error_code << std::endl;
});
```

#### 6.2.9 设备信息

| 方法 | 说明 |
|------|------|
| `GetDeviceName()` | 返回设备名称 |
| `GetPort()` | 返回串口名称 |

---

## 7. 数据结构与类型

### 7.1 控制结构

```cpp
// 单关节控制 (16 字节, 量化后)
typedef struct {
    uint8_t  id;        // 关节ID (1-11)
    uint8_t  mode;      // CtrlMode_e
    uint16_t pos;       // 位置 (encodePos)
    uint16_t vel;       // 速度 (encodeVel)
    uint16_t kp, ki, kd; // PID增益
    uint16_t torque;    // 力矩
    uint16_t current;   // 电流
} Joint_Control_t;      // 16 bytes

// 整手控制 = 11 × Joint_Control_t
typedef struct {
    Joint_Control_t joints[JOINT_COUNT];
} Hand_Control_t;       // 176 bytes
```

### 7.2 状态结构

```cpp
// 单关节状态 (10 字节, 量化)
typedef struct {
    uint8_t  id;
    uint8_t  mode;
    uint16_t pos, vel, torque, current;
} Joint_Status_t;       // 10 bytes

// 整手状态
typedef struct {
    Joint_Status_t joints[JOINT_COUNT];
} Hand_Status_t;        // 110 bytes
```

### 7.3 PID 参数

```cpp
typedef struct {
    float kp, ki, kd;           // PID增益
    float fc;                   // 截止频率
    float out_min, out_max;     // 输出限幅
} PidParam_t;                   // 24 bytes
```

### 7.4 串口配置

```cpp
struct SG100Config {
    std::string port;           // "COM3" 或 "/dev/ttyUSB0"
    unsigned long baudrate;     // 默认 1000000
    int databyte;               // 默认 8
    int parity;                 // 0=none, 1=odd, 2=even
    int stopbyte;               // 默认 1
    int flowcontrol;            // 0=none, 1=sw, 2=hw
    uint32_t timeout_ms;        // 默认 100
};
```

### 7.5 枚举类型

| 枚举 | 值 | 说明 |
|------|-----|------|
| `Cmd_Result_t` | OK=0, ERROR=1, INVALID_PARAM=2, NOT_SUPPORTED=3, CRC_ERROR=4, SEQ_ERROR=5, TIMEOUT=6 | 命令响应码 |
| `JointState_e` | IDLE=0, RUNNING=1, ERROR=2, CALIBRATING=3, HOMING=4 | 关节状态 |
| `SG100_WorkMode_e` | REALTIME=0, PASSTHROUGH=1 | SDK 工作模式 |
| `OTA_EndResult_e` | OK=0x00, CRC_FAIL=0x04, LENGTH_MISMATCH=0x07, SHA256_FAIL=0x08 | OTA 结果码 |
| `CtrlMode_e` | DUTY=0, CURRENT=1, VELOCITY=2, POSITION=3, ... | 控制模式 |

---

## 8. 协议说明

### 8.1 帧格式

```
[Header0][Header1][Length][Dst][Src][Cmd][Data...][CRC32-LE]
  0x48     0x4D     1B     1B   1B   1B   N bytes   4 bytes
```

| 字段 | 大小 | 说明 |
|------|------|------|
| Header | 2B | 固定 `0x48 0x4D` ("HM") |
| Length | 1B | Data 长度 (不含 Header/Len/Dst/Src/Cmd/CRC) |
| Dst | 1B | 目标地址 |
| Src | 1B | 源地址 |
| Cmd | 1B | 命令码 |
| Data | 0~256B | 命令数据 |
| CRC32 | 4B | CRC-32/BZIP2，小端序 |

### 8.2 CRC 算法

- **多项式**: 0x04C11DB7
- **初始值**: 0xFFFFFFFF
- **RefIn / RefOut**: false
- **XorOut**: 0xFFFFFFFF
- 校验范围: 从 Header0 到 Data 末尾 (不含 CRC 字段)

### 8.3 响应规则

- 响应帧 Cmd = 请求 Cmd | `0x80`
- Data[0] 为结果码 (`Cmd_Result_t`)

### 8.4 命令队列机制

SDK 使用 **纯 FIFO (先入先出)** 队列，无优先级排序。所有命令以追加到队尾的方式入队，从队头依次取出发送。

| 队列 | 类型 | 说明 |
|------|------|------|
| `cmd_queue_` | FIFO | 实时模式命令队列。队列为空时自动从 `cached_control_` 生成 0x05 位置控制帧 |
| `passthrough_queue_` | FIFO | 透传模式命令队列。队列为空时不发送任何数据 |

> **注意**: 头文件中仍保留了 `PRIORITY_PALM_*` / `PRIORITY_JOINT_*` 宏定义以保持向后兼容，但代码中所有命令的 `priority` 字段均设为 0，队列不进行排序。

---

## 9. OTA 固件升级

### 9.1 概述

OTA (Over-The-Air) 升级支持远程更新关节板固件，无需物理拆装。SDK 提供 **一键升级** 接口，自动完成完整流程。

### 9.2 固件包格式

原始 BIN 文件需先打包为 PKG 格式：

```
[Header: 20B][Firmware Data: N bytes]
```

| 偏移 | 大小 | 说明 |
|------|------|------|
| 0-3 | 4B | 魔数 (0x4A31AA55) |
| 4-7 | 4B | 版本号 |
| 8-11 | 4B | 固件大小 |
| 12-15 | 4B | CRC32 校验值 |
| 16-19 | 4B | 保留 |
| 20+ | NB | 固件二进制数据 |

### 9.3 打包固件

```cpp
// 将 BIN 打包为 PKG (静态方法，无需设备连接)
bool ok = SG100Device::PackFirmware(
    "firmware/SG100_Joint.bin",    // 输入: BIN文件
    "firmware/SG100_Joint.pkg",    // 输出: PKG文件
    0x00020000                      // 版本号
);
```

### 9.4 执行升级

```cpp
// 一键升级 (自动完成全部流程)
bool success = hand->OtaUpgradeJoint(
    7,                              // 关节ID
    "firmware/SG100_Joint.pkg"      // PKG文件路径
);
```

需要在上位机显示进度时，可使用带 `OTAProgressCallback` 的重载：

```cpp
OTAProgressCallback progress = [](const OTA_Progress_t& p) {
    // p.stage: 当前阶段；0 表示掌板，1..11 表示关节板
    // p.percent: 固件传输百分比；校验和重启阶段保持 100
    std::cout << static_cast<int>(p.stage) << " "
              << p.current_chunk << "/" << p.total_chunks << " "
              << p.bytes_sent << "/" << p.total_bytes << " "
              << p.percent << "%" << std::endl;
};

bool success = hand->OtaUpgradeJoint(7, "firmware/SG100_Joint.pkg", progress);
```

回调由 OTA 工作线程同步调用，应快速返回。不要在回调中再次调用运动控制或
工作模式切换接口。原有不带回调的接口保持兼容。

### 9.5 升级流程 (内部自动执行)

```
1.  切换到透传模式 (SwitchToPassthroughMode)
2.  发送 0xBB → 关节板跳转到 Bootloader
3.  等待 MCU 重启 (2秒)
4.  发送 0x86 → 查询升级状态
5.  发送 0x7F → Bootloader 握手
6.  发送 0x82 → 启动下载 (含版本/大小/CRC)
7.  发送 0x83 → 固件分片 (128B/片, 失败重试3次)
8.  发送 0x84 → 结束下载 (设备执行 CRC+SHA256 校验)
9.  等待 MCU 重启到新固件 (2秒)
10. 查询关节信息确认恢复
11. 切换回实时模式 (SwitchToRealtimeMode)
```

### 9.6 关键参数

| 参数 | 值 |
|------|----|
| 分片大小 | 128 字节 |
| 单片最大重试 | 3 次 |
| MCU 重启等待 | 2000 ms |
| Flash 擦除超时 | 5000 ms |
| 分片传输超时 | 2500 ms |

### 9.7 注意事项

- 升级期间 **请勿断电或断开连接**
- 升级耗时约 10~15 秒 (取决于固件大小和通信质量)
- 升级失败后关节板将保留在 Bootloader 模式，可重新升级
- 一次只能升级一个关节

---

## 10. 示例程序

### 10.1 hand_control_example

演示整手控制的完整流程：

- **Demo 1**: 整手精细控制 — 使用 `Hand_Control_t` 设置 11 关节的位置+PID，执行正弦波运动
- **Demo 2**: 逐指控制 — 利用 `SG100_FingerMap` 逐根手指弯曲
- **Demo 3**: 状态读取 — 从缓存获取并打印关节状态

### 10.2 ota_upgrade_example

演示 OTA 升级的完整流程：

- 修改文件顶部的用户配置区即可使用：
  ```cpp
  const int         TARGET_JOINT_ID   = 7;
  const std::string BIN_FIRMWARE_PATH = "C:/path/to/SG100_Joint.bin";
  const std::string SERIAL_PORT       = "COM6";
  const uint32_t    FIRMWARE_VERSION  = 0x00020000;
  ```
- 自动完成打包 → 连接 → 升级 → 验证全流程

---

## 11. 常见问题

### Q: 串口无法打开？

- 确认串口未被其他程序占用
- 检查设备管理器中的串口号是否正确
- Windows 下需安装对应的 USB 转串口驱动

### Q: 连接成功但没有状态返回？

- 确认波特率设置正确 (默认 1000000 bps)
- 检查硬件接线 (TX/RX 是否交叉)
- 启动后需等待约 1 秒让通信稳定

### Q: 设置位置但关节不动？

- 确认工作在实时模式 (`WORK_MODE_REALTIME`)
- 确认关节已使能 (透传模式下 `EnableJoint`)
- 检查目标角度是否在允许范围内

### Q: OTA 升级失败？

- 检查固件文件路径是否正确 (Windows 路径用 `/` 或 `\\\\`)
- 检查串口连接稳定性
- 确认目标关节 ID 正确
- 查看错误回调输出的错误码

### Q: 如何同时控制多只手？

```cpp
auto& mgr = SG100DeviceManager::Instance();
mgr.AddDeviceByParams("LEFT_HAND",  "COM3", 1000000);
mgr.AddDeviceByParams("RIGHT_HAND", "COM4", 1000000);
mgr.StartAll();

auto* left  = mgr.GetDevice("LEFT_HAND");
auto* right = mgr.GetDevice("RIGHT_HAND");
// 分别控制...
```

### Q: 路径中反斜杠问题？

C++ 字符串中的 `\` 是转义符。在路径中使用 **正斜杠** `/` 或 **双反斜杠** `\\`：

```cpp
// 正确
"C:/work/firmware/SG100_Joint.bin"
"C:\\work\\firmware\\SG100_Joint.bin"

// 错误 — \f 和 \S 会被当作转义字符
"C:\work\firmware\SG100_Joint.bin"
```

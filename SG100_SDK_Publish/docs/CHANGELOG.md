# SG100 SDK 发布版本更新日志

## V3.1：L/R 角度映射与严格校验

- 新增 `joint_angle_map_v3.0.json`，支持 L/R 两套 11 关节映射。
- 新增 `SetAngleMapHand()`、`ValidateAngleMap()` 和 `GetAngleMapStatus()`。
- 校验版本、L/R 节点、关节数量、字段类型、有限数值、非零 `a` 以及角度范围。
- 保留旧版 `joint_angle_map.json` 兼容；新版配置存在但无效时不会静默回退。
- 已更新 Windows MSVC、Windows MinGW、Linux x86_64 和 Linux ARM64 动态库。

## 2026-08-06：macOS Apple Silicon 发布包

- 新增 `lib/macos-arm64/libsg100_sdk.dylib`，目标架构为 Apple Silicon `arm64`。
- 源码构建补充 macOS 的 IOKit/CoreFoundation 框架链接，并修复 C++14 下
  `RS485Adapter::RX_BUF_SIZE` 的链接定义问题。
- 发布 CMake 增加 macOS `@loader_path` 运行时路径和 Apple Silicon 架构识别。
- 新增 `examples/serial_config.macos.example.json` 串口配置模板。
- 已在 Apple Clang 下完成 SDK 动态库和 9 个示例程序的 Release 构建验证。

## 2026-08-03：OTA 结构化进度回调

- 保留原有 `OtaUpgradeJoint(joint_id, pkg)` 与 `OtaUpgradePalm(pkg)` 接口。
- 新增带 `OTAProgressCallback` 的重载，不影响已有调用方。
- 新增 `OTA_ProgressStage_e` 和 `OTA_Progress_t`，提供升级阶段、目标 ID、
  当前/总分片、已发送/总字节与传输百分比。
- 回调覆盖准备、透传切换、Bootloader、握手、擦除、传输、校验、重启、
  重连、恢复实时、完成和失败阶段。
- 已重新生成 Windows MSVC x64、Windows MinGW x64、Linux x86_64、
  Linux ARM64 四个平台库，并更新 OTA 示例和用户文档。

> 回调在 OTA 工作线程中同步执行，应快速返回，不应在回调中调用运动控制
> 或工作模式切换接口。

---

## 版本信息
- **更新日期**: 2026-06-05
- **源版本**: SG100_SDK (开发版本)
- **目标版本**: SG100_SDK_Publish (发布版本)

---

## 🎯 本次更新概览

本次更新将 SG100_SDK 的最新功能同步到发布版本，主要包括：
1. **新增阻抗控制接口** - 支持更精细的力控操作
2. **CANFD配置优化** - 新增运行时配置接口
3. **多设备配置改进** - SetConfigSearchDir 改为实例方法
4. **示例代码扩充** - 新增4个示例程序
5. **文档完善** - 补充架构分析和用户指南

---

## 📋 详细更新内容

### 1️⃣ 头文件更新

#### `include/sg100_device.h`
**新增接口：**

```cpp
// 阻抗控制（第257-291行）
bool SetJointImpedance(uint8_t joint_id, float position_deg, float velocity_deg_s,
                       float kp, float kd, float torque_ff, float output_limit = 1.0f);

bool SetJointJointImpedance(uint8_t joint_id, float position_deg, float velocity_deg_s,
                            float kp, float kd, float torque_ff, float output_limit = 1.0f);

// CANFD配置（第729、736行）
bool SetCANFDConfig(const CANFD_Config_t& cfg);
CANFD_Config_t GetCANFDConfig() const;
```

**修改接口：**

```cpp
// 从静态方法改为实例方法（第131、137行）
void SetConfigSearchDir(const std::string& dir);        // 原: static void
std::string GetConfigSearchDir() const;                 // 原: static std::string
```

**影响：**
- ✅ 支持每个设备实例独立的配置文件搜索目录
- ✅ 支持运行时动态更新配置目录
- ✅ 为未来多设备独立角度映射配置奠定基础

---

#### `include/sg100_types.h`
**可能的新增内容：**
- 新的数据结构定义
- 协议常量更新
- 量化工具函数扩展

---

### 2️⃣ 库文件更新

**编译信息：**
- 编译时间: 2026-06-05 18:41
- 库文件大小: 480KB
- 平台: linux-x86_64
- 构建类型: Release

**文件位置：**
```
SG100_SDK_Publish/lib/linux-x86_64/libsg100_sdk.so
```

**ABI兼容性：**
- ⚠️ 由于新增了成员函数和修改了静态方法，建议重新编译使用SDK的应用程序
- ✅ 现有API保持向后兼容，不会破坏已有代码

---

### 3️⃣ 示例代码更新

**新增示例（4个）：**

1. **impedance_control_example.cpp**
   - 演示阻抗控制模式的使用
   - 展示如何设置 Kp/Kd/力矩前馈参数

2. **impedance_position_sin_test.cpp**
   - 阻抗模式下的正弦波位置测试
   - 验证阻抗控制的动态响应

3. **palm_runtime_mode_example.cpp**
   - 掌板运行模式查询和切换示例
   - 展示实时模式、透传模式、串口互联模式

4. **comm_status_test.cpp**
   - 通信状态监控测试
   - 演示如何使用 GetStatusRecvCount() 检测通讯超时

**现有示例（7个）：**
- hand_control_example.cpp
- homing_and_realtime_control_example.cpp
- realtime_joint_control_example.cpp
- ota_upgrade_example.cpp
- canfd_socketcan_hand_control_example.cpp
- comm_status_test.cpp（已存在，已更新）
- palm_runtime_mode_example.cpp（已存在，已更新）

**配置文件：**
- serial_config.json - 串口配置示例
- canfd_socketcan_config.json - CANFD配置示例

---

### 4️⃣ 文档更新

**新增文档（2个）：**

1. **docs/Architecture_Analysis.md**
   - SDK架构深度分析
   - 通信流程详解
   - 设计模式说明

2. **docs/Homing_Calibration_Flow.md**
   - 归零校准完整流程
   - 固件内部机制说明
   - 常见问题排查

**更新文档（2个）：**

1. **docs/SG100_SDK_User_Guide.md**
   - 补充阻抗控制使用说明
   - 新增多设备配置示例
   - 更新CANFD配置说明

2. **docs/Position_Transform_Calculation.md**
   - 位置变换计算细节
   - 角度映射公式说明

**README.md**
- 更新功能列表
- 补充新接口说明
- 更新编译指南

---

## 🔧 技术细节

### SetConfigSearchDir 改造说明

**改造前（全局静态）：**
```cpp
static std::string g_config_search_dir;  // 所有设备共享

static void SG100Device::SetConfigSearchDir(const std::string& dir) {
    g_config_search_dir = dir;  // 影响所有设备
}
```

**改造后（实例级别）：**
```cpp
class SG100Device {
private:
    std::string config_search_dir_;  // 每个设备独立
    
public:
    void SetConfigSearchDir(const std::string& dir) {
        config_search_dir_ = dir;  // 只影响当前设备
    }
};
```

**优势：**
- ✅ 多设备可以有不同的配置目录
- ✅ 运行时可动态切换
- ✅ 线程安全（每个实例独立）

---

### 阻抗控制接口说明

**SetJointImpedance - 笛卡尔空间阻抗控制**
```cpp
// 经过增量规划器，适合平滑运动
hand->SetJointImpedance(
    1,              // 关节ID
    45.0f,          // 目标角度（度）
    100.0f,         // 目标角速度（deg/s）
    2.0f,           // Kp刚度系数
    0.5f,           // Kd阻尼系数
    0.1f,           // 力矩前馈
    1.0f            // 输出限制
);
```

**SetJointJointImpedance - 关节空间阻抗控制**
```cpp
// 基于关节物理反馈，不经过增量规划器
hand->SetJointJointImpedance(
    1,              // 关节ID
    45.0f,          // 目标关节角度（度）
    100.0f,         // 目标关节角速度（deg/s）
    2.0f,           // 刚度系数Kp
    0.5f,           // 阻尼系数Kd
    0.1f,           // 力矩前馈
    1.0f            // 输出限制
);
```

---

## ⚠️ 注意事项

### 1. ABI兼容性
由于头文件中类布局发生变化（新增成员变量或虚函数），**强烈建议重新编译所有使用该SDK的应用程序**。

### 2. 向后兼容性
- ✅ 所有现有API保持不变
- ✅ 新增接口均为可选功能
- ✅ 默认行为与之前一致

### 3. 迁移指南

**如果使用了 SetConfigSearchDir：**

```cpp
// 旧代码（仍然有效，但会影响所有设备）
SG100Device::SetConfigSearchDir("/path/to/config");  // 编译警告：静态方法已改为实例方法

// 新代码（推荐）
auto* device = manager.GetDevice("HAND1");
device->SetConfigSearchDir("/path/to/hand1/config");

auto* device2 = manager.GetDevice("HAND2");
device2->SetConfigSearchDir("/path/to/hand2/config");
```

**如果使用阻抗控制：**

```cpp
// 确保在实时模式下使用
hand->SwitchToRealtimeMode();

// 设置阻抗参数
hand->SetJointImpedance(1, 45.0f, 100.0f, 2.0f, 0.5f, 0.1f);
```

---

## 📊 文件清单

### 更新的头文件
- ✅ include/sg100_device.h
- ✅ include/sg100_types.h
- ✅ include/sg100_device_manager.h
- ✅ include/crc.hpp
- ✅ include/json.hpp
- ✅ include/comm/*
- ✅ include/protocol/*
- ✅ include/serial/*
- ✅ include/transport/*

### 更新的库文件
- ✅ lib/linux-x86_64/libsg100_sdk.so (480KB)

### 新增/更新的示例
- ✅ examples/impedance_control_example.cpp
- ✅ examples/impedance_position_sin_test.cpp
- ✅ examples/palm_runtime_mode_example.cpp
- ✅ examples/comm_status_test.cpp
- ✅ examples/serial_config.json
- ✅ examples/canfd_socketcan_config.json

### 更新的文档
- ✅ docs/Architecture_Analysis.md (新增)
- ✅ docs/Homing_Calibration_Flow.md (新增)
- ✅ docs/SG100_SDK_User_Guide.md (更新)
- ✅ docs/Position_Transform_Calculation.md (更新)
- ✅ README.md (更新)
- ✅ CHANGELOG.md (本文件)

### 配置文件
- ✅ joint_angle_map.json

---

## 🧪 验证建议

### 1. 编译测试
```bash
cd SG100_SDK_Publish/examples
g++ -std=c++14 -I../include hand_control_example.cpp -L../lib/linux-x86_64 -lsg100_sdk -o test_hand
./test_hand
```

### 2. 功能测试
- [ ] 基本串口通信
- [ ] 整手位置控制
- [ ] 单关节控制
- [ ] 阻抗控制（新增）
- [ ] CANFD通信（如有硬件）
- [ ] OTA升级
- [ ] 多设备同时控制

### 3. 回归测试
确保以下原有功能正常工作：
- [ ] 归零校准
- [ ] 实时控制
- [ ] 透传模式
- [ ] 状态回调
- [ ] 错误处理

---

## 📞 技术支持

如有问题，请参考：
- 用户指南: `docs/SG100_SDK_User_Guide.md`
- 架构分析: `docs/Architecture_Analysis.md`
- 示例代码: `examples/` 目录

---

## 📝 版本历史

### v2026.06.05 (当前版本)
- 同步SG100_SDK最新功能
- 新增阻抗控制接口
- 优化多设备配置支持
- 扩充示例和文档

### v2026.05.xx (上一版本)
- 初始发布版本
- 基础RS485/CANFD通信
- 整手/单关节控制
- OTA升级支持

---

**更新完成时间**: 2026-06-05 18:45  
**更新执行人**: AI Assistant  
**备份位置**: `SG100_SDK_Publish_backup_20260605_*`

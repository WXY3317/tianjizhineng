# SG100 SDK 架构分析

> 最后更新：2026-05-16

---

## 一、整体分层架构（当前已重构状态）

```
┌──────────────────────────────────────────────────┐
│              应用层 / 用户代码                    │
│         examples/hand_control_example.cpp         │
└───────────────────────┬──────────────────────────┘
                        │
┌───────────────────────▼──────────────────────────┐
│           设备管理层  SG100DeviceManager          │
│  单例模式，管理多个设备（左手/右手），JSON配置加载  │
└───────────────────────┬──────────────────────────┘
                        │ 1:N
┌───────────────────────▼──────────────────────────┐
│              设备层  SG100Device                  │
│  每个实例 = 一个灵巧手（串口 + 独立通信线程）      │
│  内部通过 Pimpl (::Impl) 封装所有逻辑             │
│  ├── 整手/单关节控制 API                          │
│  ├── Flash 参数读写                               │
│  ├── 掌板串口互联 (UART Link)                     │
│  ├── OTA 升级（关节/掌板）                        │
│  └── 工作模式切换 (Realtime ↔ Passthrough)        │
├─────────────────────┬────────────────────────────┤
│   通信调度 CommScheduler    协议解析 ProtocolHandler│
│  ├─ 实时队列 (10ms周期)     ├─ 解析全部 CMD        │
│  ├─ 透传队列 (FIFO)         ├─ 更新状态缓存        │
│  └─ 后台 SerialTask 线程   └─ 触发状态/错误回调   │
└──────────────┬──────────────────────────────────┘
               │ 依赖注入 (std::function)
┌──────────────▼──────────────────────────────────┐
│           传输层  ITransportAdapter              │
│          ┌───────────┐    ┌──────────────┐       │
│          │RS485Adapter│    │CANFDAdapter  │       │
│          │ 串口+CRC32 │    │ 分片/重组    │       │
│          │ 1000000bps  │    │ 无软件CRC    │       │
│          └───────────┘    └──────────────┘       │
└──────────────────────────────────────────────────┘
               │
┌──────────────▼──────────────────────────────────┐
│           协议打包层  SG100Protocol (静态工具)    │
│  Pack([H][M][Len][Dst][Src][Cmd][Data...])        │
│  Unpack(流式解包，按帧回调)                       │
└──────────────────────────────────────────────────┘
```

---

## 二、关键设计决策

| 模块 | 模式 | 说明 |
|------|------|------|
| `SG100Device` | **Pimpl** | 对外隐藏实现，DLL 边界稳定 |
| `SG100DeviceManager` | **单例** | 统一管理多设备，枚举串口 |
| `ITransportAdapter` | **策略接口** | RS485/CANFD 可互换，依赖注入到 `CommScheduler` |
| `CommScheduler` | **生产者-消费者** | 双队列（实时+透传），10ms 周期后台线程 |
| `ProtocolHandler` | **状态缓存+回调** | 解析结果缓存在内部，外部通过 `HandStatusCallback` 推送 |
| `SG100Protocol` | **无状态工具类** | 静态 Pack/Unpack，无副作用，线程安全 |

---

## 三、通信拓扑

```
PC (0xFE)
  │ UART/RS485 (1000000bps)
  ▼
掌板 Palm (0xFD)
  │ 4路 RS485 (内部透传)
  ├──▶ 关节1~3  (拇指,   0x01~0x03)
  ├──▶ 关节4~6  (食指,   0x04~0x06)
  ├──▶ 关节7~8  (中指,   0x07~0x08)
  └──▶ 关节9~11 (无名指, 0x09~0x0B)
```

**帧格式**：`[0x48][0x4D][Len][Dst][Src][Cmd][Data...][CRC32-LE]`

**CRC 算法**：CRC-32/BZIP2（Poly=0x04C11DB7，Init=0xFFFFFFFF，RefIn=false，RefOut=false，XorOut=0xFFFFFFFF）

---

## 四、工作模式

| 模式 | 触发 | 行为 |
|------|------|------|
| **Realtime**（默认） | `SwitchToRealtimeMode()` | `CommScheduler` 每 10ms 发 `CMD_REALTIME_CTRL`，接收 11 关节位置/电流/速度反馈 |
| **Passthrough** | `SwitchToPassthroughMode()` | 暂停掌板闭环，PC 直接透传命令到单个关节（用于参数读写、OTA、标定） |

---

## 五、`sg100_device.cpp` 内部结构（重构后）

整个文件通过 **Pimpl 模式**（`SG100Device::Impl`）封装实现，各职责已向独立模块迁移：

```
sg100_device.cpp
│
├── 【全局工具函数】
│   ├── mapJointAngle()     用户角度 → 协议原始值
│   └── dmapJointAngle()    协议原始值 → 用户角度
│
└── SG100Device::Impl       (Pimpl 实现体)
    │
    ├── 【对外业务接口层】
    │   ├── 生命周期         Init / Start / Stop / IsOpen
    │   ├── 整手控制         SetHandCommand / SetJointPosition 等
    │   ├── 单关节操作       GetJointStatus / HomingZero / FocEnable 等
    │   ├── Flash 参数操作   ReadParamDevice / WriteParamControl 等
    │   ├── 掌板串口互联     UartLinkOpen / UartLinkClose 等
    │   ├── 状态查询缓存     GetCachedHandStatus / GetCachedPosition 等
    │   ├── 工作模式管理     SwitchToPassthroughMode / SwitchToRealtimeMode
    │   └── OTA 升级         OtaUpgradeJoint / OtaUpgradePalm
    │
    ├── 【适配器辅助方法】
    │   ├── SetAdapter / SendMessage / OnAdapterRx
    │   └── SendAndWaitResponse / SendOtaFrame
    │
    └── 【CANFD 专属逻辑】
        ├── InitForCANFD / SwitchToCANFD / SwitchToRS485ByParams
        └── CANFDProcessRxFrame / CANFDGetTxFrame / CANFDHasTxFrame
```

**已剥离到独立模块**：
- `CommScheduler`：原 `SerialTask` / `Communication` / `CommunicationPassthrough`
- `ProtocolHandler`：原 `ParseResponse` 大 switch
- `RS485Adapter` / `CANFDAdapter`：传输层，通过 `ITransportAdapter` 接口解耦

---

## 六、已完成的重构

| 模块 | 文件 | 状态 |
|------|------|------|
| 传输抽象接口 | `include/transport/i_transport_adapter.h` | ✅ 已创建 |
| 协议打包/解包 | `include/protocol/sg100_protocol.h` + `src/protocol/sg100_protocol.cpp` | ✅ 已创建 |
| RS485 适配器 | `include/transport/rs485_adapter.h` + `src/transport/rs485_adapter.cpp` | ✅ 已创建 |
| CANFD 适配器 | `include/transport/canfd_adapter.h` + `src/transport/canfd_adapter.cpp` | ✅ 已创建 |
| 协议解析/状态缓存 | `include/comm/protocol_handler.h` + `src/comm/protocol_handler.cpp` | ✅ 已创建 |
| 通信调度 | `include/comm/comm_scheduler.h` + `src/comm/comm_scheduler.cpp` | ✅ 已创建 |
| 设备层 | `src/sg100_device.cpp` | ✅ 已重构，使用适配器+调度器+解析器 |

---

## 七、待优化方向

1. **消除 `dynamic_cast`** — `SendAndWaitResponse` 中对 `RS485Adapter` 的 `dynamic_cast` 应通过扩展 `ITransportAdapter` 接口（添加同步收发方法）来消除，CANFD 模式下目前直接返回 `false`
2. **OTA 流程解耦** — 将 OTA 步骤编排（各约 350 行）提取为独立状态机或流程类，与帧发送逻辑分离
3. **命令注册机制** — 如果命令集继续扩展，可考虑将 `ProtocolHandler::HandleFrame` 中的 switch 改为注册回调表（观察者模式）

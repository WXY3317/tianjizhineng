# SG100 SDK 错误码与故障诊断指南

本文说明 SDK 如何接收和处理 SG100 关节固件的版本化故障信息。完整的错误码常量定义位于
`include/sg100_types.h` 的 `SG100FaultCode` 命名空间；本指南说明应用侧的调用方式和安全边界。

## 1. 机制概览

故障诊断不改变关节 ID、实时控制帧、已有参数格式或 OTA 数据格式。故障信息由主机按需轮询，
不会主动插入实时控制链路：

```text
关节固件故障检测
        |
        +-- CMD_GET_FAULT_STATUS (0x28) --> FaultStatusFrameV1_t
        |
        +-- CMD_CLEAR_FAULTS (0x29) ------> 清除已恢复的锁存故障
        |
        +-- 普通命令失败 ------------------> CmdErrorResponseV1_t（新固件）
                                             或首字节 Cmd_Result_t（旧固件）
```

SDK 仍保留 `RegisterErrorCallback`，用于兼容旧应用。新应用应同时注册
`RegisterExtendedErrorCallback`，以取得结构化错误码、详情和事件序号。

## 2. 错误码格式

错误码是 32 位无符号整数，格式为 `0xSSMMRRRR`：

| 字段 | 位段 | 说明 |
|---|---:|---|
| `SS` | 31..24 | 严重级别：`0=Info`、`1=Warning`、`2=Error`、`3=Fatal` |
| `MM` | 23..16 | 子系统：`01` 系统、`02` 通信、`03` 命令、`04` 参数/Flash、`05/06` 编码器、`07` ADC/电源、`08` 驱动器、`09` 控制、`0A` 校准、`0B` OTA、`0C` RTOS |
| `RRRR` | 15..0 | 子系统内的具体原因 |

例如：

- `0x03010001`：`SG100FaultCode::SYSTEM_HARDFAULT`，Fatal 级系统硬件异常；
- `0x02060003`：`SG100FaultCode::JOINT_ENCODER_STALE`，Error 级关节编码器长时间未更新；
- `0x01030002`：`SG100FaultCode::COMMAND_NOT_SUPPORTED`，Warning 级命令不支持。

错误码目录是协议编号，不代表每个固件版本都已经接入对应检测源。应用应同时检查快照中的
`version` 和 `size`，并对未知错误码采用按严重级别的兜底处理。

## 3. 故障快照

### 3.1 查询

`GetFaultStatus(joint_id)` 只表示请求已经加入 SDK 的透传队列；它不是同步返回故障结果。
调用前必须满足：设备已启动、`joint_id` 在 `0x01~0x0B` 范围内、当前处于透传模式。
收到响应后，使用 `GetJointFaultStatus` 读取缓存：

```cpp
FaultStatusFrameV1_t fault{};
if (hand->GetFaultStatus(joint_id)) {
    // 等待通信线程收到响应；生产代码可在自己的轮询周期中检查缓存
    if (hand->GetJointFaultStatus(joint_id, fault)) {
        std::printf("joint=%u severity=%u active=0x%08x latched=0x%08x "
                    "last=0x%08x seq=%u\n",
                    joint_id, fault.highest_severity, fault.active_flags,
                    fault.latched_flags, fault.last_error_code, fault.event_seq);
    }
}
```

`FaultStatusFrameV1_t` 当前大小为 72 字节，SDK 头文件包含 ABI 大小断言。字段含义如下：

| 字段 | 说明 |
|---|---|
| `active_flags` | 当前故障条件仍存在的位图 |
| `latched_flags` | 曾经发生、即使条件恢复仍保留的位图 |
| `highest_severity` | 当前活动故障的最高级别；无活动故障时为 Info |
| `last_error_code` / `last_error_detail` | 最近一次故障及其关联详情 |
| `event_seq` | 每次新事件递增，用于读取与清除之间的并发保护 |
| `*_count` | 通信、编码器、驱动和控制相关饱和计数器 |

### 3.2 清除锁存故障

```cpp
// fault 是此前读取的快照；0 表示清除所有已定义的锁存位
const bool queued = hand->ClearFaults(joint_id, 0, fault.event_seq);
```

清除命令的成功响应会返回最新故障快照，因此仍应以缓存中的响应为准。清除规则：

- 只清除已经恢复、当前不在 `active_flags` 中的故障；
- `clear_mask == 0` 表示全部已定义位，非零值表示指定位；
- `expected_event_seq` 非零时启用乐观并发检查，事件序号变化会返回 `CMD_BUSY`；
- 掩码包含活动故障、未定义位或请求版本不支持时，清除会整体拒绝；
- 清除锁存位不会自动重新使能 FOC，也不会清零通信/传感器计数器。

## 4. 扩展命令错误

新固件对命令失败仍使用 `request_cmd | 0x80` 作为响应命令，第 0 字节仍是旧版
`Cmd_Result_t`，后面追加 `CmdErrorResponseV1_t` 的其余字段。SDK 在版本和长度合法时缓存并触发扩展回调：

```cpp
hand->RegisterExtendedErrorCallback(
    [](uint8_t src_addr, const CmdErrorResponseV1_t& error) {
        std::printf("joint=%u result=%u code=0x%08x detail=0x%08x seq=%u\n",
                    src_addr, error.result, error.error_code,
                    error.detail, error.event_seq);
    });

// 兼容旧固件或只关心命令结果的应用仍可保留：
hand->RegisterErrorCallback(
    [](uint8_t src_addr, uint8_t result) {
        std::printf("joint=%u command result=%u\n", src_addr, result);
    });
```

`CmdErrorResponseV1_t` 当前大小为 16 字节，`detail` 对命令错误通常编码为
`(command << 24) | result`。回调应快速返回，不应在回调内执行阻塞的控制、模式切换或 OTA 操作；
需要恢复动作时，应把事件投递到应用自己的工作队列。

## 5. 推荐的应用处理策略

| 严重级别 | 默认策略 |
|---|---|
| Info | 记录日志，不停止运动 |
| Warning | 记录并提示，继续运行但增加诊断频率 |
| Error | 停止或限制该关节控制，确认 `active_flags` 消失后再考虑恢复 |
| Fatal | 立即进入安全状态，禁止重新使能；保留快照和日志，等待人工检查 |

SDK 负责协议解析和缓存，不替应用决定机械安全策略。涉及运动的产品必须把故障处理接入自己的
急停、限扭和上层状态机，并保留 `joint_id`、时间戳、`event_seq`、错误码、详情、快照和固件版本。

## 6. 兼容性与注意事项

1. 旧固件可能不支持 `0x28/0x29`，此时按 `CMD_NOT_SUPPORTED` 处理，不要把缓存中的默认零值当作“无故障”。
2. 收到快照时应验证 `version == 1` 且 `size >= sizeof(FaultStatusFrameV1_t)`；未来版本允许追加字段。
3. 多字节字段按协议使用小端序；不要在应用中手工按平台字节序重新解释结构体。
4. 清除前应重新确认活动位图；使用读取到的 `event_seq`，不要无条件传 0。
5. 错误码的完整常量以 `include/sg100_types.h` 为准；固件工程中的错误码手册用于确认具体固件版本的检测覆盖范围。

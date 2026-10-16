# SG100 归零校准流程与接口调用顺序

本文档基于以下代码整理:

- `examples/homing_and_realtime_control_example.cpp`
- `examples/realtime_joint_control_example.cpp`
- `include/sg100_device.h`
- `src/sg100_device.cpp`
- `src/sg100_device_manager.cpp`

目标是说明两件事:

1. 当前 SDK 中“校准”相关流程到底怎么走。
2. 各接口应该按什么顺序调用。

## 1. 先说结论

当前示例 `homing_and_realtime_control_example.cpp` 里的“校准”，核心不是直接做参数写入，而是对选定关节执行归零命令 `CMD_HOMING_ZERO (0x07)`。

SDK 对这个过程做了两层封装:

- `CalibrateJointZero(joint_id, timeout_ms)`
  - 单关节归零校准封装
  - 自动处理 realtime 和 passthrough 模式切换
  - 自动等待关节归零完成
- `HomingZeroMasked(joint_mask, interval_ms, manage_mode)`
  - 多关节批量归零封装
  - 本质上是循环调用 `CalibrateJointZero(...)`

因此，示例里的实际顺序是:

1. 创建设备并初始化串口
2. 启动通信线程
3. 查询掌板当前运行模式
4. 调用 `HomingZeroMasked(...)` 执行批量归零
5. SDK 内部自动切到透传模式，逐关节执行归零，再恢复原模式
6. 归零完成后再开始实时位置控制

## 2. 示例主流程

`examples/homing_and_realtime_control_example.cpp` 的主流程如下:

1. 通过 `SG100DeviceManager` 添加设备
   - 优先读取 `serial_config.json`
   - 否则枚举串口并手动选择
2. 调用 `manager.StartAll()` 启动设备通信线程
3. 调用 `QueryAndReadRuntimeMode(...)` 查询归零前的掌板运行模式
4. 调用 `hand->HomingZeroMasked(kHomingJointMask, kHomingIntervalMs, true)`
5. 再次调用 `QueryAndReadRuntimeMode(...)` 确认归零后模式已恢复
6. 调用 `InitHandCtrl(...)` 初始化整手位置控制参数
7. 启动控制线程，周期下发 `Hand_Control_t`
8. 进入命令行交互

可以简化成下面这条主线:

```text
AddDevice / AddDeviceByParams
-> StartAll
-> QueryAndReadRuntimeMode
-> HomingZeroMasked
-> QueryAndReadRuntimeMode
-> InitHandCtrl
-> 启动实时控制线程
```

## 3. 批量归零接口 `HomingZeroMasked(...)`

接口定义:

```cpp
bool HomingZeroMasked(SG100JointMask joint_mask,
                      uint32_t interval_ms = 10000,
                      bool manage_mode = true);
```

### 3.1 参数含义

- `joint_mask`
  - 需要归零的关节掩码
  - 支持按位或，例如 `SG100_JOINT_1 | SG100_JOINT_3 | SG100_JOINT_12`
- `interval_ms`
  - 相邻两个关节归零命令之间的等待时间
- `manage_mode`
  - `true`: SDK 自动处理模式切换与恢复
  - `false`: 调用方必须保证调用前已经在 `WORK_MODE_PASSTHROUGH`

### 3.2 内部调用顺序

`HomingZeroMasked(...)` 在 `src/sg100_device.cpp` 中的执行顺序如下:

1. 检查设备是否运行中
2. 检查 `joint_mask` 是否有效
3. 将掩码展开成待处理关节列表
4. 记录当前工作模式 `original_mode`
5. 如果 `manage_mode=true` 且当前不是 `WORK_MODE_PASSTHROUGH`
   - 调用 `SwitchToPassthroughMode()`
6. 依次遍历选中的关节
   - 对每个关节调用 `pimpl_->CalibrateJointZero(joint_id, 15000)`
   - 如果不是最后一个关节，等待 `interval_ms`
7. 如果开始时是 realtime 模式
   - 调用 `SwitchToRealtimeMode()` 恢复实时模式
8. 返回整体结果

所以它本质上不是一个新的底层协议，而是:

```text
HomingZeroMasked
-> SwitchToPassthroughMode   (可选)
-> CalibrateJointZero(J1)
-> sleep(interval_ms)
-> CalibrateJointZero(J2)
-> ...
-> SwitchToRealtimeMode      (可选)
```

## 4. 单关节归零接口 `CalibrateJointZero(...)`

接口定义:

```cpp
bool CalibrateJointZero(uint8_t joint_id, uint32_t timeout_ms = 15000);
```

### 4.1 它和 `HomingZero(...)` 的区别

- `HomingZero(joint_id)`
  - 只是把 `CMD_HOMING_ZERO` 放入透传队列
  - 要求当前已经处于 `WORK_MODE_PASSTHROUGH`
  - 不会自动切模式
  - 不会同步等待完成
- `CalibrateJointZero(joint_id)`
  - 是更安全的单关节封装
  - 自动 pause/resume realtime
  - 自动独占串口并等待响应
  - 更适合实际业务调用

### 4.2 内部执行顺序

`CalibrateJointZero(...)` 的流程如下:

1. 检查设备是否运行、关节 ID 是否有效
2. 记录当前是否处于 `WORK_MODE_REALTIME`
3. 如果当前是 realtime 模式
   - 调用 `SwitchToPassthroughMode()`
4. 调用 `StopSerialTask()` 暂停串口通信线程，独占串口
5. 直接发送 `CMD_HOMING_ZERO (0x07)`
6. 调用 `SendAndWaitResponse(...)` 等待响应
7. 检查响应首字节是否为 `CMD_OK`
8. 调用 `StartSerialTask()` 恢复串口通信线程
9. 如果原先是 realtime 模式
   - 调用 `SwitchToRealtimeMode()` 恢复实时调度
10. 返回归零是否成功

可以简化为:

```text
CalibrateJointZero(joint_id)
-> SwitchToPassthroughMode   (仅当原来是 realtime)
-> StopSerialTask
-> SendAndWaitResponse(dst=joint_id, cmd=CMD_HOMING_ZERO)
-> StartSerialTask
-> SwitchToRealtimeMode      (仅当原来是 realtime)
```

### 4.3 为什么这里要停掉 SerialTask

因为 `CalibrateJointZero(...)` 使用的是同步等待式的 `SendAndWaitResponse(...)`。

如果后台通信线程继续运行，就可能和当前同步等待逻辑同时读写串口，导致响应被竞争消费。因此实现中先停掉 `SerialTask`，归零完成后再恢复。

## 5. 模式切换顺序

### 5.1 切到透传模式 `SwitchToPassthroughMode()`

内部顺序如下:

1. `StopSerialTask()`
2. `FlushRealtimeQueue()`
3. 设置模式切换等待状态
4. 直接向掌板发送 `CMD_REALTIME_PAUSE (0x24)`
5. 轮询等待掌板确认响应
6. 成功后将 `work_mode_` 置为 `WORK_MODE_PASSTHROUGH`
7. `StartSerialTask()`

简化后就是:

```text
SwitchToPassthroughMode
-> StopSerialTask
-> FlushRealtimeQueue
-> SendFrame(ADDR_PALM, CMD_REALTIME_PAUSE)
-> WaitForModeSwitchResponse
-> work_mode_ = WORK_MODE_PASSTHROUGH
-> StartSerialTask
```

### 5.2 切回实时模式 `SwitchToRealtimeMode()`

内部顺序如下:

1. `StopSerialTask()`
2. `FlushPassthroughQueue()`
3. 设置模式切换等待状态
4. 直接向掌板发送 `CMD_REALTIME_RESUME (0x25)`
5. 轮询等待掌板确认响应
6. 成功后将 `work_mode_` 置为 `WORK_MODE_REALTIME`
7. `StartSerialTask()`
8. 额外等待 realtime 状态反馈，确认实时流量已恢复

简化后就是:

```text
SwitchToRealtimeMode
-> StopSerialTask
-> FlushPassthroughQueue
-> SendFrame(ADDR_PALM, CMD_REALTIME_RESUME)
-> WaitForModeSwitchResponse
-> work_mode_ = WORK_MODE_REALTIME
-> StartSerialTask
-> WaitForRealtimeStatus
```

## 6. 归零相关接口的推荐调用方式

### 6.1 场景一: 开机后做整手归零，再进入实时控制

这是当前示例采用的方式，也是最推荐的方式。

推荐顺序:

```cpp
manager.AddDevice(...);
manager.StartAll();

Palm_Runtime_Mode_Status_t mode_status{};
QueryAndReadRuntimeMode(hand, mode_status);

hand->HomingZeroMasked(SG100_JOINT_ALL, 10000, true);

QueryAndReadRuntimeMode(hand, mode_status);

InitHandCtrl(...);
std::thread ctrl_thread(ControlThreadFunc, hand);
```

适用场景:

- 开机初始化
- 全手重新建立零位基准
- 进入闭环控制之前做一次统一归零

### 6.2 场景二: 实时控制过程中，只对单个关节做归零

推荐顺序:

```cpp
暂停你自己的上层控制发送线程;
hand->CalibrateJointZero(joint_id, 15000);
恢复你自己的上层控制发送线程;
```

注意事项:

- SDK 会自动处理掌板 realtime/passthrough 的切换
- 但你的应用层如果还有独立控制线程，最好先暂停
- `examples/realtime_joint_control_example.cpp` 就是这么做的

该示例的思路是:

1. 用户输入 `cali <id>`
2. 上层先设置 `g_control_paused = true`
3. 稍等一小段时间，确保控制发送停下来
4. 调用 `hand->CalibrateJointZero(id)`
5. 校准完成后恢复控制发送

### 6.3 场景三: 你已经手动切进透传模式，只想发一个归零命令

这时才适合直接调用 `HomingZero(joint_id)`。

推荐顺序:

```cpp
hand->SwitchToPassthroughMode();
hand->HomingZero(joint_id);
// 之后由你自己决定如何等待完成、如何恢复模式
hand->SwitchToRealtimeMode();
```

这种方式更底层，但需要调用方自己保证:

- 已在 passthrough 模式
- 响应等待逻辑不会和后台线程冲突
- 校准完成后记得恢复 realtime 模式

如果不是做底层调试，通常不建议优先用这种方式。

## 7. 其他“校准相关”接口

除了归零命令 `CMD_HOMING_ZERO`，当前 SDK 里还有几类和标定相关的接口。

### 7.1 读取标定参数 `ReadParamCalib(...)`

接口:

```cpp
bool ReadParamCalib(uint8_t joint_id);
bool GetJointParamCalib(uint8_t joint_id, ParamCalibrationGroup_t& out) const;
```

推荐顺序:

```text
SwitchToPassthroughMode
-> ReadParamCalib(joint_id)
-> 等接收线程收到响应并写入缓存
-> GetJointParamCalib(joint_id, out)
```

`ParamCalibrationGroup_t` 包含:

- `electrical_angle_zero`
- `foc_mode`
- `calibration_flags`
- `align_voltage_d`
- `align_time_ms`
- `hall_zero_mark`
- `hall_zero_window`

注意:

- `ReadParamCalib(...)` 只是发命令
- 真正的数据是在接收线程 `ParseResponse(...)` 中写入缓存
- 之后再通过 `GetJointParamCalib(...)` 读取

### 7.2 FOC 零点标定 `FocCalibrateZero(...)`

接口:

```cpp
bool FocCalibrateZero(uint8_t joint_id);
bool GetJointFocCalib(uint8_t joint_id, FocZeroCalibrationResult_t& out) const;
```

推荐顺序:

```text
SwitchToPassthroughMode
-> FocCalibrateZero(joint_id)
-> 等接收线程收到响应并写入缓存
-> GetJointFocCalib(joint_id, out)
```

结果结构 `FocZeroCalibrationResult_t` 包含:

- `electrical_angle_zero`
- `calibration_flags`

### 7.3 使零点失效 `InvalidateZero(...)`

接口:

```cpp
bool InvalidateZero(uint8_t joint_id);
```

推荐顺序:

```text
SwitchToPassthroughMode
-> InvalidateZero(joint_id)
```

适用场景:

- 清除旧零点状态
- 强制重新进行零点建立

### 7.4 重新校准 ADC 偏移 `RecalibAdcOffset(...)`

接口:

```cpp
bool RecalibAdcOffset(uint8_t joint_id);
```

推荐顺序:

```text
SwitchToPassthroughMode
-> RecalibAdcOffset(joint_id)
```

## 8. 各接口之间的关系

可以用下面这张关系图理解:

```text
HomingZeroMasked
  -> CalibrateJointZero(逐关节)
       -> SwitchToPassthroughMode  (必要时)
       -> StopSerialTask
       -> SendAndWaitResponse(CMD_HOMING_ZERO)
       -> StartSerialTask
       -> SwitchToRealtimeMode     (必要时)

HomingZero
  -> SendSimplePassthroughCommand(CMD_HOMING_ZERO)
```

所以从推荐程度来说:

1. 批量归零: 优先 `HomingZeroMasked(...)`
2. 单关节归零: 优先 `CalibrateJointZero(...)`
3. 底层调试: 才考虑 `HomingZero(...)`

## 9. 使用建议

### 9.1 最稳妥的调用原则

- 在进入实时控制前，先完成归零校准
- 做业务层实时控制时，如果要临时校准某个关节，先暂停你自己的控制发送线程
- 除非你明确知道自己在做什么，否则不要优先用最底层的 `HomingZero(...)`
- 读取标定参数或 FOC 标定结果时，要记住“先发命令，再从缓存读结果”

### 9.2 如果你只是想照着当前工程用

整手归零直接使用:

```cpp
hand->HomingZeroMasked(SG100_JOINT_ALL, 10000, true);
```

单关节归零直接使用:

```cpp
hand->CalibrateJointZero(joint_id, 15000);
```

### 9.3 一个容易混淆的点

“归零校准”与“参数标定读取/写入”是两类不同动作:

- `CMD_HOMING_ZERO (0x07)`
  - 运行期执行归零动作
- `CMD_PARAM_READ_CALIB / CMD_PARAM_WRITE_CALIB`
  - 读取或写入标定参数组
- `CMD_FOC_CALIBRATE_ZERO (0x16)`
  - FOC 零点标定命令

当前 `homing_and_realtime_control_example.cpp` 用到的是第一类，也就是归零动作，不是参数组读写。

## 10. 推荐记忆版时序

如果只记一套最重要的顺序，记这个就够了:

### 整手归零

```text
连接设备
-> StartAll
-> 查询运行模式
-> HomingZeroMasked(mask, interval, true)
-> SDK 自动 pause
-> 逐关节 CalibrateJointZero
-> SDK 自动 resume
-> 查询运行模式
-> 启动实时控制
```

### 单关节归零

```text
暂停应用层控制线程
-> CalibrateJointZero(joint_id)
-> SDK 自动 pause
-> 发送 CMD_HOMING_ZERO
-> 等待完成
-> SDK 自动 resume
-> 恢复应用层控制线程
```

以上就是当前工程中“校准相关流程”和“接口调用顺序”的完整整理。
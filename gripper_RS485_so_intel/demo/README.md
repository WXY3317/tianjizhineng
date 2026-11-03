# 夹爪控制与反馈 Demo

C++17，Linux x86_64，使用现有 S-Module-R35 RS485/Modbus 驱动。夹爪位置闭环在驱动器中执行；本程序发送目标并轮询反馈，不自行实现电机 PID，也不重新定义驱动器的 Modbus 帧。

## 2026-10-09 实测端点修正

当前校准属于 Modbus ID **1** 的这台夹爪，方向 +1，串口初始为 `/dev/ttyUSB0`。程序拒绝将该表用于其他 ID；更换夹爪需重新建立对应校准。

- 用户手动确认最大张开端，实测接触面间距 **98 mm**；最大张开端已执行 `set_zero()`，随后原始电机角度为 **0 rad**，保持失能。
- 旧零位下最大张开端为 70.5596926925°，手动完全接触闭合端为 3577.59341872°。校准采用差值 **3507.0337260275° / 61.2092854977 rad** 作为闭合端角度。闭合端尚需在新零位下实测复核。
- `calibration.csv` 的 513 个点保留原库机构模型的非线性形状，并通过两端实测归一化到 0～98 mm。它不是 513 个实测点，中间位置准确度尚未实测。
- 控制和反馈使用同一张表，闭合 0 mm 对应已记录的接触位置，张开 98 mm 对应 0 rad；不再沿用旧 API 的 8 mm 软垫补偿和 82 mm 指令范围。
- 旧 `GripperController::set_gripper_distance()` 会拒绝校准闭合端所需的负模型开度。修正版通过 SDK `MotorDriver::motor_pos_cmd()` 下发校准后的角度和速度，仍由驱动器执行 POS 闭环，未修改原 `.so`。
- `motor_position_rad` 输出原始 SDK 电机角度。反馈超出实测开度行程 0.2 mm 时触发无效反馈和失能尝试；小幅超出端点的读数保留外推值，不静默截断成 0 或 98 mm。控制指令仍限制在实测端点内。
- 末端力仍是模型估算，采用原库额定力矩 0.23 N·m、效率 0.9 和校准表局部导数，未经力传感器标定。
- 修正后已执行 0↔98 mm 开合三轮，每步连续 3 次反馈处于目标 ±0.1 mm 内，错误码均为 0。闭合反馈约 -0.014～-0.010 mm，张开反馈约 98.001 mm；测试结束保持张开并失能。以上为校准模型反馈，真实接触及中间开度精度仍需现场观察/独立测量确认。
- 首次全闭合测试在接近端点时触发无效反馈并失能；原 1° 端点容差过窄，已改成物理开度 0.2 mm 反馈容差，并增加具体异常日志。控制指令范围仍为 0～98 mm，运动目标未越过记录的端点。

零位改变、机构/软垫改变后应重建端点校准。不能单靠文件存在就认为零位有效。`ZERO` 现在会检查停止位置是否接近已校准的张开端（1° 内）；遇到阻挡或已丢失的零位时会拒绝覆盖零点，需要现场确认端点后重新校准。

## 编译

```sh
cd /home/cobot/Works/gripper_RS485_so
# 依赖：g++、cmake、libeigen3-dev
cmake -S demo -B demo/build -DCMAKE_BUILD_TYPE=Release
cmake --build demo/build -j2
ctest --test-dir demo/build --output-on-failure
./demo/build/gripper_demo --help
```

## 启动与开合

### 电流触发位置保持

位置指令期间，对反馈电流绝对值进行检查。`abs(motor_current_a) >= current_threshold_a` 时，放弃原 MOVE 目标，用该次反馈的**原始电机位置**重新下发位置目标并保持使能。只捕获一次，电流回落后也不恢复旧目标或持续追随新反馈；新的 MOVE 会重置触发标志。DISABLE、ZERO 或故障会取消本次保持。

默认阈值 **0.5 A**，按用户指定设置；此前空载采样峰值约 0.22 A，此阈值并非额定限流值。启动时设置：

```sh
./demo/run.sh --use-existing-zero --current-threshold-a 0.5 /dev/ttyUSB0 1 115200
```

阈值必须有限且大于 0。位置模式保持后仍可能施力；该功能不是驱动器硬件限流或恒力夹持，也不保证瞬时电流不超过阈值。串口往返、反馈轮询和重新下发目标存在延迟；当前轮询按 100 ms 目标周期调度，实际周期受 SDK 刷新耗时影响。标零期间不应用此 MOVE 电流保持逻辑。

反馈增加：`current_threshold_a`（阈值）、`current_limit_reached`（锁存触发）、`holding_position`（触发后使能保持）、`target_active` 和 `target_position_m`（实际生效的目标，无目标时为 null）。触发后 `at_target=false`，不能误认为原命令位置到位；`command_id` 仍关联被截停的 MOVE。保持指令下发失败则按通信异常尝试失能。

将来 ROS2 可把阈值作为节点参数，新增状态字段直接映射到反馈话题。

实机验证（2026-10-09）：临时阈值 0.05 A，空载从约 98 mm 下发 90 mm；读到 0.11 A 时在 97.592 mm 捕获保持目标，约 2 s 后反馈 97.594 mm，目标未改变、`at_target=false`。完成后失能、电流 0 A。该低阈值仅用于验证触发逻辑。

随后使用默认 **0.5 A** 做硬物接触测试：从约 97.588 mm 以 10 mm/s 请求闭合到 0 mm；在反馈电流 **0.56 A**、开度 **35.240 mm** 时触发，生效目标改为 35.240 mm。保持观察约 3 s 后反馈开度 35.241 mm、电流约 0.08 A，目标未漂移，`holding_position=true`、`at_target=false`、错误码 0。测试后失能并退出，失能首帧开度约 35.384 mm、电流约 0.02 A。阈值触发与保持已在本次物体上验证；采样电流可超过阈值，这不是硬件限流，也未验证其他物体的夹持力或损伤边界。

```sh
./demo/run.sh /dev/ttyUSB0 1 115200
```

三个可选参数依次为串口、从站地址、波特率。应与驱动器实际配置一致。同一串口只运行一个控制进程。启动初始化后失能，不自动清错、不自动标零、不自动运动。

**不需要每次启动程序都标零。**首次通电或零位失效后才需标零。如果已确认设备的既有零位仍有效，可这样启动，随后直接 `ENABLE 编号`、`MOVE 编号 开度 速度`：

```sh
./demo/run.sh --use-existing-zero /dev/ttyUSB0 1 115200
```

该选项表示调用方确认零位有效，不会执行标零，也不会自动使能。现有 SDK 没有提供零位有效性查询接口，因此程序无法自行验证零点是否正确或是否在掉电后保留；首次通电、重新上电后未确认零位有效或零位失效时，请走 ZERO 流程。默认启动时零位状态为“未确认”，需要 ZERO 或使用上述显式确认选项。

从站地址（如启动参数 `1`）配置到 `MotorDriverConfig.motor_id`，已用 ID 1 成功通信并完成实机测试，未进行地址扫描或修改设备地址。命令中的编号用于关联 ACK/反馈，与从站地址不同。每个程序实例只控制一个夹爪，因此消息中不重复携带设备 ID；未来 ROS2 可使用 `/gripper_1/command`、`/gripper_1/state` 等命名空间区分实例。

以下命令逐行输入；`ZERO` 会主动张开至机械极限，执行前确保夹爪空载且运动范围畅通。每一步检查 `ack.accepted`，成功再继续：

```text
CLEAR 1
ZERO 2
ENABLE 3
MOVE 4 0.060 0.010
MOVE 5 0.020 0.010
DISABLE 6
QUIT
```

命令 `MOVE 编号 目标开度(m) 速度(m/s)` 使用 POS 模式控制：增大开度为张开，减小为闭合。当前实测校准范围为 `[0, 0.098] m`，速度必须有限且大于零。速度通过目标位置附近的局部导数换算为电机轨迹速度，完整行程中实际夹爪速度不保证恒定。小于物体尺寸的目标可能导致持续施力；位置模式的目标到位并不等同于力控制夹持。

`ZERO/ENABLE/DISABLE/CLEAR 编号` 不需要开度或速度。编号为 uint64，由发送端生成；反馈关联最后成功接收的命令。编号不作去重或排序，新到达的有效 MOVE 替换原目标。`DISABLE` 撤销使能，不进行保持位置的受控减速。退出、输入流关闭、SIGINT、SIGTERM 均尝试失能并释放通信。通信断开时失能指令可能无法到达设备。

## 消息

通过 `run.sh` 启动时，标准输出为 JSON Lines（每行一条），日志在标准错误。必须使用这个启动脚本来隔离供应商库在 main() 之前输出的注册日志；直接运行二进制时，开头会混有这些日志。`--help` 的帮助文本通过启动脚本输出到标准错误，不初始化硬件。ACK 表示指令已下发/操作完成，不表示 MOVE 已到位：

```json
{"type":"ack","command_id":4,"accepted":true,"detail":"accepted"}
{"type":"feedback","stamp_ns":1791519816648825642,"command_id":2,"valid":true,"position_m":0.0352403534,"motor_position_rad":40.2754326,"motor_current_a":0.560000002,"motor_torque_percent":16.8571415,"force_n":19.9975857,"error_code":0,"enabled":true,"homed":true,"at_target":false,"current_threshold_a":0.5,"current_limit_reached":true,"holding_position":true,"target_active":true,"target_position_m":0.0352403534}
```

反馈默认约 10 Hz，实际频率受串口往返耗时影响。时间戳在读反馈结束时取系统 Unix 纳秒时间。`ZERO` 是阻塞标零流程，采用与当前库相同的 -17% 张开输出，最多等待 10 s，期间暂停轮询/输入处理；`ENABLE` 也有约 500 ms 等待。软件退出只能在当前调用返回后处理，不可替代硬件急停。

| 字段 | 含义 |
|---|---|
| `position_m` | 实际两指软垫开度，m |
| `motor_position_rad` | 原始电机位置反馈，rad，不是夹爪开度 |
| `motor_current_a` | R35 驱动器电流反馈，A，由 `get_motor_current()` 直接读取，不使用力矩百分比换算 |
| `motor_torque_percent` | 电机额定力矩/电流百分比，100 表示 100%，不是 N·m |
| `force_n` | 由机构模型估算的末端夹持力，N，不是力传感器实测 |
| `error_code` | 驱动器错误位图，0 表示无已报告错误 |
| `valid` | 本次读取是否成功且数值有限；false 时测量值、错误码及使能状态输出 null |
| `homed` | 零位已通过 ZERO 建立或由 `--use-existing-zero` 显式确认有效；默认启动 false |
| `enabled` | 驱动器使能反馈 |
| `at_target` | 最新有效目标与开度误差 ≤1 mm 且已使能、无错误；仅为位置容差判断，不能判断机械卡滞/接触原因 |
| `current_threshold_a` | 反馈电流绝对值触发阈值，默认 0.5 A |
| `current_limit_reached` | 本次目标的电流触发标志，锁存至新 MOVE、DISABLE、ZERO 或故障 |
| `holding_position` | 电流已触发，且当前仍使能并保持捕获位置 |
| `target_active` | 当前存在有效位置目标 |
| `target_position_m` | 实际生效目标，触发后为捕获开度，无目标时为 null |

反馈字段由 SDK 顺序读取，并非一帧原子快照。通信失败、非有限测量或驱动故障会尝试失能并取消目标和本进程零位标记；恢复后需清错、重新标零、使能。没有主动输出真实 N·m 的接口，因此不臆造额定力矩换算。

## 后续 ROS2 迁移

`messages.hpp` 定义纯数据；`controller.hpp/.cpp` 包含与 ROS/命令行无关的状态机；`hardware.cpp` 只封装硬件；`main.cpp` 处理文本输入、JSON 输出和调度。`tests.cpp` 使用假硬件验证状态机，不连接串口。

将 `Command` 字段映射到 `/gripper/command`，将 `Feedback` 字段映射到 `/gripper/state`，建议 ROS2 消息如下（这里是接口草案，不是当前 ROS 包）：

```text
# GripperCommand.msg
uint8 MOVE=0
uint8 ZERO=1
uint8 ENABLE=2
uint8 DISABLE=3
uint8 CLEAR=4
uint64 command_id
uint8 action
float64 position_m
float32 speed_mps
```

```text
# GripperState.msg
builtin_interfaces/Time stamp
uint64 command_id
bool valid
float64 position_m
float64 motor_position_rad
float64 motor_current_a
float64 motor_torque_percent
float64 force_n
uint32 error_code
bool enabled
bool homed
bool at_target
float64 current_threshold_a
bool current_limit_reached
bool holding_position
bool target_active
float64 target_position_m
```

ROS 字段不支持 null：`valid=false` 时消费者必须忽略测量值和设备状态。建议使用独立串口工作线程（所有 SDK 调用均在该线程）；订阅回调只入队，定时发布反馈；运动指令可只保留最新值，避免积压。ZERO 建议进一步改为 action/service，以报告阻塞标零的开始/完成。`Controller` 本身非线程安全，不能在多个回调中并发调用。

修正前程序已执行真实设备开合，但其反馈与物理间距不一致；不能把此前测试当作新校准的物理精度验证。修正后验证范围见本文的端点修正记录。

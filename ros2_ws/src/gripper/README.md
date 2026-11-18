# gripper ROS2 功能包

ROS2 Humble / C++17。`src/` 是话题控制程序，`test/` 保存状态机与标定映射测试。SDK 位于本包 sdk/，使用配套头文件和库，按目标架构选择 x86_64 或 ARM64。
功能包实际位于 `ros2_ws/src/gripper/`，无需外部 SDK 目录。

## 目录

```text
gripper/
├── src/       # 控制节点、状态机、硬件适配
├── msg/       # 命令、状态、应答消息
├── launch/    # 启动配置
├── config/    # 标定表
├── sdk/       # 配套头文件及 x86_64/ARM64 动态库
└── test/      # 状态机与标定映射测试
```

## 构建与启动

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon --log-base log build --packages-select gripper --build-base build --install-base install --symlink-install
source install/setup.bash
ros2 launch gripper gripper.launch.py
ros2 launch gripper dual_gripper.launch.py left_use_existing_zero:=false right_use_existing_zero:=false
```

单夹爪 launch 默认选左夹爪（ID 1），也可用 `side:=right` 选右夹爪（ID 2）；两侧共用 `calibration.csv`。默认串口 `/dev/gripper_rs485`、115200 波特率、10 Hz 目标反馈频率、电流阈值 0.5 A。默认零位未确认（`use_existing_zero=false`），不自动使能或运动。
`speed_mps` 启动参数设置所有 MOVE 的速度；命令消息不包含速度字段。
驱动器断电后先通过 ZERO 话题指令重新标零；仅在驱动器未断电且零位已确认有效时，可显式使用：

```bash
ros2 launch gripper gripper.launch.py use_existing_zero:=true
```

### Tianji 单夹爪启动命令

使用 `/dev/gripper_rs485` 和右夹爪 98 mm 标定表启动。先确认没有其他程序占用串口。默认不确认零位，必须通过 `/gripper/command` 发送 ZERO，或在零位已确认有效时添加 `use_existing_zero:=true`；该参数本身不会重新标零。

```bash
cd ~/wxy/tianjizhineng/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch gripper gripper.launch.py side:=right current_threshold_a:=1.0
# 如需单独启动左夹爪：ros2 launch gripper gripper.launch.py side:=left
```

出现 `Ready on /dev/gripper_rs485 with 1 device(s); no automatic motion` 表示节点已启动；启动本身不会使能或移动夹爪。另开终端读取 `/gripper/state`，确认 `valid`、`homed` 和实际开度后再发运动命令。

## 话题

### 未建立零点时短时张开

`gripper_open` 使用底层速度模式，不读取开度标定、不清错、不写零点。
先停止占用同一串口的 ROS 节点和调试程序，确认夹爪空载、运动空间无障碍，然后执行：

```bash
ros2 run gripper gripper_open \
  /dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0 2 1000
```

参数依次为串口、设备 ID、点动时长（毫秒，可省略，默认 1000，范围 100～2000）。
波特率固定 115200，电机方向固定 +1，原生电机速度固定 -0.5 rad/s，负方向与本 SDK 的张开方向一致。
这是小幅点动，不保证到达全开位置，也不进行标零；应观察实际动作后再决定后续操作。
命令会输出点动前后的原始编码器角度、角度变化量和采样到的峰值速度、电流；指令发送成功不等于已验证机械运动。
驱动报警、反馈失败、电流绝对值达到 0.5 A 或 SIGINT/SIGTERM 时会中止，并尝试撤销速度输出、失能。
0.5 A 是软件轮询阈值，不是硬件限流；响应时间受 SDK 串口超时影响，通信中断时停止指令可能无法送达。
不要同时运行其他串口程序；端口占用检查不能阻止其他程序在检查后争用设备。


| 话题 | 类型 | 用途 |
|---|---|---|
| `/gripper/command` | `gripper/msg/GripperCommand` | 输入控制指令 |
| `/gripper/ack` | `gripper/msg/GripperAck` | 指令接受/拒绝及原因 |
| `/gripper/state` | `gripper/msg/GripperState` | 持续状态反馈 |

动作编号：MOVE=0、ZERO=1、ENABLE=2、DISABLE=3、CLEAR=4、SET_CURRENT_THRESHOLD=5。
MOVE 的 ACK 只表示已接受，需读取 state 判断结果。
`command_id` 由 ROS 节点按每侧收到命令的顺序自动生成，ACK 显示该编号，state 显示最后一次成功接受的命令编号；节点重启后重新从 1 计数。

确认零位后先使能，再发送移动指令：

```bash
ros2 topic echo /gripper/state
ros2 topic echo /gripper/ack
# 设备断电后先确认空载且张开路径畅通，再标零；完成后仍失能
ros2 topic pub --once /gripper/command gripper/msg/GripperCommand '{action: 1}'
# 逐条检查对应 ACK，再继续
ros2 topic pub --once /gripper/command gripper/msg/GripperCommand '{action: 2}'
ros2 topic pub --once /gripper/command gripper/msg/GripperCommand '{action: 0, position_m: 0.060}'
ros2 topic pub --once /gripper/command gripper/msg/GripperCommand '{action: 3}'
```

开度单位 m。左右 MOVE 范围均为 0～0.098 m。单夹爪默认速度 0.002 m/s（2 mm/s），双夹爪分别使用 launch 中 `left_speed_mps`、`right_speed_mps` 的配置；大开度为张开，小开度为闭合。
发送 `{action: 5, current_threshold_a: 0.5}` 可通过话题更新该侧电流触发保持阈值，要求有限且大于零；state 会报告当前阈值。
右夹爪空载闭合时，0.5 A 曾在约 96.865 mm 提前触发保持，导致 MOVE 的 ACK 为 accepted 但 `at_target=false`。本次通过右侧话题设为 1.0 A 后，60 mm 目标到位。1.0 A 仍是软件触发阈值，不是驱动器硬件限流；使用时应结合夹持物检查实际电流和受力。
标零发送 `action: 1`，清错发送 `action: 4`。ZERO 现调用与 `zero_demo` 相同的 SDK `GripperController::initialize_zero()`：先使能，再以张开力矩到达机械全开端并设零，完成后失能并检查原始角度接近 0 rad。此前的 -0.5 rad/s 慢速张开流程会在约 80 多 mm 处误判停点，已移除。SDK 标零成功只表示其停点判据满足；首次运行或机构变化后仍须尺量确认实际全开约 98 mm，夹爪必须空载、张开方向无遮挡。标零期间另一侧反馈暂停。

可显式在启动时执行同一找零流程，完成后节点才发布话题。单夹爪例子：

```bash
ros2 launch gripper gripper.launch.py side:=right auto_home:=true
```

双夹爪默认两侧均不自动标零，也不沿用旧零点。已在当前上电周期正确标零并确认实际全开约 98 mm 时，可按侧设置 `left_use_existing_zero:=true` 或 `right_use_existing_zero:=true`，重启 ROS 节点时沿用设备零点；断电后须重新确认。此参数只检查设备健康，不验证机械全开。左侧 SDK 曾在实际约 72 mm 时误报标零成功，确认全开前不要使能或 MOVE。

## 状态及保持

state 输出开度、原始电机角度、电流、力矩百分比、估算力、错误码、零位、使能、到位、目标和保持状态。
`valid=false` 时不能使用测量值、使能和错误码；无效浮点测量输出 NaN。`target_active=false` 时目标开度为 NaN。时间戳使用节点 ROS 时钟。

电流绝对值达到阈值时，捕获一次原始电机位置并保持；原 MOVE 目标被替换，`current_limit_reached`、`holding_position` 置位，`at_target=false`。电流回落不会恢复旧目标。新 MOVE 重置触发状态；失能、标零或故障取消保持。此功能不是硬件限流或恒力夹持，力为模型估算。

SDK 和 Controller 由单线程执行器串行访问。ZERO 运行时间由 SDK 控制，ENABLE 约 500 ms，期间暂停反馈和命令处理；正常反馈周期也受串口耗时影响。命令队列深度 10，不适合高频轨迹流或硬件急停。通信故障和退出时尝试失能，断线时不能保证指令到达。

## 同一路串口连接左右夹爪

先将实际设备设置为左 ID 1、右 ID 2，波特率一致。软件参数不会修改设备地址。
未设置前应分别单独接线设置地址，不能将两台相同 ID 的设备同时接入通信。

一个节点创建两个驱动实例，SDK 按同一串口复用 Rs485Bus，应用层使用单线程串行调用。不要启动两个进程争用同一串口。

```bash
ros2 launch gripper gripper.launch.py # 原单夹爪模式保持可用
# 双夹爪模式：左右共用 98 mm 标定表
ros2 launch gripper dual_gripper.launch.py
```

双夹爪默认 `left_auto_home=false`、`right_auto_home=false`、`left_use_existing_zero=false`、`right_use_existing_zero=false`，左右电流阈值均为 0.8 A。仅在当前上电周期已确认零点有效时，才按侧设置 `use_existing_zero:=true`；它与同侧 `auto_home:=true` 不能同时使用。
双夹爪 launch 固定两侧共用 `config/calibration.csv`，不再提供左右标定路径参数；仍可分别设置 `left_current_threshold_a`、`right_current_threshold_a`。
左右速度通过 `left_speed_mps`、`right_speed_mps` 分别设置，均默认 `0.003 m/s`，启动后只读。
两侧继续共用 `config/calibration.csv`（98 mm）。2026-11-07 本次上电确认全开标零后，左侧刚接触全闭约 63.363472 rad，右侧约 63.657230 rad；按用户要求，表的 0 mm 端点使用平均值 63.510351 rad。5 mm 以上保留原始映射，0–5 mm 用线性插值连接新端点。平均值比左侧实测接触角度大约 0.147 rad，比右侧小约 0.147 rad；空载复测两侧闭合、关注接触电流。原表曾有目标/尺量 20→24、30→34、60→62 mm 的偏差记录；ROS 开度仍是表格换算值。断电后须重新确认零点。

### 历史右表测量记录

以下记录仅用于追溯，当前 ROS 两侧均使用 `config/calibration.csv`。

右侧后续实测：原端点表反馈 60 mm 时实际约 45 mm，原始电机角度约 26.60555 rad。第一次修正使用该锚点；旧端点表保存在 `gripper_RS485/right_calibration_98mm_endpoints.csv`。

第一次修正后，右侧在 19.7244053 rad 处反馈 60 mm、实测 56 mm。第二次右表曾从端点原表使用这两处原始角度与实测开度修正；一次修正表保存在 `gripper_RS485/right_calibration_98mm_one_anchor.csv`。

第二次修正后，请求 60 mm，静止角度约 18.009 rad，现场实测 59 mm 多。该处约在 1 mm 内；不能据此推断其他开度也达到同样精度。

继续实测右侧第二次修正表的目标/实际开度：**20→13、30→25、50→50、70→66 mm**。第三次右表从端点原表用这四点及先前 45、56 mm 两点一次性重建，保持 0/98 mm 端点。静止电机角度分别约 46.655、38.732、23.528、13.588 rad。第二次修正表保存在 `gripper_RS485/right_calibration_98mm_two_anchors.csv`。

2026-11-07 重新上电、`zero_demo 2` 标零后，右侧全开仍实测 98 mm，但旧右表出现系统偏差。曾用 16 个实测点修正右表，并由用户确认本次上电约 61.209 rad 时物理全闭 0 mm。现在按用户决定移除独立右表，两侧共用左表；旧右表的中间位置实测不适用于共用表，ROS 开度仍是表格换算值。

本次上电部署新表后，重新下发目标验证：50 mm 实测 50 mm（静止角度约 33.950 rad），20 mm 实测 20 mm（静止角度约 51.612 rad）；两次结束均失能。其余目标尚未用新表独立复测。

随后重新执行 `zero_demo 2`，物理全开实测 98 mm；右侧改用左表试验时，50 mm 指令的 ROS 反馈约 50.0 mm、电机角度 31.745445 rad，而接触面尺量约 52 mm。用户决定继续直接使用左表，未保留此测量点修正；程序反馈是表格换算值，并非独立开度测量。修改 CSV 后须重启 ROS 节点。

| 夹爪 | ID 默认值 | 输入话题 | 输出话题 |
|---|---|---|---|
| 左 | 1 | `/left_gripper/command` | `/left_gripper/state`、`/left_gripper/ack` |
| 右 | 2 | `/right_gripper/command` | `/right_gripper/state`、`/right_gripper/ack` |

```bash
ros2 topic echo /left_gripper/state
ros2 topic echo /right_gripper/state
# 确认右侧本次上电零点有效，并用 right_use_existing_zero:=true 启动后再使能
ros2 topic pub --once /right_gripper/command gripper/msg/GripperCommand '{action: 2}'
ros2 topic pub --once /right_gripper/command gripper/msg/GripperCommand '{action: 0, position_m: 0.050}'
```

左右各自保存目标、电流触发和零位状态，均从同一份标定文件加载映射。单侧故障由该侧 Controller 尝试失能；总线断线可能同时影响两侧。
反馈按左右顺序轮询，目标频率是每侧 10 Hz，实际受两侧通信总耗时影响。任意一侧 ZERO/ENABLE 阻塞期间，另一侧也暂停反馈和命令处理，因此不是两个独立实时控制回路。
SDK 的总线复用及左右 ID 的物理通信已在 tianji 上验证；退出后的设备状态以现场反馈为准。

## 测试

```bash
colcon --log-base log test --packages-select gripper --build-base build --install-base install
colcon test-result --test-result-base build/gripper --verbose
```

复用原状态机与标定测试，不连接硬件。整合版本仍需 ARM64 及真实夹爪验证。

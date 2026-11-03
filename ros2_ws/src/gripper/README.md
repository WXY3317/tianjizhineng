# gripper ROS2 功能包

ROS2 Humble / C++17。`src/` 是话题控制程序，`example/` 保留原命令行示例。SDK 位于本包 sdk/，使用配套头文件和库，按目标架构选择 x86_64 或 ARM64。
功能包实际位于 `ros2_ws/src/gripper/`，无需外部 SDK 目录。

## 目录

```text
gripper/
├── src/       # 控制节点、状态机、硬件适配
├── msg/       # 命令、状态、应答消息
├── launch/    # 启动配置
├── config/    # 标定表
├── sdk/       # 配套头文件及 x86_64/ARM64 动态库
└── example/   # 命令行示例和逻辑测试
```

## 构建与启动

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon --log-base log_real build --packages-select gripper --build-base build_real --install-base install_real --symlink-install
source install_real/setup.bash
ros2 launch gripper gripper.launch.py
```

单夹爪 launch 默认选左夹爪（ID 1、左表），也可用 `side:=right` 选右夹爪（ID 2、右表）。默认串口 `/dev/gripper_rs485`、115200 波特率、10 Hz 目标反馈频率、电流阈值 0.5 A。默认使用已有零位（`use_existing_zero=true`），不自动使能或运动。
运动速度由 `gripper.launch.py` 的 `speed_mps` 配置，默认 `0.002 m/s`（2 mm/s），启动时可覆盖。速度参数只读，修改需重启节点；每条 ROS 控制消息不再携带速度。
若不使用已有零位，可显式关闭：

```bash
ros2 launch gripper gripper.launch.py use_existing_zero:=false
```

### Tianji 单夹爪启动命令

已在 `/dev/gripper_rs485`、右夹爪 85 mm 标定表下验证启动。先确认没有其他程序占用串口，并核对设备现有零位仍有效；`use_existing_zero:=true` 不会重新标零。

```bash
cd ~/wxy/tianjizhineng/ros2_ws
source /opt/ros/humble/setup.bash
source install_gripper_cal/setup.bash

ros2 launch gripper gripper.launch.py side:=right
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

动作编号：MOVE=0、ZERO=1、ENABLE=2、DISABLE=3、CLEAR=4。
MOVE 的 ACK 只表示已接受，需读取 state 判断结果。

确认零位后先使能，再发送移动指令：

```bash
ros2 topic echo /gripper/state
ros2 topic echo /gripper/ack
# 在其他已加载环境的终端逐条发送，检查 ACK 后继续
ros2 topic pub --once /gripper/command gripper/msg/GripperCommand '{action: 2}'
ros2 topic pub --once /gripper/command gripper/msg/GripperCommand '{action: 0, position_m: 0.060}'
ros2 topic pub --once /gripper/command gripper/msg/GripperCommand '{action: 3}'
```

开度单位 m，范围由各自标定表决定（左 0～0.098，右 0～0.085）；速度通过 launch 参数 `speed_mps` 配置，默认 0.005 m/s（5 mm/s），必须有限且大于零；控制消息只包含 `action` 和 `position_m`。大开度为张开，小开度为闭合。
标零发送 `action: 1`，清错发送 `action: 4`。ZERO 主动张开并检查已有标定端点，完成后失能；它不是未知零点时的通用恢复流程。

## 状态及保持

state 输出开度、原始电机角度、电流、力矩百分比、估算力、错误码、零位、使能、到位、目标和保持状态。
`valid=false` 时不能使用测量值、使能和错误码；无效浮点测量输出 NaN。`target_active=false` 时目标开度为 NaN。时间戳使用节点 ROS 时钟。

电流绝对值达到阈值时，捕获一次原始电机位置并保持；原 MOVE 目标被替换，`current_limit_reached`、`holding_position` 置位，`at_target=false`。电流回落不会恢复旧目标。新 MOVE 重置触发状态；失能、标零或故障取消保持。此功能不是硬件限流或恒力夹持，力为模型估算。

SDK 和 Controller 由单线程执行器串行访问。ZERO 最多约 10 s，ENABLE 约 500 ms，期间暂停反馈和命令处理；正常反馈周期也受串口耗时影响。命令队列深度 10，不适合高频轨迹流或硬件急停。通信故障和退出时尝试失能，断线时不能保证指令到达。

## 同一路串口连接左右夹爪

先将实际设备设置为左 ID 1、右 ID 2，波特率一致。软件参数不会修改设备地址。
未设置前应分别单独接线设置地址，不能将两台相同 ID 的设备同时接入通信。

一个节点创建两个驱动实例，SDK 按同一串口复用 Rs485Bus，应用层使用单线程串行调用。不要启动两个进程争用同一串口。

```bash
ros2 launch gripper gripper.launch.py # 原单夹爪模式保持可用
# 双夹爪模式：左表 98 mm、右表 93 mm
ros2 launch gripper dual_gripper.launch.py port:=/dev/ttyUSB0 left_id:=1 right_id:=2
```

双夹爪模式默认不自动确认零位、使能或运动。确认各自零位有效后，可添加
`left_use_existing_zero:=true right_use_existing_zero:=true`。
参数支持 `left_calibration_path`、`right_calibration_path`、`left_current_threshold_a`、`right_current_threshold_a`。
左右速度通过 `left_speed_mps`、`right_speed_mps` 分别设置，均默认 `0.005 m/s`，启动后只读。
左夹爪默认使用 `config/calibration.csv`（98 mm），右夹爪默认使用 `config/right_calibration.csv`（85 mm）；可通过 `left_calibration_path`、`right_calibration_path` 分别覆盖。右表已重新按当前实测全开 85 mm、全闭 0 mm、闭合角度 56.688259 rad 生成；中间开度来自机构模型，尚未在新端点下实测。旧 93 mm 表已备份。设备重启后必须重新确认零位。

| 夹爪 | ID 默认值 | 输入话题 | 输出话题 |
|---|---|---|---|
| 左 | 1 | `/left_gripper/command` | `/left_gripper/state`、`/left_gripper/ack` |
| 右 | 2 | `/right_gripper/command` | `/right_gripper/state`、`/right_gripper/ack` |

```bash
ros2 topic echo /left_gripper/state
ros2 topic echo /right_gripper/state
# 确认零位后，逐条发送并检查对应 ACK
ros2 topic pub --once /left_gripper/command gripper/msg/GripperCommand '{action: 2}'
ros2 topic pub --once /right_gripper/command gripper/msg/GripperCommand '{action: 2}'
ros2 topic pub --once /left_gripper/command gripper/msg/GripperCommand '{action: 0, position_m: 0.060}'
```

左右各自保存标定、目标、电流触发和零位状态。单侧故障由该侧 Controller 尝试失能；总线断线可能同时影响两侧。
反馈按左右顺序轮询，目标频率是每侧 10 Hz，实际受两侧通信总耗时影响。任意一侧 ZERO/ENABLE 阻塞期间，另一侧也暂停反馈和命令处理，因此不是两个独立实时控制回路。
SDK 的总线复用符号已检查，双设备物理通信、退出后的设备状态仍需实机验证。

## 测试

```bash
colcon --log-base log_real test --packages-select gripper --build-base build_real --install-base install_real
colcon test-result --test-result-base build_real/gripper --verbose
```

复用原状态机与标定测试，不连接硬件。整合版本仍需 ARM64 及真实夹爪验证。

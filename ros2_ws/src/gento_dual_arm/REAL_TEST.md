# 真机测试与 ROS 2 接口

单臂、双臂脚本已合并到 `gento_dual_arm` 功能包。构建后加载 `ros2_ws/install_real/setup.bash`。SDK 默认从仓库定位，独立部署时设置 `GENTO_SDK_ROOT`。


## 构建与启动

以下命令从仓库根目录开始，需要 ROS 2 Humble 和匹配控制器固件、主机架构的 SDK 动态库。

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon --log-base log_real build --base-paths src --packages-select gento_dual_arm \
  --build-base build_real --install-base install_real --symlink-install
source install_real/setup.bash

# 连接并发布反馈，收到合法目标后直接执行
ros2 launch gento_dual_arm dual_arm.launch.py
```

控制器 IP 在 [dual_arm.launch.py](launch/dual_arm.launch.py) 的 `CONTROLLER_IP` 中设置，当前为 `6.6.7.100`。此默认地址不代表已验证的设备地址；请按实际配置修改。SDK 默认从仓库定位，独立部署时传入 `sdk_root:=SDK绝对路径` 或设置 `GENTO_SDK_ROOT`。

```bash
# 临时覆盖 IP
ros2 launch gento_dual_arm dual_arm.launch.py ip:=6.6.7.100

```

也可从 `ros2_ws` 目录执行 `./run_real_robot.sh --build`，后续使用 `./run_real_robot.sh`。连接成功后依次复位双臂与躯干错误、确认 IDLE、初始化模型并读取限位，再切换双臂与躯干到 POSITION。启动不会生成运动目标或回零；复位失败保持故障锁，模式切换失败或已尝试进入 POSITION 时会请求双臂与躯干软急停并进入 fault；连接成功时保留清错话题。ROS 节点与单臂、双臂命令行脚本应分别运行，不要同时争用 SDK 连接。

## ROS 2 话题接口

默认节点名为 `/gento_dual_arm`。新终端先加载 ROS 2 和 `ros2_ws/install_real/setup.bash`。

### 控制输入

| 话题 | 消息类型 | 内容 |
|---|---|---|
| `/gento_dual_arm/left/target_joints` | `sensor_msgs/msg/JointState` | 左臂 7 个绝对关节目标，rad |
| `/gento_dual_arm/right/target_joints` | `sensor_msgs/msg/JointState` | 右臂 7 个绝对关节目标，rad |
| `/gento_dual_arm/left/target_pose` | `geometry_msgs/msg/PoseStamped` | 左臂绝对 TCP 目标，m、四元数 |
| `/gento_dual_arm/right/target_pose` | `geometry_msgs/msg/PoseStamped` | 右臂绝对 TCP 目标，m、四元数 |
| `/gento_dual_arm/stop` | `std_msgs/msg/Empty` | 软件停止并锁定后续命令 |
| `/gento_dual_arm/reset_error` | `std_msgs/msg/Empty` | 显式复位双臂和躯干错误，成功后解除命令锁 |

### 反馈输出

| 话题 | 消息类型 | 内容 |
|---|---|---|
| `/gento_dual_arm/joint_states` | `sensor_msgs/msg/JointState` | 双臂当前 14 个关节角，rad；速度、力矩数组为空 |
| `/gento_dual_arm/head/joint_states` | `sensor_msgs/msg/JointState` | 头部实际关节角，rad |
| `/gento_dual_arm/full_joint_states` | `sensor_msgs/msg/JointState` | 头部、躯干、双臂合并关节角，rad |
| `/gento_dual_arm/left/tcp_pose` | `geometry_msgs/msg/PoseStamped` | 左臂关节反馈经 FK 计算的 TCP 位姿 |
| `/gento_dual_arm/right/tcp_pose` | `geometry_msgs/msg/PoseStamped` | 右臂关节反馈经 FK 计算的 TCP 位姿 |
| `/gento_dual_arm/status` | `std_msgs/msg/String` | JSON 状态及原因或规划结果 |
| `/gento_dual_arm/joint_errors` | `std_msgs/msg/String` | JSON 对象错误类型、各轴伺服故障码及说明 |

状态包括 `ready`、`waiting_for_pair`、`running`、`completed`、`rejected`、`stopped`、`fault`。反馈默认 20 Hz；同步 SDK 调用期间可能延迟，TCP 不是外部传感器测量结果。

### 目标格式与运动方式

关节目标只需包含所选手臂的 7 个名称，顺序可变，节点按名称匹配；左右臂名称分别为：

```text
left_joint_1 left_joint_2 left_joint_3 left_joint_4 left_joint_5 left_joint_6 left_joint_7
right_joint_1 right_joint_2 right_joint_3 right_joint_4 right_joint_5 right_joint_6 right_joint_7
```

`position` 单位为 rad，`velocity`、`effort` 必须为空。关节目标仅填写对应手臂，另一臂保持接收命令时的当前角度。

末端目标使用单位四元数 `x, y, z, w`；左臂 `frame_id` 为 `left_arm_sdk_base`，右臂为 `right_arm_sdk_base`。这是各自的 SDK 基坐标系，不是共同世界坐标系，节点不执行 TF 转换。TCP 使用初始化后的 SDK 工具定义；SDK 长度转米系数 `sdk_length_to_m` 默认为 `0.001`，需确认实际模型单位。

左右末端目标必须具有完全相同的 `header.stamp`，配齐后才处理；缺少另一臂时等待，超过有效期后拒绝。只移动一臂时，另一臂填写当前 TCP。

所有运动目标必须带新时间戳，默认有效期 `command_max_age=1.0` 秒。零时间戳、启动前或上一动作完成前积压的消息、过期和明显未来消息会被拒绝。忙碌时不排队，跨机器发布需要同步系统时钟。

末端目标先以当前 7 个关节角为参考调用 IK，检查不可达、奇异、超限标志，再以 FK 回代检查误差。随后与关节目标一样执行 MoveJ 规划和轨迹校验。启动连接后自动复位双臂与躯干错误并切换 POSITION；接收目标时接受 IDLE 或 POSITION，必要时从 IDLE 切换 POSITION，到位并完成轨迹持续 0.5 秒后保持 POSITION；不保证末端直线、两臂同时到达或两手相对位姿不变。当前没有头部或灵巧手控制话题，也没有碰撞检测。

### 发布目标、查看反馈与停止

使用一次性发布器自动填写新时间戳；末端目标会自动填入相同时间戳。以下大写变量是占位符，需要替换成根据当前反馈确定的数值：

```bash
# 分别发送左臂或右臂 7 个绝对关节角，单位 rad；待完成后再发下一条。
ros2 run gento_dual_arm send_target --left-joints L1 L2 L3 L4 L5 L6 L7
ros2 run gento_dual_arm send_target --right-joints R1 R2 R3 R4 R5 R6 R7

# 每只手臂依次为 x y z qx qy qz qw
ros2 run gento_dual_arm send_target --poses \
  LX LY LZ LQX LQY LQZ LQW RX RY RZ RQX RQY RQZ RQW

ros2 topic echo /gento_dual_arm/joint_states
ros2 topic echo /gento_dual_arm/left/tcp_pose
ros2 topic echo /gento_dual_arm/right/tcp_pose
ros2 topic echo /gento_dual_arm/status

ros2 topic pub --once /gento_dual_arm/stop std_msgs/msg/Empty '{}'
```

发布器完成只代表消息已发布，需查看 `status` 判断接受或完成。ROS 节点已移除预览模式和 `execute` 参数，合法目标直接执行；停止话题请求双臂和躯干软件急停并锁定命令。停止、超时或故障后可人工排除原因并使用 reset_error 话题恢复，运行中不自动复位；重新启动节点会重新执行连接后的错误复位和位置模式设置。同步规划、上传或模式切换可能延迟停止回调，不能代替硬件急停。

下面独立命令行脚本仍保留 `--execute` 执行开关。ROS 话题使用 m/rad；下文单臂、双臂命令行脚本的关节输入使用度，两者单位不能混用。

## 躯干控制接口

| 方向 | 话题 | 消息类型 |
|---|---|---|
| 输入 | `/gento_dual_arm/body/target_joints` | `sensor_msgs/msg/JointState` |
| 输出 | `/gento_dual_arm/body/joint_states` | `sensor_msgs/msg/JointState` |

名称为 `body_joint_1` 至 `body_joint_6`，按 SDK 顺序对应，目标必须包含全部 6 关节，位置单位 rad。目标需带新时间戳，速度和力矩数组留空。先读取实际反馈，再确定目标：

```bash
ros2 topic echo /gento_dual_arm/body/joint_states
# B1..B6 为需替换的绝对目标角，rad
ros2 run gento_dual_arm send_target --body B1 B2 B3 B4 B5 B6
```

躯干通过 SDK BODY 位置接口控制，不使用手臂逆解或 50 Hz 规划；限位、单次位移、超时及到位容差均检查。双臂和躯干动作串行，忙碌时拒绝其他目标；`status` 中 `group` 区分 `arms` 和 `body`。详细启动与恢复约束见 [工作空间说明](../../README.md)。

## 双臂与躯干软件急停

在已加载 ROS 和工作空间环境的终端中运行：

```bash
ros2 topic pub --once /gento_dual_arm/stop std_msgs/msg/Empty '{}'
```

此接口需要 `gento_dual_arm` 节点正在运行并保持 SDK 连接。节点调用 `robot.emergency_stop(11)` 请求停止双臂与躯干，检查返回掩码并锁定后续运动命令。查看 `/gento_dual_arm/status`：`stopped` 表示 SDK 已确认双臂与躯干急停，`fault` 表示发生异常；发布命令成功不代表机器人已停止。

该话题不停止头部、升降机构或独立灵巧手，也不能停止另一个独立命令行脚本的 SDK 会话。命令行测试仍使用自身 Ctrl+C 异常停止流程。软件急停依赖节点、网络及 SDK，同步调用可能延迟回调，不能代替硬件急停。

停止后先排除原因，再发送 reset_error 或退出并重新启动；连接后自动复位双臂与躯干错误并切换 POSITION，进入 ready 后需重新发送目标，不续跑旧轨迹。

## 单臂真机测试

`arm_control.py` 在正逆解示例基础上增加位置模式切换、目标下发、到位监测、超时处理、异常急停和显式断开连接。Python 3.10+，仅依赖标准库与仓库 SDK；需将匹配系统架构的 `libGentoSDKPY.so` / `.dll` 放在 `PYTHON_SDK` 中。

加载上述工作空间环境后运行。脚本自动从仓库的 `tianji-robot-SDK/PYTHON_SDK` 加载 SDK；独立部署时设置 `GENTO_SDK_ROOT`。将以下 IP 换成实际控制器地址：

```bash
# 只读当前左臂关节角和末端位姿
ros2 run gento_dual_arm arm_control --ip 6.6.7.190 --arm 0

# 预览：左臂第一个关节相对当前位置增加 1 度
ros2 run gento_dual_arm arm_control --ip 6.6.7.190 --arm 0 --delta 1 0 0 0 0 0 0

# 执行同一相对动作（以本次运行读到的位置为起点）
ros2 run gento_dual_arm arm_control --ip 6.6.7.190 --arm 0 --delta 1 0 0 0 0 0 0 --execute
```

- `--arm 0/1`：左臂/右臂，每次只操作一只手臂。
- `--joints J1 J2 J3 J4 J5 J6 J7`：绝对关节角，单位为度。
- `--delta D1 D2 D3 D4 D5 D6 D7`：相对当前关节角增量，单位为度。
- `--tcp X Y Z A B C`：目标绝对 TCP 位姿，调用 SDK 的 XYZABC 转换和逆解，以当前关节角为参考，再发送关节位置目标。XYZ 与模型长度单位一致，ABC 为度；坐标系与 SDK 当前手臂 FK 输出一致，不是任意世界坐标系。脚本未另外配置工具变换，使用初始化后的 SDK 工具定义。
- 三种目标互斥。默认仅连接、读取参数和计算，不切换模式、不发送运动指令；`--execute` 是执行开关。
- `--speed` / `--acc` 默认为 5%，脚本上限 10%；`--max-delta` 默认限制每个关节单次位移为 5 度。另从控制器读取每个关节的正负限位。
- `--timeout` 默认 30 秒，`--tolerance` 默认 0.2 度，各关节误差连续 0.5 秒满足容差才判定到位。成功后切换 IDLE，再释放资源并断开。

执行前必须由操作员确认手臂处于 IDLE、运动空间无障碍且硬件急停可用。程序不会自动复位故障或接管正在其他模式运行的手臂。Ctrl+C、超时或运动异常会尝试对所选手臂发送软件急停；通信失效时无法保证软件急停送达，应使用硬件急停。失败后不自动复位或再次切换模式。

TCP 模式执行的是关节空间位置运动，**不保证末端沿直线移动**。代码没有碰撞检测、自碰撞检测或完整路径验证；终点合法不代表途中无碰撞。若需要末端直线运动，应另用 SDK 的线性规划、轨迹上传及执行接口。

逆解失败的真实返回值可能是整数错误码，因此脚本检查结果类型和超限标记，并以 FK 回代验证（位置误差阈值 0.1 个模型长度单位，旋转矩阵元素误差阈值 0.001）。这不是精度标定结果。

离线测试使用 ROS 2 桥接包中的 fake SDK 用例，不连接硬件、不加载动态库。
从仓库根目录运行：

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros2_ws/src/gento_dual_arm:$PYTHONPATH" python3 -m pytest \
  ros2_ws/src/gento_dual_arm/test/test_bridge.py -q -k 'not ros_topics_and_stamps'
```

这些用例包含目标校验、轨迹执行与异常清理；真机行为仍需现场验证。

## 双臂控制

`dual_arm_control.py` 使用一个 SDK 实例连接控制器，对左臂（0）和右臂（1）分别进行 MoveJ 规划，然后分别上传，用双臂掩码 `3` 统一请求启动。支持绝对关节目标或相对增量；参数始终是**左臂 7 个数，然后右臂 7 个数**，单位为度。

```bash
# 读取双臂当前关节角与 TCP，不运动
ros2 run gento_dual_arm dual_arm_control --ip 6.6.7.190

# 预览：左右臂的第一关节分别增加 1 度
ros2 run gento_dual_arm dual_arm_control --ip 6.6.7.190 \
  --delta 1 0 0 0 0 0 0  1 0 0 0 0 0 0

# 真机执行上述双臂增量动作
ros2 run gento_dual_arm dual_arm_control --ip 6.6.7.190 \
  --delta 1 0 0 0 0 0 0  1 0 0 0 0 0 0 --execute
```

用 `--joints` 替代 `--delta` 可传入 14 个绝对关节角。左右臂方向可能镜像；相同角度增量不等于相同空间运动方向。参数必须根据当前真机姿态确定。

执行过程：

1. 读取双臂反馈和模型，检查每个关节目标与整条离散轨迹的关节限位、单次位移上限和有限数值。
2. 默认预览；只有 `--execute` 才要求两臂均为 IDLE，检查起点未变化后切换 POSITION。
3. 两条轨迹全部上传后，再次检查连接、状态和起点，统一发送启动请求；任一臂启动失败就尝试急停双臂。
4. 监测双臂状态、连接、轨迹完成状态和最终角度。两臂到位且轨迹完成持续 0.5 秒后，分别返回 IDLE。
5. 执行期间 Ctrl+C、超时、单臂异常、上传失败或模式切换部分失败都会尝试急停双臂。最终销毁运动学句柄并断开连接。

默认控制器速度/加速度比例均为 5%，规划速度/加速度比例均为 0.05，规划采样频率为 50 Hz，等待超时 60 秒。控制器比例与规划比例分别作用，实际执行时间需真机验证，可用 `--timeout` 调整等待上限。

**能力边界：**此程序是双臂独立关节轨迹的统一启动，不是双臂同时到达规划，不保证刚性搬运时两手相对位姿不变，也不保证末端直线运动。SDK 启动返回掩码可以表示部分成功，因此程序专门处理此情况。离散关节限位检查不等于碰撞检查；本程序未包含左右臂互撞、机身/环境碰撞检测，也不应假定调用 SDK 会经过 Apex QP 保护。用于现场已确认无碰撞的低速、小幅动作测试；不得与 Apex 遥操等其他控制程序同时争用控制器。头部、机身与升降模块应保持稳定。

预览和离线单元测试不能代替真机联调；运行前需确认动态库、固件版本和型号配置匹配。掉线时软件急停可能无法送达，仍需硬件急停。

## 重复目标报规划失败 -1021

SDK 的 `OnMovJoint: Failed to generate joint trajectory` 曾将“所有关节位移均小于 0.1 度”误报为关节超限（-1021）。这可能发生在首次到位后重复发送绝对目标，或仅移动一臂、另一臂保持原位时。修复后的规划器对非零微小位移按速度/加速度规划，对完全相同的起终点生成保持轨迹，仍检查起终点和轨迹的关节限位；同时修正加速度比例误用速度比例的问题。

更新后重新构建动态库并重启 bridge，使进程加载新库：

```bash
bash ros2_ws/build_gento_sdk.sh
python3 tianji-robot-SDK/tests/run_joint_planner_regression.py
```

该回归测试离线检查零位移、微小位移、普通位移、真实超限拒绝与加速度比例生效，不连接机器人。若新库仍报 -1021，需要对照当前反馈、目标及 SDK 模型限位进一步定位；该错误仍可表示实际超限。

## 单臂保持与重复目标处理

关节目标已拆分为左右臂各 7 个关节，末端目标仍需左右配对。节点先检查两臂全部目标的有限值、关节限位和单次位移，再判断哪些手臂需要运动。

如果某臂所有关节的目标误差均不超过 `tolerance`（默认 0.2°），该臂视为已到位，不调用规划、不上传轨迹、不切换模式，也不加入启动掩码。仅左臂运动时启动掩码为 1，仅右臂为 2，两臂均运动为 3。实际运动的手臂完成后保持 POSITION，未运动手臂保持原模式。执行期间继续监测两臂目标误差。

如果两臂均已在容差内，直接报告 `completed`，不下发运动。状态包含 `moving_arms`（0 为左臂、1 为右臂）、`run_mask` 和 `points`；未运动手臂的点数为 0。该处理避免将重复目标或反馈微小抖动交给 SDK 生成零位移轨迹；其他规划错误仍会拒绝，不绕过限位检查。

启动恢复补充：SDK 的 `reset_error()` 等待 IDLE，不保证将正常 POSITION/PD 等模式切回 IDLE。连接后对非 IDLE、非 ERROR 的受控对象调用 `switch_to_idle()`，不额外清错；仅 ERROR 对象调用复位。单纯复位超时进入 fault 并保留清错话题，不重复发送急停，错误信息包含对象名及复位前后状态，不因系统错误码为 0 而忽略失败。

关节接口变更：已删除旧 `/gento_dual_arm/target_joints` 订阅及 `send_target --joints` 参数。改用 `/gento_dual_arm/left/target_joints` 或 `/gento_dual_arm/right/target_joints`，每条只含该臂 7 个绝对角（rad），需新时间戳，velocity/effort 留空。不需要双臂时间戳配对，命令到达后独立执行；共用忙碌锁，执行中其他手臂或躯干目标会被拒绝，不保证两臂同时启动。末端位姿接口的配对规则不变，独立命令行 `arm_control`/`dual_arm_control` 的 `--joints` 参数不受影响。

## 头部与整机关节反馈

新增 `/gento_dual_arm/head/joint_states` 和 `/gento_dual_arm/full_joint_states`，均为 `sensor_msgs/msg/JointState`，默认 20 Hz，位置为 rad，速度/力矩数组为空。保留 `/gento_dual_arm/joint_states`（双臂 14 关节）和 `/gento_dual_arm/body/joint_states`（躯干 6 关节）。

头部自由度通过 `get_ctrl_obj_dof(OBJ_HEAD)` 读取，仅发布实际关节数，不把 SDK 固定长度数组的占位元素当作关节。当前设备头部 2 关节，名称 `head_joint_1`、`head_joint_2`。合并顺序：头部 → `body_joint_1..6` → `left_joint_1..7` → `right_joint_1..7`，当前共 22 个关节。无头部设备只合并其余关节。

同一次回调从一次 SDK RT 字典读取各部分，使用相同 ROS 时间戳；不代表控制器底层结构体是原子快照。名称是桥接约定，未自动映射到 URDF。只新增头部反馈，不复位或切换头部模式，不新增头部控制目标。

```bash
ros2 topic echo /gento_dual_arm/head/joint_states
ros2 topic echo /gento_dual_arm/body/joint_states
ros2 topic echo /gento_dual_arm/joint_states
ros2 topic echo /gento_dual_arm/full_joint_states
```

## 显式清错与恢复话题

先处理故障原因，确保设备可恢复，再发送一次：

```bash
ros2 topic pub --once /gento_dual_arm/reset_error std_msgs/msg/Empty '{}'
ros2 topic echo /gento_dual_arm/status --qos-durability transient_local
```

该话题需要桥接节点存活、SDK 仍连接。运动中拒绝清错，需先 stop 并确认停止。清错范围为双臂和躯干，不包括头部。节点清除未配对目标，对非 IDLE/ERROR 对象先切 IDLE；仅对 ERROR 对象调用 reset_error，重新初始化运动学、读取限位、检查反馈并设置 POSITION；全部成功才解除故障锁、恢复反馈定时器并报告 ready。不上传轨迹或续跑旧目标，之后需重新发送新时间戳目标。

清错失败保持 fault；若模式切换结果不确定或已尝试进入 POSITION，再请求受控对象软急停，不能绕过伺服错误 102 等故障。连接成功但启动初始化失败时节点保留该接口；未连接成功则无法使用，需要排除网络/连接占用并重新启动。网络断开后的复位不会自动重连。

## 对象与关节故障反馈

```bash
ros2 topic echo /gento_dual_arm/joint_errors --qos-durability transient_local
```

消息为 `std_msgs/msg/String`，data 是 JSON，独立定时器默认每秒查询一次，保留最近一条。反馈、初始化或清错失败进入 fault 时仍尝试发布；链路失效时报告 available=false，不能当作无故障。SDK 同步清错或诊断调用期间发布可能延迟。

顶层含 stamp（ROS 采样时间）、available、objects。objects 按 left、right、body、head 排列，无头部设备不含 head。每组含 object_id、state（SDK 归一化状态）、state_name、error_code（对象系统错误码）、error_type（说明）、servo_available、joints。例如对象错误码 102 表示 Servo error，只能说明该对象发生伺服错误，具体轴见 joints。

每个关节含 name、servo_error_code（整数）、servo_error_hex（十六进制）、error_type（SDK 提供的说明）。头部按控制器实际自由度截取。此话题只读，不自动清错。详细伺服读取失败或不完整时 servo_available=false，附 servo_reason，不把未知故障伪装成零错误。

为避免额外同步查询延迟执行与停止回调，运动期间只报告对象错误码，暂缓各轴伺服查询，servo_available=false 并注明 deferred；停止或动作完成后恢复查询。此话题与 status 的命令执行状态分开，不替代硬件急停或现场检查。

清错来源日志：启动日志标注 `source=startup`；收到 `/gento_dual_arm/reset_error` 时打印 `Received reset_error topic request #N`，结果状态带 `source=reset_error_topic`、`request_id`。失败状态还包含 `recovery_stage` 和 `stop_requested`，便于区分清错失败与程序主动请求急停。

清错失败策略：已有 ERROR 的复位失败或模型/限位读取失败，若尚未尝试 POSITION 且没有未确认的模式切换，则保持 fault 和命令锁，不无条件再发急停。切 IDLE 失败、进入 POSITION 过程中失败或后续反馈验证失败，仍请求掩码 0x0B 软急停。stop 话题、执行异常和部分启动失败的急停行为不变。不会忽略 102 伺服错误、自动重试或自动续跑动作。

## 状态恢复与退出流程

启动与显式 reset_error 采用原厂示例的状态分支：ERROR 才调用 SDK reset_error；正常 IDLE 不复位；其他模式先切 IDLE并确认，不额外复位。随后完成模型、限位与反馈检查并设置 POSITION。复位失败仍保持故障锁，不绕过已有错误。

正常 Ctrl+C 退出时先停止节点定时回调，逐个检查双臂和躯干：正常非 IDLE 对象切回 IDLE并确认；已有 ERROR 保留、不自动清错。之后 cleanup 销毁运动学资源，unlink 断开通信。退出中的任何 IDLE 切换失败都会尝试受控对象软急停，仍释放 SDK 资源并记录错误；不会因失败而继续运动。运动中退出维持软急停流程，不自动解除 ERROR。链路断开时不能确认 IDLE，会明确报告；重复清理不会重复操作控制器。

这解决正常退出时对象仍留在 POSITION 的软件遗漏，不代表已经证明或解决控制电源欠压、PDO 异常等现场故障。

## 与厂家按钮一致的独立清错

新增 3 个 std_msgs/msg/Empty 输入话题，每次只对所选对象调用 `robot.reset_error(obj, 1000)`，不切模式、不规划、不自动急停其他对象：

```bash
ros2 topic pub --once /gento_dual_arm/left/reset_error std_msgs/msg/Empty '{}'
ros2 topic pub --once /gento_dual_arm/right/reset_error std_msgs/msg/Empty '{}'
ros2 topic pub --once /gento_dual_arm/body/reset_error std_msgs/msg/Empty '{}'
```

逐条操作并检查 status、joint_errors。成功确认所选对象 IDLE 后发布 reset_completed；这不代表整机 ready，运动锁保持，待处理完原因和其他对象后发送 `/gento_dual_arm/reset_error` 完成整组初始化并恢复 POSITION。已有动作时拒绝独立清错。连接失败无法清错，伺服故障未解除时保持 fault。

整组恢复也已调整：若某对象的 ERROR 复位失败，继续尝试其他 ERROR 对象并汇总失败原因；任一失败则不初始化模型、不进入 POSITION、不解除锁。非 ERROR 模式的切 IDLE 失败仍按模式切换失败处理，不盲目继续。独立话题可在 ERROR、IDLE 等状态下直接发送复位请求，行为与厂家 Reset 按钮一致。

ROS 节点在动作完成后保持 POSITION，仅更新 completed 并解除忙碌锁；收到下一目标无需先退回 IDLE。正常退出、清错准备阶段的 IDLE 切换仍保留，stop/异常处理不变。独立命令行 arm_control、dual_arm_control 脚本仍在任务完成后回 IDLE。

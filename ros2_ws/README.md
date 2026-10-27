# Gento 双臂 ROS 2 功能包

`real` 分支的专用入口为 `./run_real_robot.sh`，连接后等待目标，合法目标直接执行；
完整流程见 [双臂真机使用说明](REAL_ROBOT.md)。

使用本仓库 Gento SDK，运行前需确认源码、动态库与控制器固件匹配；ROS 环境为 ROS 2 Humble / Python 3.10。包含：

- `gento_dual_arm`：单 SDK 连接的双臂控制节点，以及一次性目标发布器。

控制节点复用包内 `dual_arm_control.py` 的限位读取、轨迹规划和校验函数。单臂、双臂命令行入口分别为 `ros2 run gento_dual_arm arm_control` 和 `ros2 run gento_dual_arm dual_arm_control`，详见 [测试说明](src/gento_dual_arm/REAL_TEST.md)。运行需要原厂 SDK 的 `PYTHON_SDK` 和匹配动态库；节点通过 `sdk_root` 指定 SDK 目录，命令行工具独立部署时通过 `GENTO_SDK_ROOT` 指定。

## 构建与启动

先准备与本机 CPU/系统匹配的 `PYTHON_SDK/libGentoSDKPY.so`，并确认固件兼容。从仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
./run_real_robot.sh --build
```

控制器 IP 在 [dual_arm.launch.py](src/gento_dual_arm/launch/dual_arm.launch.py) 的 `CONTROLLER_IP` 中设置，当前默认 `6.6.7.100`。可临时覆盖：

```bash
./run_real_robot.sh ip:=实际控制器IP
```

构建后也可直接使用 launch：

```bash
source install_real/setup.bash
ros2 launch gento_dual_arm dual_arm.launch.py
```

每个使用话题的新终端都需先加载 ROS 和工作空间环境（以下从仓库根目录执行）：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install_real/setup.bash
```

节点启动时连接控制器、复位双臂与躯干错误并切换位置模式，随后发布反馈，收到合法目标后直接规划、上传和执行，不再提供 `execute` 参数或预览模式。

每次连接成功后检查双臂及躯干状态，仅对 ERROR 对象调用 reset_error，确认 IDLE 后初始化模型、读取限位并切换 POSITION；任一步失败则请求双臂与躯干软急停并进入 fault；连接成功时保留清错话题。运行中故障不自动复位；重新启动节点会重新执行连接后的错误复位和位置模式设置。不得与命令行控制程序或 Apex 遥操同时争用 SDK 控制连接。

## 交叉编译 ARM SDK

ROS 功能包主要是 Python；需要针对 ARM 构建的是原厂 `libGentoSDKPY.so`。从仓库根目录运行：

```bash
# 在 x86_64 Ubuntu/Debian 编译机上安装 ARM64 交叉工具链
sudo apt-get install g++-aarch64-linux-gnu

# 默认构建 ARM64（aarch64）
./ros2_ws/cross_build_gento_sdk.sh
```

产物为 `ros2_ws/artifacts/gento-sdk/arm64/libGentoSDKPY.so`。脚本检查编译器目标和产物 ELF 架构，成功后替换输出文件；不覆盖仓库 `PYTHON_SDK` 中当前主机使用的 x86_64 库。

支持指定 SDK、工具链、目标 sysroot 和输出目录：

```bash
./ros2_ws/cross_build_gento_sdk.sh --arch arm64 \
  --compiler /opt/toolchain/bin/aarch64-linux-gnu-g++ \
  --sysroot /opt/target-sysroot \
  --output-dir /tmp/gento-arm64

# 如目标确实是 32 位 ARM hard-float，改用 armhf
sudo apt-get install g++-arm-linux-gnueabihf
./ros2_ws/cross_build_gento_sdk.sh --arch armhf
```

ARMHF 默认输出到 `ros2_ws/artifacts/gento-sdk/armhf/`。工具链及 sysroot 应与目标设备 Linux/glibc 版本匹配；静态链接 libstdc++ 和 libgcc 不意味着不依赖目标 glibc。该脚本不编译 ROS 2 本身。

将对应架构产物复制到 **ARM 目标设备** 的 `tianji-robot-SDK/PYTHON_SDK/libGentoSDKPY.so`，在目标设备安装匹配的 ROS 2/Python，运行 `./run_real_robot.sh --build` 构建 Python 包。不要把 ARM 库覆盖到仍需在 x86_64 上运行的 SDK 目录。编译和 ELF 检查不能代替目标设备的加载、固件兼容及真机验证。

## 当前 ROS 2 话题接口

默认节点名 `/gento_dual_arm`；可以使用 ROS 重映射改变话题名。下表“发布/订阅”均以桥接节点为视角，共 10 个控制输入、8 个反馈输出。当前控制双臂与躯干；没有头部、升降机构或灵巧手控制话题。

| 方向 | 话题 | 类型 | 内容/单位 |
|---|---|---|---|
| 发布 | `/gento_dual_arm/body/joint_states` | `sensor_msgs/msg/JointState` | 躯干 6 个当前关节角，rad |
| 发布 | `/gento_dual_arm/joint_states` | `sensor_msgs/msg/JointState` | 14 个当前关节位置，**rad**；速度/力矩数组留空 |
| 发布 | `/gento_dual_arm/head/joint_states` | `sensor_msgs/msg/JointState` | 头部实际关节角，rad |
| 发布 | `/gento_dual_arm/full_joint_states` | `sensor_msgs/msg/JointState` | 头部、躯干、双臂合并关节角，rad |
| 发布 | `/gento_dual_arm/left/tcp_pose` | `geometry_msgs/msg/PoseStamped` | 左臂 FK 位姿，**m + 四元数** |
| 发布 | `/gento_dual_arm/right/tcp_pose` | `geometry_msgs/msg/PoseStamped` | 右臂 FK 位姿，**m + 四元数** |
| 发布 | `/gento_dual_arm/status` | `std_msgs/msg/String` | JSON 状态：ready / waiting_for_pair / running / completed / rejected / stopped / fault |
| 发布 | `/gento_dual_arm/joint_errors` | `std_msgs/msg/String` | JSON 对象错误类型、各轴伺服故障码及说明 |
| 订阅 | `/gento_dual_arm/body/target_joints` | `sensor_msgs/msg/JointState` | 躯干 6 个绝对关节目标，rad |
| 订阅 | `/gento_dual_arm/left/target_joints` | `sensor_msgs/msg/JointState` | 左臂 7 个绝对关节目标，rad |
| 订阅 | `/gento_dual_arm/right/target_joints` | `sensor_msgs/msg/JointState` | 右臂 7 个绝对关节目标，rad |
| 订阅 | `/gento_dual_arm/left/target_pose` | `geometry_msgs/msg/PoseStamped` | 左臂绝对目标位姿 |
| 订阅 | `/gento_dual_arm/right/target_pose` | `geometry_msgs/msg/PoseStamped` | 右臂绝对目标位姿 |
| 订阅 | `/gento_dual_arm/stop` | `std_msgs/msg/Empty` | 请求双臂和躯干软件急停并锁定命令接收；重启前需人工恢复 |
| 订阅 | `/gento_dual_arm/reset_error` | `std_msgs/msg/Empty` | 显式复位双臂和躯干错误，成功后解除命令锁 |

### 状态消息

`/gento_dual_arm/status` 使用 `std_msgs/msg/String`，`data` 是 JSON。无预览状态。

| `state` | 含义 |
|---|---|
| `ready` | 连接、错误复位和位置模式设置完成，等待目标 |
| `waiting_for_pair` | 已收到一臂位姿，等待另一臂相同时间戳的目标 |
| `running` | 双臂轨迹启动成功，附带 `group`（`arms` 或 `body`）、`joints_deg`（目标角，度）；双臂另有 `points`（轨迹点数） |
| `completed` | 当前动作组到位持续 0.5 秒并保持 POSITION；双臂还检查轨迹完成 |
| `rejected` | 目标校验或规划失败，`reason` 说明原因 |
| `stopped` | 双臂和躯干 SDK 软急停已确认，节点锁定命令 |
| `fault` | 连接、执行或停止异常，`reason` 说明原因；节点锁定命令 |

例如启动完成时：`{"state": "ready"}`。`points` 是 SDK 按 50 Hz 采样频率自动生成的轨迹点数，不是用户指定点数。完整轨迹上传给控制器执行，ROS 不以 50 Hz 逐点下发。

状态话题保留最近一条消息（transient local），其余接口使用 reliable、volatile、队列深度 1。反馈与执行监测默认 20 Hz，同步 SDK 调用可能使其延迟。

### 关节目标与反馈

反馈由当前关节反馈经 FK 计算，不是外部测量位姿。时间戳是节点采样时的 ROS 系统时间，非控制器硬件时间戳；SDK 反馈本身不保证结构体的原子快照。没有发布 TF，也没有提供机器人 URDF。

关节目标按话题区分左右臂；每条 `JointState.name` 必须包含对应手臂全部 7 个名称，顺序可变，节点按名称匹配。双臂反馈仍包含以下 14 个关节：

```text
left_joint_1 left_joint_2 left_joint_3 left_joint_4 left_joint_5 left_joint_6 left_joint_7
right_joint_1 right_joint_2 right_joint_3 right_joint_4 right_joint_5 right_joint_6 right_joint_7
```

每条关节目标只控制所选手臂，不需要填写另一臂。另一臂以接收命令时的当前反馈保持，既不上传轨迹也不加入启动掩码。

这组名称是桥接节点约定，并非已校准 URDF 的关节名称。只接收 `position`；`velocity`、`effort` 必须为空。没有相对增量话题，若需要增量，调用方先读当前角度再计算绝对目标。

### 末端位姿目标与反馈：坐标系和单位

左右臂的反馈和目标均使用 ROS 2 标准 `geometry_msgs/msg/PoseStamped`：

```text
std_msgs/Header header
geometry_msgs/Pose pose
```

只移动一臂时，另一臂填写当前 TCP，仍需成对发送。

目标分别发送到左右两个话题，必须给这一对消息填写**完全相同的 `header.stamp`**（sec 和 nanosec 均相同）。节点收齐同一时间戳的双臂目标才规划；只收到一臂时报告 `waiting_for_pair`，不运动。不同时戳不会混合执行，更新的一组替换旧的未配对目标；超过 `command_max_age` 未配齐会拒绝并清空。发布顺序不限。`send_target --poses` 会自动在两个话题填入同一个时间戳。

- 左臂目标的 `header.frame_id` 默认必须为 `left_arm_sdk_base`。
- 右臂目标的 `header.frame_id` 默认必须为 `right_arm_sdk_base`。
- 两个 frame 分别代表对应手臂 SDK FK/IK 使用的参考坐标系。**不是共同世界坐标系**，节点没有两臂基座之间的变换，也不会调用 TF 转换。修改 frame 名只改变标识，不改变几何关系。
- 位置为米；姿态为单位四元数 `x,y,z,w`，不能把 XYZABC 的欧拉角直接填入四元数。
- `sdk_length_to_m` 默认 `0.001`，按 SDK 模型长度为毫米设置。现场必须确认模型单位；若模型已用米，设为 `1.0`。
- 使用 SDK 初始化后的工具定义，不另外设置夹爪 TCP。工具或基坐标不匹配时应先完成标定/配置。
- 两臂目标按时间戳配对后，以当前关节角作为各自 IK 参考。IK 失败、超限或 FK 回代误差过大时拒绝目标。成功后仍执行 **MoveJ 关节轨迹**，不是 MoveL，不保证末端直线或两手相对位姿不变。

## 发布目标

每次发送需要**当前时间戳**。零时间戳、过旧消息、明显未来时间戳、忙碌期间目标及前一次运动完成前生成的排队目标会被拒绝，不缓存待执行动作。默认时间有效期 1 秒；跨机器需同步系统时钟，不支持 `use_sim_time`。

提供一次性发布器，等待订阅发现后自动填入新时间戳：

```bash
# 以下分别为左右臂命令；每次仅执行一条，待 completed 后再发下一条。
ros2 run gento_dual_arm send_target --left-joints L1 L2 L3 L4 L5 L6 L7
ros2 run gento_dual_arm send_target --right-joints R1 R2 R3 R4 R5 R6 R7

# 左 7 个、右 7 个：各自为 x y z qx qy qz qw（米、四元数）。
ros2 run gento_dual_arm send_target --poses \
  LX LY LZ LQX LQY LQZ LQW RX RY RZ RQX RQY RQZ RQW
```

上述 `L1` 等是占位符，不是可直接执行的数值。建议第一次把收到的当前关节角或当前 TCP 原样作为目标，核对话题、单位和坐标系；发布有效目标会执行运动。`send_target` 只确认发布，不代表控制器接受或完成；查看状态话题。

查看反馈及状态：

```bash
ros2 topic echo /gento_dual_arm/joint_states
ros2 topic echo /gento_dual_arm/left/tcp_pose
ros2 topic echo /gento_dual_arm/right/tcp_pose
ros2 topic echo /gento_dual_arm/status

# 软件停止（请求双臂和躯干 SDK 软急停并锁定后续目标）
ros2 topic pub --once /gento_dual_arm/stop std_msgs/msg/Empty '{}'
```

## 躯干关节控制

躯干通过同一 SDK 连接使用 `FXObjType.OBJ_BODY=3`，共 6 个关节，名称为 `body_joint_1` 至 `body_joint_6`，按 SDK 数组顺序对应，不代表已标定模型名称。

`/gento_dual_arm/body/target_joints` 要求完整 6 关节，名称可乱序、不可重复；`position` 为 rad，`velocity`、`effort` 为空。消息时间戳要求与双臂相同。读取反馈后计算绝对目标，不要直接发送未经确认的零姿态。

```bash
ros2 topic echo /gento_dual_arm/body/joint_states
# B1..B6 替换为实际绝对关节角，单位 rad；发布器自动填写新时间戳
ros2 run gento_dual_arm send_target --body B1 B2 B3 B4 B5 B6
```

连接后自动复位躯干错误并设置 POSITION；从控制器 `R.B.L0..5.BASIC.LimitNeg/LimitPos` 读取限位，读取失败或无效则启动失败。目标同时检查有限值、限位和 `max_delta`（默认每关节 5°）。躯干使用 SDK `runtime_set_joint_pos_cmd(3, target_deg)` 位置命令，由控制器生成运动，不调用手臂 FK/IK，也不使用双臂的 50 Hz 轨迹规划。沿用 `speed`、`acc`、`timeout`、`tolerance`，到位持续 0.5 秒后保持 POSITION。

双臂和躯干共用忙碌/故障锁，节点不同时执行两类动作，忙碌时目标直接拒绝。状态以 `group: "arms"` 或 `group: "body"` 区分执行与完成。停止、异常和执行中退出时，急停掩码扩展为 `11`（双臂 `3` 加躯干 `8`）。

躯干动作会改变机械臂在整机中的位置；现有两臂 TCP 反馈仍是各自 SDK 基坐标下的位姿，没有新增世界坐标变换、整机协同 IK 或碰撞检测。

## 停止与恢复

`/gento_dual_arm/stop` 不需要时间戳，收到后调用 `robot.emergency_stop(11)` 请求双臂和躯干 SDK 软件急停，并锁定后续运动命令。它不同于 `runtime_stop_traj()` 的轨迹停止。该话题要求桥接节点运行并保持 SDK 连接，停止双臂和躯干，不停止头部、升降机构或灵巧手。

软急停后不能直接继续发送目标。先确认停止原因已排除并退出节点，再重新启动；新连接会调用双臂与躯干 `reset_error()`，确认 IDLE 后切换 POSITION。复位失败则不会进入 ready。节点提供 `/gento_dual_arm/reset_error` 复位话题，不会继续之前的轨迹；恢复后需发送新时间戳目标。同步 SDK 调用和通信异常可能延迟或阻止软件急停，不能代替硬件急停。

## launch 参数

全部为启动时只读参数，修改需重启节点。原脚本 `--joints` 改为目标关节话题，`--delta` 不再作为启动参数。

| 参数 | 默认 | 意义 |
|---|---|---|
| `sdk_root` | 自动定位仓库 SDK，可由 `GENTO_SDK_ROOT` 指定 | 完整 SDK 目录绝对路径；独立部署可显式传入 |
| `ip` | `6.6.7.100` | launch 中 `CONTROLLER_IP`，可用 `ip:=...` 覆盖 |
| `speed` / `acc` | 5.0 / 5.0 | 控制器百分比，允许 (0,10] |
| `plan_speed` / `plan_acc` | 0.05 / 0.05 | SDK 规划比例，允许 [0.01,0.1] |
| `max_delta` | 5.0 | 每关节相对起点的最大位移，**度** |
| `timeout` | 60.0 | 轨迹启动后等待完成时限，秒 |
| `tolerance` | 0.2 | 到位关节误差，**度** |
| `publish_rate` | 20.0 | 反馈发布及运动监测频率，Hz，上限 100 |
| `command_max_age` | 1.0 | 目标消息最大允许年龄，秒 |
| `sdk_length_to_m` | 0.001 | SDK 长度乘此系数转换成米 |
| `left_frame` / `right_frame` | left_arm_sdk_base / right_arm_sdk_base | 两臂各自 FK/IK 参考坐标系名称 |

规划采样频率沿用原脚本 50 Hz；发布频率不改变控制器轨迹采样周期。到位且两臂轨迹完成持续 0.5 秒后保持 POSITION。

## 运行边界

一个单线程 executor 串行使用 SDK。运动等待采用定时轮询，执行中持续发布反馈；**同步 SDK 的初始化、规划、模式切换和上传调用期间，反馈发布及 stop 回调可能延迟**。这不是硬实时控制或安全急停通道，硬件急停必须可用。网络断开时停止发布反馈，尝试停止双臂并进入 fault，不自动重连或复位。停止/故障后不再接受运动目标；处理原因后可发送 reset_error，若连接已丢失则重启重连。

对左右臂统一发送启动掩码，不承诺严格同时启动或同时到达。没有左右臂、机身、工具或环境碰撞检测；不能假定直连 SDK 经过 Apex QP 避碰层。头部与升降模块需保持稳定；躯干动作须确认整机稳定性。节点不执行 Home、不修改固件或标定参数。

## 离线测试

测试以 fake SDK 代替真实硬件，包含数值/四元数转换、双臂执行状态机、左右位姿话题配对/不匹配/过期处理、部分启动失败、IK 错误、关闭时停止、上传失败及超时，以及真实 ROS 2 消息的发布/订阅验证。不加载动态库、不连接机器人。

```bash
source /opt/ros/humble/setup.bash
source install_real/setup.bash
python3 -m pytest src/gento_dual_arm/test -q
```

接口/launch 文件改变后需重新 `colcon build`。软件包仅用于当前仓库内开发，`UNLICENSED` 不授予 SDK 的再分发权限。

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

## 一键 ARM64 编译与远端部署

从仓库根目录执行：

```bash
./ros2_ws/deploy_arm.sh
```

默认 SSH 用户为 `nvidia`，目标为 `192.168.2.120:/home/nvidia/wxy/tianjizhineng`，使用已有免密登录，不在脚本保存密码。

流程：检查目标为 aarch64 → 交叉编译 SDK → rsync 同步项目源码 → 单独上传 ARM64 库并检查 SHA256、ELF 和动态加载 → 在远端构建 `gento_dual_arm`。本机 x86_64 库保留。ROS 功能包主要是 Python，远端构建会生成目标机安装入口。

自动优先使用系统 `aarch64-linux-gnu-g++`；不存在时尝试本项目已解压的 `artifacts/cross-toolchain/root` 工具链。新环境需安装 `g++-aarch64-linux-gnu`，或显式指定工具链：

```bash
# 只展示流程，不编译、不连接、不传输
./ros2_ws/deploy_arm.sh --dry-run

# 自定义编译器及目标 sysroot
./ros2_ws/deploy_arm.sh --compiler /opt/toolchain/bin/aarch64-linux-gnu-g++ \
  --sysroot /opt/arm64-sysroot

# 自定义目标；只同步而不构建远端 ROS 包
./ros2_ws/deploy_arm.sh --host 192.168.2.120 --user nvidia \
  --remote-dir /home/nvidia/wxy/tianjizhineng --skip-remote-build
```

同步不携带 `.git`、本机 ROS 构建目录、缓存、交叉工具链、Windows 动态库或 x86 可执行文件，也不删除远端额外文件。工具链/sysroot 必须与目标系统 glibc 兼容，目标需安装 rsync、Python 3、ROS 2 Humble 和 colcon。

更新前先自行停止旧的机器人控制节点，完成部署后手动启动；脚本不停止服务、不连接控制器，不清错、不切换电机模式、不下发动作。网络、编译、库验证或构建失败时返回非零退出码。再次执行会重新编译并增量同步。

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

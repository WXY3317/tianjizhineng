# Gento 双臂 ROS 2 功能包

适用本仓库 SDK 4.4.2，已在 ROS 2 Humble / Python 3.10 下构建。包含：

- `gento_dual_arm`：单 SDK 连接的双臂控制节点，以及一次性目标发布器。

控制节点复用 `ros2_ws/real_test/dual_arm_control.py` 中的限位读取、轨迹规划/校验函数；将原来一次执行后退出的流程改为持续发布反馈、接收目标、非阻塞等待运动完成的状态机。启动时加载一次模型，连接保持到节点退出。**需要保留 SDK 的 `PYTHON_SDK`、对应动态库及辅助脚本目录 `ros2_ws/real_test`**（也兼容旧版 SDK 内或 SDK 同级的 `real_test`），`sdk_root` 指向仓库绝对路径。没有连接、启动或测试过真实硬件。

## 构建与启动

先准备与本机 CPU/系统匹配的 `PYTHON_SDK/libGentoSDKPY.so`（或适用平台动态库），并确认固件兼容。工作空间位于 `/home/wxy/code/tianji/ros2_ws`，包含 `real_test` 真机脚本目录，与 `tianji-robot-SDK` 同级。执行：

```bash
source /opt/ros/humble/setup.bash
cd /home/wxy/code/tianji/ros2_ws
# 如缺少 ROS 依赖，先执行：rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select gento_dual_arm
source install/setup.bash

# 使用实际 SDK 仓库绝对路径和实际控制器 IP
ros2 launch gento_dual_arm dual_arm.launch.py \
  sdk_root:=/home/wxy/code/tianji/tianji-robot-SDK ip:=6.6.7.190
```

默认 `execute:=false`：仍连接真机读取状态、读取模型、计算 FK/IK 和规划轨迹，但不会切换模式或下发运动。`/status` 报告 `preview`。

现场确认后，重启并启用运动：

```bash
ros2 launch gento_dual_arm dual_arm.launch.py \
  sdk_root:=/home/wxy/code/tianji/tianji-robot-SDK ip:=6.6.7.190 \
  execute:=true speed:=5.0 acc:=5.0 max_delta:=5.0 timeout:=60.0
```

不自动复位错误，也不接管非 IDLE 模式。不得与命令行控制程序或 Apex 遥操同时争用 SDK 控制连接。

## 话题

默认节点名 `/gento_dual_arm`；可以使用 ROS 重映射改变话题名。

| 方向 | 话题 | 类型 | 内容/单位 |
|---|---|---|---|
| 发布 | `/gento_dual_arm/joint_states` | `sensor_msgs/msg/JointState` | 14 个当前关节位置，**rad**；速度/力矩数组留空 |
| 发布 | `/gento_dual_arm/left/tcp_pose` | `geometry_msgs/msg/PoseStamped` | 左臂 FK 位姿，**m + 四元数** |
| 发布 | `/gento_dual_arm/right/tcp_pose` | `geometry_msgs/msg/PoseStamped` | 右臂 FK 位姿，**m + 四元数** |
| 发布 | `/gento_dual_arm/status` | `std_msgs/msg/String` | JSON 状态：ready / waiting_for_pair / preview / running / completed / rejected / stopped / fault |
| 订阅 | `/gento_dual_arm/target_joints` | `sensor_msgs/msg/JointState` | 两臂完整的**绝对关节角 rad** |
| 订阅 | `/gento_dual_arm/left/target_pose` | `geometry_msgs/msg/PoseStamped` | 左臂绝对目标位姿 |
| 订阅 | `/gento_dual_arm/right/target_pose` | `geometry_msgs/msg/PoseStamped` | 右臂绝对目标位姿 |
| 订阅 | `/gento_dual_arm/stop` | `std_msgs/msg/Empty` | 请求双臂软件急停并锁定命令接收；重启前需人工恢复 |

反馈由当前关节反馈经 FK 计算，不是外部测量位姿。时间戳是节点采样时的 ROS 系统时间，非控制器硬件时间戳；SDK 反馈本身不保证结构体的原子快照。没有发布 TF，也没有提供机器人 URDF。

`JointState.name` 必须包含以下 14 个名称，顺序可改变，节点按名称重新排序：

```text
left_joint_1 left_joint_2 left_joint_3 left_joint_4 left_joint_5 left_joint_6 left_joint_7
right_joint_1 right_joint_2 right_joint_3 right_joint_4 right_joint_5 right_joint_6 right_joint_7
```

这组名称是桥接节点约定，并非已校准 URDF 的关节名称。只接收 `position`；`velocity`、`effort` 必须为空。没有相对增量话题，若需要增量，调用方先读当前角度再计算绝对目标。

## 位姿坐标系和单位

左右臂的反馈和目标均使用 ROS 2 标准 `geometry_msgs/msg/PoseStamped`：

```text
std_msgs/Header header
geometry_msgs/Pose pose
```

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
# 将 14 个变量替换为已确认的绝对关节角（弧度），左 7 后右 7。
ros2 run gento_dual_arm send_target --joints \
  L1 L2 L3 L4 L5 L6 L7 R1 R2 R3 R4 R5 R6 R7

# 左 7 个、右 7 个：各自为 x y z qx qy qz qw（米、四元数）。
ros2 run gento_dual_arm send_target --poses \
  LX LY LZ LQX LQY LQZ LQW RX RY RZ RQX RQY RQZ RQW
```

上述 `L1` 等是占位符，不是可直接执行的数值。建议第一次把收到的当前关节角或当前 TCP 原样作为目标，在 `execute:=false` 模式验证话题、单位和坐标系。`send_target` 只确认发布，不代表控制器接受或完成；查看状态话题。

查看反馈及状态：

```bash
ros2 topic echo /gento_dual_arm/joint_states
ros2 topic echo /gento_dual_arm/left/tcp_pose
ros2 topic echo /gento_dual_arm/right/tcp_pose
ros2 topic echo /gento_dual_arm/status

# 软件停止（预览模式不会发送硬件急停，但同样锁定后续目标）
ros2 topic pub --once /gento_dual_arm/stop std_msgs/msg/Empty '{}'
```

## launch 参数

全部为启动时只读参数，修改需重启节点。原脚本 `--joints` 改为目标关节话题，`--delta` 不再作为启动参数。

| 参数 | 默认 | 意义 |
|---|---|---|
| `sdk_root` | 空，必填 | 完整 SDK 仓库绝对路径 |
| `ip` | 空，必填 | 控制器 IPv4 |
| `execute` | false | 是否执行真机运动 |
| `speed` / `acc` | 5.0 / 5.0 | 控制器百分比，允许 (0,10] |
| `plan_speed` / `plan_acc` | 0.05 / 0.05 | SDK 规划比例，允许 [0.01,0.1] |
| `max_delta` | 5.0 | 每关节相对起点的最大位移，**度** |
| `timeout` | 60.0 | 启动后等待完成时限，秒 |
| `tolerance` | 0.2 | 到位关节误差，**度** |
| `publish_rate` | 20.0 | 反馈发布及运动监测频率，Hz，上限 100 |
| `command_max_age` | 1.0 | 目标消息最大允许年龄，秒 |
| `sdk_length_to_m` | 0.001 | SDK 长度乘此系数转换成米 |
| `left_frame` / `right_frame` | left_arm_sdk_base / right_arm_sdk_base | 两臂各自 FK/IK 参考坐标系名称 |

规划采样频率沿用原脚本 50 Hz；发布频率不改变控制器轨迹采样周期。到位且两臂轨迹完成持续 0.5 秒后切回 IDLE。

## 运行边界

一个单线程 executor 串行使用 SDK。运动等待采用定时轮询，执行中持续发布反馈；**同步 SDK 的初始化、规划、模式切换和上传调用期间，反馈发布及 stop 回调可能延迟**。这不是硬实时控制或安全急停通道，硬件急停必须可用。网络断开时停止发布反馈，尝试停止双臂并进入 fault，不自动重连或复位。停止/故障后不再接受运动目标，需操作员恢复状态并重启节点。

对左右臂统一发送启动掩码，不承诺严格同时启动或同时到达。没有左右臂、机身、工具或环境碰撞检测；不能假定直连 SDK 经过 Apex QP 避碰层。头部、机身与升降模块需保持稳定。节点不执行 Home、不修改固件或标定参数。

## 离线测试

测试以 fake SDK 代替真实硬件，包含数值/四元数转换、双臂预览/执行状态机、左右位姿话题配对/不匹配/过期处理、部分启动失败、IK 错误、关闭时停止、上传失败及超时，以及真实 ROS 2 消息的发布/订阅验证。不加载动态库、不连接机器人。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest src/gento_dual_arm/test -q
```

接口/launch 文件改变后需重新 `colcon build`。软件包仅用于当前仓库内开发，`UNLICENSED` 不授予 SDK 的再分发权限。

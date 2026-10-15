# Marvin 双臂 SDK 仿真控制

本包提供双臂**运动学仿真**：原版控制 SDK 通过 UDP 驱动本机虚拟控制器，
原版计算 SDK 提供 FK/IK，`marvin_description` 提供 URDF、网格和 TF。
没有修改或替换厂商动态库，也没有连接机械臂。

```text
操作面板 / ROS 目标
    → SDK bridge → 原版 Marvin_Robot / libMarvinSDK.so
    → UDP 100343 协议 → 本机虚拟控制器
    → 仿真关节反馈 → SDK subscribe / ROS JointState / RViz

SDK libKine.so：关节 → FK → 末端；末端 + 当前关节参考 → IK → 关节目标
```

## 启动

工作目录布局：`ros2_ws/`、`marvin_description/`、`tianji_armSDK/` 同级。
支持当前 Linux / ROS 2 Humble 环境，需要 numpy、PyYAML、Tkinter、xacro、
robot_state_publisher 和 RViz2。当前环境补装了用户级 `xacro==2.1.1`。

```bash
cd /home/wxy/code/tianji/ros2_ws
./run_marvin_sim.sh --build
```

后续启动：

```bash
./run_marvin_sim.sh
# 无桌面运行
./run_marvin_sim.sh rviz:=false panel:=false
```

在图形桌面的终端运行会打开 RViz 和双臂操作面板。面板支持：

- 输入/滑动 7 个关节角，单臂或双臂发送目标。
- SDK 正解预览：计算输入关节的末端位姿，不发送运动。
- 输入 XYZ（mm）和固定轴 RPY（度），SDK 逆解后执行关节运动。
- 读取当前关节或末端位姿、停止双臂、重新启用位置模式。

面板使用度、毫米；ROS 话题使用弧度、米、四元数。初始姿态采用弯肘的非零构型，
避免把全零奇异构型作为逆解参考。SDK IK 使用当前反馈作为参考，拒绝不可达、
奇异、超限以及 FK 回代不一致的解。

## 原版 SDK 直接控制

原版 SDK 拒绝所有 `127.*` 地址。因此虚拟控制器自动选择本机一个已分配的
非回环 IPv4 地址，并绑定该地址的 UDP 4729；SDK 固定在 UDP 4730 接收反馈。
数据发往本机地址，不需要机械臂。可用 `sim_ip:=本机IPv4` 显式选择网卡；
仿真包拒绝配置远端地址，并只接受来自同一本机地址的 SDK 数据。
SDK 原生端口是固定的，同一网络命名空间只能启动一个 SDK 客户端。

默认 SDK bridge 占用该连接。要运行自己的原版 SDK 程序，关闭 bridge 和面板：

```bash
./run_marvin_sim.sh bridge:=false panel:=false
```

另一个终端运行实库端到端示例：

```bash
source /opt/ros/humble/setup.bash
cd /home/wxy/code/tianji/ros2_ws
source install/setup.bash
ros2 run marvin_sim sdk_demo --sdk-root /home/wxy/code/tianji/tianji_armSDK
```

示例执行双臂 FK → 目标 IK → 原版 SDK 发命令 → 等待 SDK 反馈到位，输出 PASS
及实际误差。运行自己的程序时，将 `robot.connect()` 的地址设为启动日志中的
**本机虚拟控制器 IP**，其余基础位置控制 API 保持原样：

```python
# 原版 SDK 调用；init、frame_serial 刷新和每次 API 返回值应按示例检查。
robot.clear_set()
robot.set_vel_acc('A', 10, 10)
robot.set_vel_acc('B', 10, 10)
robot.send_cmd_wait_response(200)
robot.clear_set()
robot.set_state('A', 1)
robot.set_state('B', 1)
robot.send_cmd_wait_response(200)
robot.clear_set()
robot.set_joint_cmd_pose('A', [10, -20, 30, -40, 20, 15, -10])  # 度
robot.send_cmd_wait_response(200)
data = robot.subscribe(DCSS())
```

虚拟控制器实现的是 **100343 协议的基础位置控制子集**：
模式 0/1、速度加速度百分比、两臂关节目标、DCSS 周期反馈、应答标签，
以及 VERSION / BASIC.Type / 伺服错误查询和软件停止参数。
未知指令、错误校验和、超限/非有限目标会拒绝整包，并在 server_status 报告错误。
**不支持** PVT/轨迹文件上传、SDK `setPln_*` 的控制器轨迹缓存协议、
阻抗/力控、CAN/485、固件更新或工具参数修改。

## ROS 接口

| 接口 | 类型 | 内容 |
|---|---|---|
| `/joint_states` | JointState | 仿真模型实际状态，14 关节，rad/rad/s |
| `/marvin_sim/sdk_joint_states` | JointState | 经过原版 SDK 接收的反馈，rad |
| `/marvin_sim/target_joints` | JointState | 一整条手臂或完整双臂绝对目标，rad |
| `/marvin_sim/left/target_pose` | PoseStamped | 左末端绝对目标，frame=`Base_L` |
| `/marvin_sim/right/target_pose` | PoseStamped | 右末端绝对目标，frame=`Base_R` |
| `/marvin_sim/{left,right}/tcp_pose` | PoseStamped | SDK 反馈关节经 SDK FK 计算，m + xyzw |
| `/marvin_sim/{left,right}/model_pose` | PoseStamped | 模型关节经 SDK FK 计算，m + xyzw |
| `/marvin_sim/status` | String(JSON) | ready/moving/completed/stopped/fault 和错误 |
| `/marvin_sim/server_status` | String(JSON) | 模式、运动状态、收包/拒包数及错误 |
| `/marvin_sim/stop` | Trigger | 停止双臂并切到 idle |
| `/marvin_sim/enable` | Trigger | 进入位置模式 |

关节名为 `Joint1_L..Joint7_L`、`Joint1_R..Joint7_R`。目标按名称重排，
不接收 velocity/effort 目标。两臂位姿话题独立执行；需要同时发起双臂运动时，
先分别计算 IK，再通过完整 14 关节目标发送。

末端是原生法兰 `flange_L/R`，没有工具偏移。目标必须在对应 `Base_L/R` 中，
不隐式把 world 或 base_link 坐标解释成机械臂基座坐标。
面板 RPY 为 `Rz(yaw) Ry(pitch) Rx(roll)`。

停止后需 enable 才能继续；反馈超过 0.5 秒未刷新时桥接器报 fault 并拒绝新运动。
轨迹按单调墙钟执行，不使用 `/clock` 或 ROS 仿真时间。

## 模型和执行边界

- 使用 `ccs_m6_40.MvKDCfg`、type=1017 和当前 description 模型。
  测试对左右臂多个构型实际比较了 URDF 与 SDK FK（位置 <5 µm，姿态 <30 µrad）。
  仓库未明确列出 `MarvinM6SCCS—696` 的完整型号映射，仍需按机械版本确认。
- 关节目标采用五次平滑插值，速度/加速度受 SDK 参数与模型限位共同约束。
  插值由虚拟控制器实现，不声称复现真机控制器算法；末端目标执行 MoveJ，
  不保证末端直线。运行中更新目标会从当前位置重新插值，速度可能不连续。
- 不模拟惯性、重力、接触、力矩、伺服延迟或力控。反馈力矩字段为零，ROS 不发布 effort。
- 本版没有自动避碰，description 的碰撞网格也不会让 RViz 自动阻止运动。
- `marvin_description` 保持纯模型职责，厂商 SDK 文件保持原样；仿真逻辑均在本包。

## 验证

先退出运行中的仿真和 SDK 客户端，测试会使用相同的 UDP 4729/4730 端口。

```bash
source /opt/ros/humble/setup.bash
cd /home/wxy/code/tianji/ros2_ws
source install/setup.bash
python3 -m pytest src/marvin_sim/test -q
```

测试调用实际 `libKine.so` 和 `libMarvinSDK.so`，覆盖正逆解、URDF 一致性、
非法输入、协议校验、批量原子拒绝、速度加速度限制、原版 SDK 双臂 UDP 控制及停止恢复。
端到端 ROS 测试启动独立虚拟控制器和 SDK bridge，不连接真实机械臂。

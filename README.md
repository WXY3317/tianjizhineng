# 天机双臂机器人开发与仿真（end-Modification 分支）

本分支用于整机运动学仿真，包含原厂 SDK、双臂/灵巧手模型、ROS 2 控制节点和可视化面板。
仿真通过原版 Gento、SG100 SDK 连接本机虚拟控制器，运行时不需要连接真实机器人。

## 分支用途

| 分支 | 用途 |
|---|---|
| `end-Modification` | 整机仿真：双臂、灵巧手、躯干和头部控制，统一面板与手掌 TCP 正逆解 |
| `real` | 双臂真机 ROS 2 控制：关节目标、末端 FK/IK、反馈与停止；已移除仿真功能包 |

真机运行请使用 [`real` 分支说明](https://github.com/WXY3317/tianjizhineng/blob/real/ros2_ws/REAL_ROBOT.md)。
两个分支的启动入口、目标话题和坐标系不同，不能直接混用。

## 目录结构

| 目录 | 内容 |
|---|---|
| `tianji_armSDK` | Marvin 控制、运动学和干涉检测 SDK |
| `tianji-robot-SDK` | Gento SDK 与示例 |
| `SG100_SDK_Publish` | SG100 灵巧手原版 SDK 与左右手角度配置 |
| `marvin_description` | Marvin CCS M6 ROS 2 模型 |
| `mjcf` | 原始 MJCF 与 STL/OBJ 资源 |
| `ros2_ws/src/marvin_mesh_sim` | STL 整机仿真，接通双臂原版 SDK 正逆解与控制 |
| `ros2_ws/src/marvin_sim` | 原版 SDK 的本机虚拟控制器及运动学仿真基础 |
| `ros2_ws/src/gento_dual_arm` | Gento 双臂 ROS 2 接口 |
| `ros2_ws/real_test` | 真机控制脚本 |

## 获取代码

两个大型 OBJ 资源使用 Git LFS，克隆前请安装 Git LFS：

```bash
git lfs install
git clone --branch end-Modification git@github.com:WXY3317/tianjizhineng.git
cd tianjizhineng
git lfs pull
```

## 启动双臂与灵巧手仿真

环境：Linux、ROS 2 Humble、Python 3.10，以及 NumPy、PyYAML、Tkinter、xacro、
RViz2、robot_state_publisher、colcon。需要与本机架构匹配的 SDK 动态库。

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
./run_marvin_mesh_sim.sh --build
# 后续启动（新终端先加载 ROS 环境）：
./run_marvin_mesh_sim.sh
# 无图形界面：
./run_marvin_mesh_sim.sh rviz:=false panel:=false
```

包含底盘、腰腿/躯干、头部、双臂和双手，共 47 个真实 STL，
支持 14 个机械臂关节、22 个手指关节和 8 个本体/头部关节。
双臂位置控制、正逆解以及躯干/头部控制统一通过原版 Gento SDK，
双手通过原版 SG100 SDK。双臂对外正逆解统一参考躯干 `Gento_Luna_Leg_J4_Link`，末端控制点为手掌中心 `palm_center_L/R`。
手指按 SDK 的 Joint1–11 编号，第四指使用 FINGER_RING（无名指）命名。
重复启动会复用后台并重新打开界面。
本版是运动学仿真，不含接触动力学或自动避碰。

## ROS 2 控制接口

新终端先加载 ROS 和工作空间环境：

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws  # 从仓库根目录进入
source install/setup.bash
```

| 话题 | 用途 | 消息类型 |
|---|---|---|
| `/marvin_sim/target_joints` | 完整单臂 7 关节或双臂 14 关节绝对目标 | `sensor_msgs/msg/JointState` |
| `/marvin_sim/left/target_pose`、`right/target_pose` | 左右手掌 TCP 目标 | `geometry_msgs/msg/PoseStamped` |
| `/marvin_sim/left/target_rpy`、`right/target_rpy` | `[x, y, z, roll, pitch, yaw]` 末端目标 | `std_msgs/msg/Float64MultiArray` |
| `/marvin_mesh_sim/body_targets` | 躯干和头部关节目标 | `sensor_msgs/msg/JointState` |
| `/marvin_mesh_sim/hand_targets` | 完整单手 11 关节目标 | `sensor_msgs/msg/JointState` |
| `/marvin_sim/sdk_joint_states` | 双臂 SDK 反馈 | `sensor_msgs/msg/JointState` |
| `/marvin_sim/status` | 双臂命令执行状态 | `std_msgs/msg/String` |

ROS 位置使用 m、角度使用 rad；控制面板支持角度/弧度切换。
末端目标参考系为 `Gento_Luna_Leg_J4_Link`，控制点为手掌中心，姿态可使用四元数或 RPY。
IK 后执行关节运动，不保证末端直线；执行末端目标时应保持躯干 J5/J6 不动。

```bash
# 查看反馈与命令状态
ros2 topic echo /marvin_sim/sdk_joint_states
ros2 topic echo /marvin_sim/status
# 停止双臂 / 重新启用
ros2 service call /marvin_sim/stop std_srvs/srv/Trigger '{}'
ros2 service call /marvin_sim/enable std_srvs/srv/Trigger '{}'
```

## 运行与使用说明

关闭 RViz 或面板只关闭窗口；在后台启动终端按 Ctrl+C 停止仿真。
原版 SDK 的反馈端口固定，不要同时启动仿真、真机节点或多个 SDK 控制客户端。
仿真模型的运动学配置不代表已完成真机标定，不能用于真机执行。

详细话题示例、构建验证和 Python API 见 [marvin_mesh_sim 使用说明](ros2_ws/src/marvin_mesh_sim/README.md)。
SDK 和第三方模型保留各自随附的许可证与说明。

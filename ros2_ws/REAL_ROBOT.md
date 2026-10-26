# 双臂真机 ROS 2 控制

`real` 分支通过 `gento_dual_arm` 直接连接实体控制器。第一阶段控制双臂 14 个关节；
躯干、头部和灵巧手保持原位。真机入口不会启动虚拟控制器、加载仿真运动学参数或运行仿真面板。
本分支已移除两个仿真功能包及其启动入口；需要仿真时请使用仿真分支。

## 启动

控制器 IP 在 `src/gento_dual_arm/launch/dual_arm.launch.py` 的 `CONTROLLER_IP` 中设置，当前为 `6.6.7.100`（沿用急停脚本地址，请按实际配置修改）。从工作空间目录构建并加载环境后，可直接启动：

```bash
source /opt/ros/humble/setup.bash
source install_real/setup.bash
ros2 launch gento_dual_arm dual_arm.launch.py
# 临时覆盖 IP：
ros2 launch gento_dual_arm dual_arm.launch.py ip:=6.6.7.190
```

默认 `execute:=false`。仓库中自动定位 SDK，独立部署时通过 `sdk_root:=SDK绝对路径` 或 `GENTO_SDK_ROOT` 指定。启动脚本也支持不传 IP：`./run_real_robot.sh --build`。

需要 ROS 2 Humble、Python 3.10、colcon，以及匹配控制器固件和主机架构的原厂
`tianji-robot-SDK/PYTHON_SDK/libGentoSDKPY.so`。主机应能访问控制器 IP。
不要与仿真后台、Apex 或其他 SDK 客户端同时运行，避免反馈端口和控制权冲突。

从仓库根目录执行，替换控制器 IP：

```bash
cd ros2_ws
# 首次构建并连接真机，默认仅预览
./run_real_robot.sh --build ip:=6.6.7.190
# 后续连接
./run_real_robot.sh ip:=6.6.7.190
```

预览模式读取真机反馈、计算 FK/IK 并规划轨迹，不切换控制模式或上传运动。
确认反馈、SDK 模型、工具定义、单位和运动空间后，停止预览进程，再显式启用运动：

```bash
./run_real_robot.sh ip:=6.6.7.190 execute:=true
```

默认速度/加速度为 5%，各关节单次位移上限 5°。真机模型从控制器读取，
不能使用仿真的 `mesh_arms.MvKDCfg` 或假定仿真的手掌 TCP 已用于真机。
该入口使用独立 `build_real`、`install_real`、`log_real`，避免复用仿真构建产物。
只构建、不连接可执行：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base log_real build --base-paths src --packages-select gento_dual_arm \
  --build-base build_real --install-base install_real --symlink-install
```

## 反馈与控制话题

新终端加载环境：

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws  # 从仓库根目录进入
source install_real/setup.bash
```

| 方向 | 话题 | 类型 | 含义 |
|---|---|---|---|
| 发布 | `/gento_dual_arm/joint_states` | `sensor_msgs/msg/JointState` | 当前双臂 14 个关节角，rad |
| 发布 | `/gento_dual_arm/left/tcp_pose` | `geometry_msgs/msg/PoseStamped` | 左臂反馈经 FK 得到的 TCP，m、四元数 |
| 发布 | `/gento_dual_arm/right/tcp_pose` | `geometry_msgs/msg/PoseStamped` | 右臂反馈经 FK 得到的 TCP，m、四元数 |
| 发布 | `/gento_dual_arm/status` | `std_msgs/msg/String` | JSON 接受、预览、运行、完成或故障状态 |
| 订阅 | `/gento_dual_arm/target_joints` | `sensor_msgs/msg/JointState` | 完整双臂绝对关节目标，rad |
| 订阅 | `/gento_dual_arm/left/target_pose` | `geometry_msgs/msg/PoseStamped` | 左臂绝对 TCP 目标 |
| 订阅 | `/gento_dual_arm/right/target_pose` | `geometry_msgs/msg/PoseStamped` | 右臂绝对 TCP 目标 |
| 订阅 | `/gento_dual_arm/stop` | `std_msgs/msg/Empty` | 软件停止并锁定后续命令 |

```bash
ros2 topic echo /gento_dual_arm/joint_states
ros2 topic echo /gento_dual_arm/left/tcp_pose
ros2 topic echo /gento_dual_arm/right/tcp_pose
ros2 topic echo /gento_dual_arm/status
```

### 关节运动

名称为 `left_joint_1` 至 `left_joint_7`，随后 `right_joint_1` 至 `right_joint_7`。
每条目标必须包含全部 14 个关节，`velocity` 和 `effort` 为空。
消息必须有新时间戳；可用发布器自动填写：

```bash
# L1...R7 替换为基于当前反馈确定的绝对关节角（rad），不能直接照抄占位符
ros2 run gento_dual_arm send_target --joints \
  L1 L2 L3 L4 L5 L6 L7 R1 R2 R3 R4 R5 R6 R7
```

第一次建议在预览模式把当前反馈原样作为目标，核对 status 中的规划结果。

### 末端运动学控制

两臂目标必须以完全相同的 `header.stamp` 成对发送，配齐后才执行 IK 和关节轨迹规划。
左臂 `frame_id` 是 `left_arm_sdk_base`，右臂是 `right_arm_sdk_base`；它们分别对应
SDK 当前模型的基坐标系，不是仿真的共同 J4 坐标系，也不做 TF 转换。
TCP 使用控制器初始化后的工具定义，位置单位 m，姿态为单位四元数。

```bash
# 各臂依次为 x y z qx qy qz qw；替换为各自基坐标系下的有效目标
ros2 run gento_dual_arm send_target --poses \
  LX LY LZ LQX LQY LQZ LQW RX RY RZ RQX RQY RQZ RQW
```

发布器自动给两臂填同一个新时间戳。默认目标有效期 1 秒，零时间戳、过期、
忙碌期间和运动前积压的目标会被拒绝。规划结果经过限位、位移及 FK 回代检查。
执行的是 MoveJ，不保证末端直线，也不保证两手相对位姿不变或同时到达。
只希望移动一臂时，另一臂填写其当前关节角或当前 TCP。

### 停止

```bash
ros2 topic pub --once /gento_dual_arm/stop std_msgs/msg/Empty '{}'
```

停止、超时或故障后锁定命令，需人工恢复并重启节点；不自动复位。
软件停止回调可能被同步 SDK 调用延迟，不能代替硬件急停。
当前没有碰撞检测；实机动作需要现场确认。代码验证仅使用 fake SDK，未实际驱动机器人。

## 离线验证

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/src/gento_dual_arm:$PYTHONPATH" \
  python3 -m pytest src/gento_dual_arm/test -q
```

包含 fake SDK 状态机、真实 ROS 话题收发和原生 IK 结构体的 C/Python 布局对照。
完整参数及接口约束见 [工作空间说明](README.md)。

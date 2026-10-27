# 双臂真机 ROS 2 控制

`real` 分支通过 `gento_dual_arm` 直接连接实体控制器。控制双臂 14 个关节及躯干 6 个关节；
头部和灵巧手保持原位。真机入口不会启动虚拟控制器、加载仿真运动学参数或运行仿真面板。
本分支已移除两个仿真功能包及其启动入口；需要仿真时请使用仿真分支。

## 启动

控制器 IP 在 `src/gento_dual_arm/launch/dual_arm.launch.py` 的 `CONTROLLER_IP` 中设置，当前为 `6.6.7.100`（请按实际控制器配置修改）。从工作空间目录构建并加载环境后，可直接启动：

```bash
source /opt/ros/humble/setup.bash
source install_real/setup.bash
ros2 launch gento_dual_arm dual_arm.launch.py
# 临时覆盖 IP：
ros2 launch gento_dual_arm dual_arm.launch.py ip:=6.6.7.190
```

收到合法目标后直接执行，无需 `execute` 参数。仓库中自动定位 SDK，独立部署时通过 `sdk_root:=SDK绝对路径` 或 `GENTO_SDK_ROOT` 指定。启动脚本也支持不传 IP：`./run_real_robot.sh --build`。

需要 ROS 2 Humble、Python 3.10、colcon，以及匹配控制器固件和主机架构的原厂
`tianji-robot-SDK/PYTHON_SDK/libGentoSDKPY.so`。主机应能访问控制器 IP。
不要与仿真后台、Apex 或其他 SDK 客户端同时运行，避免反馈端口和控制权冲突。

从仓库根目录执行，替换控制器 IP：

```bash
cd ros2_ws
# 首次构建并连接真机，等待目标命令
./run_real_robot.sh --build ip:=6.6.7.190
# 后续连接
./run_real_robot.sh ip:=6.6.7.190
```

节点连接后自动清除双臂与躯干错误，确认 IDLE 后初始化模型、读取限位并切换 POSITION，随后读取反馈并等待命令，收到合法目标后直接执行。发布前确认反馈、SDK 模型、工具定义、单位和运动空间。

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
| 发布 | `/gento_dual_arm/head/joint_states` | `sensor_msgs/msg/JointState` | 头部实际关节角，rad |
| 发布 | `/gento_dual_arm/full_joint_states` | `sensor_msgs/msg/JointState` | 头部、躯干、双臂合并关节角，rad |
| 发布 | `/gento_dual_arm/left/tcp_pose` | `geometry_msgs/msg/PoseStamped` | 左臂反馈经 FK 得到的 TCP，m、四元数 |
| 发布 | `/gento_dual_arm/right/tcp_pose` | `geometry_msgs/msg/PoseStamped` | 右臂反馈经 FK 得到的 TCP，m、四元数 |
| 发布 | `/gento_dual_arm/status` | `std_msgs/msg/String` | JSON 接受、运行、完成或故障状态 |
| 发布 | `/gento_dual_arm/joint_errors` | `std_msgs/msg/String` | JSON 对象错误类型、各轴伺服故障码及说明 |
| 订阅 | `/gento_dual_arm/left/target_joints` | `sensor_msgs/msg/JointState` | 左臂 7 个绝对关节目标，rad |
| 订阅 | `/gento_dual_arm/right/target_joints` | `sensor_msgs/msg/JointState` | 右臂 7 个绝对关节目标，rad |
| 订阅 | `/gento_dual_arm/left/target_pose` | `geometry_msgs/msg/PoseStamped` | 左臂绝对 TCP 目标 |
| 订阅 | `/gento_dual_arm/right/target_pose` | `geometry_msgs/msg/PoseStamped` | 右臂绝对 TCP 目标 |
| 订阅 | `/gento_dual_arm/stop` | `std_msgs/msg/Empty` | 软件停止并锁定后续命令 |
| 订阅 | `/gento_dual_arm/reset_error` | `std_msgs/msg/Empty` | 显式复位双臂和躯干错误，成功后解除命令锁 |

```bash
ros2 topic echo /gento_dual_arm/joint_states
ros2 topic echo /gento_dual_arm/left/tcp_pose
ros2 topic echo /gento_dual_arm/right/tcp_pose
ros2 topic echo /gento_dual_arm/status
```

### 关节运动

名称为 `left_joint_1` 至 `left_joint_7`，随后 `right_joint_1` 至 `right_joint_7`。
每条关节目标必须包含对应左臂或右臂全部 7 个关节，`velocity` 和 `effort` 为空。
消息必须有新时间戳；可用发布器自动填写：

```bash
# 用当前反馈确定单臂 7 个绝对关节角（rad）；占位符不能直接执行
ros2 run gento_dual_arm send_target --left-joints L1 L2 L3 L4 L5 L6 L7
ros2 run gento_dual_arm send_target --right-joints R1 R2 R3 R4 R5 R6 R7
```

第一次建议把当前反馈原样作为目标；该目标也会进入执行流程，需查看 status 确认结果。

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

停止、超时或故障后锁定命令，可人工排除原因后使用 reset_error 话题恢复；运行中不自动复位；重新启动节点会重新执行连接后的错误复位和位置模式设置。
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

## 躯干控制

新增 `/gento_dual_arm/body/target_joints`（完整 6 关节绝对目标）和 `/gento_dual_arm/body/joint_states`（反馈），均使用 `sensor_msgs/msg/JointState`、rad，名称 `body_joint_1..6`。使用 `ros2 run gento_dual_arm send_target --body B1 B2 B3 B4 B5 B6` 自动填时间戳；数值需替换为已确认目标。双臂和躯干串行执行；stop 话题现请求双臂及躯干软急停（掩码 11）。详见 [工作空间话题说明](README.md)。

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

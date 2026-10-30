# Luna 六轴躯干正逆解

实现位于 ROS 2 功能包 `gento_driver` 的 `gento_driver/body_kinematics.py`。
厂商 SDK 保持原样。模块依赖 NumPy、SciPy，已在 `package.xml` 和 `setup.py` 声明。
以下命令在 `ros2_ws` 目录执行：

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select gento_driver --symlink-install
source install/setup.bash
```

## 基座和末端

- **固定基座 B**：原点在 `BD_J1` 旋转中心，轴方向与配置中的 `AGV` 平行；
  位于第 1 轴运动之前，不随任何躯干关节转动。
- **末端 E**：`BD_FL`，包含 `BD_J6 -> BD_FL` 的固定安装变换。
  当前配置为第 6 轴局部 Z 方向 391.5 mm；不是 `BD_J6` 原点。
- 输出/输入位姿均为 **`T_B_BD_FL`**，4×4 齐次矩阵；平移单位 mm，关节角单位度。
  位姿矩阵左上角是旋转矩阵，不是欧拉角。

几何参数读取包内 `gento_driver/data/luna_body.Cord`，随功能包一起安装，
运行时无需访问 SDK 源码目录。该文件是原有
`C_SDK/FXConfig/InterferenceCfg/CordDef.Cord` 的工作空间副本，
可通过构造函数的 `cord_path` 显式指定经过标定的配置。
`<m3>` 的三组数依次是局部 X 轴、局部 Z 轴和安装位移，不是普通 3×3 旋转矩阵。
每个关节使用 `T_parent_joint(q) = T_install @ Rz(q)`，与
`ICBase::FX_OnLoadCord`、`FX_OnUpdateCord` 的定义一致。

当前配置中 `T_AGV_B` 只有 Z=100 mm 的平移。B 的全零位末端位置为
`[0, 0, 1156.5]` mm，末端朝向为单位旋转矩阵。模型的“全零位”不表示推荐的实机姿态。
如需转换现有 AGV 坐标下的目标：

```python
import numpy as np
target_in_base = np.linalg.inv(np.array(model.agv_to_base)) @ target_in_agv
```

这是仓库配置描述的几何模型。尚未与当前实体机器的标定参数、零位和反馈进行实机比对。
离线测试会与 SDK 原有 C++ 坐标链比较，但这不代替机械标定。

## 限位

连接控制器时，从 `R.B.L0..5.BASIC.LimitNeg/LimitPos` 读取六对有符号限位，
读取失败、非六轴躯干、非有限值或无有效区间都会报错，不会使用猜测值继续求解。
再与用户指定的以下范围取交集，绝不放宽控制器给出的范围：

| 关节 | 额外范围（度） |
|---|---|
| J1 | [-11, 11] |
| J2 | **[-50, 50]** |
| J3 | 保留控制器有符号限位 |
| J4 | 保留控制器有符号限位 |
| J5 | [-30, 30] |
| J6 | [-170, 170] |

用户图中 J3 的 60°/140°、J4 的 55°/110°没有标明正负方向；按已确认的方式，
这两轴直接采用控制器限位。有效限位通过 `model.joint_limits_deg` 获取，可打印与图纸核对。
以上是求解器本地约束，不会修改控制器参数。
逆解返回的度数会重新检查并保持在精确限位内。执行端仅允许度/弧度往返
产生的 `1e-10` 度以内舍入误差，并在下发前把关节目标归回精确限位；
实际超限的目标仍会拒绝。

## 在功能包内使用

在 driver 内，直接复用已读取的 `engine.body_limits` 和当前关节角；
计算模块不创建 SDK 连接，不发送运动指令。这里保留 SDK 的度/mm 单位，
在 ROS 消息边界按约定转换成 rad/m。

```python
from gento_driver.body_kinematics import LunaBodyKinematics

model = LunaBodyKinematics(engine.body_limits)
print("六轴有效限位（度）:", model.joint_limits_deg)

# 直接读取原始反馈，避免 get_rt_dict 的小数舍入影响精度。
current = list(engine.robot.rt.m_BODY.m_BODY_OUT.m_BODY_FBK_Joint_Pos)
current_pose = model.forward(current)
target = [row[:] for row in current_pose]
target[0][3] += 10.0  # 沿固定基座 X 方向移动 10 mm

# 完整位姿：保留当前末端朝向，并求目标位置。
result = model.inverse(
    target, current,
    max_joint_delta_deg=5.0,  # 可选，每轴相对 current 的最大变化；不是速度限制
)

# 如果只关心位置，允许朝向变化以减小关节位移，使用：
# result = model.inverse(
#     [target[i][3] for i in range(3)], current, position_only=True)

if not result.success:
    raise RuntimeError(result.message)
print("求解关节角（度）:", result.joints_deg)
print("位置误差（mm）:", result.position_error_mm)
print("朝向误差（度）:", result.orientation_error_deg)
```

也可在唯一持有 SDK 连接的 driver 内调用 `LunaBodyKinematics.from_robot(robot)`，
通过 SDK 原有的只读参数接口初始化。其他节点应使用 driver 提供的反馈和限位，
不要为了求解建立第二个 SDK 连接。

## ROS 话题反馈和控制

| 话题 | 消息 | 含义 |
|---|---|---|
| `/gento_feedback/body/tcp_pose` | `geometry_msgs/msg/PoseStamped` | 当前六轴反馈角正解得到的 `BD_FL` 位姿 |
| `/gento_control/body/target_pose` | `geometry_msgs/msg/PoseStamped` | `BD_FL` 的绝对目标位置及朝向 |

两者默认 `frame_id: body_base`，位置单位 **m**，朝向为单位四元数；此处固定
使用 mm→m 的换算，不受双臂 `sdk_length_to_m` 参数影响。可通过 driver/launch
的 `body_frame` 参数统一修改名称，不自动做 TF 转换。

driver 从当前反馈角计算 `body_matrix`，与同一帧关节反馈一起发给 feedback，
feedback 转换为带相同时间戳的 `PoseStamped`。只读模式也可发布当前位姿，
无需读取限位或开启控制。

control 收到新鲜目标后，向唯一 SDK owner driver 请求 `body_ik`，以当前反馈
关节角为参考，约束完整位姿、有效限位和 `max_delta`，再提交现有躯干关节执行
Action。driver 会独立检查起点、限位、模式、心跳和最大变化量。
控制器采用单点关节位置通道执行，末端移动路径不保证为直线。
若逆解失败或消息非法，不下发关节目标。停止请求可中断数值求解。
本地额外限位也应用于现有 `body/target_joints` 通道。

躯干任务不读取双臂限速参数。执行完成须同时满足关节角 `tolerance`、
末端位置 `body_position_tolerance_mm`（默认 1 mm）和朝向
`body_orientation_tolerance_deg`（默认 0.2°），并持续 0.5 秒。
执行位姿误差以目标关节角正解得到的位姿为基准；逆解与原始目标之间另有
默认 0.1 mm / 0.01° 的数值容差。到位容差与逆解数值容差是两套独立条件。
未达到位姿容差时继续等待，超过 `timeout` 则停止并报告失败。

先启动 driver、feedback、control（在 `ros2_ws` 目录）：

```bash
./run_real_robot.sh --build
```

在另一个已加载 ROS 和工作空间环境的终端读取当前位置：

```bash
ros2 topic echo --once /gento_feedback/body/tcp_pose
```

发送目标的格式如下。`X/Y/Z/QX/QY/QZ/QW` 必须替换为现场选定的数值，
可从当前反馈位姿开始小幅调整；四元数若不需要旋转可沿用当前反馈值。

```bash
ros2 topic pub --once /gento_control/body/target_pose geometry_msgs/msg/PoseStamped \
"{header: {stamp: now, frame_id: body_base},
  pose: {position: {x: X, y: Y, z: Z},
         orientation: {x: QX, y: QY, z: QZ, w: QW}}}"
```

当前通道只接受完整位置和朝向；`position_only=True` 仍用于直接调用计算模块，
没有把“忽略朝向”隐含在 ROS 位姿消息中。忙碌时不排队，任务结束后需重新发带新时间戳的目标。

`max_joint_delta_deg` 可以是一个正数，也可以是六个正数。
即使当前角度超出用户收紧的范围（例如 J2 当前为 60°），仍可以计算正解；
逆解的输出必须满足所有有效限位。若额外变化范围与限位没有交集，会直接报错。
若交集恰好是一个角度，该轴会固定在此角度，只求解其余可动关节。

`BodyIKResult` 字段：

| 字段 | 含义 |
|---|---|
| `success` | 最终解是否同时满足目标容差及关节限制 |
| `joints_deg` | 六轴结果，失败时为 `None` |
| `position_error_mm` | 最终或最优尝试的位置误差范数 |
| `orientation_error_deg` | 旋转误差角；仅位置模式下为 `None` |
| `reference_distance_deg` | 与参考角的欧氏距离；失败时为 `None` |
| `attempts` | 尝试的初值数量，最多 10 个 |
| `message` | 求解状态说明 |

默认位置容差 0.1 mm、朝向容差 0.01°。先用带边界的最小二乘寻找可行解，
再在保持目标位姿约束的条件下局部最小化 `sum((q-current)**2)`，最后选择
候选中关节变化最小的可行解；不会通过牺牲目标误差来交换接近当前角度。
仅位置模式下朝向完全自由，完整位姿模式下约束位置及朝向。
这是有界、多初值的局部数值搜索，不保证全局最近解；搜索失败也不等于数学上证明无解。
它计算一个静态末端目标，不包含轨迹规划、速度/加速度限制或碰撞检测。

## 不连接机器使用

独立计算无需实例化 `GentoRobot` 或加载 `.so/.dll`：

```python
from gento_driver.body_kinematics import LunaBodyKinematics

# controller_limits_deg 是已经导出的六对有符号限位；没有硬编码的 J3/J4 默认值。
model = LunaBodyKinematics(controller_limits_deg)
pose = model.forward(current_joints_deg)
result = model.inverse(target_matrix, current_joints_deg)
```

将六对控制器限位保存为 JSON 数组后，可运行离线示例：

```bash
ros2 run gento_driver body_ik_example \
  --limits body_limits.json --reference 0 10 -20 10 0 0 \
  --offset 10 0 0 --position-only
```

运行回归测试（包含编译独立 C++ 正解对照程序，需要 g++）：

```bash
PYTHONPATH=src/gento_driver python3 -m pytest -q \
  src/gento_driver/test/test_body_kinematics.py
```

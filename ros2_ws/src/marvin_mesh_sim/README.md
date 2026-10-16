# STL 整机与双臂 SDK 仿真

模型来自 `/home/wxy/code/tianji/mjcf/luna_m6_robot.xml` 和其引用的 STL。
独立包 `marvin_mesh_sim`，双臂、躯干和头部统一使用 Gento SDK，双手使用 SG100 SDK。

当前资源审计：底盘 1 个、腰腿/躯干 6 个、头部 2 个、双臂 14 个、左右手 24 个，
共 47 个真实 STL 网格。网格逐字节复制到本包，保留 44 个可动关节：
腰腿/躯干 6 + 头部 2 + 双臂 14 + 双手 22。
每只手按 SG100 SDK 命名为拇指、食指、中指、无名指，共 4 指。
原 MJCF 的 `little` 对应 SDK 的 `FINGER_RING`，仅改变命名，不改变几何或关节轴。
更新源网格后，重新执行 `--build` 会重新审计并生成模型。
如果后续出现空网格，生成器会明确记录，并用原 MJCF 的橙色碰撞盒占位。

## 启动

Gento SDK 使用 UDP 3720–3724，手部使用 UDP 4741/4742，不能同时启动两套后台。
启动脚本会复用本工作空间已经运行的后台，并重新打开界面。
复用前检查 `/robot_description` 发布者；若模型节点退出，会补起 robot_state_publisher。
若端口被其他程序或旧版启动入口占用，会在打开窗口前给出明确提示。
关闭 RViz 或操作面板仅关闭窗口；在后台的启动终端按 Ctrl+C 才会停止仿真。

```bash
cd /home/wxy/code/tianji/ros2_ws
./run_marvin_mesh_sim.sh --build
# 后续启动
./run_marvin_mesh_sim.sh
# 无图形运行
./run_marvin_mesh_sim.sh rviz:=false panel:=false
```

桌面显示整机 RViz、双臂 FK/IK 面板、灵巧手面板以及本体/头部面板。
机械臂面板使用度/mm，手指面板使用度，本体/头部面板使用度。
支持单指调节、手部张开/弯曲、双臂关节运动、末端 IK 运动和本体/头部姿态调节。
本体/头部面板通过原版 Gento SDK 的位置控制 API 向本机虚拟控制器发送目标；反馈也由原版 SDK 接收。

## SDK 调用

原版 `tianji-robot-SDK/PYTHON_SDK/libGentoSDKPY.so` 统一控制双臂、躯干和头部，
机械臂 FK/IK 也使用该库的运动学接口。默认整机入口不再加载 `libMarvinSDK.so` 或 `libKine.so`。
新模型是 264/264 mm 连杆、87 mm 腕端偏移，肩坐标原点在第一关节，
不能使用上一版 stock `ccs_m6_40` 的 287/314 mm 几何参数。
本包生成 `config/mesh_arms.MvKDCfg`，并配置法兰旋转使 SDK 与模型对齐。
此配置仅用于运动学，动力学参数未标定，不用于真实机械臂。

运行自己的 SDK 客户端时关闭默认桥接，避免争用原版 SDK 的固定反馈端口：

```bash
./run_marvin_mesh_sim.sh bridge:=false panel:=false
```

另一个终端：

```bash
source /opt/ros/humble/setup.bash
cd /home/wxy/code/tianji/ros2_ws
source install/setup.bash
ros2 run marvin_mesh_sim sdk_demo --sdk-root /home/wxy/code/tianji/tianji-robot-SDK
```

Python API 示例（机械臂关节角为度，位姿矩阵平移为米，手指角为 rad）：

```python
from marvin_mesh_sim.sdk import MeshSimulationSDK

with MeshSimulationSDK('/home/wxy/code/tianji/tianji-robot-SDK') as sdk:
    q = sdk.get_joint_positions('A')
    q[0] += 3
    pose = sdk.fk('A', q)         # Gento SDK 正解 + 坐标转换，J4 → flange_L
    solution = sdk.ik('A', pose)  # 输入 J4 下的目标，转换到肩部后调用 SDK IK
    sdk.move_joints('A', solution)
    # 也可以 sdk.move_pose('A', pose)
    hands = sdk.get_hand_state()
    sdk.set_hand_joints('L', [0.0] * 11, duration=1.0)
```

`sdk.robot` 是原版 `GentoRobot`，可调用 `runtime_set_joint_pos_cmd(0, angles_deg)`
和 `runtime_set_joint_pos_cmd(1, angles_deg)` 分别控制左右臂。
默认 ROS 桥接占用 Gento SDK 固定接收端口，独立客户端运行前应关闭默认桥接。
`gento_kinematics.py` 在适配层按 C 头文件修正 Python IK 结构体的 FX_BOOL 字段；
厂商文件保持原样，修正后的结果仍须通过限位与 FK 回代校验。
`MeshSimulationSDK` 的手指接口为兼容入口（UDP 4741）；后台同样经原版 SG100 SDK 打包后执行。
直接调用 SG100 SDK 的示例见下文。
SDK 目标地址只允许本机已分配的非回环 IPv4，原版库会拒绝 127.*。

## 原版 Gento SDK 控制双臂、躯干和头部

使用 `tianji-robot-SDK/PYTHON_SDK/GentoRobot.py` 和从原始 C++ 源码编译的
`libGentoSDKPY.so`，不修改厂商 SDK。启动脚本在缺少 SO 时自动编译；也可以手动执行：

```bash
./build_gento_sdk.sh
```

控制链路：面板/ROS → 原版 Gento Python API → 原版 SO → UDP → 本机虚拟控制器
→ 关节插值/整机 TF；反馈沿 UDP → 原版 SO → Python API 返回。
虚拟控制器实现连接握手、版本/自由度查询、位置模式、速度加速度比例和空闲停止。
当前支持双臂、躯干和头部的位置控制；其他协议指令被拒绝。

直接运行原版 SDK 示例前，关闭已有后台，然后启动不带桥接的服务器：

```bash
./run_marvin_mesh_sim.sh bridge:=false panel:=false
```

另一个终端加载 ROS 和工作空间环境后运行：

```bash
ros2 run marvin_mesh_sim gento_demo --sdk-root /home/wxy/code/tianji/tianji-robot-SDK
```

示例直接调用 `link`、`switch_to_position_mode`、`runtime_set_joint_pos_cmd`、
读取 `robot.rt`，最终调用 `switch_to_idle`、`unlink`。
BODY 控制对象编号为 3，6 个角度通道；HEAD 为 2，SDK 数据格式保留 3 个通道，
本模型只使用前两个，第 3 通道必须为零。原版位置 API 单位为度；ROS 话题单位为 rad。
仿真关节映射、轴和限位取自 MJCF，尚未经真机标定。
手部独立使用 SG100 SDK，控制对象不经过 Gento 手部通道。

## 原版 SG100 SDK 控制双手

使用 `SG100_SDK_Publish/lib/linux-*/libsg100_sdk.so` 原版预编译库。
`native/sg100_bridge.cpp` 只提供 C ABI，供 Python 调用 C++ SDK；没有替换厂商实现。
启动脚本自动编译适配库；手动构建命令为 `./build_sg100_sdk.sh`。
若直接执行 colcon，请先运行此命令，再构建功能包。

| SDK 手指枚举 | SDK 关节 ID | ROS/URDF 关节名（S 为 L/R） |
|---|---|---|
| FINGER_THUMB | 1、2、3 | S_Joint1、S_Joint2、S_Joint3 |
| FINGER_INDEX | 4、5、6 | S_Joint4、S_Joint5、S_Joint6 |
| FINGER_MIDDLE | 7、8 | S_Joint7、S_Joint8 |
| FINGER_RING | 9、10、11 | S_Joint9、S_Joint10、S_Joint11 |

`config/hands.yaml` 保存 sdk_id、sdk_finger、finger_joint、source_name，
以及源限位与 SDK/MJCF 交集限位。旧 `*_thumb_j*` / `*_little_j*` ROS 关节名不再接受。
SDK 位置 API 与界面使用度，ROS 接口与兼容 Python 接口继续使用 rad。
左右手 a/b 映射使用 SDK 的 `joint_angle_map_v3.0.json`，不在调用前重复反转角度。

控制链路：面板/ROS → 原版 SDK `SetJointPositions` → CANFD 分片与 CRC
→ 本机 UDP 4742 → 虚拟掌板 → 关节插值；反馈反向进入原版
`CANFDProcessRxFrame`，通过 `GetCachedPositions` 发布 `/hand_states`。
UDP 仅承载 CANFD 分片，没有连接实际 CAN 总线。原版 CANFD 适配器的帧内
CRC-32/BZIP2 完整保留；本包只实现位置模式和整手状态查询。
SDK 的阻抗、力矩、触觉、标定、Flash 和 OTA 功能未在仿真中实现，不能用于验证其物理效果。
停止兼容 API 通过 SDK 发送当前姿态作为保持目标；不提供真机急停等效行为。

默认桥接运行时也可运行以下独立客户端示例（增加左手无名指 J9 目标 5°）：

```bash
source /opt/ros/humble/setup.bash
source /home/wxy/code/tianji/ros2_ws/install/setup.bash
ros2 run marvin_mesh_sim sg100_demo --sdk-root /home/wxy/code/tianji/SG100_SDK_Publish --hand L
```

直接 Python 调用：

```python
from marvin_mesh_sim.sg100_client import SG100Client
from ament_index_python.packages import get_package_share_directory

client = SG100Client('/home/wxy/code/tianji/SG100_SDK_Publish',
                     get_package_share_directory('marvin_mesh_sim'))
try:
    client.devices['L'].SetJointPositions([0.0] * 11)  # 原版 SDK，度
    client.flush()                                   # 发送原版 CANFD 分片
    # 控制循环内重复调用 client.read()，得到原版 SDK 解析的左右手反馈(rad)。
    # 初始反馈尚未到达或反馈超过 0.5 秒未更新时，read() 抛出 RuntimeError。
finally:
    client.close()
```

多个客户端同时运行时，同一只手以最后收到的有效目标为准。
SDK 库为预编译发布文件，当前 Linux x86_64 环境已经验证；其他平台需单独验证。

## ROS 接口

机械臂接口沿用 `marvin_sim`：

- `/marvin_sim/target_joints`：完整单臂或双臂 JointState，rad。
- `/marvin_sim/{left,right}/target_pose`：PoseStamped，m + xyzw；frame 必须是 `Gento_Luna_Leg_J4_Link`。
- `/marvin_sim/sdk_joint_states`：原版 SDK 收到的双臂反馈。
- `/marvin_sim/{left,right}/tcp_pose`：Gento SDK FK 经转换得到的 J4 下法兰位姿。
- `/marvin_sim/stop`、`/marvin_sim/enable`：双臂停止/启用服务。
- `/joint_states`：全部 44 个关节，用于整机 TF。

手部接口：

- `/marvin_mesh_sim/hand_targets`：一整只手的 11 关节 JointState，rad。
- `/marvin_mesh_sim/hand_states`：原版 SG100 SDK 解析的双手 22 关节反馈。
- `/marvin_mesh_sim/hand_status`：目标接受/拒绝信息。

本体/头部接口：

- `/marvin_mesh_sim/body_targets`：JointState，可发送一个或多个本体/头部位置目标，rad。
- `/marvin_mesh_sim/body_states`：原版 Gento SDK 接收的 8 个本体/头部关节反馈，rad。
- `/marvin_mesh_sim/body_status`：目标接受/拒绝信息。
- 关节名称和限位见 `config/body.yaml`。腰腿/躯干为 `Gento_Luna_Leg_J1_Joint`
  至 `Gento_Luna_Leg_J6_Joint`，头部为 `Head_J1_joint`、`Head_J2_Joint`。

底盘在原 MJCF 的地面位置固定，`world → base_link → 六段腰腿/躯干`；
头部与左右肩挂在第六段躯干上，躯干运动通过 TF 自然带动两臂。
本包的面板、`MeshSimulationSDK.fk()/ik()/move_pose()` 和 ROS 末端位姿接口
统一以 `Gento_Luna_Leg_J4_Link` 为参考系，末端仍为 `flange_L/flange_R`。
`frames.py` 根据生成 URDF 和当前躯干 SDK 反馈计算 J4 到左右肩的变换：

```text
正解：T_J4_flange = T_J4_Base × SDK_FK(q)
逆解：SDK_IK(inverse(T_J4_Base) × T_J4_target)
```

单个参考关节角导致 SDK 返回超限，不代表目标没有可行解。适配层优先使用当前参考角；
失败后最多尝试 18 个确定性的备用参考角，仍由原版 SDK 求解，选择候选中关节变化较小的有效解。
每个候选都必须通过原有限位、奇异标志和完整位姿的 FK 回代校验，不修改位置、朝向或关节限位。
界面会提示备用求解及最大关节变化；没有有效候选时保留 SDK 错误码并拒绝执行。

厂商原生 `GentoRobot.forward_kinematics()/inverse_kinematics()` 和底层
`GentoMeshKinematics` 仍使用各臂肩部基准，不能将 J4 位姿直接传给这些底层函数。
J4 后的躯干 J5/J6 运动会改变肩部相对 J4 的位置，转换使用当前反馈，不能写死变换。
J5/J6 尚未到达目标时拒绝新的 IK 目标；本版为单次 IK 求解，不做躯干与双臂的连续协同跟踪。
执行末端运动期间应保持 J5/J6 不动。J4 上游的躯干运动会带着 J4 参考系一起移动。
`world` 等其他坐标系的视觉目标需要先转换到 J4 后再发送。

可与 TF 对照：

```bash
ros2 run tf2_ros tf2_echo Gento_Luna_Leg_J4_Link flange_L
ros2 run tf2_ros tf2_echo Gento_Luna_Leg_J4_Link flange_R
```

机械臂名称为 Joint1_L..Joint7_L / Joint1_R..Joint7_R；手指名称和顺序
见 `config/hands.yaml`，名称为 L_Joint1..L_Joint11 / R_Joint1..R_Joint11。
手指几何和关节轴来自原 MJCF，未添加额外手指或假设耦合传动。

## 模型来源与边界

- `config/asset_report.json` 记录每个 STL 的大小、三角形数、包围盒和 SHA256，
  以及每个缺失网格的占位来源。
- 原 MJCF 的机械臂旋转含约数微弧度的导出舍入。为与 SDK 的标准 CCS 几何一致，
  机械臂旋转规范化到最近的精确 90° 转角；修正量记录在报告中。手指旋转保持原值。
- 基座使用真实底盘外形，移除了原先的圆柱支架；没有添加资源中不存在的轮关节。
  当前不模拟移动底盘行驶或浮动基座动力学。
- 底盘和第六段躯干使用原资源中同坐标系的较轻 STL 文件，避免加载大型 OBJ。
  本体/头部的装配变换、关节轴和限位保持原 MJCF，未做机械臂的转角规范化处理。
- 关节与惯性来自 MJCF。SDK 派生配置中的动力学项置零，避免误用其他机型数据；
  原始惯性保存在 URDF 中，但本版执行的是运动学插值，不模拟动力学/接触。
- 不包含自动避碰；生成的碰撞盒用于几何描述。SDK 手腕耦合限位没有该机型标定数据，
  采用原 MJCF 的独立关节限位，不能作为真机碰撞或安全验证依据。
- 末端目标执行关节运动，不保证 Cartesian 直线路径。只支持原版 SDK 基础位置控制子集，
  不支持力控、PVT 或 setPln_* 控制器轨迹上传协议。

## 测试

退出仿真及 SDK 客户端后执行：

```bash
source /opt/ros/humble/setup.bash
cd /home/wxy/code/tianji/ros2_ws
source install/setup.bash
python3 -m pytest src/marvin_mesh_sim/test -q
```

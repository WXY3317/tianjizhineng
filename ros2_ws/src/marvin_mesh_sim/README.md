# STL 双臂与灵巧手仿真

模型来自 `/home/wxy/code/tianji/mjcf/luna_m6_robot.xml` 和其引用的 STL。
独立包 `marvin_mesh_sim`，复用 `marvin_sim` 的原版 SDK UDP 传输层。

当前资源审计：双臂 14 个、左右手 24 个 STL 均已补齐，共 38 个真实网格。
网格逐字节复制到本包，36 个可动关节（双臂 14 + 双手 22）均保留。
每只手有拇指、食指、中指、小指共 4 指，原资源没有无名指。
更新源网格后，重新执行 `--build` 会重新审计并生成模型。
如果后续出现空网格，生成器会明确记录，并用原 MJCF 的橙色碰撞盒占位。

## 启动

原版 SDK 固定使用 UDP 4729/4730，不能同时启动两套后台。
启动脚本会复用本工作空间已经运行的后台，并重新打开界面，避免重复启动导致窗口闪退。
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

桌面显示 RViz、双臂 FK/IK 面板和独立灵巧手面板。机械臂面板使用度/mm，
手指面板使用 rad。支持单指调节、手部张开/弯曲、双臂关节运动和末端 IK 运动。

## SDK 调用

原版 `libMarvinSDK.so` 控制双臂，`libKine.so` 计算 FK/IK。
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
ros2 run marvin_mesh_sim sdk_demo --sdk-root /home/wxy/code/tianji/tianji_armSDK
```

Python API 示例（机械臂关节角为度，位姿矩阵平移为米，手指角为 rad）：

```python
from marvin_mesh_sim.sdk import MeshSimulationSDK

with MeshSimulationSDK('/home/wxy/code/tianji/tianji_armSDK') as sdk:
    q = sdk.get_joint_positions('A')
    q[0] += 3
    pose = sdk.fk('A', q)         # 原版 SDK 正解，Base_L → flange_L
    solution = sdk.ik('A', pose)  # 原版 SDK 逆解，以当前反馈为参考
    sdk.move_joints('A', solution)
    # 也可以 sdk.move_pose('A', pose)
    hands = sdk.get_hand_state()
    sdk.set_hand_joints('L', [0.0] * 11, duration=1.0)
```

`sdk.robot` 是原版 `Marvin_Robot`，可直接调用原有位置控制 API。
手指是本包增加的仿真接口（UDP 4741），不宣称厂商机械臂 SDK 原生支持手指。
SDK 目标地址只允许本机已分配的非回环 IPv4，原版库会拒绝 127.*。

## ROS 接口

机械臂接口沿用 `marvin_sim`：

- `/marvin_sim/target_joints`：完整单臂或双臂 JointState，rad。
- `/marvin_sim/{left,right}/target_pose`：PoseStamped，m + xyzw；frame 必须是 Base_L/Base_R。
- `/marvin_sim/sdk_joint_states`：原版 SDK 收到的双臂反馈。
- `/marvin_sim/{left,right}/tcp_pose`：原版 SDK FK 得到的法兰位姿。
- `/marvin_sim/stop`、`/marvin_sim/enable`：双臂停止/启用服务。
- `/joint_states`：全部 36 个关节，用于模型 TF。

手部接口：

- `/marvin_mesh_sim/hand_targets`：一整只手的 11 关节 JointState，rad。
- `/marvin_mesh_sim/hand_states`：双手 22 关节反馈。
- `/marvin_mesh_sim/hand_status`：目标接受/拒绝信息。

机械臂名称为 Joint1_L..Joint7_L / Joint1_R..Joint7_R；手指名称和顺序
见 `config/hands.yaml`，例如 L_thumb_j1..L_thumb_j3。
手指参数和关节轴来自原 MJCF；未添加无名指或假设手部耦合传动。

## 模型来源与边界

- `config/asset_report.json` 记录每个 STL 的大小、三角形数、包围盒和 SHA256，
  以及每个缺失网格的占位来源。
- 原 MJCF 的机械臂旋转含约数微弧度的导出舍入。为与 SDK 的标准 CCS 几何一致，
  机械臂旋转规范化到最近的精确 90° 转角；修正量记录在报告中。手指旋转保持原值。
- 基座是新增固定支架，不模拟源机器人移动底盘、腰腿或头部。
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

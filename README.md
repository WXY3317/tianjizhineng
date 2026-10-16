# 天机双臂机器人开发与仿真

包含天机机器人 SDK、双臂/灵巧手模型、ROS 2 控制节点和运动学仿真功能包。

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
| `real_test` | 真机控制脚本 |

## 获取代码

两个大型 OBJ 资源使用 Git LFS，克隆前请安装 Git LFS：

```bash
git lfs install
git clone git@github.com:WXY3317/tianjizhineng.git
cd tianjizhineng
git lfs pull
```

## 启动双臂与灵巧手仿真

环境：Linux、ROS 2 Humble、Python 3.10，以及 NumPy、PyYAML、Tkinter、xacro、
RViz2、robot_state_publisher、colcon。需要与本机架构匹配的 SDK 动态库。

```bash
cd ros2_ws
./run_marvin_mesh_sim.sh --build
# 后续启动：
./run_marvin_mesh_sim.sh
```

包含底盘、腰腿/躯干、头部、双臂和双手，共 47 个真实 STL，
支持 14 个机械臂关节、22 个手指关节和 8 个本体/头部关节。
双臂位置控制、正逆解以及躯干/头部控制统一通过原版 Gento SDK，
双手通过原版 SG100 SDK。双臂对外正逆解统一参考躯干 `Gento_Luna_Leg_J4_Link`。
手指按 SDK 的 Joint1–11 编号，第四指使用 FINGER_RING（无名指）命名。
重复启动会复用后台并重新打开界面。
本版是运动学仿真，不含接触动力学或自动避碰。

详细用法和 Python API 见 [marvin_mesh_sim 使用说明](ros2_ws/src/marvin_mesh_sim/README.md)。
SDK 和第三方模型保留各自随附的许可证与说明。

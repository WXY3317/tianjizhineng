# 天机双臂机器人真机控制（real 分支）

通过 ROS 2 话题连接实体控制器，读取双臂关节反馈和 TCP 位姿，
调用 Gento SDK 完成 FK/IK、关节轨迹规划及执行。

| 目录 | 内容 |
|---|---|
| `ros2_ws/src/gento_dual_arm` | 双臂真机 ROS 2 控制功能包 |
| `ros2_ws/src/gento_dual_arm/REAL_TEST.md` | 包内单臂、双臂命令行测试说明 |
| `tianji-robot-SDK` | Gento 原厂 SDK 与示例 |
| `tianji_armSDK` | Marvin 原厂 SDK，保留供参考 |
| `SG100_SDK_Publish` | SG100 原厂 SDK；当前双臂节点不控制灵巧手 |
| `marvin_description` | 机器人描述和网格资源 |
| `mjcf` | 原始模型及资源 |

## 启动

需要 ROS 2 Humble、Python 3.10、colcon，以及与真机固件和主机架构匹配的 SDK 动态库。
将 IP 替换为实际控制器地址：

```bash
cd ros2_ws
# 首次构建并连接真机，默认只预览
./run_real_robot.sh --build ip:=6.6.7.190
# 显式启用运动
./run_real_robot.sh ip:=6.6.7.190 execute:=true
```

话题格式、坐标系、目标发布和停止示例见 [双臂真机使用说明](ros2_ws/REAL_ROBOT.md)，
完整参数见 [工作空间说明](ros2_ws/README.md)。

本分支已移除仿真功能包及启动脚本；仿真代码保留在其他分支。
模型资源不代表已完成真机标定。当前接口没有碰撞检测，尚未进行硬件联调。

## 获取代码

两个大型 OBJ 资源使用 Git LFS：

```bash
git lfs install
git clone --branch real git@github.com:WXY3317/tianjizhineng.git
cd tianjizhineng
git lfs pull
```

SDK 和第三方模型保留各自随附的许可证与说明。

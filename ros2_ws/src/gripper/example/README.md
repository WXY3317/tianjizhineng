# 命令行示例

在 `ros2_ws/src/gripper/` 下执行：

```sh
cmake -S example -B example/build -DCMAKE_BUILD_TYPE=Release
cmake --build example/build -j2
ctest --test-dir example/build --output-on-failure
./example/run.sh --help
```

示例使用包内 sdk/，与 ROS2 节点分开构建。ROS2 使用说明见 ../README.md。

输入命令不带命令编号，例如 `ENABLE`、`MOVE 0.020 0.005`、`DISABLE`。
设备地址仍通过启动参数 `motor_id` 指定。

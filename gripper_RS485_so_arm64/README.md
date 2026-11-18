# 动态库部署

程序通过 `$ORIGIN`（程序或动态库自身所在目录）查找依赖，
无需设置 `LD_LIBRARY_PATH`。

整体复制本目录即可保留 `build/`、`build_arm/` 和 `lib/` 的相对位置。
也可以只交付程序与对应架构的三个动态库，例如：

```text
gripper/
├── bin/
│   ├── position_loop_demo
│   ├── velocity_loop_demo
│   ├── torque_loop_demo
│   └── zero_demo
└── lib/
    └── x86/                 # ARM64 使用 arm/
        ├── libgripper_lib.so
        ├── libgrip_pawl_lib.so
        └── libcan_motor_driver.so
```

把程序和三个库全部放在同一个目录也可以。请保持同一架构、配套版本，
目标机器仍需提供兼容的 Linux、glibc 和 libstdc++。

更新构建配置后重新编译：

```bash
# x86_64
cmake -S . -B build
cmake --build build --parallel 2

# ARM64 交叉编译，使用独立构建目录
cmake -S . -B build_arm -DCMAKE_TOOLCHAIN_FILE=aarch64-toolchain.cmake
cmake --build build_arm --parallel 2
```

配置只影响重新链接的文件；更新 `gripper_RS485` 后，应将新生成的
`libgripper_lib.so`、`libgrip_pawl_lib.so` 复制到本目录对应的 `lib/` 子目录。
第三方预编译库的运行路径不会因编译示例而改变。

## 交互式夹爪控制 Demo

此版本直接编译 `../gripper_RS485_so/demo` 的程序代码，使用本目录的
`include/` 和 `lib/arm/`。在 tianji (aarch64) 上将两个目录保持相邻，执行：

```bash
cmake -S ../gripper_RS485_so/demo -B bin -DSDK_ROOT="$PWD" -DBUILD_TESTING=OFF
cmake --build bin --parallel 2
./run_demo.sh --help
```

也可在装有 `aarch64-linux-gnu-g++` 及 ARM64 Eigen 开发文件的主机上交叉编译，
配置时追加 `-DCMAKE_TOOLCHAIN_FILE="$PWD/aarch64-toolchain.cmake"`。
`bin/gripper_demo`、`bin/calibration.csv`、`lib/arm/*.so` 和 `run_demo.sh`
需保持上述相对位置。每次修改校准表后重新运行 CMake 配置以复制新表。

驱动器断电后零位若丢失，先在空载且张开方向安全的条件下建立零位；
当前交互式 demo 的 `ZERO` 仍要求旧零点距离张开端小于 1°，无法处理
任意零位丢失。可先构建并运行本目录的 ARM `zero_demo`：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target zero_demo --parallel 2
./build/zero_demo
```

它使用 `/dev/ttyUSB0`、ID 1、115200；确认标零成功且张开端反馈与
校准表一致后，在同一次通电期间运行：

```bash
./run_demo.sh --use-existing-zero /dev/ttyUSB0 1 115200
```

然后逐行输入 `ENABLE 1`、`MOVE 2 0.060 0.010`、`DISABLE 3`、`QUIT`。
当前校准表仅对应 ID 1 的特定夹爪；换设备须重新校准。

右夹爪使用相同 RS485 串口、Modbus ID 2 和独立的 `bin/right_calibration.csv`。
2026-11-06 重新标定：自动张开标零后，实测全开 98 mm；手动确认两指完全闭合、
静止复读角度 63.5037384033 rad。中间 511 个点由机构模型推算，尚未逐点实测。
后续在原表反馈 60 mm 时实测约 45 mm，已按此增加中间校正点；其余
中间开度仍需卡尺复核。更新表后需重新配置 CMake 以复制到 `bin/`，
并重启控制进程。
第一次修正后又在 19.7244053 rad 处实测 56 mm；第二次右表曾从端点
原表按两处实测点重建。
随后实测 20/30/50/70 mm 目标分别对应 13/25/50/66 mm；当前右表
合并六个中间实测点重建，仍需验证其他位置及重复性。
2026-11-07 重新上电后右侧出现新的中段偏差；当前右表按本次 5～80 mm
的 16 个实测点再修正，仍需复测全闭角度及下次上电后的重复性。
在这次零位保持有效时，使用 `./run_demo.sh --use-existing-zero /dev/gripper_rs485 2 115200`
启动右侧，`MOVE` 的编号仍只是命令编号。右侧和左侧共用串口，不能同时运行两个独立控制进程。

终端反馈默认每秒一条，内部电流检查仍约每 100 ms 执行一次。
如需更密集的反馈，在启动命令中加入 `--feedback-ms 100`；
`--feedback-ms` 可设置 100～60000 ms。电流首次触发、反馈首次失效或新故障会立即输出。

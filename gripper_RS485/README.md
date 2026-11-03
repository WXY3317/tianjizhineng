# RS485 夹爪控制工程

整合日期：2026-10-29。

业务代码与标定表来自 `gripper_RS485_so_intel/demo/`；SDK 头文件、x86_64/ARM64 动态库及供应商示例来自 `gripper_RS485_so_arm64/`。原始目录未修改。未复制历史临时日志、远程同步脚本和旧构建产物。

## 目录

```text
gripper_RS485/
├── CMakeLists.txt               # 统一构建入口
├── aarch64-toolchain.cmake      # ARM64 交叉编译工具链
├── run.sh                      # 根目录构建的启动入口
├── demo/                       # ROS2 功能包，example/ 保留命令行示例
├── include/                    # 配套 SDK 头文件
├── lib/arm/                    # ARM64 SDK
├── lib/x86/                    # x86_64 SDK（同样来自 ARM64 压缩包）
├── src/                        # 供应商示例及 CSV 日志工具
└── docs/                       # API 手册、Intel demo 历史说明
```

## 原生构建（x86_64 或 ARM64 Linux）

依赖：C++17 编译器、CMake >= 3.16、Eigen3 开发包。根据目标处理器自动选择 `lib/x86` 或 `lib/arm`，其他架构会报错。

```sh
cd gripper_RS485
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
./run.sh --help
```

默认构建业务程序 `build/demo/example/gripper_demo`、诊断工具 `gripper_inspect` 和测试。供应商运动示例默认关闭，增加 `-DGRIPPER_BUILD_EXAMPLES=ON` 可构建四个示例。`combined_loop_demo.cpp` 保留为参考，未加入构建。

## ARM64 交叉编译

主机需安装 `aarch64-linux-gnu-g++`，并准备目标平台兼容的头文件/开发环境与 Eigen3。

```sh
cmake -S . -B build_arm -DCMAKE_TOOLCHAIN_FILE=aarch64-toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build_arm -j2
```

交叉编译产物须在 ARM64 Linux 上运行；当前没有配置模拟器。目标端重新原生构建最直接。复制交叉构建的产物时，须保证动态库及标定表路径可用：当前业务程序内置构建时的标定表绝对路径，尚未实现可搬迁安装包；根目录 `run.sh` 默认运行 `build/demo/example/gripper_demo`。

## 运行

确认同一台 ID 1 夹爪的既有零位有效后：

```sh
./run.sh --use-existing-zero --current-threshold-a 0.5 /dev/ttyUSB0 1 115200
```

程序启动后不自动使能或运动，保持标准输入打开，按步骤发送命令并读取 ACK 和反馈：

```text
ENABLE 1
MOVE 2 0.060 0.010
DISABLE 3
QUIT
```

MOVE 参数为命令编号、开度（m）、速度（m/s）。开度范围 0～0.098 m。ACK 不表示运动到位，必须读取反馈再决定后续动作。标准输出为 JSON Lines，SDK 日志通过启动脚本输出到标准错误。

支持 `CLEAR 编号`、`ZERO 编号`。ZERO 会主动张开，且依赖现有标定端点检查，不是未知零位下的通用恢复方法。标定表仅适用于原 ID 1 夹爪；零位或机构变化后需重新标定。

反馈电流绝对值达到默认 0.5 A 时，取消原目标并保持捕获的原始电机位置。它不是硬件限流或恒力控制；力反馈是模型估算。控制逻辑及原标定表保持原样，所有 SDK 头文件与库均使用 ARM64 包版本。

## 验证状态

本机 x86_64：业务程序、诊断工具、四个供应商示例编译通过；状态机和标定测试 2/2 通过；根启动脚本 `--help` 通过。SDK 头文件有既有成员初始化顺序警告。

ARM64 交叉编译及真实夹爪通信/运动未验证。历史实机记录位于 `docs/Intel-demo历史说明.md`，不代表当前 SDK 组合的实机验证结果。

## ROS2 话题功能包

功能包已迁移至 `ros2_ws/src/gripper/`，包名 `gripper`，内部包含 SDK、业务代码、消息、配置和示例。使用说明见 [gripper README](../ros2_ws/src/gripper/README.md)。本目录根 CMake 仍可构建迁移后的命令行示例。

## 右夹爪 ID 2：点动与端点标定

新增的 `build/gripper_calibrate` 是独立串口工具，操作前停止占用该串口的 ROS 节点。它固定 115200 波特率、电机方向 `+1`。`open` 发 `-0.5 rad/s`，`close` 发 `+0.5 rad/s`，每次最多 5 秒；结束时发送停止与失能。电流达到 0.5 A、驱动报错或通信失败时尝试停止并失能；这是软件采样阈值，不是硬件限流。操作时夹爪应空载、行程无遮挡，观察实物运动；不要连续顶住机械限位。命令均不自动清错。

在目标机器上构建（或用 ARM64 工具链交叉编译），以下命令在 `gripper_RS485/` 目录执行：

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target gripper_calibrate -j2
PORT=/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0
./build/gripper_calibrate "$PORT" 2 status
./build/gripper_calibrate "$PORT" 2 open 1000
```

逐次点动，靠近全开时缩短到 `open 100` 或 `open 200`。确认**机械全开**后停止运动，测量接触面间距，执行 `status`，记下 `angle_rad` 作为 `OPEN_BEFORE_ZERO`。同一位置再读一次，检查角度稳定。确认位置未变后用记录值设零；命令要求当前角度与记录值相差不超过 0.03 rad，并在写入后检查反馈接近 0 rad：

```sh
./build/gripper_calibrate "$PORT" 2 set-open-zero OPEN_BEFORE_ZERO
./build/gripper_calibrate "$PORT" 2 status
```

**保留 `OPEN_BEFORE_ZERO` 的原始数字。**零点建立后，逐次执行 `close 1000`，接近闭合时改用 `close 100` 或 `close 200`。在两侧接触面刚接触时停止，不要持续挤压。执行 `status`，记录 `angle_rad` 作为 `CLOSED_AFTER_ZERO`；用卡尺确认闭合开度约为 0 mm。此时右夹爪的零位后闭合行程应为正值。若为负值，停止，不要套用左夹爪模型。

为避免混用设零前后坐标，生成脚本要求**两个角度都在设零前的同一坐标系**。因此将闭合端读数还原为 `CLOSED_BEFORE_ZERO = OPEN_BEFORE_ZERO + CLOSED_AFTER_ZERO`，再生成右夹爪表。以下示例中的角度和宽度须替换成现场记录，`--model` 必须指向当前机器架构的动态库：

```sh
python3 generate_calibration.py \
  --model ./lib/arm/libgrip_pawl_lib.so \
  --open-width-mm 98 \
  --open-angle-rad OPEN_BEFORE_ZERO \
  --closed-angle-rad CLOSED_BEFORE_ZERO \
  --output right_calibration.csv
```

表中间 511 点来自原机构模型，不是实测值；用卡尺核对几个中间开度后再用于位置控制。ROS2 节点可通过参数 `right.calibration_path` 指向此文件；不要让 ID 2 使用默认的左夹爪表。设零或端点记录前不要用旧表发送位置控制命令。`set-open-zero` 会修改驱动器零位，仅在确认全开位置时执行。



angle_rad=-49.512298584 current_a=0 speed_rad_s=0
nvidia@vision:~/wxy/tianjizhineng/gripper_RS485$ ./build/gripper_calibrate "$PORT" 2 set-open-zero -49.5127754211
Bus SocketCan registered
SerialBus Rs485 registered
ProtocolFactory register: kaiser
ProtocolFactory register: hdriver
ProtocolFactory register: lens
ModbusProtocolFactory register: smodule_modbus
open_zero_set previous_angle_rad=-49.512298584 angle_rad=0
nvidia@vision:~/wxy/tianjizhineng/gripper_RS485$ ./build/gripper_calibrate "$PORT" 2 status
Bus SocketCan registered
SerialBus Rs485 registered
ProtocolFactory register: kaiser
ProtocolFactory register: hdriver
ProtocolFactory register: lens
ModbusProtocolFactory register: smodule_modbus
angle_rad=0 current_a=0 speed_rad_s=0
nvidia@vision:~/wxy/tianjizhineng/gripper_RS485$

右夹爪 ID 2 的 `right_calibration.csv` 已按目标/实测开度 15→14、20→18、30→27 mm 修正；原始仅端点归一化的表保存在 `right_calibration_initial.csv`。用 `refine_calibration.py` 可从原表和新的测量点重新生成，不要对已修正表重复应用同一组修正。中间位置仍需独立实测复核。

右表第二次修正使用目标/实测开度 10→10、20→20、30→30、40→38、50→47、60→56、70→65、80→74、90→82 mm；上一版本保存在 `right_calibration_v2.csv`。90～93 mm 段因实测 90→82 与全开 93→93 差异较大，必须复测，不要把该段反馈当作独立测量。

右表第三次修正仅调整 80～93 mm，使用最新复测 84→83、86→84、90→85 mm，80 mm 以下映射保持不变；上一版本保存在 `right_calibration_v3.csv`。90 mm 目标对应电机角度约 0.212 rad，接近全开机械端，复测时应监测电流并避免连续顶限位。

当前右夹爪重新确认全开实际宽度为 85 mm，设零前角度约 -51.4629478455 rad，已在该位置重新设零。原 93 mm 的 `right_calibration.csv` 暂不可用于位置控制；需在此次零位下实测完全闭合角度后，用 `--open-width-mm 85 --open-angle-rad 0 --closed-angle-rad 实测闭合角度` 生成新表。

## Tianji 串口固定名称

`udev/99-gripper-rs485.rules` 将当前 CH340 RS485 适配器固定为 `/dev/gripper_rs485`。该适配器没有唯一 USB 序列号，规则同时匹配 VID/PID `1a86:7523` 和当前物理 USB 路径 `platform-14180000.pcie-pci-0000:01:00.0-usb-0:2.1:1.0`。更换 USB 插口或集线器后必须重新核对 `udevadm info --query=property --name=/dev/ttyUSB0` 中的 `ID_PATH`。Modbus ID 1/2 在 USB 适配器后面，udev 无法据此区分夹爪；两只夹爪共用这个串口时仍分别用电机 ID 选设备。

在 Tianji 上安装（需要 sudo）：

```sh
cd ~/wxy/tianjizhineng/gripper_RS485
sudo install -m 0644 udev/99-gripper-rs485.rules /etc/udev/rules.d/99-gripper-rs485.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty --action=add
udevadm settle
readlink -f /dev/gripper_rs485
```

最后一条应解析到当前 RS485 适配器，例如 `/dev/ttyUSB0`。后续可用 `port:=/dev/gripper_rs485` 和 `PORT=/dev/gripper_rs485`。这个规则只创建稳定设备名，不影响电机零位或标定表。

2026-11-02 右夹爪 ID 2 重新确认全开 85 mm 并在全开处设零，完全闭合角度 56.6882591248 rad。当前 `right_calibration.csv` 已按 85 mm/0 mm 端点重新生成，中间开度仍是机构模型估算；先前 93 mm 表保存在 `right_calibration_93mm.csv`，不可用于当前零位/机构状态。

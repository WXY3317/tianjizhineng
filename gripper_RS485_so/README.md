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

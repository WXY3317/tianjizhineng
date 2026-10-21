#!/usr/bin/env bash
# 编译自研 SG100 C ABI 适配层并链接原版 SDK，供仿真中的 Python 调用。
set -e
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sdk_dir="${1:-$(dirname -- "$workspace_dir")/SG100_SDK_Publish}"
package_dir="$workspace_dir/src/marvin_mesh_sim"
case "$(uname -m)" in
  x86_64) platform=linux-x86_64 ;;
  aarch64|arm64) platform=linux-arm64 ;;
  *) echo "Unsupported SG100 SDK platform: $(uname -m)" >&2; exit 1 ;;
esac
mkdir -p "$package_dir/native/lib"
g++ -std=c++14 -O2 -fPIC -shared "$package_dir/native/sg100_bridge.cpp" \
  -I"$sdk_dir/include" -L"$sdk_dir/lib/$platform" -lsg100_sdk \
  -Wl,-rpath,"$sdk_dir/lib/$platform" -pthread \
  -o "$package_dir/native/lib/libsg100_sim_bridge.so"

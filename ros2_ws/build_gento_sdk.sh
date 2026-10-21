#!/usr/bin/env bash
# 从本地原版 Gento 源码构建 SDK 动态库，供 Python 接口加载。
set -e
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
gento_root="${1:-$(dirname -- "$workspace_dir")/tianji-robot-SDK}"
python3 - "$gento_root" <<'PY'
from pathlib import Path
import subprocess
import sys

sdk=Path(sys.argv[1]).expanduser().resolve()
root=sdk/'C_SDK'
sources=sorted(root.rglob('*.cpp'))
if not sources:raise SystemExit(f'Gento SDK sources missing: {root}')
includes=sorted({p.parent for p in root.rglob('*.h')})
target=sdk/'PYTHON_SDK/libGentoSDKPY.so'
command=['g++',*map(str,sources),*[f'-I{p}' for p in includes],'-O2','-fPIC','-shared',
         '-DL1_SDK_EXPORTS','-DCMPL_LIN','-static-libgcc','-static-libstdc++',
         '-lpthread','-ldl','-lm','-o',str(target)]
print(f'Compiling original Gento SDK -> {target}',flush=True)
subprocess.run(command,check=True)
print('Original Gento SDK build completed.',flush=True)
PY

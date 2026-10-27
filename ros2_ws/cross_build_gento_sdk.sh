#!/usr/bin/env bash
# Cross-compile the vendor Python SDK without replacing the host library.
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 - "$workspace_dir" "$@" <<'PY'
import argparse
import os
from pathlib import Path
import shlex
import shutil
import struct
import subprocess
import sys
import tempfile

workspace = Path(sys.argv[1])
parser = argparse.ArgumentParser(prog='cross_build_gento_sdk.sh', description='Cross-compile Gento SDK for ARM Linux (default: ARM64).')
parser.add_argument('--arch', choices=('arm64', 'armhf'), default='arm64',
                    help='arm64: 64-bit aarch64; armhf: 32-bit ARM hard-float')
parser.add_argument('--sdk-root', type=Path, default=workspace.parent / 'tianji-robot-SDK')
parser.add_argument('--output-dir', type=Path, help='Default: ros2_ws/artifacts/gento-sdk/ARCH')
parser.add_argument('--compiler', help='Cross C++ compiler executable; default depends on --arch')
parser.add_argument('--sysroot', type=Path, help='Optional target Linux sysroot with headers and libraries')
args = parser.parse_args(sys.argv[2:])

triplet, machine, elf_class = {'arm64': ('aarch64-linux-gnu', 183, 2),
                              'armhf': ('arm-linux-gnueabihf', 40, 1)}[args.arch]
compiler = args.compiler or triplet + '-g++'
if shutil.which(compiler) is None:
    parser.error(f'Compiler not found: {compiler}. On Ubuntu/Debian install g++-{triplet}.')
try:
    actual_target = subprocess.check_output([compiler, '-dumpmachine'], text=True).strip()
except subprocess.CalledProcessError as exc:
    parser.error(f'Cannot query compiler target: {exc}')
if (args.arch == 'arm64' and not actual_target.startswith('aarch64')) or (
        args.arch == 'armhf' and (not actual_target.startswith('arm') or 'hf' not in actual_target)):
    parser.error(f'Compiler target {actual_target!r} does not match {args.arch}')

sdk = args.sdk_root.expanduser().resolve()
source_root = sdk / 'C_SDK'
sources = sorted(source_root.rglob('*.cpp'))
if not sources:
    parser.error(f'No SDK C++ sources found: {source_root}')
includes = sorted({p.parent for p in source_root.rglob('*.h')})
output = (args.output_dir or workspace / 'artifacts/gento-sdk' / args.arch).expanduser().resolve()
# Keep cross outputs separate from the SDK library used by this host.
if output == (sdk / 'PYTHON_SDK').resolve():
    parser.error('--output-dir must not be the host SDK PYTHON_SDK directory')
if args.sysroot is not None:
    args.sysroot = args.sysroot.expanduser().resolve()
    if not args.sysroot.is_dir():
        parser.error(f'Sysroot directory not found: {args.sysroot}')
output.mkdir(parents=True, exist_ok=True)
target = output / 'libGentoSDKPY.so'
with tempfile.TemporaryDirectory(prefix='.cross-build-', dir=output) as temp_dir:
    temporary = Path(temp_dir) / target.name
    command = [compiler]
    if args.sysroot is not None:
        command.append(f'--sysroot={args.sysroot}')
    command += [*map(str, sources), *[f'-I{p}' for p in includes],
                '-O2', '-fPIC', '-shared', '-DL1_SDK_EXPORTS', '-DCMPL_LIN',
                '-static-libgcc', '-static-libstdc++', '-Wl,-soname,libGentoSDKPY.so',
                '-o', str(temporary), '-lpthread', '-ldl', '-lm']
    print(f'Compiler target: {actual_target}', flush=True)
    print(shlex.join(command), flush=True)
    subprocess.run(command, check=True)
    header = temporary.read_bytes()[:20]
    if len(header) < 20 or header[:4] != b'\x7fELF' or header[4] != elf_class or header[5] != 1:
        raise SystemExit('Build output has an unexpected ELF format; existing output was preserved')
    kind, actual_machine = struct.unpack_from('<HH', header, 16)
    if kind != 3 or actual_machine != machine:
        raise SystemExit(f'Unexpected ELF type/machine: {kind}/{actual_machine}; existing output was preserved')
    os.replace(temporary, target)
print(f'Built and verified {args.arch} library: {target}')
print('Deploy this library to PYTHON_SDK/libGentoSDKPY.so on the matching ARM Linux host.')
print('Use a toolchain/sysroot compatible with the target glibc. Target hardware execution was not tested.')
PY

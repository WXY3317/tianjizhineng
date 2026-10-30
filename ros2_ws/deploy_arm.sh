#!/usr/bin/env bash
# Build ARM64 SDK, sync sources, and build the ROS package without starting it.
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 - "$workspace_dir" "$@" <<'PY'
import argparse
import hashlib
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys

workspace = Path(sys.argv[1])
project = workspace.parent
parser = argparse.ArgumentParser(prog='deploy_arm.sh', description='Cross-build ARM64 SDK and deploy the project via SSH; never starts robot control.')
parser.add_argument('--host', default='tianji', help='SSH host or configured alias')
parser.add_argument('--user', default='nvidia')
parser.add_argument('--remote-dir', default='/home/nvidia/wxy/tianjizhineng')
parser.add_argument('--ros-distro', default='humble')
parser.add_argument('--compiler', help='ARM64 g++ executable')
parser.add_argument('--sysroot', type=Path)
parser.add_argument('--skip-remote-build', action='store_true', help='Sync only after cross-compilation')
parser.add_argument('--dry-run', action='store_true', help='Print operations without compiling, connecting, or writing files')
args = parser.parse_args(sys.argv[2:])
if not args.remote_dir.startswith('/') or args.remote_dir.rstrip('/') in ('', '/','/home','/home/nvidia'):
    parser.error('--remote-dir must be an absolute project directory')
if args.host.startswith('-') or args.user.startswith('-') or not args.host or not args.user:
    parser.error('Invalid SSH host or user')
if '/' in args.ros_distro or not args.ros_distro:
    parser.error('Invalid ROS distribution')
for name in ('ssh', 'rsync'):
    if shutil.which(name) is None:
        parser.error(f'Required executable not found: {name}')

env = os.environ.copy()
compiler = args.compiler or shutil.which('aarch64-linux-gnu-g++')
sysroot = args.sysroot
# Support the unprivileged Ubuntu toolchain previously extracted in this project.
local_root = workspace / 'artifacts/cross-toolchain/root'
local_compiler = local_root / 'usr/bin/aarch64-linux-gnu-g++-11'
if compiler is None and local_compiler.is_file():
    compiler = str(local_compiler)
    sysroot = sysroot or local_root
    libraries = str(local_root / 'usr/lib/x86_64-linux-gnu')
    env['LD_LIBRARY_PATH'] = libraries + (':' + env['LD_LIBRARY_PATH'] if env.get('LD_LIBRARY_PATH') else '')
if compiler is None:
    parser.error('Install g++-aarch64-linux-gnu or specify --compiler and optionally --sysroot')

remote = f'{args.user}@{args.host}'
ssh = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10',
       '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=6']
remote_dir = args.remote_dir.rstrip('/')
remote_library = remote_dir + '/tianji-robot-SDK/PYTHON_SDK/libGentoSDKPY.so'
library = workspace / 'artifacts/gento-sdk/arm64/libGentoSDKPY.so'

def run(command, **kwargs):
    print('+ ' + shlex.join(map(str, command)), flush=True)
    if args.dry_run:
        if 'input' in kwargs:
            print('Remote script:\n' + kwargs['input'], flush=True)
        return
    try:
        return subprocess.run(command, check=True, **kwargs)
    except subprocess.CalledProcessError as exc:
        if command[0] == 'ssh' and exc.returncode == 255:
            print(f'SSH connection failed: {remote} (exit 255).', file=sys.stderr)
            print('Check network routing/VPN, the target IP, and target ssh service. '
                  'A key-exchange or banner failure occurs before public-key authentication.', file=sys.stderr)
            print('Diagnose with: ' + shlex.join([*ssh, '-v', remote, 'uname -m']), file=sys.stderr)
        else:
            print(f'Command failed (exit {exc.returncode}): {shlex.join(map(str, command))}', file=sys.stderr)
        raise SystemExit(exc.returncode) from None

def remote_script(script):
    run([*ssh, remote, 'bash -se'], input=script, text=True)

# Read-only preflight: ensure the destination really is ARM64 before transferring.
setup = f'/opt/ros/{args.ros_distro}/setup.bash'
preflight = 'test "$(uname -m)" = aarch64\ncommand -v rsync >/dev/null\n'
preflight += r"""python3 - <<'CHECK_ROBOT_PROCESSES'
import sys
from pathlib import Path
executables = ('/gento_driver/bridge', '/gento_control/control', '/gento_feedback/feedback',
               '/gento_dual_arm/bridge', '/gento_driver/arm_control', '/gento_driver/dual_arm_control')
active = []
for process in Path('/proc').iterdir():
    if not process.name.isdigit():
        continue
    try:
        args = [arg.decode(errors='replace') for arg in (process / 'cmdline').read_bytes().split(bytes([0])) if arg]
    except OSError:
        continue
    if any(arg.endswith(executables) for arg in args):
        active.append(process.name + ': ' + ' '.join(args))
if active:
    sys.exit('Stop robot nodes before deploying; no process was stopped automatically:\n' + '\n'.join(active))
print('No running robot nodes detected')
CHECK_ROBOT_PROCESSES
"""
if not args.skip_remote_build:
    preflight += f'test -f {shlex.quote(setup)}\ncommand -v colcon >/dev/null\n'
print('1/4 Verify remote ARM64 environment', flush=True)
remote_script(preflight)

print('2/4 Cross-compile ARM64 SDK', flush=True)
command = [str(workspace / 'cross_build_gento_sdk.sh'), '--arch', 'arm64', '--compiler', compiler]
if sysroot is not None:
    command += ['--sysroot', str(sysroot.expanduser().resolve())]
run(command, env=env)

print('3/4 Sync project sources and ARM64 library', flush=True)
remote_script('mkdir -p ' + shlex.quote(remote_dir) + '\n')
exclusions = '''
.git/ .vscode/ .idea/ __pycache__/ .pytest_cache/ *.egg-info/
build/ install/ log/ build_real/ install_real/ log_real/
artifacts/ *.so *.dll *.exe /tianji-robot-SDK/FXPlatform
'''.split()
rsync = ['rsync', '-az', '--stats', '--protect-args', '-e', shlex.join(ssh)]
run([*rsync, *(f'--exclude={item}' for item in exclusions), str(project) + '/', remote + ':' + remote_dir + '/'])
# Stage separately so an incomplete upload never replaces the target library.
staged = remote_library + '.upload'
run([*rsync, str(library), remote + ':' + staged])
digest = '<compiled-library-sha256>' if args.dry_run else hashlib.sha256(library.read_bytes()).hexdigest()
verify = f'''import ctypes, hashlib, os, struct
from pathlib import Path
p = Path({staged!r})
data = p.read_bytes()
assert hashlib.sha256(data).hexdigest() == {digest!r}, 'Library SHA256 mismatch'
header = data[:20]
assert header[:4] == bytes([127, 69, 76, 70]) and header[4:6] == bytes([2, 1]) and struct.unpack_from('<H', header, 18)[0] == 183, 'Not ARM64 ELF'
dll = ctypes.CDLL(str(p))
for name in ('FX_L1_State_ResetError', 'FX_L1_Runtime_SetJointPosCmd', 'FX_L1_Kinematics_InverseKinematics',
             'FX_L1_Runtime_SetSpeedRatio', 'FX_L1_Config_SetTraj', 'FX_L1_Runtime_RunTraj'):
    getattr(dll, name)
os.replace(p, {remote_library!r})
print('ARM64 SDK checksum, architecture and loading verified')
'''
remote_script('python3 - <<\'VERIFY_ARM_SDK\'\n' + verify + 'VERIFY_ARM_SDK\n')

if not args.skip_remote_build:
    print('4/4 Build ROS package on ARM64 host (no robot connection)', flush=True)
    # rsync intentionally keeps unrelated files; retire only known removed implementations.
    obsolete = ['src/gento_dual_arm',
                'build_real/gento_dual_arm',
                'install_real/gento_dual_arm',
                'build/gento_dual_arm',
                'install/gento_dual_arm',
                'src/gento_driver/gento_driver/arm_control.py',
                'src/gento_driver/gento_driver/dual_arm_control.py',
                'src/gento_driver/gento_driver/cartesian.py',
                'src/gento_driver/gento_driver/trajectory_diagnostics.py',
                'src/gento_driver/gento_driver/diagnostics.py',
                'install_real/gento_driver/lib/gento_driver/arm_control',
                'install_real/gento_driver/lib/gento_driver/dual_arm_control']
    cleanup = f'''import shutil, uuid
from pathlib import Path
workspace = Path({(remote_dir + '/ros2_ws')!r})
archive = workspace / 'artifacts' / ('retired-split-' + uuid.uuid4().hex[:8])
for relative in {obsolete!r}:
    source = workspace / relative
    if source.exists() or source.is_symlink():
        destination = archive / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(source), str(destination))
'''
    remote_script("python3 - <<'RETIRE_REMOVED_MODULES'\n" + cleanup + 'RETIRE_REMOVED_MODULES\n')
    remote_script(f'''source {shlex.quote(setup)}
cd {shlex.quote(remote_dir + '/ros2_ws')}
colcon --log-base log_real build --base-paths src --packages-up-to gento_driver --build-base build_real --install-base install_real --symlink-install
source install_real/setup.bash
ros2 launch gento_driver dual_arm.launch.py --show-args
''')
print(('Dry run finished. ' if args.dry_run else 'Deployment complete. ') + remote + ':' + remote_dir)
print('No controller connection, error reset, mode switch, or motion was requested.')
PY

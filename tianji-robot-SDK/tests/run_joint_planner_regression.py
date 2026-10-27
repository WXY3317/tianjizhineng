"""Run offline after: bash ros2_ws/build_gento_sdk.sh"""
from pathlib import Path
import subprocess
import tempfile

sdk = Path(__file__).resolve().parents[1]
includes = sorted({p.parent for p in (sdk / 'C_SDK').rglob('*.h')})
with tempfile.TemporaryDirectory() as tmp:
    binary = str(Path(tmp) / 'joint_planner_regression')
    subprocess.run([
        'g++', '-std=c++11', '-DCMPL_LIN',
        str(sdk / 'tests/joint_planner_regression.cpp'),
        *[f'-I{p}' for p in includes],
        f'-L{sdk / "PYTHON_SDK"}', f'-Wl,-rpath,{sdk / "PYTHON_SDK"}',
        '-lGentoSDKPY', '-o', binary,
    ], check=True)
    subprocess.run([binary], check=True)

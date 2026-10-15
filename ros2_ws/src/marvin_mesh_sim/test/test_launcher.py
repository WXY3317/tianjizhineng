"""Starting the launcher twice must preserve the backend, not flash/exit."""
import os
from pathlib import Path
import signal
import subprocess
import time


WORKSPACE=Path(__file__).resolve().parents[3]


def test_repeated_headless_launcher_reuses_backend(tmp_path):
    env=dict(os.environ,ROS_DOMAIN_ID='97',XDG_RUNTIME_DIR=str(tmp_path))
    command=[str(WORKSPACE/'run_marvin_mesh_sim.sh'),'rviz:=false','panel:=false']
    log_path=tmp_path/'first.log'
    with log_path.open('w') as log:
        first=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+12
            while time.monotonic()<deadline:
                assert first.poll() is None,log_path.read_text()
                if 'Original libMarvinSDK.so connected' in log_path.read_text():break
                time.sleep(.05)
            else:raise AssertionError(log_path.read_text())
            second=subprocess.run(command,env=env,capture_output=True,text=True,timeout=8)
            output=second.stdout+second.stderr
            assert second.returncode==0,output
            assert '复用已运行的仿真后台' in output,output
            assert 'Address already in use' not in output,output
            assert 'process started with pid' not in output,output
            assert first.poll() is None
        finally:
            if first.poll() is None:
                os.killpg(first.pid,signal.SIGINT)
                try:first.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(first.pid,signal.SIGTERM);first.wait(timeout=5)

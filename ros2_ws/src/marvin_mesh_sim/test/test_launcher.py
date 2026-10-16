"""Starting the launcher twice must preserve the backend, not flash/exit."""
import os
import re
import sys
from pathlib import Path
import signal
import subprocess
import time


WORKSPACE=Path(__file__).resolve().parents[3]


def test_repeated_headless_launcher_reuses_backend(tmp_path):
    env=dict(os.environ,ROS_DOMAIN_ID='97',XDG_RUNTIME_DIR=str(tmp_path))
    command=[str(WORKSPACE/'run_marvin_mesh_sim.sh'),'rviz:=false','panel:=false']
    log_path=tmp_path/'first.log'
    recovery=None
    with log_path.open('w') as log:
        first=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+12
            while time.monotonic()<deadline:
                assert first.poll() is None,log_path.read_text()
                if 'Original libGentoSDKPY.so connected' in log_path.read_text():break
                time.sleep(.05)
            else:raise AssertionError(log_path.read_text())
            second=subprocess.run(command,env=env,capture_output=True,text=True,timeout=12)
            output=second.stdout+second.stderr
            assert second.returncode==0,output
            assert '复用已运行的仿真后台' in output,output
            assert 'Address already in use' not in output,output
            assert 'process started with pid' not in output,output
            assert first.poll() is None
            # Reproduce the actual bug: SDK nodes remain, description publisher dies.
            match=re.search(r'robot_state_publisher-\d+.*process started with pid \[(\d+)\]',log_path.read_text())
            assert match,log_path.read_text()
            os.kill(int(match.group(1)),signal.SIGTERM)
            probe="""
import rclpy,time
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
rclpy.init();n=Node('model_recovery_probe');seen={}
n.create_subscription(String,'/robot_description',lambda m:seen.update(description=len(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(JointState,'/joint_states',lambda m:seen.update(joints=len(m.name)),10)
end=time.monotonic()+6
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.03)
n.destroy_node();rclpy.shutdown()
assert seen.get('description',0)>1000 and seen.get('joints')==44,seen
"""
            missing=subprocess.run([sys.executable,'-c',probe],env=env,capture_output=True,text=True,timeout=10)
            assert missing.returncode!=0,'Missing description publisher must reproduce invisible model'
            with (tmp_path/'recovery.log').open('w') as recovery_log:
                recovery=subprocess.Popen(command,env=env,stdout=recovery_log,stderr=subprocess.STDOUT,start_new_session=True)
                deadline=time.monotonic()+12
                while 'got segment world' not in (tmp_path/'recovery.log').read_text():
                    assert recovery.poll() is None,(tmp_path/'recovery.log').read_text()
                    assert time.monotonic()<deadline,(tmp_path/'recovery.log').read_text()
                    time.sleep(.05)
                restored=subprocess.run([sys.executable,'-c',probe],env=env,capture_output=True,text=True,timeout=10)
                assert restored.returncode==0,restored.stdout+restored.stderr
        finally:
            if recovery is not None and recovery.poll() is None:
                os.killpg(recovery.pid,signal.SIGINT);recovery.wait(timeout=8)
            if first.poll() is None:
                os.killpg(first.pid,signal.SIGINT)
                try:first.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(first.pid,signal.SIGTERM);first.wait(timeout=5)

# 命令行仿真示例：连接 SDK 后展示状态读取及运动控制流程，参数由命令行提供。
import argparse
import json
import time
import numpy as np

from .sdk import MeshSimulationSDK
from .hands import closed_posture
from marvin_sim.kinematics import pose_error


def main():
    p=argparse.ArgumentParser(description='Actual SDK arm FK/IK and hand simulation control')
    p.add_argument('--sdk-root',required=True);p.add_argument('--model-root');p.add_argument('--sim-ip',default='')
    args=p.parse_args()
    with MeshSimulationSDK(args.sdk_root,args.model_root,args.sim_ip) as sdk:
        goals={};errors=[]
        for arm in ('A','B'):
            q=sdk.get_joint_positions(arm);q[0]+=3 if arm=='A' else -3
            target=sdk.fk(arm,q);sol=sdk.ik(arm,target)
            sdk.move_joints(arm,sol);goals[arm]=sol
            errors.append(pose_error(sdk.fk(arm,sol),target))
        state=sdk.get_hand_state()
        for side in ('L','R'):sdk.set_hand_joints(side,closed_posture(state[side]['names']))
        end=time.monotonic()+10
        while time.monotonic()<end:
            arm_error=max(float(np.max(np.abs(np.array(sdk.get_joint_positions(a))-q))) for a,q in goals.items())
            hand=sdk.get_hand_state()
            if arm_error<1e-3 and all(not h['moving'] for h in hand.values()):break
            time.sleep(.03)
        else:raise RuntimeError('SDK simulation demo timed out')
        print(json.dumps({'result':'PASS','sdk_fk_ik_errors_m_rad':errors,
                          'arm_feedback_error_deg':arm_error,'hands_dof':sum(len(h['names']) for h in hand.values())},indent=2))

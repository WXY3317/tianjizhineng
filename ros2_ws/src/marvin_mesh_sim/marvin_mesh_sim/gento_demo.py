"""Direct original Gento SDK position-control example; angles in degrees."""
import argparse
from pathlib import Path
import time

from marvin_sim.local_address import local_address
from .gento_protocol import vendor


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk-root',required=True)
    parser.add_argument('--sim-ip',default='')
    args=parser.parse_args()
    module=vendor(Path(args.sdk_root))
    robot=module.GentoRobot(precision=7)
    connected=False
    try:
        address=local_address(args.sim_ip)
        result=robot.link(*map(int,address.split('.')),log_level=8)
        if result<0:raise RuntimeError(f'SDK link failed: {result}')
        connected=True
        for obj in (3,2):
            if robot.switch_to_position_mode(obj,1000,50,50)!=0:
                raise RuntimeError('SDK position-mode transition failed')
        rt=module.ROBOT_RT.from_buffer_copy(bytes(robot.rt))
        body=list(rt.m_BODY.m_BODY_OUT.m_BODY_FBK_Joint_Pos)
        head=list(rt.m_HEAD.m_HEAD_OUT.m_HEAD_FBK_Joint_Pos)
        body[1]+=2.;head[0]+=2.;head[2]=0.
        if robot.runtime_set_joint_pos_cmd(3,body)!=0:raise RuntimeError('Body command failed')
        if robot.runtime_set_joint_pos_cmd(2,head)!=0:raise RuntimeError('Head command failed')
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            time.sleep(.05)
            rt=module.ROBOT_RT.from_buffer_copy(bytes(robot.rt))
            fb_body=list(rt.m_BODY.m_BODY_OUT.m_BODY_FBK_Joint_Pos)
            fb_head=list(rt.m_HEAD.m_HEAD_OUT.m_HEAD_FBK_Joint_Pos)
            if max(abs(x-y) for x,y in zip(fb_body+fb_head,body+head))<1e-3:
                print(f'Original SDK feedback reached target (deg): body={fb_body}, head={fb_head}')
                return
        raise RuntimeError('SDK feedback did not reach target within 8 seconds')
    finally:
        if connected:
            for obj in (3,2):robot.switch_to_idle(obj,500)
            robot.unlink()

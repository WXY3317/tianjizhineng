"""One original Gento SDK connection controls both arms, body and head."""
import numpy as np

from .gento_client import GentoBodyClient


class GentoRobotClient(GentoBodyClient):
    def __init__(self,kine,model_root,sim_ip=''):
        self.kine=kine
        super().__init__(kine.sdk_root,model_root,sim_ip)
        try:
            if any(self.robot.get_ctrl_obj_dof(a)!=7 for a in (0,1)):
                raise RuntimeError('Gento simulator requires two seven-joint arms')
            self.enable()
        except BaseException:self.close();raise

    def read(self):
        GentoBodyClient.read(self)
        rt=self.module.ROBOT_RT.from_buffer_copy(bytes(self.robot.rt))
        self.feedback=rt
        return np.radians([float(v) for arm in rt.m_ARMS for v in arm.m_ARM_OUT.m_ARM_FBK_Joint_Pos])

    def body_read(self):return GentoBodyClient.read(self)
    def move_body(self,names,positions):return GentoBodyClient.move(self,names,positions)

    def move(self,goals):
        validated={a:self.kine.validate_joints(a,q) for a,q in goals.items()}
        self.read()
        for arm in validated:
            if self.feedback.m_ARMS[arm].m_ARM_State.m_CurState!=1:
                raise ValueError('Gento arm is not in position mode')
        for arm,q in validated.items():
            self.check(self.robot.runtime_set_joint_pos_cmd(arm,np.degrees(q).tolist()),'arm joint command')

    def enable(self):
        for arm in (0,1):self.check(self.robot.switch_to_position_mode(arm,1000,25,25),'arm position mode')

    def stop(self):
        for arm in (0,1):self.check(self.robot.switch_to_idle(arm,500),'arm idle mode')

    def close(self):
        if self.connected:
            try:
                for obj in (0,1,2,3):self.check(self.robot.switch_to_idle(obj,500),'idle mode')
            except (RuntimeError,KeyboardInterrupt):pass
            finally:self.robot.unlink();self.connected=False

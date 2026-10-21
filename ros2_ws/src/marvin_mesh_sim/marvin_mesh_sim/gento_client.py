# 封装原版 Gento SDK 的躯干和头部模式切换、反馈读取与连接生命周期。
# 仿真地址必须是本机已分配的 IPv4；原厂 SDK 对外使用的角度需在模型边界转换为弧度。
"""Body/head commands and feedback through unmodified GentoRobot + native SO."""
import time

import numpy as np

from marvin_sim.local_address import local_address
from .body import Body
from .gento_protocol import vendor


class GentoBodyClient:
    def __init__(self,sdk_root,model_root,sim_ip=''):
        self.module=vendor(sdk_root)
        self.robot=self.module.GentoRobot(precision=7)
        self.model=Body(model_root)
        # 连接目标由本机网卡选择器约束，不将仿真命令发送给任意外部 IP。
        self.address=local_address(sim_ip)
        self.connected=False
        self.serial=None;self.last_refresh=time.monotonic()
        try:
            result=self.robot.link(*map(int,self.address.split('.')),log_level=8)
            if result<0:raise RuntimeError(f'Original Gento SDK connection failed: {result}')
            self.connected=True
            if self.robot.get_ctrl_obj_dof(3)!=6 or self.robot.get_ctrl_obj_dof(2)!=2:
                raise RuntimeError('Gento/model mapping requires 6 body and 2 head joints')
            for obj in (3,2):
                self.check(self.robot.switch_to_position_mode(obj,1000,50,50),'position mode')
            self.read()
        except BaseException:
            self.close();raise

    @staticmethod
    def check(result,operation):
        if result!=0:raise RuntimeError(f'Original Gento SDK {operation} failed: {result}')

    def read(self):
        rt=self.module.ROBOT_RT.from_buffer_copy(bytes(self.robot.rt))
        if rt.m_RT_FrameSerial!=self.serial:
            self.serial=rt.m_RT_FrameSerial;self.last_refresh=time.monotonic()
        if not self.serial or time.monotonic()-self.last_refresh>.5:
            raise RuntimeError('Gento SDK feedback stale for more than 0.5 seconds')
        body=list(rt.m_BODY.m_BODY_OUT.m_BODY_FBK_Joint_Pos)
        head=list(rt.m_HEAD.m_HEAD_OUT.m_HEAD_FBK_Joint_Pos)[:2]
        self.q=np.radians(body+head)
        self.model.q=self.q.copy()
        self.model.target=np.radians(list(rt.m_BODY.m_BODY_IN.m_BODY_CMD_Joint_Pos)
                                     +list(rt.m_HEAD.m_HEAD_IN.m_HEAD_CMD_Joint_Pos)[:2])
        return self.q.copy()

    def move(self,names,positions):
        GentoBodyClient.read(self)
        self.model.set_target(names,positions)  # complete validation before SDK writes
        desired=np.degrees(self.model.target).tolist()
        if any(name in self.model.names[:6] for name in names):
            self.check(self.robot.runtime_set_joint_pos_cmd(3,desired[:6]),'body position')
        if any(name in self.model.names[6:] for name in names):
            self.check(self.robot.runtime_set_joint_pos_cmd(2,desired[6:]+[0.]),'head position')

    def stop(self):
        for obj in (3,2):self.check(self.robot.switch_to_idle(obj,500),'idle mode')

    def close(self):
        if self.connected:
            try:self.stop()
            except RuntimeError:pass
            finally:self.robot.unlink();self.connected=False

"""SDK client restricted to this host. Motion uses the original library."""
import ctypes
import time

import numpy as np
from .local_address import local_address


class SDKClient:
    def __init__(self, kine, timeout=3., sim_ip=''):
        self.kine = kine
        self.robot = kine.robot_module.Marvin_Robot()
        self.feedback = kine.robot_module.DCSS()
        self.connected = False
        self.address = local_address(sim_ip)
        # Complete the prototypes used here; do not change vendor files.
        lib = self.robot.robot
        for name, args in {
            'OnLinkTo': [ctypes.c_ubyte]*4,
            'OnRelease': [], 'OnClearSet': [], 'OnSetSend': [],
            'OnGetBuf': [ctypes.POINTER(type(self.feedback))],
            'OnSetTargetState_A': [ctypes.c_int], 'OnSetTargetState_B': [ctypes.c_int],
            'OnSetJointLmt_A': [ctypes.c_int]*2, 'OnSetJointLmt_B': [ctypes.c_int]*2,
            'OnSetJointCmdPos_A': [ctypes.POINTER(ctypes.c_double)],
            'OnSetJointCmdPos_B': [ctypes.POINTER(ctypes.c_double)],
        }.items():
            fn = getattr(lib, name)
            fn.argtypes, fn.restype = args, ctypes.c_bool
        try:
            if not self.robot.connect(self.address):
                raise RuntimeError('SDK cannot bind port 4730; another SDK client may be running')
            self.connected = True
            deadline = time.monotonic()+timeout
            previous = None
            while time.monotonic() < deadline:
                self.read()
                serial = self.feedback.m_Out[0].m_OutFrameSerial
                if serial and previous is not None and serial != previous:
                    break
                previous = serial
                time.sleep(.01)
            else:
                raise RuntimeError(f'No refreshing feedback from virtual controller on {self.address}:4729')
            self._batch(lambda: [self.robot.set_vel_acc(a, 10, 10) for a in ('A', 'B')])
            self._batch(lambda: [self.robot.set_state(a, 1) for a in ('A', 'B')])
            self.read()
            if any(s.m_CurState != 1 for s in self.feedback.m_State):
                raise RuntimeError('Virtual controller did not enter position mode')
        except BaseException:
            self.close()
            raise

    def _batch(self, operation):
        if not self.robot.clear_set():
            raise RuntimeError('SDK command buffer busy')
        if not all(operation()):
            raise RuntimeError('SDK rejected command construction')
        if self.robot.send_cmd_wait_response(200) <= 0:
            raise RuntimeError('SDK command acknowledgment timeout')

    def read(self):
        if not self.robot.robot.OnGetBuf(ctypes.byref(self.feedback)):
            raise RuntimeError('SDK feedback unavailable')
        return np.radians([float(v) for arm in self.feedback.m_Out for v in arm.m_FB_Joint_Pos])

    def move(self, goals):
        checked = {arm: self.kine.validate_joints(arm, q) for arm, q in goals.items()}
        self._batch(lambda: [self.robot.set_joint_cmd_pose('AB'[arm], np.degrees(q).tolist())
                             for arm, q in checked.items()])

    def stop(self):
        self._batch(lambda: [self.robot.set_state(a, 0) for a in ('A', 'B')])

    def enable(self):
        self._batch(lambda: [self.robot.set_state(a, 1) for a in ('A', 'B')])

    def close(self):
        if self.connected:
            try:
                self.stop()
            except RuntimeError:
                pass
            finally:
                self.robot.release_robot()
                self.connected = False

# 用 ctypes 加载自研 C ABI 适配层，将 Python 调用转交原版 SG100 C++ SDK。
"""C ABI adapter to the original, unmodified SG100 C++ SDK."""
import ctypes as C
from pathlib import Path

import numpy as np


class SG100Device:
    def __init__(self,sdk_root,side,bridge_library=None):
        if side not in ('L','R'):raise ValueError('SG100 hand must be L/R')
        root=Path(sdk_root).resolve()
        if bridge_library is None:
            source=Path(__file__).resolve().parents[1]/'native/lib/libsg100_sim_bridge.so'
            if source.exists():bridge_library=source
            else:
                from ament_index_python.packages import get_package_share_directory
                bridge_library=Path(get_package_share_directory('marvin_mesh_sim'))/'native/lib/libsg100_sim_bridge.so'
        if not Path(bridge_library).exists():
            raise RuntimeError('Build the SG100 adapter with ros2_ws/build_sg100_sdk.sh')
        self.lib=C.CDLL(str(bridge_library))
        specs={'sg_open':([C.c_char_p,C.c_int],C.c_void_p),
               'sg_error':([],C.c_char_p),'sg_close':([C.c_void_p],None),
               'sg_set':([C.c_void_p,C.POINTER(C.c_float)],C.c_int),
               'sg_status':([C.c_void_p],C.c_int),
               'sg_set_one':([C.c_void_p,C.c_int,C.c_float],C.c_int),
               'sg_tx':([C.c_void_p,C.POINTER(C.c_uint32),C.POINTER(C.c_ubyte)],C.c_int),
               'sg_rx':([C.c_void_p,C.c_uint32,C.POINTER(C.c_ubyte),C.c_int],C.c_int),
               'sg_positions':([C.c_void_p,C.POINTER(C.c_float)],None),
               'sg_updates':([C.c_void_p],C.c_ulonglong)}
        for name,(args,result) in specs.items():
            f=getattr(self.lib,name);f.argtypes=args;f.restype=result
        self.handle=self.lib.sg_open(str(root).encode(),int(side=='L'))
        if not self.handle:raise RuntimeError(self.lib.sg_error().decode())

    def SetJointPositions(self,positions_deg):
        q=np.asarray(positions_deg,dtype=float)
        if q.shape!=(11,) or not np.isfinite(q).all():raise ValueError('Expected 11 finite SG100 angles in degrees')
        if not self.lib.sg_set(self.handle,(C.c_float*11)(*q)):
            raise RuntimeError('Original SG100 SDK rejected SetJointPositions')

    def SetJointPosition(self,joint_id,position_deg):
        if not 1<=joint_id<=11 or not np.isfinite(position_deg):raise ValueError('Invalid SG100 joint ID/angle')
        if not self.lib.sg_set_one(self.handle,joint_id,position_deg):
            raise RuntimeError('Original SG100 SDK rejected SetJointPosition')

    def RequestHandStatus(self):
        if not self.lib.sg_status(self.handle):raise RuntimeError('Original SG100 status request failed')

    def CANFDGetTxFrames(self):
        data=(C.c_ubyte*64)();can_id=C.c_uint32()
        while True:
            size=self.lib.sg_tx(self.handle,C.byref(can_id),data)
            if not size:return
            yield can_id.value,bytes(data[:size])

    def CANFDProcessRxFrame(self,can_id,data):
        if not 1<=len(data)<=64:raise ValueError('Invalid CANFD fragment length')
        if not self.lib.sg_rx(self.handle,can_id,(C.c_ubyte*len(data)).from_buffer_copy(data),len(data)):
            raise RuntimeError('Original SG100 SDK rejected CANFD feedback')

    def GetCachedPositions(self):
        q=(C.c_float*11)();self.lib.sg_positions(self.handle,q)
        return np.array(q,dtype=float)

    @property
    def updates(self):return self.lib.sg_updates(self.handle)

    def close(self):
        if self.handle:self.lib.sg_close(self.handle);self.handle=None

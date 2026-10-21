# 通过原版 SG100 SDK 打包命令，将 CANFD 帧封装进本机 UDP，接收反馈后交回原版 SDK 解码。
"""Original SG100 SDK client using local UDP to carry its CANFD fragments."""
import json
from pathlib import Path
import socket
import struct
import time

import numpy as np
import yaml

from marvin_sim.local_address import local_address
from .sg100_native import SG100Device


class SG100Client:
    def __init__(self,sdk_root,model_root,sim_ip='',port=4742):
        self.names={};self.limits={};self.devices={}
        self.address=local_address(sim_ip);self.port=port
        self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.socket.bind((self.address,0));self.socket.setblocking(False)
        self.serial={s:0 for s in 'LR'};self.refreshed={s:0. for s in 'LR'}
        self.last_poll=0.
        config=yaml.safe_load((Path(model_root)/'config/hands.yaml').read_text())
        mapping=json.loads((Path(sdk_root)/'joint_angle_map_v3.0.json').read_text())['joint_angle_map']
        try:
            for side in 'LR':
                self.names[side]=[j['name'] for j in config[side]['joints']]
                model=np.array([j['range'] for j in config[side]['joints']])
                sdk=np.radians([[j['min_angle'],j['max_angle']] for j in mapping[side]])
                # 仅接受模型限位与原厂角度映射限位的交集，避免两个控制约定不一致。
                self.limits[side]=np.column_stack((np.maximum(model[:,0],sdk[:,0]),np.minimum(model[:,1],sdk[:,1])))
                self.devices[side]=SG100Device(sdk_root,side)
        except BaseException:self.close();raise

    # 从原版 SDK 取出待发送 CANFD 帧；外加本机 UDP 信封，不改变原厂帧载荷。
    def flush(self):
        for side,device in self.devices.items():
            for can_id,data in device.CANFDGetTxFrames():
                self.socket.sendto(side.encode()+struct.pack('<I',can_id)+data,(self.address,self.port))

    def move(self,side,joints_rad):
        if side not in self.devices:raise ValueError('SG100 hand must be L/R')
        q=np.asarray(joints_rad,dtype=float);bounds=self.limits[side]
        if q.shape!=(11,) or not np.isfinite(q).all():raise ValueError('Expected 11 finite hand angles')
        if np.any(q<bounds[:,0]) or np.any(q>bounds[:,1]):raise ValueError('Hand target exceeds SG100/model limits')
        # 模型用弧度，原厂 SG100 位置 API 用度，在此统一转换。
        self.devices[side].SetJointPositions(np.degrees(q))
        self.flush()

    def pump(self):
        now=time.monotonic()
        # 约每 20 毫秒请求一次状态，非阻塞接收并交给原厂库解析；有效更新才刷新反馈时间。
        if now-self.last_poll>=.02:
            for device in self.devices.values():device.RequestHandStatus()
            self.last_poll=now;self.flush()
        for _ in range(400):
            try:packet,sender=self.socket.recvfrom(128)
            except BlockingIOError:break
            if sender!=(self.address,self.port) or not 6<=len(packet)<=69:continue
            side=packet[:1].decode('ascii',errors='replace')
            if side not in self.devices:continue
            device=self.devices[side]
            device.CANFDProcessRxFrame(struct.unpack('<I',packet[1:5])[0],packet[5:])
            if device.updates!=self.serial[side]:
                self.serial[side]=device.updates;self.refreshed[side]=now

    def read(self):
        self.pump()
        # 拒绝使用超过 0.5 秒没有更新的缓存反馈，避免把断开的仿真状态当作当前状态。
        if any(time.monotonic()-t>.5 for t in self.refreshed.values()):
            raise RuntimeError('Original SG100 SDK feedback missing/stale (>0.5 seconds)')
        return {s:np.radians(d.GetCachedPositions()) for s,d in self.devices.items()}

    def close(self):
        for device in self.devices.values():device.close()
        self.devices.clear();self.socket.close()

"""Hand model driven exclusively by decoded original SG100 SDK packets."""
import json
from pathlib import Path
import socket
import struct

from .hands import Hands
from .sg100_client import SG100Client
from .sg100_protocol import Palm,Stream


class SG100Hands(Hands):
    def __init__(self,model_root,address,sdk_root,port=4741,wire_port=4742,**kwargs):
        super().__init__(model_root,address,port=port,**kwargs)
        self.wire=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.compat=None
        try:
            self.wire.bind((address,wire_port));self.wire.setblocking(False)
            self.wire_port=self.wire.getsockname()[1]
            self.duration={s:1. for s in 'LR'}
            mapping=json.loads((Path(sdk_root)/'joint_angle_map_v3.0.json').read_text())['joint_angle_map']
            self.palms={s:Palm(self,s,mapping[s]) for s in 'LR'}
            self.streams={}
            # Existing UDP API also serializes via the vendor library.
            self.compat=SG100Client(sdk_root,model_root,address,port=self.wire_port)
        except BaseException:self.close();raise

    def set_target(self,side,joints,duration=1.):
        import numpy as np
        duration=float(duration)
        if not np.isfinite(duration) or not .05<=duration<=30.:raise ValueError('Invalid hand duration')
        self.compat.move(side,joints)
        self.duration[side]=duration

    def stop(self):
        for side in 'LR':self.compat.move(side,self.q[side])

    def tick(self):
        super().tick()
        self.compat.pump()
        for _ in range(400):
            try:packet,sender=self.wire.recvfrom(128)
            except BlockingIOError:break
            if sender[0]!=self.address or not 6<=len(packet)<=69:continue
            side=packet[:1].decode('ascii',errors='replace')
            if side not in self.palms:continue
            palm=self.palms[side]
            try:
                key=(sender,side)
                if key not in self.streams:
                    if len(self.streams)>=64:
                        oldest=min(self.streams,key=lambda k:self.streams[k].last)
                        del self.streams[oldest]
                    self.streams[key]=Stream()
                for command in self.streams[key].feed(packet[5:]):
                    response=palm.handle(command)
                    for index,offset in enumerate(range(0,len(response),64)):
                        self.wire.sendto(side.encode()+struct.pack('<I',0x200+index)+response[offset:offset+64],sender)
            except (ValueError,TypeError,struct.error) as exc:
                palm.rejected+=1;palm.last_error=str(exc)

    def close(self):
        if self.compat:self.compat.close()
        self.wire.close();super().close()

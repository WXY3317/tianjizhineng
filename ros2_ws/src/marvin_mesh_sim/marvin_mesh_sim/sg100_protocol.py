"""Virtual SG100 palm position-control subset. CANFD fragments travel over UDP.

UDP envelope: side ASCII L/R + uint32 little-endian CAN ID + 1..64 payload
bytes. Vendor CANFD framing and CRC are preserved inside this local transport.
"""
import struct
import time

import numpy as np


def crc32(data):
    value=0xffffffff
    for byte in data:
        value^=byte<<24
        for _ in range(8):
            value=((value<<1)^0x04c11db7 if value&0x80000000 else value<<1)&0xffffffff
    return value^0xffffffff


def frame(cmd,data=b'',dst=0xfe,src=0xfd):
    body=b'HM'+bytes([len(data),dst,src,cmd])+data
    return body+struct.pack('<I',crc32(body))


class Stream:
    def __init__(self):self.buffer=bytearray();self.last=time.monotonic()
    def feed(self,data):
        now=time.monotonic()
        if now-self.last>.5:self.buffer.clear()
        self.last=now;self.buffer.extend(data)
        if len(self.buffer)>2048:self.buffer.clear();raise ValueError('SG100 stream overflow')
        packets=[]
        while len(self.buffer)>=2:
            offset=self.buffer.find(b'HM')
            if offset<0:
                self.buffer[:]=self.buffer[-1:] if self.buffer[-1:]==b'H' else b''
                break
            del self.buffer[:offset]
            if len(self.buffer)<6:break
            size=self.buffer[2]+10
            if len(self.buffer)<size:break
            packet=bytes(self.buffer[:size]);del self.buffer[:size]
            if crc32(packet[:-4])!=struct.unpack('<I',packet[-4:])[0]:
                raise ValueError('SG100 CRC-32/BZIP2 mismatch')
            packets.append(packet)
        return packets


class Palm:
    def __init__(self,hands,side,mapping):
        self.hands,self.side=hands,side
        self.a=np.array([j['a'] for j in mapping])
        self.b=np.array([j['b'] for j in mapping])
        self.sdk_limits=np.radians([[j['min_angle'],j['max_angle']] for j in mapping])
        self.received=0;self.rejected=0;self.last_error=''

    def handle(self,packet):
        if packet[3:5]!=b'\xfd\xfe':raise ValueError('SG100 target must be palm FD from PC FE')
        command=packet[5];data=packet[6:-4]
        if command==0x05:
            if len(data)!=176:raise ValueError('SG100 position frame requires 11 x 16 bytes')
            target=np.zeros(11);seen=set()
            for offset in range(0,176,16):
                joint_id,mode,raw=struct.unpack_from('<BBf',data,offset)
                if not 1<=joint_id<=11 or joint_id in seen:raise ValueError('Invalid/duplicate SG100 joint ID')
                if mode!=1:raise ValueError('Only SG100 MODE_POSITION is simulated')
                seen.add(joint_id);i=joint_id-1
                # SDK encodes raw = radians(a * user_degrees + b).
                target[i]=(raw-np.radians(self.b[i]))/self.a[i]
            if not np.isfinite(target).all():raise ValueError('Nonfinite SG100 target')
            if np.any(target<self.sdk_limits[:,0]-1e-6) or np.any(target>self.sdk_limits[:,1]+1e-6):
                raise ValueError('SG100 target exceeds SDK angle limits')
            # float32 wire rounding at an exact model boundary is harmless.
            limits=self.hands.limits[self.side]
            if np.any(target<limits[:,0]-1e-6) or np.any(target>limits[:,1]+1e-6):
                raise ValueError('SG100 target exceeds MJCF model limits')
            target=np.clip(target,limits[:,0],limits[:,1])
            if not np.allclose(target,self.hands.target[self.side],atol=1e-8,rtol=0):
                # Apply only decoded SDK commands, never the pre-serialization input.
                from .hands import Hands
                Hands.set_target(self.hands,self.side,target,self.hands.duration[self.side])
        elif command==0x26 and not data:pass
        else:raise ValueError(f'Unsupported SG100 simulation command {command:#x}')
        self.received+=1
        return self.feedback(command)

    def feedback(self,command=0x26):
        raw=self.a*self.hands.q[self.side]+np.radians(self.b)
        data=b''.join(struct.pack('<BBfHHH',i+1,1,float(value),32767,32767,32767)
                      for i,value in enumerate(raw))
        return frame(command,data)

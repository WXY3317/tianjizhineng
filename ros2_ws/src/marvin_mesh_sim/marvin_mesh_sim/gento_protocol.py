"""Local Gento 4.4.2 protocol subset, exercised by the unmodified vendor SDK.

SDK binds wildcard receive ports equal to its destination ports. The server
binds a specific assigned host IP with SO_REUSEADDR; replies go to loopback,
which reaches the SDK wildcard sockets without looping back into this server.
Position-mode ARM0/ARM1/BODY/HEAD commands share this one Gento connection.
"""
import importlib
from pathlib import Path
import socket
import struct
import sys

import numpy as np


def vendor(sdk_root):
    root=Path(sdk_root).resolve()
    sys.path.insert(0,str(root))
    module=importlib.import_module('PYTHON_SDK.GentoRobot')
    if Path(module.__file__).resolve().parents[1]!=root:
        raise ValueError('A different Gento SDK is already imported')
    return module


def decode(packet):
    if (len(packet)<6 or packet[:2]!=b'Fa'
            or int.from_bytes(packet[2:4],'little')!=len(packet)-6
            or sum(packet[5:])%256!=packet[4]):
        raise ValueError('Invalid Gento packet header/length/checksum')
    offset=6;commands=[]
    for _ in range(packet[5]):
        if offset+4>len(packet):raise ValueError('Truncated Gento command')
        size=int.from_bytes(packet[offset+1:offset+3],'little')
        end=offset+size+4
        if end>len(packet) or sum(packet[offset:end])%256:
            raise ValueError('Invalid Gento command length/checksum')
        commands.append((packet[offset],packet[offset+3:end-1]));offset=end
    if offset!=len(packet):raise ValueError('Unexpected Gento suffix')
    return commands


class GentoController:
    def __init__(self,body,sdk_root,address,arms=None):
        self.body,self.address=body,address
        self.arms=arms
        self.module=vendor(sdk_root)
        self.rt,self.sg=self.module.ROBOT_RT(),self.module.ROBOT_SG()
        self.sockets={};self.connected=False
        self.modes={60:0,90:0};self.ratios={62:10.,63:10.,93:10.,94:10.}
        self.received,self.rejected=0,0;self.last_error=''
        try:
            for port in (3720,3721,3722):
                sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
                self.sockets[port]=sock
                sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
                sock.bind((address,port));sock.setblocking(False)
        except BaseException:
            self.close();raise

    def close(self):
        for sock in self.sockets.values():sock.close()

    def operation(self,payload):
        if len(payload)!=44:raise ValueError('Invalid Gento operation size')
        op=self.module.OP_SET.from_buffer_copy(payload)
        name=bytes(op.m_OpValueS).split(b'\0',1)[0].decode('ascii')
        success=True
        if op.m_OpIns==12:op.m_OpValueI=0x040402
        elif op.m_OpIns==13:op.m_OpValueI=int(op.m_OpValueI>>8==0x0404)
        elif op.m_OpIns==5 and name=='R.BASIC.Name':
            for i,value in enumerate(b'GentoLuna\0'):op.m_OpValueS[i]=value
        elif op.m_OpIns==3:
            values={'R.A0.BASIC.Dof':7,'R.A1.BASIC.Dof':7,'R.B.BASIC.Dof':6,
                    'R.H.BASIC.Dof':2,'R.L.BASIC.Dof':0}
            if name not in values:success=False
            else:op.m_OpValueI=values[name]
        else:success=False
        op.m_OpRetSerial=op.m_OpCmdSerial+(0 if success else 100)
        return op

    def handle(self,packet):
        commands=decode(packet)
        modes,ratios=self.modes.copy(),self.ratios.copy()
        goals={};operation=None;ack=self.rt.wait_serial
        arm_modes=self.arms.modes.copy() if self.arms is not None else []
        arm_ratios=[r.copy() for r in self.arms.ratios] if self.arms is not None else []
        arm_goals={}
        for code,data in commands:
            if self.arms is not None and code in (0,30,1,31,5,6,35,36,200,201):
                arm=1 if code in (30,31,35,36,201) else 0
                if code in (0,30) and len(data)==2:
                    mode,=struct.unpack('<h',data)
                    if mode not in (0,1):raise ValueError('Only Gento arm idle/position modes are simulated')
                    arm_modes[arm]=mode
                elif code in (1,31) and len(data)==28:
                    q=np.radians(struct.unpack('<7f',data))
                    section=slice(arm*7,arm*7+7)
                    lower=self.arms.kine.lower[section];upper=self.arms.kine.upper[section]
                    if (not np.isfinite(q).all() or np.any(q<lower-1e-6) or np.any(q>upper+1e-6)):
                        raise ValueError('Gento arm command exceeds model limits')
                    arm_goals[arm]=self.arms.kine.validate_joints(arm,np.clip(q,lower,upper))
                    if arm_modes[arm]!=1:raise ValueError('Gento arm requires position mode')
                elif code in (5,6,35,36) and len(data)==4:
                    ratio,=struct.unpack('<f',data)
                    if not np.isfinite(ratio) or not 1<=ratio<=100:raise ValueError('Invalid Gento arm VA ratio')
                    arm_ratios[arm][0 if code in (5,35) else 1]=ratio
                elif code in (200,201) and not data:arm_modes[arm]=0
                else:raise ValueError('Invalid Gento arm instruction size')
            elif code==254:operation=self.operation(data)
            elif code==251 and len(data)==1:ack=data[0]
            elif code in (60,90) and len(data)==2:
                mode,=struct.unpack('<h',data)
                if mode not in (0,1):raise ValueError('Only Gento idle/position modes are simulated')
                modes[code]=mode
            elif code in ratios and len(data)==4:
                ratio,=struct.unpack('<f',data)
                if not np.isfinite(ratio) or not 1<=ratio<=100:raise ValueError('Invalid Gento VA ratio')
                ratios[code]=ratio
            elif code in (61,91):
                count=3 if code==61 else 6
                if len(data)!=4*count:raise ValueError('Invalid Gento joint command size')
                values=np.array(struct.unpack('<'+'f'*count,data))
                if not np.isfinite(values).all():raise ValueError('Nonfinite Gento joint command')
                if code==61 and abs(values[2])>1e-7:
                    raise ValueError('This model has two head joints; SDK channel 3 must be zero')
                for name,value in zip(self.body.names[6:] if code==61 else self.body.names[:6],values):
                    goals[name]=float(np.radians(value))
                if modes[60 if code==61 else 90]!=1:
                    raise ValueError('Gento motion requires position mode')
            elif code in (202,203) and not data:
                modes[60 if code==202 else 90]=0
            else:raise ValueError(f'Unsupported Gento instruction: {code}')
        for group,indices in ((60,range(6,8)),(90,range(6))):
            if modes[group]==0:
                for index in indices:goals[self.body.names[index]]=float(self.body.q[index])
        # Body validates the entire merged batch before state changes.
        if goals:
            speed=.5*min(ratios[62],ratios[93])/100
            acc=.5*min(ratios[63],ratios[94])/100
            self.body.set_target(list(goals),list(goals.values()),speed=speed,acceleration=acc)
        if self.arms is not None:
            self.arms.modes,self.arms.ratios=arm_modes,arm_ratios
            for arm in range(2):
                section=slice(arm*7,arm*7+7)
                if arm_modes[arm]==0:
                    self.arms.moves[arm]=None;self.arms.v[section]=0.
                    self.arms.target[section]=self.arms.q[section]
                elif arm in arm_goals:
                    self.arms.target[section]=arm_goals[arm]
                    self.arms._start(arm,arm_goals[arm])
            self.arms.received+=1
        self.modes,self.ratios=modes,ratios
        self.rt.wait_serial=ack
        if operation is not None:self.sg.m_OP_SET=operation
        self.received+=1

    def feedback(self):
        self.rt.m_RT_FrameSerial=self.rt.m_RT_FrameSerial%999999+1
        self.sg.m_RT_FrameSerial=self.rt.m_RT_FrameSerial
        for code,group in ((60,self.rt.m_HEAD),(90,self.rt.m_BODY)):
            state=group.m_HEAD_State if code==60 else group.m_BODY_State
            state.m_CurState=state.m_CmdState=self.modes[code]
        q=np.degrees(self.body.q);target=np.degrees(self.body.target)
        for i in range(6):
            self.rt.m_BODY.m_BODY_OUT.m_BODY_FBK_Joint_Pos[i]=q[i]
            self.rt.m_BODY.m_BODY_OUT.m_BODY_FBK_Joint_Vel[i]=np.degrees(self.body.velocity[i])
            self.rt.m_BODY.m_BODY_IN.m_BODY_CMD_Joint_Pos[i]=target[i]
        for i in range(2):
            self.rt.m_HEAD.m_HEAD_OUT.m_HEAD_FBK_Joint_Pos[i]=q[6+i]
            self.rt.m_HEAD.m_HEAD_IN.m_HEAD_CMD_Joint_Pos[i]=target[6+i]
        self.sg.m_BODY.m_BODY_SET.m_BODY_Ctrl_VelRatio=self.ratios[93]
        self.sg.m_BODY.m_BODY_SET.m_BODY_Ctrl_AccRatio=self.ratios[94]
        self.sg.m_HEAD.m_HEAD_SET.m_HEAD_Ctrl_VelRatio=self.ratios[62]
        self.sg.m_HEAD.m_HEAD_SET.m_HEAD_Ctrl_AccRatio=self.ratios[63]
        if self.arms is not None:
            for a in range(2):
                arm=self.rt.m_ARMS[a]
                arm.m_ARM_State.m_CurState=arm.m_ARM_State.m_CmdState=self.arms.modes[a]
                arm_sg=self.sg.m_ARMS[a]
                arm_sg.m_ARM_SET.m_ARM_Ctrl_VelRatio=self.arms.ratios[a][0]
                arm_sg.m_ARM_SET.m_ARM_Ctrl_AccRatio=self.arms.ratios[a][1]
                arm_sg.m_ARM_GET.m_ARM_FBK_LowSpdFlag=int(self.arms.moves[a] is None)
                for i in range(7):
                    arm.m_ARM_IN.m_ARM_CMD_Joint_Pos[i]=np.degrees(self.arms.target[7*a+i])
                    arm.m_ARM_OUT.m_ARM_FBK_Joint_Cmd[i]=np.degrees(self.arms.target[7*a+i])
                    arm.m_ARM_OUT.m_ARM_FBK_Joint_Vel[i]=np.degrees(self.arms.v[7*a+i])
                    arm.m_ARM_OUT.m_ARM_FBK_Joint_Pos[i]=np.degrees(self.arms.q[7*a+i])
        self.sockets[3721].sendto(b'FX'+bytes(self.rt),('127.0.0.1',3721))
        self.sockets[3722].sendto(b'FX'+bytes(self.sg),('127.0.0.1',3722))

    def tick(self):
        for port,sock in self.sockets.items():
            for _ in range(100):
                try:packet,sender=sock.recvfrom(2048)
                except BlockingIOError:break
                if sender[0]!=self.address:continue
                try:
                    if port==3720 and packet.startswith(b'FXASK') and len(packet)==6:
                        sock.sendto(b'FXACK'+packet[-1:]+b'\x01',('127.0.0.1',3720))
                        self.connected=True
                    elif packet==b'FX' or packet==b'FXBEAT':pass
                    elif port==3721:self.handle(packet)
                    else:raise ValueError('Unsupported Gento channel packet')
                except (ValueError,TypeError,struct.error) as exc:
                    self.rejected+=1;self.last_error=str(exc)
                    if self.arms is not None:
                        self.arms.rejected+=1;self.arms.last_error=str(exc)
        if self.connected:self.feedback()

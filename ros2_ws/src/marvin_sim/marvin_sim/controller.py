# 实现旧版 Marvin 虚拟控制器的 UDP 协议子集；固定接收端口要求避免多个 SDK 客户端争用。
"""100343 UDP protocol subset for an unmodified libMarvinSDK.so client.

Local host only, one SDK client (the SDK fixes its receive port at 4730).
Supported: parameter queries listed below, idle/position mode, VA ratios,
joint targets, response tags. Unsupported commands reject the whole packet.
This is a kinematic controller, not a firmware or dynamics emulator.
"""
import copy
import ctypes
import math
import socket
import struct
import time

import numpy as np

from .kinematics import INITIAL_DEG
from .local_address import local_address


def decode_packet(packet):
    if len(packet) < 7 or packet[:2] != b'FY':
        raise ValueError('Invalid SDK packet header')
    size = int.from_bytes(packet[3:5], 'little')
    if (size != len(packet)-7 or sum(packet[:6]) % 256
            or sum(packet[7:]) % 256 != packet[6]):
        raise ValueError('Invalid SDK packet length/checksum')
    commands, offset = [], 7
    for _ in range(packet[2]):
        if offset+5 > len(packet):
            raise ValueError('Truncated SDK command')
        head = packet[offset:offset+5]
        count = int.from_bytes(head[1:3], 'big')
        payload = packet[offset+5:offset+5+count]
        if (len(payload) != count or sum(head[:4]) % 256
                or (sum(payload)+head[4]) % 256):
            raise ValueError('Invalid SDK command checksum/length')
        commands.append((head[0], payload))
        offset += 5+count
    if offset != len(packet):
        raise ValueError('Unexpected SDK packet suffix')
    return commands


class VirtualController:
    def __init__(self, kine, port=4729, feedback_port=4730, now=time.monotonic, sim_ip=''):
        self.kine, self.now = kine, now
        self.feedback_port = feedback_port
        self.address = local_address(sim_ip)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind((self.address, port))
            self.socket.setblocking(False)
        except BaseException:
            self.socket.close()
            raise
        self.port = self.socket.getsockname()[1]
        self.q = np.radians(INITIAL_DEG).ravel()
        self.v = np.zeros(14)
        self.target = self.q.copy()
        self.modes = [0, 0]
        self.ratios = [[10, 10], [10, 10]]
        self.moves = [None, None]
        self.dcss = kine.robot_module.DCSS()
        self.serial, self.ack = 0, 0
        self.received, self.rejected = 0, 0
        self.last_error = ''
        self.last_client = None
        self.last_rx = 0.
        self.last_tick = now()
        self._feedback()

    def close(self):
        self.socket.close()

    def stop(self):
        self.moves = [None, None]
        self.target = self.q.copy()
        self.v[:] = 0

    def _start(self, arm, target):
        s = slice(7*arm, 7*arm+7)
        distance = float(np.max(np.abs(target-self.q[s])))
        if distance < 1e-10:
            self.moves[arm] = None
            self.v[s] = 0
            return
        delta = np.abs(target-self.q[s])
        vel = self.kine.velocity[s] * self.ratios[arm][0]/100.
        acc = self.kine.acceleration[s] * self.ratios[arm][1]/100.
        # Quintic smoothstep: maximum slope 1.875 and curvature < 5.774.
        duration = max(.05, float(np.max(1.875*delta/vel)),
                       float(np.max(np.sqrt(5.774*delta/acc))))
        self.moves[arm] = (self.now(), duration, self.q[s].copy(), target.copy())

    def _parameter(self, payload):
        if len(payload) != 42:
            raise ValueError('Invalid parameter command size')
        name, kind, operation, ivalue, fvalue, serial = struct.unpack('<30sBBifh', payload)
        name = name.split(b'\0', 1)[0].decode('ascii')
        values = {'VERSION': 100343014, 'R.A0.BASIC.Type': 1017, 'R.A1.BASIC.Type': 1017}
        if name.startswith(('SERVO0ERR', 'SERVO1ERR')) and name[-1:] in '0123456':
            values[name] = 0
        if operation == 103 and name in values:
            ivalue = values[name]
        elif operation == 101 and name in ('EMCY0', 'EMCY1', 'STOPRUN0', 'STOPRUN1'):
            pass
        else:
            raise ValueError(f'Unsupported simulator parameter: {name}/{operation}')
        return name, kind, operation, ivalue, fvalue, serial

    def handle_packet(self, packet):
        """Validate all commands before applying any part of a batch."""
        commands = decode_packet(packet)
        modes = self.modes.copy()
        ratios = copy.deepcopy(self.ratios)
        goals, parameters, ack = {}, [], self.ack
        for instruction, payload in commands:
            if instruction == 150:
                parameters.append(self._parameter(payload))
                continue
            if instruction == 251 and len(payload) == 1:
                ack = payload[0]
                continue
            arm = 0 if 100 < instruction < 150 else 1 if 200 < instruction < 250 else -1
            op = instruction - (100 if arm == 0 else 200)
            if arm < 0:
                raise ValueError(f'Unsupported SDK instruction: {instruction}')
            if op == 1 and len(payload) == 4:
                mode, = struct.unpack('<i', payload)
                if mode not in (0, 1):
                    raise ValueError('Simulator supports only idle(0) and position(1)')
                modes[arm] = mode
            elif op == 3 and len(payload) == 4:
                ratio = list(struct.unpack('<hh', payload))
                if any(not 1 <= v <= 100 for v in ratio):
                    raise ValueError('VA ratio must be 1..100')
                ratios[arm] = ratio
            elif op == 8 and len(payload) == 32:
                values = struct.unpack('<i7f', payload)
                goals[arm] = self.kine.validate_joints(arm, np.radians(values[1:]))
            else:
                raise ValueError(f'Unsupported SDK instruction or size: {instruction}')
        for arm in goals:
            if modes[arm] != 1:
                raise ValueError('Joint motion requires position mode')
        self.modes, self.ratios, self.ack = modes, ratios, ack
        for arm in range(2):
            s = slice(7*arm, 7*arm+7)
            if modes[arm] == 0:
                self.moves[arm] = None
                self.target[s] = self.q[s]
                self.v[s] = 0
            elif arm in goals:
                self.target[s] = goals[arm]
                self._start(arm, goals[arm])
        for name, kind, operation, ivalue, fvalue, serial in parameters:
            self.dcss.m_ParaName = name.encode()
            self.dcss.m_ParaType, self.dcss.m_ParaIns = kind, operation
            self.dcss.m_ParaValueI, self.dcss.m_ParaValueF = ivalue, fvalue
            self.dcss.m_ParaCmdSerial = self.dcss.m_ParaRetSerial = serial
            if operation == 101:
                arm = int(name[-1])
                s = slice(7*arm, 7*arm+7)
                self.moves[arm] = None
                self.target[s], self.v[s] = self.q[s], 0
        self.received += 1

    def _feedback(self):
        self.serial = self.serial % 999999 + 1
        for arm in range(2):
            s = slice(7*arm, 7*arm+7)
            state, incoming, out = self.dcss.m_State[arm], self.dcss.m_In[arm], self.dcss.m_Out[arm]
            state.m_CurState = state.m_CmdState = self.modes[arm]
            incoming.m_Joint_Vel_Ratio, incoming.m_Joint_Acc_Ratio = self.ratios[arm]
            out.m_OutFrameSerial = self.serial
            out.m_LowSpdFlag = bytes([int(self.moves[arm] is None)])
            out.m_pad = bytes([self.ack])
            for j in range(7):
                out.m_FB_Joint_Pos[j] = math.degrees(self.q[s][j])
                out.m_FB_Joint_PosE[j] = out.m_FB_Joint_Pos[j]
                out.m_FB_Joint_Cmd[j] = incoming.m_Joint_CMD_Pos[j] = math.degrees(self.target[s][j])
                out.m_FB_Joint_Vel[j] = math.degrees(self.v[s][j])
        return b'FX' + bytes(self.dcss)

    def tick(self):
        # Bound receive work so a sender cannot starve motion/feedback updates.
        for _ in range(100):
            try:
                packet, sender = self.socket.recvfrom(2048)
            except BlockingIOError:
                break
            try:
                if sender[0] != self.address:
                    raise ValueError('Only SDK clients on this host are accepted')
                self.handle_packet(packet)
                self.last_client = (sender[0], self.feedback_port)
                self.last_rx = self.now()
                self.last_error = ''
            except (ValueError, UnicodeError, struct.error) as exc:
                self.rejected += 1
                self.last_error = str(exc)
        current = self.now()
        for arm, move in enumerate(self.moves):
            if move is None:
                continue
            started, duration, start, goal = move
            u = min(1., max(0., (current-started)/duration))
            blend = 10*u**3-15*u**4+6*u**5
            slope = (30*u**2-60*u**3+30*u**4)/duration
            s = slice(7*arm, 7*arm+7)
            self.q[s] = start + (goal-start)*blend
            self.v[s] = (goal-start)*slope
            if u == 1.:
                self.moves[arm] = None
                self.v[s] = 0
        feedback = self._feedback()
        if self.last_client:
            self.socket.sendto(feedback, self.last_client)
        self.last_tick = current

"""Joint-level hand simulation. This API is separate from vendor arm firmware."""
import json
from pathlib import Path
import socket
import time

import numpy as np
import yaml


class Hands:
    def __init__(self, model_root, address, port=4741, now=time.monotonic):
        self.config = yaml.safe_load((Path(model_root)/'config/hands.yaml').read_text())
        self.names = {s:[j['name'] for j in c['joints']] for s,c in self.config.items()}
        self.limits = {s:np.array([j['range'] for j in c['joints']]) for s,c in self.config.items()}
        self.q = {s:np.zeros(len(n)) for s,n in self.names.items()}
        self.target = {s:q.copy() for s,q in self.q.items()}
        self.moves = {s:None for s in self.q}
        self.now, self.address = now, address
        self.socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        try:
            self.socket.bind((address,port))
            self.socket.setblocking(False)
        except BaseException:
            self.socket.close()
            raise
        self.port=self.socket.getsockname()[1]

    def close(self):
        self.socket.close()

    def set_target(self, side, joints, duration=1.):
        if side not in self.q:
            raise ValueError('Hand side must be L or R')
        q=np.asarray(joints,dtype=float)
        bounds=self.limits[side]
        if q.shape!=self.q[side].shape or not np.isfinite(q).all():
            raise ValueError('Expected 11 finite hand angles in radians')
        if np.any(q<bounds[:,0]) or np.any(q>bounds[:,1]):
            raise ValueError('Hand target exceeds source MJCF joint limits')
        duration=float(duration)
        if not np.isfinite(duration) or not .05<=duration<=30.:
            raise ValueError('Hand move duration must be 0.05..30 seconds')
        duration=max(duration,1.875*float(np.max(np.abs(q-self.q[side])))/2.)
        self.target[side]=q.copy()
        self.moves[side]=(self.now(),duration,self.q[side].copy(),q.copy())

    def state(self):
        return {s:{'names':self.names[s],'positions_rad':q.tolist(),
                   'moving':self.moves[s] is not None} for s,q in self.q.items()}

    def stop(self):
        self.moves={s:None for s in self.q}
        self.target={s:q.copy() for s,q in self.q.items()}

    def tick(self):
        for _ in range(50):
            try:
                raw,sender=self.socket.recvfrom(8192)
            except BlockingIOError:
                break
            request={}
            try:
                if sender[0]!=self.address:
                    continue
                request=json.loads(raw)
                if not isinstance(request,dict):
                    raise ValueError('Request must be a JSON object')
                op=request.get('op')
                if op=='set':
                    self.set_target(request.get('side'),request.get('joints_rad'),request.get('duration',1.))
                elif op=='stop':
                    self.stop()
                elif op!='state':
                    raise ValueError('Unknown hand operation')
                response={'ok':True,'state':self.state()}
            except (ValueError,TypeError,OverflowError) as exc:
                response={'ok':False,'error':str(exc)}
            response['id']=request.get('id') if isinstance(request,dict) else None
            self.socket.sendto(json.dumps(response).encode(),sender)
        for side,move in self.moves.items():
            if move is None:
                continue
            started,duration,start,goal=move
            u=min(1.,max(0.,(self.now()-started)/duration))
            self.q[side]=start+(goal-start)*(10*u**3-15*u**4+6*u**5)
            if u==1.:
                self.moves[side]=None


def closed_posture(names):
    # SDK ID order: thumb 1-3, index 4-6, middle 7-8, ring 9-11.
    values=[.4,-1.1,.5,-.2,1.1,.7,1.1,.7,.2,1.1,.7]
    return [values[int(n.split('_Joint')[1])-1] for n in names]

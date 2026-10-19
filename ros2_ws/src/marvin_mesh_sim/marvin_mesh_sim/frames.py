"""J4 palm-center TCP conversion; native SDK still solves shoulder-to-flange."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from marvin_sim.kinematics import validate_transform

TCP_FRAMES=('palm_center_L','palm_center_R')

J4_FRAME='Gento_Luna_Leg_J4_Link'


def origin(element):
    matrix=np.eye(4)
    if element is None:return matrix
    matrix[:3,3]=list(map(float,element.get('xyz','0 0 0').split()))
    r,p,y=map(float,element.get('rpy','0 0 0').split())
    cr,sr,cp,sp,cy,sy=np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
    matrix[:3,:3]=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                   [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
    return matrix


class J4Frames:
    def __init__(self,model_root):
        model=Path(model_root)
        root=ET.parse(model/'urdf/marvin_mesh_hands.urdf').getroot()
        self.joints={j.find('child').get('link'):j for j in root.findall('joint')}
        self.body_names=[j['name'] for j in yaml.safe_load((model/'config/body.yaml').read_text())['joints']]
        self.tools=[]
        for arm,side in enumerate('LR'):
            path=[];link=TCP_FRAMES[arm]
            while link!=f'flange_{side}':
                if link not in self.joints:raise ValueError(f'Missing palm TCP frame: {link}; rebuild model')
                joint=self.joints[link]
                if joint.get('type')!='fixed':raise ValueError('Palm TCP must be rigidly attached to flange')
                path.append(joint);link=joint.find('parent').get('link')
            tool=np.eye(4)
            for joint in reversed(path):tool=tool@origin(joint.find('origin'))
            self.tools.append(tool)
        self.paths=[]
        for side in 'LR':
            path=[];link=f'Base_{side}'
            while link!=J4_FRAME:
                if link not in self.joints:raise ValueError(f'{J4_FRAME} is not an ancestor of Base_{side}')
                j=self.joints[link];path.append(j);link=j.find('parent').get('link')
            self.paths.append(list(reversed(path)))

    @staticmethod
    def body_from_rt(rt,require_stationary=False):
        body=rt.m_BODY
        q=list(body.m_BODY_OUT.m_BODY_FBK_Joint_Pos)
        if require_stationary:
            target=list(body.m_BODY_IN.m_BODY_CMD_Joint_Pos)
            velocity=list(body.m_BODY_OUT.m_BODY_FBK_Joint_Vel)
            if any(abs(target[i]-q[i])>1e-4 or abs(velocity[i])>1e-3 for i in (4,5)):
                raise ValueError('J4 IK requires torso J5/J6 to finish moving first')
        return np.radians(q+list(rt.m_HEAD.m_HEAD_OUT.m_HEAD_FBK_Joint_Pos)[:2])

    def shoulder(self,arm,body_positions):
        if arm not in (0,1):raise ValueError('Arm must be 0/1')
        q=np.asarray(body_positions,dtype=float)
        if q.shape!=(8,) or not np.isfinite(q).all():raise ValueError('Expected current eight body/head angles')
        values=dict(zip(self.body_names,q));result=np.eye(4)
        for joint in self.paths[arm]:
            step=origin(joint.find('origin'))
            if joint.get('type')!='fixed':
                name=joint.get('name')
                if name not in values:raise ValueError(f'Missing current joint position: {name}')
                axis=np.array(list(map(float,joint.find('axis').get('xyz').split())))
                axis/=np.linalg.norm(axis);x,y,z=axis
                skew=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
                angle=values[name];rotation=np.eye(4)
                rotation[:3,:3]=np.eye(3)+np.sin(angle)*skew+(1-np.cos(angle))*(skew@skew)
                step=step@rotation
            result=result@step
        return result

    def fk(self,arm,shoulder_pose,body_positions):
        return self.shoulder(arm,body_positions)@validate_transform(shoulder_pose)@self.tools[arm]

    def ik_target(self,arm,j4_pose,body_positions):
        return np.linalg.inv(self.shoulder(arm,body_positions))@validate_transform(j4_pose)@np.linalg.inv(self.tools[arm])

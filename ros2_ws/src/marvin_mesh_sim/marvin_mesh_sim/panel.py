import json
import time
import numpy as np
from pathlib import Path
import tkinter as tk
from tkinter import ttk

import rclpy
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from marvin_sim.panel import Panel
from .gento_kinematics import GentoMeshKinematics
from .hands import closed_posture
from .frames import J4Frames,J4_FRAME


class MeshPanel(Panel):
    def __init__(self):
        self.j4_body=None;self.j4_refreshed=0.
        super().__init__(kine_factory=lambda root,model:GentoMeshKinematics(
            root if (Path(root)/'PYTHON_SDK/GentoRobot.py').exists() else Path(root).parent/'tianji-robot-SDK',model),
                         sdk_label='原版 Gento SDK · 双臂正逆解统一参考躯干 J4',
                         pose_frame=J4_FRAME,initialize_pose=False,
                         content_factory=self.create_tabs)
        self.root.title('整机仿真 · 双臂、灵巧手与本体控制')
        self.root.geometry('1120x850')
        model=Path(self.get_parameter('description_root').value)
        self.frames=J4Frames(model)
        import yaml
        config=yaml.safe_load((model/'config/hands.yaml').read_text())
        report=json.loads((model/'config/asset_report.json').read_text())
        if report['empty_mesh_count']:
            banner=(f"已读取 {report['mesh_count']} 个真实 STL；{report['empty_mesh_count']} 个空 STL "
                    '暂用原 MJCF 碰撞盒显示（橙色），控制关节保留。')
        else:
            banner=f"已读取全部 {report['mesh_count']} 个真实 STL：底盘、躯干、头部、双臂和双手，无占位外形。"
        ttk.Label(self.hand_tab,text=banner,wraplength=1000).pack(padx=15,pady=12)
        ttk.Label(self.hand_tab,text='SG100 原版 SDK 控制：拇指 J1–3、食指 J4–6、中指 J7–8、无名指 J9–11。').pack(pady=3)
        self.hand_pub=self.create_publisher(JointState,'/marvin_mesh_sim/hand_targets',10)
        self.create_subscription(JointState,'/marvin_mesh_sim/hand_states',self.hand_feedback,10)
        self.create_subscription(String,'/marvin_mesh_sim/hand_status',
                                 lambda m:self.hand_message.set(m.data),10)
        self.hand_message=tk.StringVar(value='SG100 目标单位：度；滑块不会自动发送运动')
        ttk.Label(self.hand_tab,textvariable=self.hand_message,wraplength=1000).pack(pady=5)
        self.hand_names,self.hand_vars,self.hand_current,self.hand_text={},{},{},{}
        body=ttk.Frame(self.hand_tab);body.pack(fill='both',expand=True,padx=10)
        for side,title in (('L','左手'),('R','右手')):
            frame=ttk.LabelFrame(body,text=title,padding=8)
            frame.pack(side='left',fill='both',expand=True,padx=5)
            self.hand_names[side]=[j['name'] for j in config[side]['joints']]
            self.hand_vars[side]=[]
            for joint in config[side]['joints']:
                row=ttk.Frame(frame);row.pack(fill='x',pady=3)
                finger={'FINGER_THUMB':'拇指','FINGER_INDEX':'食指','FINGER_MIDDLE':'中指','FINGER_RING':'无名指'}[joint['sdk_finger']]
                ttk.Label(row,text=f"J{joint['sdk_id']} {finger}{joint['finger_joint']} (°)",width=19).pack(side='left')
                var=tk.DoubleVar(value=0.)
                ttk.Scale(row,from_=math.degrees(joint['range'][0]),to=math.degrees(joint['range'][1]),variable=var).pack(side='left',fill='x',expand=True)
                ttk.Entry(row,textvariable=var,width=8).pack(side='left',padx=4)
                self.hand_vars[side].append(var)
            row=ttk.Frame(frame);row.pack(fill='x',pady=12)
            ttk.Button(row,text='张开',command=lambda s=side:self.preset(s,False)).pack(side='left',padx=3)
            ttk.Button(row,text='弯曲',command=lambda s=side:self.preset(s,True)).pack(side='left',padx=3)
            ttk.Button(row,text='发送目标',command=lambda s=side:self.send_hand(s)).pack(side='left',padx=3)
            value=tk.StringVar(value='等待手部反馈');self.hand_text[side]=value
            ttk.Label(frame,textvariable=value,wraplength=400).pack(fill='x',pady=8)
        self.create_body_panel(model)

    def create_tabs(self,root):
        self.notebook=ttk.Notebook(root)
        self.notebook.pack(fill='both',expand=True,padx=8,pady=8)
        self.arm_tab=ttk.Frame(self.notebook)
        self.hand_tab=ttk.Frame(self.notebook)
        self.body_tab=ttk.Frame(self.notebook)
        self.notebook.add(self.arm_tab,text='双臂控制')
        self.notebook.add(self.hand_tab,text='灵巧手控制')
        self.notebook.add(self.body_tab,text='本体与头部')
        return self.arm_tab

    def fk_pose(self,arm,joints):
        if self.j4_body is None or time.monotonic()-self.j4_refreshed>.5:
            raise ValueError('等待当前躯干 SDK 反馈，无法计算 J4 下的末端位姿')
        return self.frames.fk(arm,self.kine.fk(arm,joints),self.j4_body)

    def feedback(self,msg):
        if self.j4_body is None or time.monotonic()-self.j4_refreshed>.5:
            self.received=False
            return
        return super().feedback(msg)

    def create_body_panel(self,model):
        import yaml
        config=yaml.safe_load((model/'config/body.yaml').read_text())
        ttk.Label(self.body_tab,text='底盘固定于地面；本体姿态变化会带动头部、肩部和双臂。',
                  wraplength=1000).pack(padx=15,pady=12)
        ttk.Label(self.body_tab,text='原版 Gento SDK → 本机虚拟控制器 → 躯干/头部关节反馈。').pack(pady=5)
        self.body_pub=self.create_publisher(JointState,'/marvin_mesh_sim/body_targets',10)
        self.body_vars=[];self.body_names=[j['name'] for j in config['joints']]
        self.body_current={}
        for index,joint in enumerate(config['joints']):
            row=ttk.Frame(self.body_tab);row.pack(fill='x',padx=80,pady=6)
            label=f'躯干 J{index+1}' if joint['group']=='torso' else f'头部 J{index-5}'
            ttk.Label(row,text=label+' (°)',width=16).pack(side='left')
            var=tk.DoubleVar(value=0.)
            ttk.Scale(row,from_=math.degrees(joint['range'][0]),
                      to=math.degrees(joint['range'][1]),variable=var).pack(side='left',fill='x',expand=True)
            ttk.Entry(row,textvariable=var,width=12).pack(side='left',padx=6)
            self.body_vars.append(var)
        actions=ttk.Frame(self.body_tab);actions.pack(pady=10)
        ttk.Button(actions,text='读取当前姿态',command=self.read_body).pack(side='left',padx=6)
        ttk.Button(actions,text='发送本体目标',command=self.send_body).pack(side='left',padx=6)
        ttk.Button(actions,text='恢复初始姿态',command=self.reset_body).pack(side='left',padx=6)
        self.body_message=tk.StringVar(value='目标单位：度；滑块不会自动发送运动')
        ttk.Label(self.body_tab,textvariable=self.body_message,wraplength=1000).pack(padx=15,pady=8)
        self.create_subscription(JointState,'/marvin_mesh_sim/body_states',self.body_feedback,10)
        self.create_subscription(String,'/marvin_mesh_sim/body_status',
                                 lambda m:self.body_message.set(m.data),10)

    def body_feedback(self,msg):
        self.body_current=dict(zip(msg.name,msg.position))
        if all(name in self.body_current for name in self.frames.body_names):
            self.j4_body=np.array([self.body_current[name] for name in self.frames.body_names])
            self.j4_refreshed=time.monotonic()

    def read_body(self):
        if not all(n in self.body_current for n in self.body_names):
            self.body_message.set('等待本体反馈');return
        for var,name in zip(self.body_vars,self.body_names):
            var.set(round(math.degrees(self.body_current[name]),5))

    def send_body(self):
        try:
            msg=JointState();msg.name=self.body_names
            msg.position=[math.radians(v.get()) for v in self.body_vars]
            self.body_pub.publish(msg)
        except tk.TclError as exc:self.body_message.set(str(exc))

    def reset_body(self):
        for var in self.body_vars:var.set(0.)
        self.send_body()

    def hand_feedback(self,msg):
        values=dict(zip(msg.name,msg.position))
        for side,names in self.hand_names.items():
            if all(n in values for n in names):
                self.hand_current[side]=[values[n] for n in names]
                self.hand_text[side].set('SDK 反馈 (°)：\n'+', '.join(f'{math.degrees(values[n]):.2f}' for n in names))

    def preset(self,side,closed):
        values=closed_posture(self.hand_names[side]) if closed else [0.]*11
        for var,value in zip(self.hand_vars[side],values):var.set(math.degrees(value))
        self.send_hand(side)

    def send_hand(self,side):
        try:
            msg=JointState();msg.name=self.hand_names[side]
            msg.position=[math.radians(v.get()) for v in self.hand_vars[side]]
            self.hand_pub.publish(msg)
        except tk.TclError as exc:
            self.hand_message.set(str(exc))


def main(args=None):
    rclpy.init(args=args);node=None
    try:
        node=MeshPanel();node.poll();node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.root.destroy();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

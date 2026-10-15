import json
from pathlib import Path
import tkinter as tk
from tkinter import ttk

import rclpy
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from marvin_sim.panel import Panel
from .kinematics import MeshKinematics
from .hands import closed_posture


class MeshPanel(Panel):
    def __init__(self):
        super().__init__(kine_factory=MeshKinematics)
        self.root.title('整机仿真 · 双臂 SDK 正逆解与控制')
        self.hand_window=tk.Toplevel(self.root)
        self.hand_window.title('灵巧手 · 每侧 4 指 / 11 关节')
        self.hand_window.geometry('950x720')
        self.hand_window.protocol('WM_DELETE_WINDOW',self.hand_window.withdraw)
        model=Path(self.get_parameter('description_root').value)
        import yaml
        config=yaml.safe_load((model/'config/hands.yaml').read_text())
        report=json.loads((model/'config/asset_report.json').read_text())
        if report['empty_mesh_count']:
            banner=(f"已读取 {report['mesh_count']} 个真实 STL；{report['empty_mesh_count']} 个空 STL "
                    '暂用原 MJCF 碰撞盒显示（橙色），控制关节保留。')
        else:
            banner=f"已读取全部 {report['mesh_count']} 个真实 STL：底盘、躯干、头部、双臂和双手，无占位外形。"
        ttk.Label(self.hand_window,text=banner,wraplength=900).pack(padx=15,pady=12)
        ttk.Label(self.hand_window,text='手部控制是仿真扩展接口；机械臂运动使用原版厂商 SDK。').pack(pady=3)
        self.hand_pub=self.create_publisher(JointState,'/marvin_mesh_sim/hand_targets',10)
        self.create_subscription(JointState,'/marvin_mesh_sim/hand_states',self.hand_feedback,10)
        self.create_subscription(String,'/marvin_mesh_sim/hand_status',
                                 lambda m:self.hand_message.set(m.data),10)
        self.hand_message=tk.StringVar(value='手指目标单位：rad；滑块不会自动发送运动')
        ttk.Label(self.hand_window,textvariable=self.hand_message,wraplength=900).pack(pady=5)
        self.hand_names,self.hand_vars,self.hand_current,self.hand_text={},{},{},{}
        body=ttk.Frame(self.hand_window);body.pack(fill='both',expand=True,padx=10)
        for side,title in (('L','左手'),('R','右手')):
            frame=ttk.LabelFrame(body,text=title,padding=8)
            frame.pack(side='left',fill='both',expand=True,padx=5)
            self.hand_names[side]=[j['name'] for j in config[side]['joints']]
            self.hand_vars[side]=[]
            for joint in config[side]['joints']:
                row=ttk.Frame(frame);row.pack(fill='x',pady=3)
                ttk.Label(row,text=joint['name'],width=15).pack(side='left')
                var=tk.DoubleVar(value=0.)
                ttk.Scale(row,from_=joint['range'][0],to=joint['range'][1],variable=var).pack(side='left',fill='x',expand=True)
                ttk.Entry(row,textvariable=var,width=8).pack(side='left',padx=4)
                self.hand_vars[side].append(var)
            row=ttk.Frame(frame);row.pack(fill='x',pady=12)
            ttk.Button(row,text='张开',command=lambda s=side:self.preset(s,False)).pack(side='left',padx=3)
            ttk.Button(row,text='弯曲',command=lambda s=side:self.preset(s,True)).pack(side='left',padx=3)
            ttk.Button(row,text='发送目标',command=lambda s=side:self.send_hand(s)).pack(side='left',padx=3)
            value=tk.StringVar(value='等待手部反馈');self.hand_text[side]=value
            ttk.Label(frame,textvariable=value,wraplength=400).pack(fill='x',pady=8)
        ttk.Button(self.root,text='显示灵巧手控制窗口',command=self.hand_window.deiconify).pack(pady=3)
        self.create_body_panel(model)

    def create_body_panel(self,model):
        import yaml
        config=yaml.safe_load((model/'config/body.yaml').read_text())
        self.body_window=tk.Toplevel(self.root)
        self.body_window.title('整机本体 · 躯干与头部姿态')
        self.body_window.geometry('760x600')
        self.body_window.protocol('WM_DELETE_WINDOW',self.body_window.withdraw)
        ttk.Label(self.body_window,text='底盘固定于地面；本体姿态变化会带动头部、肩部和双臂。',
                  wraplength=700).pack(padx=15,pady=12)
        ttk.Label(self.body_window,text='本窗口仅调节仿真关节，不向真实机器人发送本体指令。').pack(pady=5)
        self.body_pub=self.create_publisher(JointState,'/marvin_mesh_sim/body_targets',10)
        self.body_vars=[];self.body_names=[j['name'] for j in config['joints']]
        self.body_current={}
        for index,joint in enumerate(config['joints']):
            row=ttk.Frame(self.body_window);row.pack(fill='x',padx=15,pady=6)
            label=f'躯干 J{index+1}' if joint['group']=='torso' else f'头部 J{index-5}'
            ttk.Label(row,text=label+' (°)',width=16).pack(side='left')
            var=tk.DoubleVar(value=0.)
            ttk.Scale(row,from_=math.degrees(joint['range'][0]),
                      to=math.degrees(joint['range'][1]),variable=var).pack(side='left',fill='x',expand=True)
            ttk.Entry(row,textvariable=var,width=12).pack(side='left',padx=6)
            self.body_vars.append(var)
        actions=ttk.Frame(self.body_window);actions.pack(pady=10)
        ttk.Button(actions,text='读取当前姿态',command=self.read_body).pack(side='left',padx=6)
        ttk.Button(actions,text='发送本体目标',command=self.send_body).pack(side='left',padx=6)
        ttk.Button(actions,text='恢复初始姿态',command=self.reset_body).pack(side='left',padx=6)
        self.body_message=tk.StringVar(value='目标单位：度；滑块不会自动发送运动')
        ttk.Label(self.body_window,textvariable=self.body_message,wraplength=700).pack(padx=15,pady=8)
        self.create_subscription(JointState,'/marvin_mesh_sim/body_states',self.body_feedback,10)
        self.create_subscription(String,'/marvin_mesh_sim/body_status',
                                 lambda m:self.body_message.set(m.data),10)
        ttk.Button(self.hand_window,text='显示本体/头部控制窗口',command=self.body_window.deiconify).pack(pady=5)

    def body_feedback(self,msg):
        self.body_current=dict(zip(msg.name,msg.position))

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
                self.hand_text[side].set('当前角度 (rad)：\n'+', '.join(f'{values[n]:.2f}' for n in names))

    def preset(self,side,closed):
        values=closed_posture(self.hand_names[side]) if closed else [0.]*11
        for var,value in zip(self.hand_vars[side],values):var.set(value)
        self.send_hand(side)

    def send_hand(self,side):
        try:
            msg=JointState();msg.name=self.hand_names[side]
            msg.position=[v.get() for v in self.hand_vars[side]]
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

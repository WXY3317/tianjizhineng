"""Interactive joint/FK/IK panel. ROS bridge sends every move through SDK UDP."""
import json
import math
import tkinter as tk
from tkinter import ttk

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .kinematics import JOINT_NAMES, INITIAL_DEG, Kinematics, matrix_quaternion, quaternion_matrix


def rpy_matrix(degrees):
    roll, pitch, yaw = np.radians(degrees)
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def matrix_rpy(matrix):
    pitch = math.atan2(-matrix[2, 0], math.hypot(matrix[0, 0], matrix[1, 0]))
    if abs(math.cos(pitch)) < 1e-8:
        roll, yaw = 0., math.atan2(-matrix[0, 1], matrix[1, 1])
    else:
        roll, yaw = math.atan2(matrix[2, 1], matrix[2, 2]), math.atan2(matrix[1, 0], matrix[0, 0])
    return np.degrees([roll, pitch, yaw]).tolist()


class Panel(Node):
    def __init__(self, kine_factory=Kinematics, sdk_label='原版 libMarvinSDK.so → 本机虚拟控制器 → URDF / RViz',
                 pose_frame=None,initialize_pose=True,content_factory=None,endpoint_label='末端法兰'):
        super().__init__('marvin_sim_panel')
        self.declare_parameter('sdk_root', '')
        self.declare_parameter('description_root', '')
        self.kine = kine_factory(self.get_parameter('sdk_root').value,
                               self.get_parameter('description_root').value)
        self.pose_frame=pose_frame
        self.root = tk.Tk()
        self.root.title('Marvin M6 · SDK 正逆解与双臂仿真')
        self.root.geometry('1080x790')
        self.root.protocol('WM_DELETE_WINDOW', self.root.quit)
        self.angle_unit='deg';self.angle_controls=[]
        content = content_factory(self.root) if content_factory else self.root
        self.q = np.radians(INITIAL_DEG).ravel()
        self.received = False
        self.joint_pub = self.create_publisher(JointState, '/marvin_sim/target_joints', 10)
        self.pose_pubs = [self.create_publisher(PoseStamped, f'/marvin_sim/{s}/target_pose', 10)
                          for s in ('left', 'right')]
        self.stop_client = self.create_client(Trigger, '/marvin_sim/stop')
        self.enable_client = self.create_client(Trigger, '/marvin_sim/enable')
        self.create_subscription(JointState, '/marvin_sim/sdk_joint_states', self.feedback, 10)
        self.create_subscription(String, '/marvin_sim/status', self.status, 10)
        self.message = tk.StringVar(value='等待 SDK 反馈…')
        ttk.Label(content, text=sdk_label,
                  font=('', 14)).pack(pady=10)
        toolbar = ttk.Frame(content)
        toolbar.pack(fill='x', padx=18)
        ttk.Button(toolbar, text='停止双臂', command=lambda: self.service(self.stop_client)).pack(side='left', padx=5)
        ttk.Button(toolbar, text='启用位置模式', command=lambda: self.service(self.enable_client)).pack(side='left', padx=5)
        ttk.Button(toolbar, text='同时发送双臂关节目标', command=self.send_both).pack(side='left', padx=5)
        ttk.Button(toolbar, text='回到初始位置', command=self.move_initial).pack(side='left', padx=5)
        ttk.Button(toolbar, text='回到全零位置', command=self.move_zero).pack(side='left', padx=5)
        self.add_unit_selector(toolbar)
        ttk.Label(content, textvariable=self.message, wraplength=1020).pack(fill='x', padx=20, pady=8)
        body = ttk.Frame(content)
        body.pack(fill='both', expand=True, padx=12)
        self.entries, self.pose_entries, self.feedback_text = [], [], []
        for arm, title in enumerate((f"左臂 A · {pose_frame or 'Base_L'}", f"右臂 B · {pose_frame or 'Base_R'}")):
            box = ttk.LabelFrame(body, text=title, padding=10)
            box.pack(side='left', fill='both', expand=True, padx=5)
            variables = []
            for j in range(7):
                row = ttk.Frame(box)
                row.pack(fill='x', pady=3)
                label=tk.StringVar(value=f'J{j+1} (°)')
                ttk.Label(row, textvariable=label, width=9).pack(side='left')
                v = tk.DoubleVar(value=INITIAL_DEG[arm][j])
                scale=ttk.Scale(row, from_=math.degrees(self.kine.lower[7*arm+j]),
                                to=math.degrees(self.kine.upper[7*arm+j]), variable=v)
                scale.pack(side='left', fill='x', expand=True)
                ttk.Entry(row, textvariable=v, width=10).pack(side='left', padx=5)
                self.register_angle(v,label,f'J{j+1}',scale,
                                    self.kine.lower[7*arm+j],self.kine.upper[7*arm+j])
                variables.append(v)
            self.entries.append(variables)
            actions = ttk.Frame(box)
            actions.pack(fill='x', pady=8)
            ttk.Button(actions, text='读取当前关节', command=lambda a=arm: self.fill_joints(a)).pack(side='left')
            ttk.Button(actions, text='SDK 正解预览', command=lambda a=arm: self.preview_fk(a)).pack(side='left', padx=4)
            ttk.Button(actions, text='执行关节目标', command=lambda a=arm: self.send_joint(a)).pack(side='left')
            ttk.Separator(box).pack(fill='x', pady=8)
            ttk.Label(box, text=f"{endpoint_label}目标（相对 {pose_frame or '本臂基座'}）").pack(anchor='w')
            pose_vars = []
            for index,name in enumerate(('X (mm)', 'Y (mm)', 'Z (mm)', 'Roll', 'Pitch', 'Yaw')):
                row = ttk.Frame(box)
                row.pack(fill='x', pady=2)
                label=tk.StringVar(value=name if index<3 else f'{name} (°)')
                ttk.Label(row, textvariable=label, width=13).pack(side='left')
                v = tk.StringVar(value='0')
                ttk.Entry(row, textvariable=v).pack(side='left', fill='x', expand=True)
                if index>=3:self.register_angle(v,label,name)
                pose_vars.append(v)
            self.pose_entries.append(pose_vars)
            actions = ttk.Frame(box)
            actions.pack(fill='x', pady=8)
            ttk.Button(actions, text='读取当前末端', command=lambda a=arm: self.fill_pose(a)).pack(side='left')
            ttk.Button(actions, text='SDK 逆解并运动', command=lambda a=arm: self.send_pose(a)).pack(side='left', padx=5)
            text = tk.StringVar(value='等待反馈')
            ttk.Label(box, textvariable=text, wraplength=475).pack(anchor='w', pady=8)
            self.feedback_text.append(text)
            if initialize_pose:self.set_pose_fields(arm, self.fk_pose(arm, self.q[7*arm:7*arm+7]))
        ttk.Label(content, text='关节目标采用平滑关节插值；末端目标通过 SDK IK 求解后执行关节运动。此版本不模拟接触力或自动避碰。',
                  wraplength=1040).pack(padx=20, pady=12)

    def add_unit_selector(self,parent):
        frame=ttk.Frame(parent);frame.pack(side='right',padx=8)
        ttk.Label(frame,text='角度单位：').pack(side='left')
        ttk.Button(frame,text='角度 (°)',command=lambda:self.set_angle_unit('deg')).pack(side='left',padx=2)
        ttk.Button(frame,text='弧度 (rad)',command=lambda:self.set_angle_unit('rad')).pack(side='left',padx=2)

    def register_angle(self,var,label,name,scale=None,lower=None,upper=None):
        self.angle_controls.append((var,label,name,scale,lower,upper))

    def angle_values(self,values):
        values=np.asarray(values,dtype=float)
        return np.degrees(values) if self.angle_unit=='deg' else values

    def radians(self,values):
        values=np.asarray(values,dtype=float)
        return np.radians(values) if self.angle_unit=='deg' else values

    def angle_suffix(self):
        return '°' if self.angle_unit=='deg' else 'rad'

    def set_angle_unit(self,unit):
        if unit==self.angle_unit:return
        old=self.angle_unit
        converted=[]
        for var,*_ in self.angle_controls:
            try:
                value=float(var.get())
                converted.append(math.radians(value) if old=='deg' else math.degrees(value))
            except (ValueError,tk.TclError):converted.append(None)
        self.angle_unit=unit;suffix=self.angle_suffix()
        for control,value in zip(self.angle_controls,converted):
            var,label,name,scale,lower,upper=control
            label.set(f'{name} ({suffix})')
            if scale is not None:
                bounds=self.angle_values([lower,upper])
                scale.configure(from_=bounds[0],to=bounds[1])
            if value is not None:var.set(round(value,6))
        self.refresh_angle_displays()

    def refresh_angle_displays(self):
        if self.received:
            try:
                for arm in range(2):self.update_arm_feedback(arm)
            except ValueError as exc:
                self.received=False;self.message.set(str(exc))

    def fk_pose(self,arm,joints):
        return self.kine.fk(arm,joints)

    def status(self, msg):
        data = json.loads(msg.data)
        if data.get('accepted') is False:
            self.message.set(f"目标未执行：{data.get('error', '')}")
            return
        info=data.get('ik_info',{})
        change=info.get('max_joint_change_deg')
        if change is not None and self.angle_unit=='rad':change=math.radians(change)
        extra=(f" SDK 已换参考角求得有效解；最大关节变化 {change:.4f}{self.angle_suffix()}"
               if info.get('fallback') and change is not None else '')
        self.message.set(f"状态：{data.get('state')}  {data.get('error', '')}{extra}")

    def feedback(self, msg):
        mapping = dict(zip(msg.name, msg.position))
        if not all(n in mapping for n in JOINT_NAMES):
            return
        self.q = np.array([mapping[n] for n in JOINT_NAMES])
        first = not self.received
        self.received = True
        for arm in range(2):
            matrix=self.update_arm_feedback(arm)
            if first:
                self.fill_joints(arm)
                self.set_pose_fields(arm, matrix)

    def update_arm_feedback(self,arm):
        q=self.q[7*arm:7*arm+7];matrix=self.fk_pose(arm,q)
        self.feedback_text[arm].set(f'SDK 反馈关节 ({self.angle_suffix()})：\n'
            +', '.join(f'{x:.4f}' for x in self.angle_values(q))
            +'\nSDK 正解 XYZ (mm)：'+', '.join(f'{x:.2f}' for x in matrix[:3,3]*1000))
        return matrix

    def set_pose_fields(self, arm, matrix):
        rpy=self.angle_values(np.radians(matrix_rpy(matrix)))
        for v, value in zip(self.pose_entries[arm], [*(matrix[:3,3]*1000), *rpy]):
            v.set(f'{value:.6f}')

    def fill_joints(self, arm):
        for v, value in zip(self.entries[arm], self.angle_values(self.q[7*arm:7*arm+7])):
            v.set(round(float(value), 5))

    def fill_pose(self, arm):
        try:self.set_pose_fields(arm,self.fk_pose(arm,self.q[7*arm:7*arm+7]))
        except ValueError as exc:self.message.set(str(exc))

    def selected_joints(self, arm):
        return self.kine.validate_joints(arm, self.radians([v.get() for v in self.entries[arm]]))

    def preview_fk(self, arm):
        try:
            self.set_pose_fields(arm, self.fk_pose(arm, self.selected_joints(arm)))
            self.message.set('SDK 正解完成，结果已填入末端目标栏；尚未发送运动')
        except (ValueError, tk.TclError) as exc:
            self.message.set(str(exc))

    def send_both(self):
        self.send_joint(None)

    def move_preset(self,joints):
        shown=self.angle_values(joints)
        for arm in range(2):
            for var,value in zip(self.entries[arm],shown[7*arm:7*arm+7]):
                var.set(round(float(value),6))
        self.send_both()

    def move_initial(self):
        self.move_preset(np.radians(INITIAL_DEG).ravel())

    def move_zero(self):
        self.move_preset(np.zeros(14))

    def send_joint(self, arm):
        try:
            if not self.received:
                raise ValueError('尚未收到 SDK 反馈')
            arms = range(2) if arm is None else [arm]
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            for a in arms:
                msg.name.extend(JOINT_NAMES[7*a:7*a+7])
                msg.position.extend(self.selected_joints(a).tolist())
            self.joint_pub.publish(msg)
        except (ValueError, tk.TclError) as exc:
            self.message.set(str(exc))

    def send_pose(self, arm):
        try:
            if not self.received:
                raise ValueError('尚未收到 SDK 反馈')
            values = np.array([float(v.get()) for v in self.pose_entries[arm]])
            if not np.isfinite(values).all():
                raise ValueError('末端目标必须为有限数值')
            msg = PoseStamped()
            msg.header.frame_id = self.pose_frame or ('Base_L', 'Base_R')[arm]
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (values[:3]/1000).tolist()
            q = matrix_quaternion(rpy_matrix(np.degrees(self.radians(values[3:]))))
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
            self.pose_pubs[arm].publish(msg)
        except ValueError as exc:
            self.message.set(str(exc))

    def service(self, client):
        if not client.service_is_ready():
            self.message.set('SDK 桥接服务尚未就绪')
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self.message.set(f.result().message))

    def poll(self):
        if rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.)
            self.root.after(10, self.poll)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Panel()
        node.poll()
        node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.root.destroy()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

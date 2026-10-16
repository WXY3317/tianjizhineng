import json
from pathlib import Path

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from marvin_sim.node import Simulation
from .sg100_hands import SG100Hands
from .sg100_client import SG100Client
from .gento_kinematics import GentoMeshKinematics
from .gento_arms import GentoArms
from .gento_bridge import GentoBridge,resolve_gento
from .body import Body
from .frames import J4Frames,J4_FRAME
from .gento_protocol import GentoController


def sg100_root(node):
    node.declare_parameter('sg100_sdk_root','')
    configured=node.get_parameter('sg100_sdk_root').value
    return configured or str(Path(node.get_parameter('sdk_root').value).parent/'SG100_SDK_Publish')


class MeshSimulation(Simulation):
    def __init__(self):
        super().__init__(kine_factory=lambda _,model:GentoMeshKinematics(resolve_gento(self),model),
                         controller_factory=GentoArms)
        model=Path(self.get_parameter('description_root').value)
        self.hands=SG100Hands(model,self.controller.address,sg100_root(self))
        self.body=Body(model)
        self.frames=J4Frames(model)
        self.gento=GentoController(self.body,self.kine.sdk_root,self.controller.address,arms=self.controller)
        self.create_timer(.005,self.gento.tick)
        self.body_state=self.create_publisher(JointState,'/marvin_mesh_sim/model_body_states',10)
        self.create_timer(.01,self.body.tick)
        self.hand_state=self.create_publisher(JointState,'/marvin_mesh_sim/model_hand_states',10)
        self.hand_status=self.create_publisher(String,'/marvin_mesh_sim/hand_status',10)
        self.create_timer(.01,self.hands.tick)
        report=json.loads((model/'config/asset_report.json').read_text())
        if report['empty_mesh_count']:
            self.get_logger().warning(f"Assets: {report['mesh_count']} real STL, "
                                      f"{report['empty_mesh_count']} missing STL shown as MJCF collision boxes")
        else:
            self.get_logger().info(f"Assets: all {report['mesh_count']} STL meshes loaded; no placeholders")

    def publish(self):
        # Core publisher remains authoritative for arm poses and feedback.
        # Publish one combined JointState so TF never loses either arm or hand.
        stamp=self.get_clock().now().to_msg()
        msg=JointState()
        from marvin_sim.kinematics import JOINT_NAMES
        from marvin_sim.node import make_pose
        msg.header.stamp=stamp
        msg.name=JOINT_NAMES.copy();msg.position=self.controller.q.tolist()
        for side in ('L','R'):
            msg.name.extend(self.hands.names[side]);msg.position.extend(self.hands.q[side].tolist())
        msg.name.extend(self.body.names);msg.position.extend(self.body.q.tolist())
        self.joints.publish(msg)
        hand=JointState();hand.header.stamp=stamp
        hand.name=msg.name[14:36];hand.position=msg.position[14:36]
        self.hand_state.publish(hand)
        body=JointState();body.header.stamp=stamp
        body.name=self.body.names;body.position=self.body.q.tolist()
        self.body_state.publish(body)
        for arm,pub in enumerate(self.poses):
            pub.publish(make_pose(self.frames.fk(arm,self.kine.fk(arm,self.controller.q[7*arm:7*arm+7]),
                                  self.body.q),J4_FRAME,stamp))

    def publish_status(self):
        c=self.controller
        self.status.publish(String(data=json.dumps({
            'backend':'original_gento_sdk','modes':c.modes,
            'moving':[move is not None for move in c.moves],
            'received_packets':c.received,'rejected_packets':c.rejected,'last_error':c.last_error})))

    def destroy_node(self):
        self.gento.close()
        self.hands.close()
        return super().destroy_node()


class WholeBridge(GentoBridge):
    def __init__(self):
        super().__init__()
        self.gento_client=self.client
        self.body_state=self.create_publisher(JointState,'/marvin_mesh_sim/body_states',10)
        self.body_status=self.create_publisher(String,'/marvin_mesh_sim/body_status',10)
        self.create_subscription(JointState,'/marvin_mesh_sim/body_targets',self.body_goal,10)
        self.sg100=SG100Client(sg100_root(self),self.get_parameter('description_root').value,
                               self.get_parameter('sim_ip').value)
        self.hand_state=self.create_publisher(JointState,'/marvin_mesh_sim/hand_states',10)
        self.hand_status=self.create_publisher(String,'/marvin_mesh_sim/hand_status',10)
        self.create_subscription(JointState,'/marvin_mesh_sim/hand_targets',self.hand_goal,10)
        self.hand_started=self.get_clock().now()
        self.create_timer(.01,self.publish_hands)
        self.get_logger().info('Original libsg100_sdk.so connected via passive CANFD: L/R 11 joints, FINGER_RING J9-11')
        self.body_fault=False
        self.create_timer(.02,self.publish_body)
        self.get_logger().info('Original libGentoSDKPY.so connected: body 6 + head 2; UDP command and feedback')

    def hand_goal(self,msg):
        try:
            if (len(msg.name)!=len(msg.position) or len(set(msg.name))!=len(msg.name)
                    or msg.velocity or msg.effort):
                raise ValueError('Provide unique SG100 hand names with positions only')
            for side,names in self.sg100.names.items():
                if set(msg.name)==set(names):
                    values=dict(zip(msg.name,msg.position))
                    self.sg100.move(side,[values[n] for n in names])
                    self.hand_status.publish(String(data=json.dumps({'ok':True,'backend':'original_sg100_sdk'})))
                    return
            raise ValueError('Provide exactly one complete SG100 11-joint hand')
        except (ValueError,TypeError,RuntimeError) as exc:
            self.hand_status.publish(String(data=json.dumps({'ok':False,'error':str(exc)})))

    def publish_hands(self):
        try:
            state=self.sg100.read()
            msg=JointState();msg.header.stamp=self.get_clock().now().to_msg()
            for side in 'LR':
                msg.name.extend(self.sg100.names[side]);msg.position.extend(state[side].tolist())
            self.hand_state.publish(msg)
        except RuntimeError as exc:
            if (self.get_clock().now()-self.hand_started).nanoseconds>1000000000:
                self.get_logger().error(str(exc),throttle_duration_sec=5.)

    def body_goal(self,msg):
        try:
            if msg.velocity or msg.effort:
                raise ValueError('Torso/head commands accept position only')
            if self.body_fault:raise RuntimeError('Gento bridge faulted; restart to recover')
            self.gento_client.move_body(msg.name,msg.position)
            self.body_status.publish(String(data=json.dumps({'ok':True,'backend':'original_gento_sdk'})))
        except (ValueError,TypeError,RuntimeError) as exc:
            self.body_status.publish(String(data=json.dumps({'ok':False,'error':str(exc)})))

    def publish_body(self):
        try:
            q=self.gento_client.body_read()
            msg=JointState();msg.header.stamp=self.get_clock().now().to_msg()
            msg.name=self.gento_client.model.names;msg.position=q.tolist()
            self.body_state.publish(msg)
        except RuntimeError as exc:
            if not self.body_fault:
                self.body_fault=True
                self.body_status.publish(String(data=json.dumps({'ok':False,'error':str(exc)})))

    def destroy_node(self):
        self.sg100.close()
        return super().destroy_node()


def main(args=None):
    import sys
    bridge='--bridge' in sys.argv
    if bridge:sys.argv.remove('--bridge')
    rclpy.init(args=args)
    node=None
    try:
        node=WholeBridge() if bridge else MeshSimulation()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

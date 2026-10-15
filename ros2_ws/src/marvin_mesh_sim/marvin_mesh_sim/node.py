import json
from pathlib import Path

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from marvin_sim.node import Simulation,Bridge
from .hands import Hands
from .kinematics import MeshKinematics


class MeshSimulation(Simulation):
    def __init__(self):
        super().__init__(kine_factory=MeshKinematics)
        model=Path(self.get_parameter('description_root').value)
        self.hands=Hands(model,self.controller.address)
        self.hand_state=self.create_publisher(JointState,'/marvin_mesh_sim/hand_states',10)
        self.hand_status=self.create_publisher(String,'/marvin_mesh_sim/hand_status',10)
        self.create_subscription(JointState,'/marvin_mesh_sim/hand_targets',self.hand_goal,10)
        self.create_timer(.01,self.hands.tick)
        report=json.loads((model/'config/asset_report.json').read_text())
        if report['empty_mesh_count']:
            self.get_logger().warning(f"Assets: {report['mesh_count']} real STL, "
                                      f"{report['empty_mesh_count']} missing STL shown as MJCF collision boxes")
        else:
            self.get_logger().info(f"Assets: all {report['mesh_count']} STL meshes loaded; no placeholders")

    def hand_goal(self,msg):
        try:
            if (len(msg.name)!=len(msg.position) or len(set(msg.name))!=len(msg.name)
                    or msg.velocity or msg.effort):
                raise ValueError('Provide unique hand names with positions only')
            goals={}
            for side,names in self.hands.names.items():
                if set(msg.name)==set(names):
                    mapping=dict(zip(msg.name,msg.position))
                    goals[side]=[mapping[n] for n in names]
            if not goals:
                raise ValueError('Hand target must contain exactly one complete 11-joint hand')
            for side,values in goals.items():
                self.hands.set_target(side,values)
            self.hand_status.publish(String(data=json.dumps({'ok':True})))
        except (ValueError,TypeError) as exc:
            self.hand_status.publish(String(data=json.dumps({'ok':False,'error':str(exc)})))

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
        self.joints.publish(msg)
        hand=JointState();hand.header.stamp=stamp
        hand.name=msg.name[14:];hand.position=msg.position[14:]
        self.hand_state.publish(hand)
        for arm,pub in enumerate(self.poses):
            pub.publish(make_pose(self.kine.fk(arm,self.controller.q[7*arm:7*arm+7]),
                                  ('Base_L','Base_R')[arm],stamp))

    def destroy_node(self):
        self.hands.close()
        return super().destroy_node()


def main(args=None):
    import sys
    bridge='--bridge' in sys.argv
    if bridge:sys.argv.remove('--bridge')
    rclpy.init(args=args)
    node=None
    try:
        node=Bridge(kine_factory=MeshKinematics) if bridge else MeshSimulation()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from .dh_kin import SCARAKinematics

class DHMotionDemo(Node):
    def __init__(self):
        super().__init__('dh_motion_demo')
        self.kin = SCARAKinematics()
        self.joint_names = ['joint1','joint2','joint3']
        self.js_sub  = self.create_subscription(JointState,'/joint_states', self.on_js, 10)
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory', 10
        )
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(2.0, self.run_sequence)
        self.seq_step = 0
        self.last_js = None
        self.get_logger().info('DHMotionDemo pronto.')

    def on_js(self, msg):
        self.last_js = msg

    def publish_tf(self, q):
        pos, yaw, _ = self.kin.fk(q)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'ee_link'
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        cy, sy = np.cos(yaw*0.5), np.sin(yaw*0.5)
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        self.br.sendTransform(t)

    def send_q(self, q, duration=2.0):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        # usa só as 3 primeiras juntas para o controlador
        q_cmd = q[:3]

        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in q_cmd]
        p.time_from_start.sec = int(duration)
        traj.points = [p]

        self.traj_pub.publish(traj)
        self.publish_tf(q)

        pos, yaw, _ = self.kin.fk(q)
        self.get_logger().info(
            f"CMD q={np.round(q,3)} -> pose xyz={np.round(pos,3)}, yaw={yaw:.3f} rad"
        )

    def run_sequence(self):
        if   self.seq_step == 0:
            q = np.array([0.0, 0.0, 0.05, 0.0])
        elif self.seq_step == 1:
            r = self.kin.a1 + self.kin.a2 - 0.01
            x, y, z, phi = r, 0.0, 0.02, 0.0
            q = self.kin.ik(x, y, z, phi, elbow='up')
        elif self.seq_step == 2:
            q = np.array([0.0, 0.01, 0.02, 0.0])  # quase singularidade
        elif self.seq_step == 3:
            q = np.array([
                np.deg2rad(170),
                np.deg2rad(-135),
                0.10,
                np.deg2rad(180)
            ])
        else:
            self.get_logger().info("Sequência finalizada. Ctrl+C para sair.")
            self.timer.cancel()
            return

        self.send_q(q, 2.0)
        self.seq_step += 1

def main():
    rclpy.init()
    node = DHMotionDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

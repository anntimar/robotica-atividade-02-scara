import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np

from .dh_kin import SCARAKinematics

# ======= ajustes de duração =======
# mais pontos = trajetória mais suave e mais longa
CIRCLE_POINTS = 300   # antes: 100
SINE_POINTS   = 360   # antes: 120
TIMER_DT      = 0.1   # tempo entre pontos (s). Se quiser mais devagar, aumenta p/ 0.2, 0.3...
# ================================


class SCARATrajDemo(Node):
    def __init__(self):
        super().__init__('scara_traj_demo')

        self.kin = SCARAKinematics()
        self.joint_names = ['joint1', 'joint2', 'joint3']

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.br = TransformBroadcaster(self)

        # monta toda a trajetória (home -> círculo -> home -> senoide -> home)
        self.path = self.build_path()
        self.index = 0

        # publica um ponto a cada TIMER_DT segundos
        self.dt = TIMER_DT
        self.timer = self.create_timer(self.dt, self.timer_cb)

        self.get_logger().info(
            f'SCARATrajDemo pronto. Trajetória com {len(self.path)} pontos.'
        )

    # ---------- construção das trajetórias ----------

    def build_circle(self,
                     center=(0.4, 0.0),
                     radius=0.05,
                     z=0.05,
                     points=CIRCLE_POINTS,
                     elbow='up'):
        """Trajetória circular no plano xy."""
        qs = []
        for i in range(points):
            theta = 2.0 * np.pi * i / points
            x = center[0] + radius * np.cos(theta)
            y = center[1] + radius * np.sin(theta)
            phi = 0.0  # orientação da ferramenta

            try:
                q = self.kin.ik(x, y, z, phi, elbow=elbow)
                qs.append(q)
            except Exception as e:
                self.get_logger().warn(
                    f'IK falhou para círculo em ({x:.3f}, {y:.3f}, {z:.3f}): {e}'
                )
        return qs

    def build_sine(self,
                   x0=0.35,
                   x1=0.55,
                   y0=0.0,
                   A=0.03,
                   periods=2,
                   z=0.05,
                   points=SINE_POINTS,
                   elbow='up'):
        """Trajetória senoidal (como solda/desenho em onda)."""
        xs = np.linspace(x0, x1, points)
        k = 2.0 * np.pi * periods / (x1 - x0)

        qs = []
        for x in xs:
            y = y0 + A * np.sin(k * (x - x0))
            phi = 0.0
            try:
                q = self.kin.ik(x, y, z, phi, elbow=elbow)
                qs.append(q)
            except Exception as e:
                self.get_logger().warn(
                    f'IK falhou para senoide em ({x:.3f}, {y:.3f}, {z:.3f}): {e}'
                )
        return qs

    def build_path(self):
        """Monta sequência completa: home -> círculo -> home -> senoide -> home."""
        path = []

        # pose "segura" inicial (4 DOF, mas só 3 são enviados ao controlador)
        q_home = np.array([0.0, 0.0, 0.05, 0.0])
        path.append(q_home)

        # círculo
        circle_qs = self.build_circle()
        path.extend(circle_qs)

        # volta pra home
        path.append(q_home)

        # senoide
        sine_qs = self.build_sine()
        path.extend(sine_qs)

        # volta pra home de novo
        path.append(q_home)

        return path

    # ---------- publicação TF do efetuador ----------

    def publish_tf(self, q):
        pos, yaw, _ = self.kin.fk(q)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'ee_link'
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        self.br.sendTransform(t)

    # ---------- timer: envia um ponto por vez ----------

    def timer_cb(self):
        if self.index >= len(self.path):
            self.get_logger().info('Trajetória finalizada.')
            self.timer.cancel()
            return

        q = self.path[self.index]
        # controlador tem 3 juntas (joint1, joint2, joint3)
        q_cmd = q[:3]

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in q_cmd]
        # tempo relativo até este ponto (controlador vai pegando ponto a ponto)
        p.time_from_start.sec = 1
        traj.points = [p]

        self.traj_pub.publish(traj)
        self.publish_tf(q)

        pos, yaw, _ = self.kin.fk(q)
        self.get_logger().info(
            f'Ponto {self.index+1}/{len(self.path)} '
            f'q={np.round(q,3)} -> xyz={np.round(pos,3)}, yaw={yaw:.3f}'
        )

        self.index += 1


def main():
    rclpy.init()
    node = SCARATrajDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


#!/usr/bin/env python3
"""オドメトリの twist に、時変の共分散（観測ノイズ R）と倍率の補正を付けて中継するノード.

  odom → odom_out（twist の v・ω に odom_scale を掛け、twist.covariance を置き換える。他はそのまま）

odom_scale [v, ω] はオドメトリの倍率の補正（実効的な車輪間隔・車輪半径が設定と違う分）。v・ω に掛けてから使う。

R は分散で
  R_v     = R_odom[0] + alpha[0]·v² + alpha[1]·ω² + (accel_noise_tau[0]·a)²
  R_omega = R_odom[1] + alpha[2]·v² + alpha[3]·ω² + (accel_noise_tau[1]·α)²
  （v = twist.linear.x、ω = twist.angular.z、a = dv/dt、α = dω/dt は直前のメッセージとの差）。
  alpha の項は速さに応じた分（Nav2 AMCL の odom_alpha と同じ考え方）、accel_noise_tau の項は加速度に応じた分
  （遅れ τ のあるオドメトリの誤差は約 τ·加速度）。それ以外の成分（vy, vz, vroll, vpitch）は R_odom_other。
odom_scale が [1, 1]、alpha・accel_noise_tau がすべて 0 なら、固定の R_odom で値はそのまま。
ekf_localizer は odom_covariance_source: message で [0]・[35] を使う。
オドメトリの遅れの補正は ekf_localizer の odom_delay で行う（2026-10-15 にここの odom_delay は外した。同じことをしていたため）。
"""

from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


class OdomCovariance(Node):

    def __init__(self):
        super().__init__('odom_covariance')
        # yaml で [0, 0, 0, 0] のように整数で書いても受け付ける
        any_type = ParameterDescriptor(dynamic_typing=True)
        self.r0 = [float(v) for v in self.declare_parameter(
            'R_odom', [2.5e-3, 5.0e-3], any_type).value]
        self.alpha = [float(v) for v in self.declare_parameter(
            'odom_noise_alpha', [0.0, 0.0, 0.0, 0.0], any_type).value]
        self.r_other = float(self.declare_parameter('R_odom_other', 1.0e-6, any_type).value)
        self.tau = [float(v) for v in self.declare_parameter(
            'accel_noise_tau', [0.0, 0.0], any_type).value]
        self.scale = [float(v) for v in self.declare_parameter(
            'odom_scale', [1.0, 1.0], any_type).value]
        if len(self.r0) != 2 or len(self.alpha) != 4 or len(self.tau) != 2 or len(self.scale) != 2:
            raise ValueError('R_odom, accel_noise_tau and odom_scale need 2 values, '
                             'odom_noise_alpha needs 4 values')
        self.prev = None   # 直前の (stamp [s], v, ω)
        self.accel = (0.0, 0.0)
        self.pub = self.create_publisher(
            Odometry, 'odom_out', QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.get_logger().info(
            f'R_odom: {self.r0}, odom_noise_alpha: {self.alpha}, '
            f'accel_noise_tau: {self.tau}, odom_scale: {self.scale}')

    def on_odom(self, msg):
        msg.twist.twist.linear.x *= self.scale[0]
        msg.twist.twist.angular.z *= self.scale[1]
        v, w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev is not None and t > self.prev[0]:
            dt = t - self.prev[0]
            self.accel = ((v - self.prev[1]) / dt, (w - self.prev[2]) / dt)
        self.prev = (t, v, w)
        a = self.alpha
        cov = [0.0] * 36
        for i in (7, 14, 21, 28):
            cov[i] = self.r_other
        cov[0] = self.r0[0] + a[0] * v * v + a[1] * w * w + (self.tau[0] * self.accel[0]) ** 2
        cov[35] = self.r0[1] + a[2] * v * v + a[3] * w * w + (self.tau[1] * self.accel[1]) ** 2
        msg.twist.covariance = cov
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = OdomCovariance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

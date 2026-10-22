#!/usr/bin/env python3
"""オドメトリの twist に、時変の共分散（観測ノイズ R）と倍率の補正を付け、時刻を決めて中継するノード.

  odom → odom_out（twist の v・ω に odom_scale を掛け、twist.covariance を置き換え、stamp_mode で header.stamp を決める。他はそのまま）

stamp_mode（localization_bringup の環境ファイルの stamp_mode がそのまま入る。点群の cloud_stamp_corrector と同じ決め方）：
  stamp  ：stamp をそのまま使う
  receipt：このノードが受信した時刻にする
  auto   ：|受信した時刻 − stamp| が auto_threshold 以上なら受信した時刻、小さければ stamp（1 つずつ判断する）
  sim time で /clock をまだ受け取っていない間（このノードの時刻が 0）は、どの stamp_mode でも stamp のまま

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
        self.stamp_mode = self.declare_parameter('stamp_mode', 'stamp').value
        self.auto_threshold = float(self.declare_parameter('auto_threshold', 0.1, any_type).value)
        if self.stamp_mode not in ('stamp', 'receipt', 'auto'):
            raise ValueError(f'stamp_mode must be stamp, receipt or auto: {self.stamp_mode}')
        self.count = self.count_receipt = 0
        self.max_abs_diff = 0.0
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
            f'accel_noise_tau: {self.tau}, odom_scale: {self.scale}, '
            f'stamp_mode: {self.stamp_mode} (auto_threshold {self.auto_threshold} s)')

    def on_odom(self, msg):
        if self.stamp_mode != 'stamp':
            self.restamp(msg)
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

    def restamp(self, msg):
        """stamp_mode が receipt か、auto で受信した時刻と auto_threshold 以上ずれていれば、受信した時刻にする."""
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return   # sim time で /clock をまだ受け取っていない（受信した時刻が分からない）。stamp のまま
        diff = (now.nanoseconds - (msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)) * 1e-9
        use_receipt = self.stamp_mode == 'receipt' or abs(diff) >= self.auto_threshold
        if use_receipt:
            msg.header.stamp = now.to_msg()
        if self.stamp_mode == 'auto':
            self.count += 1
            self.count_receipt += use_receipt
            self.max_abs_diff = max(self.max_abs_diff, abs(diff))
            self.get_logger().info(
                f'auto: receipt time used for {self.count_receipt} of {self.count} messages '
                f'(|receipt - stamp| latest {diff * 1e3:.1f} ms, max {self.max_abs_diff * 1e3:.1f} ms)',
                throttle_duration_sec=10.0)


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

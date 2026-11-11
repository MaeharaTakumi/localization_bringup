#!/usr/bin/env python3
"""オドメトリの twist に倍率を掛け、時刻を決めて中継するノード（2026-11-11 に odom_covariance から名前を変えた。共分散は扱わない）.

  odom → odom_out（twist の v・ω に odom_scale を掛け、stamp_mode で header.stamp を決める。他はそのまま）

stamp_mode（localization_bringup の環境ファイルの stamp_mode がそのまま入る。NDT の stamp_mode と同じ名前・値・決め方）：
  stamp  ：stamp をそのまま使う
  receipt：このノードが受信した時刻にする
  auto   ：|受信した時刻 − stamp| が stamp_auto_threshold 以上なら受信した時刻、小さければ stamp（1 つずつ判断する）
  sim time で /clock をまだ受け取っていない間（このノードの時刻が 0）は、どの stamp_mode でも stamp のまま

odom_scale [v, ω] はオドメトリの倍率の補正（実効的な車輪間隔・車輪半径が設定と違う分）。
観測ノイズ R は ekf_localizer が計算する（R_odom・odom_noise_alpha・accel_noise_tau。2026-11-09 にここから移した）。
オドメトリの遅れの補正は ekf_localizer の odom_delay で行う（2026-10-15 にここの odom_delay は外した。同じことをしていたため）。
"""

from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


class OdomRelay(Node):

    def __init__(self):
        super().__init__('odom_relay')
        # yaml で [1, 1] のように整数で書いても受け付ける
        any_type = ParameterDescriptor(dynamic_typing=True)
        self.scale = [float(v) for v in self.declare_parameter(
            'odom_scale', [1.0, 1.0], any_type).value]
        self.stamp_mode = self.declare_parameter('stamp_mode', 'stamp').value
        self.stamp_auto_threshold = float(self.declare_parameter('stamp_auto_threshold', 0.1, any_type).value)
        if self.stamp_mode not in ('stamp', 'receipt', 'auto'):
            raise ValueError(f'stamp_mode must be stamp, receipt or auto: {self.stamp_mode}')
        self.count = self.count_receipt = 0
        self.max_abs_diff = 0.0
        if len(self.scale) != 2:
            raise ValueError('odom_scale needs 2 values')
        self.pub = self.create_publisher(
            Odometry, 'odom_out', QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.get_logger().info(
            f'odom_scale: {self.scale}, '
            f'stamp_mode: {self.stamp_mode} (stamp_auto_threshold {self.stamp_auto_threshold} s)')

    def on_odom(self, msg):
        if self.stamp_mode != 'stamp':
            self.restamp(msg)
        msg.twist.twist.linear.x *= self.scale[0]
        msg.twist.twist.angular.z *= self.scale[1]
        self.pub.publish(msg)

    def restamp(self, msg):
        """stamp_mode が receipt か、auto で受信した時刻と stamp_auto_threshold 以上ずれていれば、受信した時刻にする."""
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return   # sim time で /clock をまだ受け取っていない（受信した時刻が分からない）。stamp のまま
        diff = (now.nanoseconds - (msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)) * 1e-9
        use_receipt = self.stamp_mode == 'receipt' or abs(diff) >= self.stamp_auto_threshold
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
    node = OdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

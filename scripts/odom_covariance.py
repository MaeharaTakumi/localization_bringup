#!/usr/bin/env python3
"""オドメトリの twist に、速さに応じた共分散（観測ノイズ R）を付けて中継するノード.

  odom → odom_out（twist.covariance だけ置き換える。他はそのまま）

R は Nav2 AMCL のオドメトリの動きのモデル（odom_alpha）と同じく、ばらつきが動きに比例するとして分散で
  R_v     = R_odom[0] + alpha[0]·v² + alpha[1]·ω²
  R_omega = R_odom[1] + alpha[2]·v² + alpha[3]·ω²
  （v = twist.linear.x、ω = twist.angular.z）。それ以外の成分（vy, vz, vroll, vpitch）は R_odom_other。
alpha がすべて 0 なら固定の R_odom（ekf_localizer の odom_covariance_source: param と同じ）。
ekf_localizer は odom_covariance_source: message で [0]・[35] を、robot_localization はそのまま使う。
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
        if len(self.r0) != 2 or len(self.alpha) != 4:
            raise ValueError('R_odom needs 2 values and odom_noise_alpha needs 4 values')
        self.pub = self.create_publisher(
            Odometry, 'odom_out', QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.get_logger().info(f'R_odom: {self.r0}, odom_noise_alpha: {self.alpha}')

    def on_odom(self, msg):
        v2 = msg.twist.twist.linear.x ** 2
        w2 = msg.twist.twist.angular.z ** 2
        a = self.alpha
        cov = [0.0] * 36
        for i in (7, 14, 21, 28):
            cov[i] = self.r_other
        cov[0] = self.r0[0] + a[0] * v2 + a[1] * w2
        cov[35] = self.r0[1] + a[2] * v2 + a[3] * w2
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

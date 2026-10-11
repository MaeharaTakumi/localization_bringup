#!/usr/bin/env python3
"""左右の車輪の角速度（sensor_msgs/JointState）から v・omega を計算し、オドメトリとして配信するノード.

  joint_states → odom（nav_msgs/Odometry。twist だけ。stamp は joint_states の stamp）

  v = r (wR + wL) / 2、omega = r (wR - wL) / b（差動二輪。実機の whill_odometry と同じ式）

Gazebo の diff_drive_controller のオドメトリ（/wheelchair/odom）は車輪の回転角の差分を 0.2 s 移動平均しているため、
速度の変化が 0.2 s かけて斜めに現れる（中央で約 0.1 s 遅れる）。joint_states の角速度は移動平均がかかっていないので、
こちらを使う。velocity が無い・有限でないときは position の差分で求める。
共分散は付けない（odom_covariance が付ける）。
"""

import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState


class JointStateOdometry(Node):

    def __init__(self):
        super().__init__('joint_state_odometry')
        self.left = self.declare_parameter('left_wheel_joint', 'left_wheel_joint').value
        self.right = self.declare_parameter('right_wheel_joint', 'right_wheel_joint').value
        self.radius = float(self.declare_parameter('wheel_radius', 0.1325).value)
        self.separation = float(self.declare_parameter('wheel_separation', 0.492).value)
        self.odom_frame = self.declare_parameter('odom_frame_id', 'odom').value
        self.base_frame = self.declare_parameter('base_frame_id', 'base_link').value
        self.prev = None   # (stamp [s], 左の角度, 右の角度)
        self.pub = self.create_publisher(
            Odometry, 'odom', QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(JointState, 'joint_states', self.on_joint_states,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            f'joints: {self.left}, {self.right}, wheel_radius: {self.radius}, '
            f'wheel_separation: {self.separation}')

    def on_joint_states(self, msg):
        try:
            il, ir = msg.name.index(self.left), msg.name.index(self.right)
        except ValueError:
            self.get_logger().warn(f'{self.left} / {self.right} not in joint_states',
                                   throttle_duration_sec=5.0)
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        wl = wr = None
        if len(msg.velocity) > max(il, ir):
            wl, wr = msg.velocity[il], msg.velocity[ir]
            if not (math.isfinite(wl) and math.isfinite(wr)):
                wl = wr = None
        if wl is None:
            if len(msg.position) <= max(il, ir):
                return
            pl, pr = msg.position[il], msg.position[ir]
            prev, self.prev = self.prev, (t, pl, pr)
            if prev is None or t <= prev[0]:
                return
            wl, wr = (pl - prev[1]) / (t - prev[0]), (pr - prev[2]) / (t - prev[0])
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = self.radius * (wr + wl) / 2.0
        odom.twist.twist.angular.z = self.radius * (wr - wl) / self.separation
        self.pub.publish(odom)


def main():
    rclpy.init()
    node = JointStateOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

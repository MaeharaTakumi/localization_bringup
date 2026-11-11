#!/usr/bin/env python3
"""EKF を起動しないときに、NDT・lidarslam の解（map -> LiDAR）を取付で map -> base の TF に直して配信する.

localization.launch.py が EKF を起動しないときだけ起動する（NDT・scan_matcher は TF を出さない）。
  購読  pose（PoseWithCovarianceStamped。launch が /ndt_pose か /slam_pose に付け替える）
  配信  /tf  map_frame_id -> base_frame_id（stamp は解の stamp = 点群の時刻）
取付 base_frame_id -> lidar_frame_id は TF（静的）から読む（読めるまで配信しない）。
フレームの名前は launch が ekf_localizer.yaml の値を入れる（EKF と同じ TF になる）。
T_map_base = T_map_lidar · T_base_lidar⁻¹（pcl_localization が publish_tf で出していたものと同じ計算）。
"""

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
from tf2_ros import TransformListener


def q_mul(a, b):
    """クォータニオンの積 a·b（(x, y, z, w)）."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def q_rotate(q, v):
    """ベクトル v をクォータニオン q で回す."""
    x, y, z, w = q_mul(q_mul(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return (x, y, z)


class PoseToTf(Node):

    def __init__(self):
        super().__init__('pose_to_tf')
        self.map_frame = self.declare_parameter('map_frame_id', 'map').value
        self.base_frame = self.declare_parameter('base_frame_id', 'base_link').value
        self.lidar_frame = self.declare_parameter('lidar_frame_id', 'velodyne').value
        self.mount = None   # base -> LiDAR（平行移動、クォータニオン）
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        # NDT の ndt_pose・scan_matcher の slam_pose と同じ reliable・深さ 10
        self.create_subscription(
            PoseWithCovarianceStamped, 'pose', self.pose_received,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.get_logger().info(
            f'TF {self.map_frame} -> {self.base_frame} from the pose of {self.lidar_frame}')

    def lookup_mount(self):
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.lidar_frame, Time()).transform
        except TransformException as e:
            self.get_logger().warn(f'mount {self.base_frame} -> {self.lidar_frame} not available: {e}',
                                   throttle_duration_sec=5.0)
            return False
        self.mount = ((t.translation.x, t.translation.y, t.translation.z),
                      (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))
        self.get_logger().info(f'mount {self.base_frame} -> {self.lidar_frame} (from TF): '
                               f'xyz {self.mount[0]}, q {self.mount[1]}')
        return True

    def pose_received(self, msg):
        if self.mount is None and not self.lookup_mount():
            return
        p = msg.pose.pose
        q_ml = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        t_bl, q_bl = self.mount
        q_mb = q_mul(q_ml, (-q_bl[0], -q_bl[1], -q_bl[2], q_bl[3]))
        r = q_rotate(q_mb, t_bl)
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = p.position.x - r[0]
        tf.transform.translation.y = p.position.y - r[1]
        tf.transform.translation.z = p.position.z - r[2]
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q_mb
        self.broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = PoseToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""robot_localization と NDT・オドメトリ・/ekf_pose の利用者（MATLAB）をつなぐノード.

robot_localization は、姿勢の観測を base の姿勢として扱い（センサの取付を補正しない）、メッセージの
共分散をそのまま使う（0 は 1e-9 になる）。そのため、このノードで次のように直してから渡す。

  NDT        ndt_pose（map → LiDAR、共分散なし）
             → ndt_pose_base（map → base、共分散は R_ndt を取付で伝播したもの）
  オドメトリ odom → odom_out（child_frame_id を base にする。共分散は odom_covariance が付けたものをそのまま使う）
             最初の NDT が届くまでは流さない（robot_localization を NDT の姿勢で初期化するため）
             ekf_localizer と同じく twist を base の速度として扱う。Gazebo の child_frame_id は
             wheelchair/base_link で TF に無く、そのままだと robot_localization が黙って捨てる
  初期姿勢   initialpose（map → LiDAR。NDT と同じ扱い）
             → set_pose_map（map → base）と set_pose_odom（同じ値を odom フレームとして）
  出力       odometry（robot_localization の map → base）
             → ekf_pose（map → LiDAR、ekf_localizer の /ekf_pose と同じ型・QoS）
             publish_trigger: "timer"  odometry が届くたびに作る（robot_localization の frequency。既定）
                              "cloud"  点群（points）が届くたびに、その stamp の時刻の TF から作る

取付 base → LiDAR は静的 TF から読む。読めるまでは NDT・初期姿勢・出力を捨てる。
"""

from collections import deque
import math

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
import tf2_ros


# ---------- 姿勢の計算（[x, y, z, roll, pitch, yaw] と 4x4 同次変換） ----------

def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def matrix_to_rpy(r):
    roll = math.atan2(r[2, 1], r[2, 2])
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    yaw = math.atan2(r[1, 0], r[0, 0])
    return roll, pitch, yaw


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(r):
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0.0:
        s = 2.0 * math.sqrt(tr + 1.0)
        return ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s,
                (r[1, 0] - r[0, 1]) / s, 0.25 * s)
    if r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        return (0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s,
                (r[2, 1] - r[1, 2]) / s)
    if r[1, 1] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        return ((r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s,
                (r[0, 2] - r[2, 0]) / s)
    s = 2.0 * math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
    return ((r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s,
            (r[1, 0] - r[0, 1]) / s)


def vec_to_matrix(v):
    t = np.eye(4)
    t[:3, :3] = rpy_to_matrix(v[3], v[4], v[5])
    t[:3, 3] = v[:3]
    return t


def matrix_to_vec(t):
    return np.array([t[0, 3], t[1, 3], t[2, 3], *matrix_to_rpy(t[:3, :3])])


def pose_msg_to_matrix(p):
    t = np.eye(4)
    t[:3, :3] = quat_to_matrix(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
    t[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return t


def transform_msg_to_matrix(tr):
    t = np.eye(4)
    q = tr.rotation
    t[:3, :3] = quat_to_matrix(q.x, q.y, q.z, q.w)
    t[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
    return t


def fill_pose_msg(p, t):
    p.position.x, p.position.y, p.position.z = (float(v) for v in t[:3, 3])
    qx, qy, qz, qw = matrix_to_quat(t[:3, :3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def propagate_covariance(t_in, t_rel, cov_in, eps=1.0e-6):
    """t_out = t_in · t_rel の [x, y, z, roll, pitch, yaw] の共分散（数値ヤコビアンによる 1 次の伝播）."""
    v0 = matrix_to_vec(t_in)
    f0 = matrix_to_vec(t_in @ t_rel)
    jac = np.zeros((6, 6))
    for i in range(6):
        v = v0.copy()
        v[i] += eps
        d = matrix_to_vec(vec_to_matrix(v) @ t_rel) - f0
        d[3:] = [wrap(a) for a in d[3:]]
        jac[:, i] = d / eps
    return jac @ cov_in @ jac.T


# ---------- ノード ----------

class RobotLocalizationAdapter(Node):

    def __init__(self):
        super().__init__('robot_localization_adapter')
        self.map_frame = self.declare_parameter('map_frame_id', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame_id', 'odom').value
        self.base_frame = self.declare_parameter('base_frame_id', 'base_link').value
        self.lidar_frame = self.declare_parameter('lidar_frame_id', 'velodyne').value
        self.r_ndt = np.diag(self.declare_parameter(
            'R_ndt', [1.0e-2, 1.0e-2, 1.0e-2, 0.03, 0.03, 1.0e-3]).value)
        self.p_init = np.diag(self.declare_parameter(
            'P_init', [2.0e-1, 2.0e-1, 2.0e-1, 0.01, 0.01, 0.01]).value)
        self.publish_trigger = self.declare_parameter('publish_trigger', 'timer').value
        self.cloud_tf_timeout = self.declare_parameter('cloud_tf_timeout', 0.2).value
        if self.publish_trigger not in ('cloud', 'timer'):
            raise ValueError(f'publish_trigger must be "cloud" or "timer": {self.publish_trigger}')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.t_bl = None          # 取付 base → LiDAR
        self.t_lb = None
        self.ndt_received = False
        self.last_cov_base = None  # 最新の robot_localization の共分散（base）
        self.pending_clouds = deque()  # (点群の stamp, 受信時刻)
        self.last_publish_stamp = None

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.ndt_base_pub = self.create_publisher(PoseWithCovarianceStamped, 'ndt_pose_base', reliable)
        self.odom_pub = self.create_publisher(Odometry, 'odom_out', reliable)
        self.set_pose_map_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'set_pose_map', QoSProfile(depth=1))
        self.set_pose_odom_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'set_pose_odom', QoSProfile(depth=1))
        self.ekf_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'ekf_pose',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.create_subscription(PoseWithCovarianceStamped, 'ndt_pose', self.on_ndt_pose, reliable)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self.on_initial_pose, QoSProfile(depth=10))
        self.create_subscription(Odometry, 'odometry', self.on_odometry, QoSProfile(depth=10))
        if self.publish_trigger == 'cloud':
            # 点群は stamp しか使わない。best effort は reliable / best effort どちらの publisher とも接続できる
            self.create_subscription(PointCloud2, 'points', self.on_points, qos_profile_sensor_data)
            self.create_timer(0.01, self.publish_pending_clouds)

        self.get_logger().info(
            f'base: {self.base_frame}, lidar: {self.lidar_frame}, '
            f'publish_trigger: {self.publish_trigger}')

    def mount_ready(self):
        if self.t_bl is not None:
            return True
        try:
            tr = self.tf_buffer.lookup_transform(self.base_frame, self.lidar_frame, Time())
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f'mount {self.base_frame} -> {self.lidar_frame} not available: {e}',
                throttle_duration_sec=2.0)
            return False
        self.t_bl = transform_msg_to_matrix(tr.transform)
        self.t_lb = np.linalg.inv(self.t_bl)
        self.get_logger().info(
            f'mount {self.base_frame} -> {self.lidar_frame}: {matrix_to_vec(self.t_bl).round(4)}')
        return True

    # ---- 入力 ----

    def on_ndt_pose(self, msg):
        if not self.mount_ready():
            return
        t_ml = pose_msg_to_matrix(msg.pose.pose)
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        fill_pose_msg(out.pose.pose, t_ml @ self.t_lb)
        out.pose.covariance = propagate_covariance(t_ml, self.t_lb, self.r_ndt).flatten().tolist()
        self.ndt_base_pub.publish(out)
        self.ndt_received = True

    def on_odom(self, msg):
        if not self.ndt_received:
            return
        msg.child_frame_id = self.base_frame
        self.odom_pub.publish(msg)

    def on_initial_pose(self, msg):
        if msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f'initialpose frame_id "{msg.header.frame_id}" != "{self.map_frame}". Ignored.')
            return
        if not self.mount_ready():
            return
        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        if Time.from_msg(msg.header.stamp).nanoseconds == 0:
            out.header.stamp = self.get_clock().now().to_msg()
        fill_pose_msg(out.pose.pose, pose_msg_to_matrix(msg.pose.pose) @ self.t_lb)
        out.pose.covariance = self.p_init.flatten().tolist()
        out.header.frame_id = self.map_frame
        self.set_pose_map_pub.publish(out)
        # odom も同じ姿勢にする（リセット直後は map → odom が恒等）
        out.header.frame_id = self.odom_frame
        self.set_pose_odom_pub.publish(out)
        self.pending_clouds.clear()
        self.last_publish_stamp = None
        self.get_logger().info('initialpose: robot_localization (map, odom) reset.')

    # ---- 出力 ----

    def on_odometry(self, msg):
        self.last_cov_base = np.array(msg.pose.covariance).reshape(6, 6)
        if self.publish_trigger != 'timer' or not self.mount_ready():
            return
        self.publish_ekf_pose(pose_msg_to_matrix(msg.pose.pose) @ self.t_bl, msg.header.stamp)

    def on_points(self, msg):
        self.pending_clouds.append((Time.from_msg(msg.header.stamp), self.get_clock().now()))

    def publish_pending_clouds(self):
        """点群の stamp の時刻の map → LiDAR が TF で引けたら配信する（引けるまで待ち、古いものは捨てる）."""
        while self.pending_clouds:
            stamp, received = self.pending_clouds[0]
            if self.last_publish_stamp is not None and stamp <= self.last_publish_stamp:
                self.pending_clouds.popleft()
                continue
            if self.tf_buffer.can_transform(self.map_frame, self.lidar_frame, stamp):
                self.pending_clouds.popleft()
                if not self.mount_ready():
                    continue
                tr = self.tf_buffer.lookup_transform(self.map_frame, self.lidar_frame, stamp)
                self.publish_ekf_pose(transform_msg_to_matrix(tr.transform), stamp.to_msg())
                self.last_publish_stamp = stamp
                continue
            if self.get_clock().now() - received > Duration(seconds=self.cloud_tf_timeout):
                self.pending_clouds.popleft()
                self.get_logger().warn(
                    f'TF {self.map_frame} -> {self.lidar_frame} at cloud stamp '
                    f'{stamp.nanoseconds * 1e-9:.3f} not available. Skipped.',
                    throttle_duration_sec=2.0)
                continue
            break

    def publish_ekf_pose(self, t_ml, stamp):
        out = PoseWithCovarianceStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.map_frame
        fill_pose_msg(out.pose.pose, t_ml)
        if self.last_cov_base is not None:
            out.pose.covariance = propagate_covariance(
                t_ml @ self.t_lb, self.t_bl, self.last_cov_base).flatten().tolist()
        self.ekf_pose_pub.publish(out)


def main():
    rclpy.init()
    node = RobotLocalizationAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

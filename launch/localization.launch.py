"""NDT（pcl_localization_ros2）と EKF（ekf_localizer）を環境ごとの設定で起動する.

環境ファイルの launch_odometry が true なら whill_odometry も起動する（実機）.

  ros2 launch localization_bringup localization.launch.py env:=gazebo
  ros2 launch localization_bringup localization.launch.py env:=real map_path:=/path/to/map.pcd

環境は param/env/<名前>.yaml に 1 ファイルで書く（env:=<名前>）。追加するにはファイルを置いて
colcon build する。ビルドせずに試すなら env:=/path/to/file.yaml でも読める。

ノードの共通パラメータは各パッケージの param/*.yaml。環境ファイルの parameters で上書きする。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg
import yaml

# 点群の stamp を直したときの配信先
CORRECTED_POINTS = '/localization/points'


def _env_dir():
    return os.path.join(get_package_share_directory('localization_bringup'), 'param', 'env')


def _load_env(env):
    """env（名前かファイルのパス）を読み、(パス, 内容) を返す."""
    if env.endswith('.yaml') or os.sep in env:
        path = os.path.abspath(os.path.expanduser(env))
    else:
        path = os.path.join(_env_dir(), env + '.yaml')
    if not os.path.isfile(path):
        available = sorted(
            os.path.splitext(f)[0] for f in os.listdir(_env_dir()) if f.endswith('.yaml'))
        raise RuntimeError(
            f'env "{env}" not found ({path}). Available: {", ".join(available)}')
    with open(path) as f:
        return path, (yaml.safe_load(f) or {})


def _require(env, path, key):
    if key not in env:
        raise RuntimeError(f'"{key}" is missing in {path}')
    return env[key]


def _launch_setup(context):
    path, env = _load_env(LaunchConfiguration('env').perform(context))
    use_ekf = LaunchConfiguration('use_ekf').perform(context).lower() == 'true'
    use_rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    map_path = LaunchConfiguration('map_path').perform(context)

    use_sim_time = bool(env.get('use_sim_time', False))
    topics = _require(env, path, 'topics')
    base_frame_id = _require(env, path, 'base_frame_id')
    lidar_frame_id = _require(env, path, 'lidar_frame_id')
    correction = env.get('cloud_stamp_correction', 'none')
    launch_odometry = bool(env.get('launch_odometry', False))
    overrides = env.get('parameters') or {}
    sim_time = {'use_sim_time': use_sim_time}

    def node_params(name):
        return overrides.get(name) or {}

    actions = [LogInfo(
        msg=f'[localization_bringup] env: {path} '
            f'(use_ekf: {use_ekf}, launch_odometry: {launch_odometry})')]

    # ---- オドメトリ（Gazebo などオドメトリを別に配信する環境では起動しない）----
    if launch_odometry:
        actions.append(Node(
            package='whill_odometry', executable='whill_odometry_node',
            name='whill_odometry', output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('whill_odometry'), 'config', 'odometry.yaml'),
                node_params('whill_odometry'),
                {'input_topic_name': _require(topics, path, 'motor_speed')},
                sim_time],
            # 出力はコードで /wheelchair/odom 固定なので、topics.odom に付け替える
            remappings=[('/wheelchair/odom', topics['odom'])]))

    # ---- 静的 TF（URDF などが配信しない分）----
    for tf in env.get('static_tfs') or []:
        xyz = tf.get('xyz', [0.0, 0.0, 0.0])
        rpy = tf.get('rpy', [0.0, 0.0, 0.0])
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'static_tf_{tf["child"]}',
            arguments=[
                '--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
                '--frame-id', tf['parent'], '--child-frame-id', tf['child']],
            parameters=[sim_time]))

    # ---- 点群の stamp（時計が同期していない環境だけ直す）----
    points = topics['points']
    if correction != 'none':
        actions.append(Node(
            package='pcl_localization_ros2', executable='cloud_stamp_corrector',
            name='cloud_stamp_corrector', output='screen',
            parameters=[node_params('cloud_stamp_corrector'), {'mode': correction}, sim_time],
            remappings=[('input', points), ('output', CORRECTED_POINTS)]))
        points = CORRECTED_POINTS

    # ---- NDT ----
    ndt_params = {
        'base_frame_id': base_frame_id,
        # 点群の stamp は上で直してあるので、そのまま使う
        'scan_time_source': 'message',
        'publish_rviz_tf': False,
        # EKF と起動するときは TF を EKF に任せ、初期値を EKF の TF から取る
        'publish_tf': not use_ekf,
        'use_tf_init_guess': use_ekf,
    }
    if map_path:
        ndt_params['map_path'] = map_path
    ndt = LifecycleNode(
        package='pcl_localization_ros2', executable='pcl_localization_node',
        name='pcl_localization', namespace='', output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('pcl_localization_ros2'), 'param',
                'localization.yaml'),
            node_params('pcl_localization'), ndt_params, sim_time],
        remappings=[('velodyne_points', points), ('pcl_pose', '/current_pose')])
    actions += [
        ndt,
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=ndt, start_state='configuring', goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(ndt),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))])),
        EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(ndt),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE)),
    ]

    # ---- EKF ----
    if use_ekf:
        actions.append(Node(
            package='ekf_localizer', executable='ekf_localizer_node',
            name='ekf_localizer', output='screen',
            parameters=[
                os.path.join(get_package_share_directory('ekf_localizer'), 'param', 'ekf.yaml'),
                node_params('ekf_localizer'),
                {'base_frame_id': base_frame_id, 'lidar_frame_id': lidar_frame_id,
                 'odom_topic': topics['odom']},
                sim_time]))

    # ---- RViz ----
    if use_rviz:
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('pcl_localization_ros2'), 'rviz',
                'localization2.rviz')],
            parameters=[sim_time],
            # 設定ファイルの /velodyne_points を、stamp を直した点群に差し替える
            remappings=[('/velodyne_points', points)] if points != '/velodyne_points' else []))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'env', default_value='real',
            description='Environment name (param/env/<env>.yaml) or path to an env yaml file'),
        DeclareLaunchArgument(
            'use_ekf', default_value='true',
            description='Run ekf_localizer. If false, NDT publishes the TF itself'),
        DeclareLaunchArgument(
            'map_path', default_value='',
            description='PCD map path (overrides map_path in the parameter files)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz'),
        OpaqueFunction(function=_launch_setup),
    ])

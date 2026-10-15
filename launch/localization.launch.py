"""NDT（pcl_localization_ros2）と EKF（ekf_localizer）を環境ごとの設定で起動する.

環境ファイルの launch_odometry が true なら whill_odometry も起動する（実機）.
環境ファイルに joint_state_odometry があれば、車輪の角速度（JointState）からオドメトリを計算するノードを起動する（任意）.

  ros2 launch localization_bringup localization.launch.py env:=gazebo
  ros2 launch localization_bringup localization.launch.py map_path:=/path/to/map.pcd          # 実機の車椅子 red（既定）
  ros2 launch localization_bringup localization.launch.py env:=white map_path:=/path/to/map.pcd

実機は車椅子の色ごとの環境（env:=red・blue・white）で選ぶ（トピックの Drp5_whill_<色> が違う）。

環境は param/env/<名前>.yaml に 1 ファイルで書く（env:=<名前>）。追加するにはファイルを置いて
colcon build する。ビルドせずに試すなら env:=/path/to/file.yaml でも読める。

ノードの共通パラメータは各パッケージの param/*.yaml。環境ファイルの parameters で上書きする。
上書き（と launch が計算した値）は、ノード名の節の一時ファイルにして渡す（_scoped）。辞書のまま渡すと launch_ros が
/** の節にするので、パッケージの yaml にノード名の節があると、そちらが優先されて上書きが効かない（rclcpp は /** の節を先、
ノード名の節を後に当てるため）。
"""

import atexit
import os
import shutil
import tempfile

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

# 環境ファイルの clock_synced（点群とオドメトリの時刻の決め方）→ cloud_stamp_corrector の mode・odom_covariance の stamp_mode
#   true ：stamp をそのまま使う（時刻同期済み・Gazebo。cloud_stamp_corrector は起動しない）
#   false：受信した時刻にする
#   auto ：|受信した時刻 − stamp| が clock_offset_threshold（既定 0.1 s）以上なら受信した時刻、小さければ stamp（1 つずつ判断する）
CLOCK_SYNCED = {True: ('stamp', None), False: ('receipt', 'receipt'), 'auto': ('auto', 'auto')}   # → (odom, 点群)
# 点群の stamp を直したときの配信先
CORRECTED_POINTS = '/localization/points'
# odom_covariance が共分散を付けたオドメトリ（ekf_localizer が購読する）
ODOM_WITH_COV = '/localization/odom'



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


def _normalize(value):
    """launch_ros と同じく、整数と小数が混ざった数のリストは小数にそろえる（rcl は型の違う要素のリストを読めない）."""
    if isinstance(value, dict):
        return {k: _normalize(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        items = [_normalize(v) for v in value]
        nums = [v for v in items if isinstance(v, (int, float)) and not isinstance(v, bool)]
        if items and len(nums) == len(items) and any(isinstance(v, float) for v in nums):
            return [float(v) for v in items]
        return items
    return value


_TMP_DIR = None


def _tmp_dir():
    """一時 yaml の置き場所（launch ごとに 1 つ。launch が終わるときに消す。/tmp に溜まらないように）."""
    global _TMP_DIR
    if _TMP_DIR is None:
        _TMP_DIR = tempfile.mkdtemp(prefix='localization_bringup_')
        atexit.register(shutil.rmtree, _TMP_DIR, ignore_errors=True)
    return _TMP_DIR


def _scoped(name, *dicts):
    """パラメータの辞書（後ろほど優先）を、ノード名の節の一時 yaml にしてそのパスを返す.

    パッケージの yaml のノード名の節より後に読まれるので、同じ名前のパラメータを確実に上書きできる。
    """
    params = {}
    for d in dicts:
        params.update(d or {})
    params = _normalize(params)
    with tempfile.NamedTemporaryFile(
            'w', prefix=f'{name}_', suffix='.yaml', dir=_tmp_dir(), delete=False) as f:
        yaml.safe_dump({name: {'ros__parameters': params}}, f)
        return f.name


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
    topics = dict(_require(env, path, 'topics'))
    base_frame_id = _require(env, path, 'base_frame_id')
    lidar_frame_id = _require(env, path, 'lidar_frame_id')
    if 'cloud_stamp_correction' in env:
        raise RuntimeError(
            f'cloud_stamp_correction in {path} was replaced by clock_synced (true / false / auto) on 2026-10-15')
    clock_synced = env.get('clock_synced', True)
    if isinstance(clock_synced, str) and clock_synced.lower() in ('true', 'false'):
        clock_synced = clock_synced.lower() == 'true'
    if clock_synced not in CLOCK_SYNCED:
        raise RuntimeError(f'clock_synced "{clock_synced}" is not supported in {path}. Available: true, false, auto')
    odom_stamp_mode, cloud_mode = CLOCK_SYNCED[clock_synced]
    clock_threshold = {'auto_threshold': float(env.get('clock_offset_threshold', 0.1))}
    launch_odometry = bool(env.get('launch_odometry', False))
    overrides = env.get('parameters') or {}
    sim_time = {'use_sim_time': use_sim_time}

    def node_params(name):
        return overrides.get(name) or {}

    actions = [LogInfo(
        msg=f'[localization_bringup] env: {path} '
            f'(use_ekf: {use_ekf}, launch_odometry: {launch_odometry}, clock_synced: {clock_synced})')]

    # ---- オドメトリ（Gazebo などオドメトリを別に配信する環境では起動しない）----
    if launch_odometry:
        actions.append(Node(
            package='whill_odometry', executable='whill_odometry_node',
            name='whill_odometry', output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('whill_odometry'), 'config', 'odometry.yaml'),
                _scoped('whill_odometry', node_params('whill_odometry'),
                        {'input_topic_name': _require(topics, path, 'motor_speed')}, sim_time)],
            # 出力はコードで /wheelchair/odom 固定なので、topics.odom に付け替える
            remappings=[('/wheelchair/odom', topics['odom'])]))

    # ---- 車輪の角速度からのオドメトリ（joint_state_odometry があるとき。出力は topics.odom）----
    jso = env.get('joint_state_odometry')
    if jso:
        jso = dict(jso)
        joint_states = jso.pop('topic', '/joint_states')
        actions.append(Node(
            package='localization_bringup', executable='joint_state_odometry.py',
            name='joint_state_odometry', output='screen',
            parameters=[jso, {'base_frame_id': base_frame_id}, sim_time],
            remappings=[('joint_states', joint_states), ('odom', topics['odom'])]))

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

    # ---- 点群の stamp（clock_synced が true でなければ、受信した時刻にする・ずれていたら受信した時刻にする）----
    points = topics['points']
    if cloud_mode:
        actions.append(Node(
            package='pcl_localization_ros2', executable='cloud_stamp_corrector',
            name='cloud_stamp_corrector', output='screen',
            parameters=[_scoped('cloud_stamp_corrector', node_params('cloud_stamp_corrector'),
                                {'mode': cloud_mode}, clock_threshold, sim_time)],
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
            _scoped('pcl_localization', node_params('pcl_localization'), ndt_params, sim_time)],
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

    # ---- オドメトリの共分散・倍率・時刻（clock_synced。ekf_localizer が使う）----
    if use_ekf:
        actions.append(Node(
            package='localization_bringup', executable='odom_covariance.py',
            name='odom_covariance', output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('localization_bringup'), 'param',
                    'odom_covariance.yaml'),
                _scoped('odom_covariance', node_params('odom_covariance'),
                        {'stamp_mode': odom_stamp_mode}, clock_threshold, sim_time)],
            remappings=[('odom', topics['odom']), ('odom_out', ODOM_WITH_COV)]))

    # ---- EKF ----
    if use_ekf:
        actions.append(Node(
            package='ekf_localizer', executable='ekf_localizer_node',
            name='ekf_localizer', output='screen',
            parameters=[
                os.path.join(get_package_share_directory('ekf_localizer'), 'param', 'ekf.yaml'),
                _scoped(
                    'ekf_localizer', node_params('ekf_localizer'),
                    # points_topic は publish_trigger: cloud のときに購読する（stamp を直した点群）。
                    # オドメトリの R は odom_covariance が付けた共分散を使う
                    {'base_frame_id': base_frame_id, 'lidar_frame_id': lidar_frame_id,
                     'odom_topic': ODOM_WITH_COV, 'odom_covariance_source': 'message',
                     'points_topic': points},
                    sim_time)]))

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
            'env', default_value='red',
            description='Environment name (param/env/<env>.yaml: gazebo, red, blue, white) or path to an env yaml file'),
        DeclareLaunchArgument(
            'use_ekf', default_value='true',
            description='Run the EKF (ekf_localizer). If false, NDT publishes the TF itself'),
        DeclareLaunchArgument(
            'map_path', default_value='',
            description='PCD map path (overrides map_path in the parameter files)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz'),
        OpaqueFunction(function=_launch_setup),
    ])

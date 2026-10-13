"""NDT（pcl_localization_ros2）と EKF（ekf_localizer）を環境ごとの設定で起動する.

環境ファイルの launch_odometry が true なら whill_odometry も起動する（実機）.
環境ファイルに joint_state_odometry があれば、車輪の角速度（JointState）からオドメトリを計算するノードを起動する（任意）.

  ros2 launch localization_bringup localization.launch.py env:=gazebo
  ros2 launch localization_bringup localization.launch.py env:=real map_path:=/path/to/map.pcd

EKF は estimator で選ぶ。ekf_localizer（既定）か robot_localization（2 段 EKF、param/robot_localization.yaml）.

  ros2 launch localization_bringup localization.launch.py env:=gazebo estimator:=robot_localization

実機の車椅子は whill:=<色>（blue・red・white。既定は env の whill）。env の topics の {whill} が Drp5_whill_<色> になる。

環境は param/env/<名前>.yaml に 1 ファイルで書く（env:=<名前>）。追加するにはファイルを置いて
colcon build する。ビルドせずに試すなら env:=/path/to/file.yaml でも読める。

ノードの共通パラメータは各パッケージの param/*.yaml。環境ファイルの parameters で上書きする。
上書き（と launch が計算した値）は、ノード名の節の一時ファイルにして渡す（_scoped）。辞書のまま渡すと launch_ros が
/** の節にするので、パッケージの yaml にノード名の節があると、そちらが優先されて上書きが効かない（rclcpp は /** の節を先、
ノード名の節を後に当てるため。例：robot_localization.yaml の robot_localization_adapter の R_ndt）。
"""

import os
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

# 点群の stamp を直したときの配信先
CORRECTED_POINTS = '/localization/points'
# odom_covariance が共分散を付けたオドメトリ（両方の推定器が購読する）
ODOM_WITH_COV = '/localization/odom'

ESTIMATORS = ('ekf_localizer', 'robot_localization')
WHILL_COLORS = ('blue', 'red', 'white')

# robot_localization と robot_localization_adapter の間のトピック（param/robot_localization.yaml と合わせる）
RL_NDT_POSE = '/robot_localization/ndt_pose_base'
RL_ODOM = '/robot_localization/odom'
RL_SET_POSE_ODOM = '/ekf_filter_node_odom/set_pose'
RL_SET_POSE_MAP = '/ekf_filter_node_map/set_pose'
RL_ODOMETRY_ODOM = '/odometry/local'
RL_ODOMETRY_MAP = '/odometry/global'


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


def _expand_diag(params):
    """robot_localization の 15x15 の行列を、対角 15 個（<名前>_diag）で書けるようにする."""
    out = {}
    for key, value in params.items():
        if key.endswith('_diag'):
            n = len(value)
            out[key[:-len('_diag')]] = [
                float(value[i]) if i == j else 0.0 for i in range(n) for j in range(n)]
        else:
            out[key] = value
    return out


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


def _scoped(name, *dicts):
    """パラメータの辞書（後ろほど優先）を、ノード名の節の一時 yaml にしてそのパスを返す.

    パッケージの yaml のノード名の節より後に読まれるので、同じ名前のパラメータを確実に上書きできる。
    """
    params = {}
    for d in dicts:
        params.update(d or {})
    params = _normalize(params)
    with tempfile.NamedTemporaryFile(
            'w', prefix=f'localization_bringup_{name}_', suffix='.yaml', delete=False) as f:
        yaml.safe_dump({name: {'ros__parameters': params}}, f)
        return f.name


def _require(env, path, key):
    if key not in env:
        raise RuntimeError(f'"{key}" is missing in {path}')
    return env[key]


def _launch_setup(context):
    path, env = _load_env(LaunchConfiguration('env').perform(context))
    use_ekf = LaunchConfiguration('use_ekf').perform(context).lower() == 'true'
    estimator = LaunchConfiguration('estimator').perform(context)
    if estimator not in ESTIMATORS:
        raise RuntimeError(
            f'estimator "{estimator}" is not supported. Available: {", ".join(ESTIMATORS)}')
    use_rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    map_path = LaunchConfiguration('map_path').perform(context)

    use_sim_time = bool(env.get('use_sim_time', False))
    topics = dict(_require(env, path, 'topics'))
    # 実機の車椅子：topics の {whill} を Drp5_whill_<色> にする（launch の whill が env の whill より優先）
    whill = LaunchConfiguration('whill').perform(context) or env.get('whill', '')
    if whill and whill not in WHILL_COLORS:
        raise RuntimeError(f'whill "{whill}" is not supported. Available: {", ".join(WHILL_COLORS)}')
    for key, value in topics.items():
        if isinstance(value, str) and '{whill}' in value:
            if not whill:
                raise RuntimeError(f'topics.{key} uses {{whill}} but whill is not set (whill:=<color> or env whill)')
            topics[key] = value.replace('{whill}', f'Drp5_whill_{whill}')
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
            f'(use_ekf: {use_ekf}, estimator: {estimator}, launch_odometry: {launch_odometry}, '
            f'whill: {whill or "-"})')]

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

    # ---- 点群の stamp（時計が同期していない環境だけ直す）----
    points = topics['points']
    if correction != 'none':
        actions.append(Node(
            package='pcl_localization_ros2', executable='cloud_stamp_corrector',
            name='cloud_stamp_corrector', output='screen',
            parameters=[_scoped('cloud_stamp_corrector', node_params('cloud_stamp_corrector'),
                                {'mode': correction}, sim_time)],
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

    # ---- オドメトリの共分散（速さに応じた R。両方の推定器で使う）----
    if use_ekf:
        actions.append(Node(
            package='localization_bringup', executable='odom_covariance.py',
            name='odom_covariance', output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('localization_bringup'), 'param',
                    'odom_covariance.yaml'),
                _scoped('odom_covariance', node_params('odom_covariance'), sim_time)],
            remappings=[('odom', topics['odom']), ('odom_out', ODOM_WITH_COV)]))

    # ---- EKF ----
    if use_ekf and estimator == 'ekf_localizer':
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

    # ---- robot_localization（2 段 EKF）----
    #   adapter        NDT（LiDAR）を base に直し、オドメトリ・NDT に共分散を付けて渡す。
    #                  /initialpose を 2 つの EKF の set_pose に、map -> base を /ekf_pose（LiDAR）に直す
    #   ekf_filter_node_odom  オドメトリだけ。odom -> base の TF
    #   ekf_filter_node_map   NDT + オドメトリ。map -> odom の TF
    if use_ekf and estimator == 'robot_localization':
        rl_params = os.path.join(
            get_package_share_directory('localization_bringup'), 'param', 'robot_localization.yaml')
        actions.append(Node(
            package='localization_bringup', executable='robot_localization_adapter.py',
            name='robot_localization_adapter', output='screen',
            parameters=[
                rl_params,
                _scoped('robot_localization_adapter', node_params('robot_localization_adapter'),
                        {'base_frame_id': base_frame_id, 'lidar_frame_id': lidar_frame_id},
                        sim_time)],
            remappings=[
                ('ndt_pose', '/ndt_pose'), ('ndt_pose_base', RL_NDT_POSE),
                ('odom', ODOM_WITH_COV), ('odom_out', RL_ODOM),
                ('initialpose', '/initialpose'),
                ('set_pose_odom', RL_SET_POSE_ODOM), ('set_pose_map', RL_SET_POSE_MAP),
                ('odometry', RL_ODOMETRY_MAP), ('points', points), ('ekf_pose', '/ekf_pose')]))
        for name, odometry, set_pose in (
                ('ekf_filter_node_odom', RL_ODOMETRY_ODOM, RL_SET_POSE_ODOM),
                ('ekf_filter_node_map', RL_ODOMETRY_MAP, RL_SET_POSE_MAP)):
            actions.append(Node(
                package='robot_localization', executable='ekf_node', name=name, output='screen',
                parameters=[
                    rl_params,
                    _scoped(name, _expand_diag(node_params(name)),
                            {'base_link_frame': base_frame_id}, sim_time)],
                remappings=[('odometry/filtered', odometry), ('set_pose', set_pose)]))

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
            description='Run an EKF (estimator). If false, NDT publishes the TF itself'),
        DeclareLaunchArgument(
            'estimator', default_value='ekf_localizer', choices=list(ESTIMATORS),
            description='EKF to run when use_ekf is true'),
        DeclareLaunchArgument(
            'map_path', default_value='',
            description='PCD map path (overrides map_path in the parameter files)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz'),
        DeclareLaunchArgument(
            'whill', default_value='',
            description='Wheelchair color for the real robot (blue, red, white). Default: whill in the env'),
        OpaqueFunction(function=_launch_setup),
    ])

"""NDT（pcl_localization_ros2）と EKF（ekf_localizer）を環境ごとの設定で起動する.

起動するノードと各ノードのトピックは、環境ファイルの nodes.<ノード名> に書く：
  enable：起動するか（cloud_stamp_corrector は書かない。stamp_mode が receipt・auto のときだけ起動する）
  topics：ノードの中の名前: 実際の名前。そのまま remap に渡す（つなぐときは配信側と購読側を同じ名前にする）
止めたノードの出力は、録画や別の PC から来るものとして、つなぎ方は変えない。例外は 2 つ：
  - ekf_localizer を起動しないときは NDT が TF を出し、RViz の /tf も付け替えない
  - cloud_stamp_corrector を起動するときは、ほかのノードで、その input と同じ名前のトピックを output に付け替える

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

# 環境ファイルの stamp_mode（点群とオドメトリの時刻の決め方）。odom_covariance の stamp_mode・cloud_stamp_corrector の mode にそのまま渡す
#   stamp  ：stamp をそのまま使う（時刻同期済み・Gazebo。cloud_stamp_corrector は起動しない）
#   receipt：受信した時刻にする
#   auto   ：|受信した時刻 − stamp| が clock_offset_threshold（既定 0.1 s）以上なら受信した時刻、小さければ stamp（1 つずつ判断する）
STAMP_MODES = ('stamp', 'receipt', 'auto')
# 環境ファイルの nodes に書くノード
NODES = ('whill_odometry', 'joint_state_odometry', 'cloud_stamp_corrector', 'pcl_localization',
         'odom_covariance', 'ekf_localizer', 'rviz')
# 2026-10-22 に nodes に移した環境ファイルの項目
MOVED_TO_NODES = {
    'topics': 'nodes.<node>.topics',
    'launch_odometry': 'nodes.whill_odometry.enable',
    'joint_state_odometry': 'nodes.joint_state_odometry (enable, topics) and parameters.joint_state_odometry',
}



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


def _arg_bool(context, name):
    """launch 引数の true / false。空なら None（環境ファイルの値を使う）."""
    value = LaunchConfiguration(name).perform(context).lower()
    return None if value == '' else value == 'true'


def _load_nodes(env, path, stamp_mode):
    """環境ファイルの nodes から、(起動するノードの集合, ノードごとのトピックの remap の辞書) を返す."""
    nodes = _require(env, path, 'nodes')
    unknown = sorted(set(nodes) - set(NODES))
    if unknown:
        raise RuntimeError(f'unknown node in nodes of {path}: {", ".join(unknown)}. Available: {", ".join(NODES)}')
    enabled = set()
    topics = {}
    for name in NODES:
        node = nodes.get(name)
        if not isinstance(node, dict):
            raise RuntimeError(f'"nodes.{name}" is missing in {path} (write enable and topics under it)')
        if name == 'cloud_stamp_corrector':
            if 'enable' in node:
                raise RuntimeError(
                    f'nodes.cloud_stamp_corrector.enable in {path} is not used: '
                    'it runs only when stamp_mode is receipt or auto')
            if stamp_mode != 'stamp':
                enabled.add(name)
        elif _require(node, f'{path} (nodes.{name})', 'enable'):
            enabled.add(name)
        topics[name] = dict(node.get('topics') or {})
    return enabled, topics


def _launch_setup(context):
    path, env = _load_env(LaunchConfiguration('env').perform(context))
    use_ekf = _arg_bool(context, 'use_ekf')
    use_rviz = _arg_bool(context, 'rviz')
    map_path = LaunchConfiguration('map_path').perform(context)

    use_sim_time = bool(env.get('use_sim_time', False))
    base_frame_id = _require(env, path, 'base_frame_id')
    lidar_frame_id = _require(env, path, 'lidar_frame_id')
    for old in ('cloud_stamp_correction', 'clock_synced'):
        if old in env:
            raise RuntimeError(
                f'{old} in {path} was replaced by stamp_mode (stamp / receipt / auto) on 2026-10-22 '
                '(clock_synced: true -> stamp, false -> receipt, auto -> auto)')
    for old, new in MOVED_TO_NODES.items():
        if old in env:
            raise RuntimeError(f'{old} in {path} was moved to {new} on 2026-10-22')
    stamp_mode = env.get('stamp_mode', 'stamp')
    if stamp_mode not in STAMP_MODES:
        raise RuntimeError(
            f'stamp_mode "{stamp_mode}" is not supported in {path}. Available: {", ".join(STAMP_MODES)}')
    clock_threshold = {'auto_threshold': float(env.get('clock_offset_threshold', 0.1))}
    overrides = env.get('parameters') or {}
    sim_time = {'use_sim_time': use_sim_time}

    def node_params(name):
        return overrides.get(name) or {}

    enabled, topics = _load_nodes(env, path, stamp_mode)
    # launch 引数は環境ファイルより優先（use_ekf は odom_covariance も一緒に）
    for name, value in (('ekf_localizer', use_ekf), ('odom_covariance', use_ekf), ('rviz', use_rviz)):
        if value is not None:
            (enabled.add if value else enabled.discard)(name)
    use_ekf = 'ekf_localizer' in enabled

    # cloud_stamp_corrector を起動するときは、ほかのノードの点群の入力（input と同じ名前）を output に付け替える
    if 'cloud_stamp_corrector' in enabled:
        cloud = topics['cloud_stamp_corrector']
        where = f'{path} (nodes.cloud_stamp_corrector.topics)'
        cloud_in, cloud_out = _require(cloud, where, 'input'), _require(cloud, where, 'output')
        for name in NODES:
            if name != 'cloud_stamp_corrector':
                topics[name] = {k: cloud_out if v == cloud_in else v for k, v in topics[name].items()}

    def remappings(name):
        return list(topics[name].items())

    actions = [LogInfo(
        msg=f'[localization_bringup] env: {path} (stamp_mode: {stamp_mode}, '
            f'nodes: {", ".join(n for n in NODES if n in enabled)})')]

    # ---- オドメトリ（Gazebo などオドメトリを別に配信する環境では起動しない）----
    if 'whill_odometry' in enabled:
        actions.append(Node(
            package='whill_odometry', executable='whill_odometry_node',
            name='whill_odometry', output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('whill_odometry'), 'param', 'odometry.yaml'),
                # 入力はパラメータなので、中の名前 motor_speed にして remap する
                _scoped('whill_odometry', node_params('whill_odometry'),
                        {'input_topic_name': 'motor_speed'}, sim_time)],
            remappings=remappings('whill_odometry')))

    # ---- 車輪の角速度（JointState）からのオドメトリ（任意）----
    if 'joint_state_odometry' in enabled:
        actions.append(Node(
            package='localization_bringup', executable='joint_state_odometry.py',
            name='joint_state_odometry', output='screen',
            parameters=[_scoped('joint_state_odometry', node_params('joint_state_odometry'),
                                {'base_frame_id': base_frame_id}, sim_time)],
            remappings=remappings('joint_state_odometry')))

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

    # ---- 点群の stamp（stamp_mode が receipt・auto のとき、受信した時刻にする・ずれていたら受信した時刻にする）----
    if 'cloud_stamp_corrector' in enabled:
        actions.append(Node(
            package='pcl_localization_ros2', executable='cloud_stamp_corrector',
            name='cloud_stamp_corrector', output='screen',
            parameters=[_scoped('cloud_stamp_corrector', node_params('cloud_stamp_corrector'),
                                {'mode': stamp_mode}, clock_threshold, sim_time)],
            remappings=remappings('cloud_stamp_corrector')))

    # ---- NDT ----
    if 'pcl_localization' in enabled:
        ndt_params = {
            'base_frame_id': base_frame_id,
            # 点群の stamp は cloud_stamp_corrector で直してあるので、そのまま使う
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
            remappings=remappings('pcl_localization'))
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

    # ---- オドメトリの共分散・倍率・時刻（stamp_mode。ekf_localizer が使う）----
    if 'odom_covariance' in enabled:
        actions.append(Node(
            package='localization_bringup', executable='odom_covariance.py',
            name='odom_covariance', output='screen',
            # パラメータは環境ファイルの parameters.odom_covariance（書かなければ odom_covariance.py の既定値）
            parameters=[
                _scoped('odom_covariance', node_params('odom_covariance'),
                        {'stamp_mode': stamp_mode}, clock_threshold, sim_time)],
            remappings=remappings('odom_covariance')))

    # ---- EKF ----
    if use_ekf:
        actions.append(Node(
            package='ekf_localizer', executable='ekf_localizer_node',
            name='ekf_localizer', output='screen',
            parameters=[
                os.path.join(get_package_share_directory('ekf_localizer'), 'param', 'ekf.yaml'),
                _scoped(
                    'ekf_localizer', node_params('ekf_localizer'),
                    # 購読するトピックはパラメータなので、中の名前にして remap する。
                    # オドメトリの R は odom_covariance が付けた共分散を使う
                    {'base_frame_id': base_frame_id, 'lidar_frame_id': lidar_frame_id,
                     'ndt_pose_topic': 'ndt_pose', 'odom_topic': 'odom', 'initialpose_topic': 'initialpose',
                     'points_topic': 'velodyne_points', 'odom_covariance_source': 'message'},
                    sim_time)],
            remappings=remappings('ekf_localizer')))

    # ---- RViz ----
    if 'rviz' in enabled:
        rviz_remappings = remappings('rviz')
        if use_ekf:
            # 点群を、同じ時刻の EKF の NDT の更新値（事後推定値）で座標変換する。ekf_localizer が
            # publish_ndt_update_tf で tf_ndt_update に出す map -> base を /tf の代わりに読む
            # （/tf の map -> base は点群の時刻の予測値。取付などの静的 TF は /tf_static のまま）
            rviz_remappings.append(
                ('/tf', topics['ekf_localizer'].get('tf_ndt_update', '/tf_ndt_update')))
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('pcl_localization_ros2'), 'rviz',
                'localization2.rviz')],
            parameters=[sim_time],
            remappings=rviz_remappings))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'env', default_value='red',
            description='Environment name (param/env/<env>.yaml: gazebo, red, blue, white) or path to an env yaml file'),
        DeclareLaunchArgument(
            'use_ekf', default_value='',
            description='Run the EKF (ekf_localizer) and odom_covariance (overrides nodes.*.enable in the env; '
                        'empty: use the env). If the EKF does not run, NDT publishes the TF itself'),
        DeclareLaunchArgument(
            'map_path', default_value='',
            description='PCD map path (overrides map_path in the parameter files)'),
        DeclareLaunchArgument(
            'rviz', default_value='',
            description='Launch RViz (overrides nodes.rviz.enable in the env; empty: use the env)'),
        OpaqueFunction(function=_launch_setup),
    ])

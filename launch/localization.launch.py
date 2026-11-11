"""NDT（pcl_localization_ros2）か lidarslam（scan_matcher・graph_based_slam）と、EKF（ekf_localizer）を環境ごとの設定で起動する.

  ros2 launch localization_bringup localization.launch.py env:=gazebo
  ros2 launch localization_bringup localization.launch.py env:=blue map_path:=/path/to/map.pcd
  ros2 launch localization_bringup localization.launch.py                 # env なし：NDT と EKF をコードの既定値で起動するだけ
  ros2 launch localization_bringup lidarslam.launch.py env:=blue          # SLAM（matcher:=lidarslam を付けてこのファイルを読む）

環境は param/<名前>/ のフォルダ（env:=<名前>。フォルダのパスでもよい。追加したら colcon build）：
  env.yaml          launch が読む：use_sim_time、起動するノード（nodes.<ノード名>: true / false）、静的 TF（static_tfs）
  <ノード名>.yaml   そのノードのパラメータ（ROS のパラメータファイル。ekf_localizer・pcl_localization・whill_odometry）
  lidarslam.yaml    lidarslam の scan_matcher・graph_based_slam のパラメータ（matcher:=lidarslam のとき）
yaml が無いノード、yaml に書いていない項目は、そのノードのコードの既定値になる。env を指定しなければ、どのノードにも yaml を渡さない。

matcher（ndt / lidarslam）：点群を合わせるノード。lidarslam のときは pcl_localization の代わりに scan_matcher と graph_based_slam を起動する
（env.yaml の nodes.pcl_localization は見ない）。EKF は scan_matcher の slam_pose（NDT の ndt_pose と同じ型）を購読する。

launch が入れるのは use_sim_time（全ノード）と、NDT・scan_matcher の publish_tf（いつも false）・use_tf_init_guess（EKF を起動するときは true）、
map_path（launch 引数を付けたとき）だけ。TF map -> base は EKF が出す。EKF を起動しないときは pose_to_tf（このパッケージ）が
NDT・scan_matcher の解を取付で base に直して出す（フレームの名前は ekf_localizer.yaml の値）。
トピックは各ノードの名前のままでつながる。付け替えるのは ekf_localizer の odom（→ /wheelchair/odom）・ndt_pose（lidarslam のとき → /slam_pose）、
NDT の pcl_pose（→ /current_pose。RViz と、研究室の MATLAB（whill_common_matlab の CR）の実機モードが購読する）、
scan_matcher の input_cloud（→ /velodyne_points）、pose_to_tf の pose（→ /ndt_pose か /slam_pose）だけ。
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

# env.yaml の nodes に書けるノードと、env を指定しないとき（nodes を書かないとき）に起動するもの
NODES = ('whill_odometry', 'pcl_localization', 'ekf_localizer', 'rviz')
DEFAULT_NODES = ('pcl_localization', 'ekf_localizer')
MATCHERS = ('ndt', 'lidarslam')
# matcher ごとの、解のトピックと RViz の設定（パッケージ、ファイル）
POSE_TOPIC = {'ndt': '/ndt_pose', 'lidarslam': '/slam_pose'}
RVIZ = {'ndt': ('pcl_localization_ros2', 'localization2.rviz'), 'lidarslam': ('lidarslam', 'mapping.rviz')}


def _param_dir():
    return os.path.join(get_package_share_directory('localization_bringup'), 'param')


def _env_dir(env):
    """env（名前かフォルダのパス）からフォルダのパスを返す。空なら None."""
    if not env:
        return None
    path = os.path.abspath(os.path.expanduser(env)) if os.sep in env else os.path.join(_param_dir(), env)
    if not os.path.isdir(path):
        available = sorted(d for d in os.listdir(_param_dir()) if os.path.isdir(os.path.join(_param_dir(), d)))
        raise RuntimeError(
            f'env "{env}" not found ({path}). Available: {", ".join(available)} '
            '(env is a folder param/<name>/ since 2026-11-11, not param/env/<name>.yaml)')
    return path


def _load_yaml(path):
    if not path or not os.path.isfile(path):
        return None
    with open(path) as f:
        return yaml.safe_load(f) or {}


def _node_params(env_dir, name, file=None):
    """param/<env>/<file か name>.yaml のパラメータ（ノード名か /** の節の ros__parameters）と、そのパス。無ければ ({}, None)."""
    path = os.path.join(env_dir, f'{file or name}.yaml') if env_dir else None
    data = _load_yaml(path)
    if data is None:
        return {}, None
    section = data.get(name) or data.get(f'/{name}') or data.get('/**') or {}
    return dict(section.get('ros__parameters') or {}), path


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
    """一時 yaml の置き場所（launch ごとに 1 つ。launch が終わるときに消す）."""
    global _TMP_DIR
    if _TMP_DIR is None:
        _TMP_DIR = tempfile.mkdtemp(prefix='localization_bringup_')
        atexit.register(shutil.rmtree, _TMP_DIR, ignore_errors=True)
    return _TMP_DIR


def _scoped(name, *dicts):
    """パラメータの辞書（後ろほど優先）を、ノード名の節の一時 yaml にしてそのパスを返す."""
    params = {}
    for d in dicts:
        params.update(d or {})
    with tempfile.NamedTemporaryFile(
            'w', prefix=f'{name}_', suffix='.yaml', dir=_tmp_dir(), delete=False) as f:
        yaml.safe_dump({name: {'ros__parameters': _normalize(params)}}, f)
        return f.name


def _arg_bool(context, name):
    """launch 引数の true / false。空なら None（env.yaml の値を使う）."""
    value = LaunchConfiguration(name).perform(context).lower()
    return None if value == '' else value == 'true'


def _launch_setup(context):
    env_dir = _env_dir(LaunchConfiguration('env').perform(context))
    env = _load_yaml(os.path.join(env_dir, 'env.yaml')) if env_dir else None
    if env_dir and env is None:
        raise RuntimeError(f'env.yaml is missing in {env_dir}')
    env = env or {}
    map_path = LaunchConfiguration('map_path').perform(context)
    matcher = LaunchConfiguration('matcher').perform(context)
    if matcher not in MATCHERS:
        raise RuntimeError(f'unknown matcher "{matcher}". Available: {", ".join(MATCHERS)}')

    nodes = env.get('nodes') or {n: n in DEFAULT_NODES for n in NODES}
    unknown = sorted(set(nodes) - set(NODES))
    if unknown:
        raise RuntimeError(f'unknown node in nodes of {env_dir}/env.yaml: {", ".join(unknown)}. Available: {", ".join(NODES)}')
    enabled = {n for n in NODES if nodes.get(n)}
    # launch 引数は env.yaml より優先
    for name, arg in (('ekf_localizer', 'use_ekf'), ('rviz', 'rviz')):
        value = _arg_bool(context, arg)
        if value is not None:
            (enabled.add if value else enabled.discard)(name)
    # lidarslam のときは NDT の代わりに scan_matcher・graph_based_slam を起動する
    use_slam = matcher == 'lidarslam'
    if use_slam:
        enabled.discard('pcl_localization')
    use_ekf = 'ekf_localizer' in enabled
    sim_time = {'use_sim_time': bool(env.get('use_sim_time', False))}
    # TF は EKF（起動しないときは pose_to_tf）が出す。NDT・scan_matcher は EKF の TF を初期値にする
    matcher_values = {'publish_tf': False, 'use_tf_init_guess': use_ekf}
    listed = [n for n in NODES if n in enabled] + (['scan_matcher', 'graph_based_slam'] if use_slam else [])
    if not use_ekf and (use_slam or 'pcl_localization' in enabled):
        listed.append('pose_to_tf')

    actions = [LogInfo(msg=f'[localization_bringup] env: {env_dir or "(none: code defaults)"}, matcher: {matcher}, '
                           f'nodes: {", ".join(listed)}')]

    def parameters(name, launch_values=None, file=None):
        """ノードに渡すパラメータ：param/<env>/<file か name>.yaml（無ければコードの既定値）に launch が入れる値を重ねた一時 yaml."""
        params, path = _node_params(env_dir, name, file)
        actions.append(LogInfo(msg=f'[localization_bringup] {name}: {path or "code defaults"}'))
        return [_scoped(name, params, launch_values, sim_time)]

    # ---- オドメトリ（実機。WHILL の左右輪の速度から /wheelchair/odom）----
    if 'whill_odometry' in enabled:
        actions.append(Node(
            package='whill_odometry', executable='whill_odometry_node',
            name='whill_odometry', output='screen', parameters=parameters('whill_odometry')))

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

    # ---- NDT ----
    if 'pcl_localization' in enabled:
        # TF は EKF か pose_to_tf に任せ、EKF と起動するときは初期値を EKF の TF（点群の時刻）から取る
        ndt_values = dict(matcher_values)
        if map_path:
            ndt_values['map_path'] = map_path
        ndt = LifecycleNode(
            package='pcl_localization_ros2', executable='pcl_localization_node',
            name='pcl_localization', namespace='', output='screen',
            parameters=parameters('pcl_localization', ndt_values),
            # NDT の解（RViz と、研究室の MATLAB（whill_common_matlab の CR）の実機モードが購読する）
            remappings=[('pcl_pose', '/current_pose')])
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

    # ---- SLAM（lidarslam_ros2）----
    if use_slam:
        actions += [
            # 逐次の位置合わせと地図作り。解は /current_pose（PoseStamped。MATLAB CR の sensor 4）と /slam_pose（EKF へ）
            Node(
                package='scanmatcher', executable='scanmatcher_node', name='scan_matcher', output='screen',
                parameters=parameters('scan_matcher', matcher_values, file='lidarslam'),
                remappings=[('input_cloud', '/velodyne_points')]),
            # ループ閉じ込みと地図の保存（サービス /map_save。コマンド savemap）。map.pcd は launch の作業ディレクトリに書く
            Node(
                package='graph_based_slam', executable='graph_based_slam_node', name='graph_based_slam',
                output='screen', parameters=parameters('graph_based_slam', file='lidarslam')),
        ]

    # ---- EKF ----
    if use_ekf:
        remappings = [('odom', '/wheelchair/odom')]   # オドメトリ（実機は whill_odometry、Gazebo は gazebo_bridge）
        if use_slam:
            remappings.append(('ndt_pose', '/slam_pose'))
        actions.append(Node(
            package='ekf_localizer', executable='ekf_localizer_node',
            name='ekf_localizer', output='screen', parameters=parameters('ekf_localizer'),
            remappings=remappings))
    elif use_slam or 'pcl_localization' in enabled:
        # EKF を起動しないとき：NDT・scan_matcher の解（map -> LiDAR）を取付で map -> base の TF に直す。フレームは ekf_localizer.yaml と同じ
        ekf_params, _ = _node_params(env_dir, 'ekf_localizer')
        frames = {k: ekf_params[k] for k in ('map_frame_id', 'base_frame_id', 'lidar_frame_id') if k in ekf_params}
        actions.append(Node(
            package='localization_bringup', executable='pose_to_tf.py', name='pose_to_tf', output='screen',
            parameters=[{**frames, **sim_time}], remappings=[('pose', POSE_TOPIC[matcher])]))

    # ---- RViz（/tf をそのまま読む。点群はその stamp の時刻の TF、視点の Target Frame base_link は最新の値）----
    # lidarslam のときは lidarslam の mapping.rviz（/map・/modified_map・/path・/current_pose）
    if 'rviz' in enabled:
        rviz_package, rviz_file = RVIZ[matcher]
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', os.path.join(get_package_share_directory(rviz_package), 'rviz', rviz_file)],
            parameters=[sim_time]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'env', default_value='',
            description='Environment (param/<env>/: gazebo, red, blue, white) or path to an env folder. '
                        'Empty: NDT and EKF with the code defaults'),
        DeclareLaunchArgument(
            'use_ekf', default_value='',
            description='Run the EKF (ekf_localizer) (overrides nodes.ekf_localizer in env.yaml; empty: use env.yaml). '
                        'If the EKF does not run, pose_to_tf publishes the TF from the NDT or lidarslam pose'),
        DeclareLaunchArgument(
            'map_path', default_value='',
            description='PCD map path (overrides map_path in pcl_localization.yaml)'),
        DeclareLaunchArgument(
            'matcher', default_value='ndt',
            description='Scan matcher: ndt (pcl_localization) or lidarslam (scan_matcher and graph_based_slam). '
                        'lidarslam.launch.py sets lidarslam'),
        DeclareLaunchArgument(
            'rviz', default_value='',
            description='Launch RViz (overrides nodes.rviz in env.yaml; empty: use env.yaml)'),
        OpaqueFunction(function=_launch_setup),
    ])

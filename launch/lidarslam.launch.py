"""lidarslam（scan_matcher・graph_based_slam）と EKF（ekf_localizer）を環境ごとの設定で起動する（コマンド lidarslam）.

  ros2 launch localization_bringup lidarslam.launch.py env:=gazebo
  ros2 launch localization_bringup lidarslam.launch.py env:=blue

localization.launch.py を matcher:=lidarslam で読むだけ（中身は共通）。環境は NDT のときと同じ param/<env>/ で、
SLAM のパラメータは param/<env>/lidarslam.yaml。地図は savemap（サービス /map_save）で保存する。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ARGS = (
    ('env', 'Environment (param/<env>/: gazebo, red, blue, white) or path to an env folder. '
            'Empty: scan_matcher, graph_based_slam and EKF with the code defaults'),
    ('use_ekf', 'Run the EKF (overrides nodes.ekf_localizer in env.yaml; empty: use env.yaml). '
                'If the EKF does not run, pose_to_tf publishes the TF from /slam_pose'),
    ('rviz', 'Launch RViz (lidarslam mapping.rviz) (overrides nodes.rviz in env.yaml; empty: use env.yaml)'),
)


def generate_launch_description():
    localization = os.path.join(
        get_package_share_directory('localization_bringup'), 'launch', 'localization.launch.py')
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value='', description=desc) for name, desc in ARGS] + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localization),
                launch_arguments={'matcher': 'lidarslam',
                                  **{name: LaunchConfiguration(name) for name, _ in ARGS}}.items()),
        ])

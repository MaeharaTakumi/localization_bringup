# localization_bringup

NDT（[lidar_localization](https://github.com/MaeharaTakumi/lidar_localization)、パッケージ名 `pcl_localization_ros2`）と
EKF（[ekf_localizer](https://github.com/MaeharaTakumi/ekf_localizer)）を、環境ごとの設定で起動します。

```bash
ros2 launch localization_bringup localization.launch.py env:=gazebo
ros2 launch localization_bringup localization.launch.py env:=real map_path:=/path/to/map.pcd
```

| 引数 | 既定値 | 説明 |
|---|---|---|
| `env` | `real` | 環境名（`param/env/<env>.yaml`）か、環境ファイルのパス |
| `use_ekf` | `true` | `false` なら EKF を起動せず、NDT が TF を配信する |
| `map_path` | （空） | PCD 地図のパス。環境ファイルの値より優先 |
| `rviz` | `true` | RViz を起動する |

## 構成

```
点群 ─▶ cloud_stamp_corrector ─▶ pcl_localization (NDT) ─▶ /ndt_pose ─▶ ekf_localizer ◀─ オドメトリ
          （時計が同期していない      ▲                                      │
           環境だけ起動）             └── TF map→LiDAR（点群の取得時刻）◀─────┘ TF map→base, /ekf_pose
```

## 環境を追加する

`param/env/<名前>.yaml` を 1 つ置いて `colcon build` すると `env:=<名前>` で使えます
（ビルドせずに試すなら `env:=/path/to/<名前>.yaml`）。項目は [param/env/real.yaml](param/env/real.yaml) のコメントを参照してください。

| 項目 | 内容 |
|---|---|
| `use_sim_time` | Gazebo なら `true` |
| `topics.points` / `topics.odom` | 点群とオドメトリのトピック（`topics.odom` は EKF の `odom_topic` に渡す） |
| `base_frame_id` / `lidar_frame_id` | EKF の状態・TF の基準フレームと、点群のフレーム |
| `cloud_stamp_correction` | `none`（stamp をそのまま使う）/ `estimate`（時計のずれを推定して直す）/ `receipt`（受信時刻にする） |
| `static_tfs` | launch が配信する静的 TF。URDF などが配信するものは書かない |
| `parameters.<ノード名>` | ノードのパラメータの上書き（地図・初期姿勢など） |

| 環境 | 内容 |
|---|---|
| `real` | 実機。点群は別 PC（時計が同期していない）なので `estimate` で直す。取付は launch が静的 TF で配信 |
| `gazebo` | シミュレーション。sim time。取付は URDF が配信。基準フレームは `base_footprint`。オドメトリの遅れ（`odom_delay` 0.09 s）と EKF の `Q` を設定 |

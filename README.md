# localization_bringup

NDT（[lidar_localization](https://github.com/MaeharaTakumi/lidar_localization)、パッケージ名 `pcl_localization_ros2`）と
EKF（[ekf_localizer](https://github.com/MaeharaTakumi/ekf_localizer) か
[robot_localization](https://github.com/cra-ros-pkg/robot_localization)）を、環境ごとの設定で起動します。
実機では [whill_odometry](https://github.com/MaeharaTakumi/whill_odometry) も起動します。

```bash
ros2 launch localization_bringup localization.launch.py env:=gazebo
ros2 launch localization_bringup localization.launch.py env:=real map_path:=/path/to/map.pcd
ros2 launch localization_bringup localization.launch.py env:=gazebo estimator:=robot_localization
```

| 引数 | 既定値 | 説明 |
|---|---|---|
| `env` | `real` | 環境名（`param/env/<env>.yaml`）か、環境ファイルのパス |
| `use_ekf` | `true` | `false` なら EKF を起動せず、NDT が TF を配信する |
| `estimator` | `ekf_localizer` | 使う EKF。`ekf_localizer` か `robot_localization`（下記） |
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
| `launch_odometry` | `true` なら whill_odometry を起動する。書かなければ `false` |
| `topics.motor_speed` | whill_odometry の入力（WHILL の左右輪の速度）。`launch_odometry: true` のとき必須。出力は `topics.odom` |
| `base_frame_id` / `lidar_frame_id` | EKF の状態・TF の基準フレームと、点群のフレーム |
| `cloud_stamp_correction` | `none`（stamp をそのまま使う）/ `estimate`（時計のずれを推定して直す）/ `receipt`（受信時刻にする） |
| `static_tfs` | launch が配信する静的 TF。URDF などが配信するものは書かない |
| `parameters.<ノード名>` | ノードのパラメータの上書き（地図・初期姿勢など）。`whill_odometry` も上書きできる |

| 環境 | 内容 |
|---|---|
| `real` | 実機。点群は別 PC（時計が同期していない）なので `estimate` で直す。取付は launch が静的 TF で配信。whill_odometry を起動 |
| `gazebo` | シミュレーション。sim time。取付は URDF が配信。基準フレームは `base_footprint`。オドメトリの遅れ（`odom_delay` 0.09 s）と EKF の `Q` を設定 |

## robot_localization（`estimator:=robot_localization`）

`ekf_localizer` の代わりに robot_localization の EKF を 2 段で使います。パラメータは
[param/robot_localization.yaml](param/robot_localization.yaml)（環境ファイルの `parameters.<ノード名>` で上書き）。

```
/ndt_pose（LiDAR）─▶ robot_localization_adapter ─▶ /robot_localization/ndt_pose_base（base、共分散 R_ndt）─▶ ekf_filter_node_map ─▶ TF map→odom, /odometry/global
オドメトリ ───────▶ robot_localization_adapter ─▶ /robot_localization/odom（共分散 R_odom）─┬▶ ekf_filter_node_map
                                                                                          └▶ ekf_filter_node_odom ─▶ TF odom→base, /odometry/local
/initialpose（LiDAR）─▶ robot_localization_adapter ─▶ 2 つの EKF の set_pose（base。odom も同じ値）
/odometry/global・TF ─▶ robot_localization_adapter ─▶ /ekf_pose（map→LiDAR。ekf_localizer と同じ型・QoS）
```

- robot_localization は姿勢の観測を base の姿勢として扱い（センサの取付を補正しない）、メッセージの共分散を使う
  （0 は 1e-9 になる）。そのため `robot_localization_adapter` が NDT の解を取付（静的 TF）で base に直し、
  NDT・オドメトリに `ekf_localizer` と同じ観測ノイズ（`R_ndt`・`R_odom`）を共分散として付ける。
- オドメトリは最初の NDT が届くまで流さない（robot_localization を NDT の姿勢で初期化するため）。
- `/ekf_pose` は `publish_trigger: "cloud"`（既定。点群が届くたびに、その stamp の時刻の TF map→LiDAR から作る）か
  `"timer"`（`/odometry/global` が届くたび。現在時刻まで予測した値）。
- `Q`・`P_init` は `ekf_localizer` の値（robot_localization の `process_noise_covariance` も時間でスケールされる）。
  NDT の遅れは `smooth_lagged_data`（`history_length` 1.0 s）で巻き戻して入れ直す。
- `ekf_localizer` の `odom_delay`・ゲート・ロックアウトに当たる機能は使っていない。
- 環境ファイルの `parameters.ekf_filter_node_*` では、15x15 の行列（`process_noise_covariance` など）を
  `<名前>_diag: [対角 15 個]` で書ける（launch が行列に直す）。
- `ekf_filter_node_map` は map→odom を出すとき現在時刻の odom→base を TF から引き、引けないと**警告なしに最新の TF で代用する**。
  odom 側の周期が遅いと、その分だけ回転中に yaw がずれる。Gazebo（`gazebo.yaml`）では odom 側を 200 Hz、
  `transform_timeout` を 0.05 s、`vyaw` のプロセスノイズを 1.0 にしている（左右 1 周 + 前進の走行で調整）。

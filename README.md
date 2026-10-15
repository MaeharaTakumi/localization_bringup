# localization_bringup

NDT（[lidar_localization](https://github.com/MaeharaTakumi/lidar_localization)、パッケージ名 `pcl_localization_ros2`）と
EKF（[ekf_localizer](https://github.com/MaeharaTakumi/ekf_localizer)）を、環境ごとの設定で起動します。
実機では [whill_odometry](https://github.com/MaeharaTakumi/whill_odometry) も起動します。

```bash
ros2 launch localization_bringup localization.launch.py env:=gazebo
ros2 launch localization_bringup localization.launch.py map_path:=/path/to/map.pcd             # 実機（車椅子 red。既定）
ros2 launch localization_bringup localization.launch.py env:=white map_path:=/path/to/map.pcd   # 実機（車椅子 white）
```

| 引数 | 既定値 | 説明 |
|---|---|---|
| `env` | `red` | 環境名（`param/env/<env>.yaml`。`gazebo`・`red`・`blue`・`white`）か、環境ファイルのパス |
| `use_ekf` | `true` | `false` なら EKF を起動せず、NDT が TF を配信する |
| `map_path` | （空） | PCD 地図のパス。環境ファイルの値より優先 |
| `rviz` | `true` | RViz を起動する |

## 構成

```
点群 ─▶ cloud_stamp_corrector ─▶ pcl_localization (NDT) ─▶ /ndt_pose ─▶ ekf_localizer ◀─ odom_covariance ◀─ オドメトリ
          （clock_synced が true      ▲                                      │      （倍率・観測ノイズ R・時刻）
           でないときだけ起動）       └── TF map→LiDAR（点群の取得時刻）◀─────┘ TF map→base, /ekf_pose
```

## 環境を追加する

`param/env/<名前>.yaml` を 1 つ置いて `colcon build` すると `env:=<名前>` で使えます
（ビルドせずに試すなら `env:=/path/to/<名前>.yaml`）。項目は [param/env/red.yaml](param/env/red.yaml) のコメントを参照してください。

| 項目 | 内容 |
|---|---|
| `use_sim_time` | Gazebo なら `true` |
| `topics.points` / `topics.odom` | 点群とオドメトリのトピック（`topics.odom` は EKF の `odom_topic` に渡す） |
| `launch_odometry` | `true` なら whill_odometry を起動する。書かなければ `false` |
| `topics.motor_speed` | whill_odometry の入力（WHILL の左右輪の速度）。`launch_odometry: true` のとき必須。出力は `topics.odom` |
| `base_frame_id` / `lidar_frame_id` | EKF の状態・TF の基準フレームと、点群のフレーム |
| `clock_synced` | 点群とオドメトリの時刻の決め方（両方そろえる）。`true`（既定）：stamp をそのまま使う / `false`：この PC で受信した時刻にする / `auto`：\|受信した時刻 − stamp\| が `clock_offset_threshold`（既定 0.1 s）以上なら受信した時刻、小さければ stamp（1 つずつ判断する）。点群は `cloud_stamp_corrector`（`true` 以外のとき起動）、オドメトリは `odom_covariance` が行う |
| `static_tfs` | launch が配信する静的 TF。URDF などが配信するものは書かない |
| `parameters.<ノード名>` | ノードのパラメータの上書き（地図・初期姿勢など）。`whill_odometry` も上書きできる。launch がノード名の節の一時ファイルにして渡すので、各パッケージの yaml のノード名の節より優先される |

| 環境 | 内容 |
|---|---|
| `red`（既定）・`blue`・`white` | 実機。車椅子（`topics.motor_speed` の `Drp5_whill_<色>`。ラズパイのホスト名）だけが違い、ほかは同じ（変えるときは 3 つとも直す）。取付は launch が静的 TF で配信。whill_odometry を起動。点群・オドメトリの stamp はそのまま使う（`clock_synced: true`。自己位置推定の PC とラズパイは時刻同期している） |
| `gazebo` | シミュレーション。sim time。取付は URDF が配信。基準フレームは `base_footprint`。オドメトリは `/wheelchair/odom`（Gazebo の diff_drive_controller の速度の移動平均を 1 にしたもの）。omega の倍率（`odom_scale`）と R・Q を設定 |

## robot_localization（廃止）

robot_localization（2 段 EKF）に切り替えて比べる機能（`estimator:=robot_localization`）は 2026-10-15 に廃止しました。
Gazebo の比較で ekf_localizer と同じか、その場の回転・曲線では ekf_localizer のほうが良かったためです。
コードは廃止する前の最後の commit（`robot_localization` ブランチの 23767ee）に残っています。

## オドメトリの観測ノイズ（`odom_covariance`）

`use_ekf:=true` のとき、オドメトリは `odom_covariance` を通して `/localization/odom` に中継され、v・omega に倍率（`odom_scale`）が掛かり、
twist の共分散に速さに応じた観測ノイズ R が入る（Nav2 AMCL の `odom_alpha` と同じ考え方）。パラメータは
[param/odom_covariance.yaml](param/odom_covariance.yaml)（環境ファイルの `parameters.odom_covariance` で上書き）。

```
R_v     = R_odom[0] + odom_noise_alpha[0]·v² + odom_noise_alpha[1]·ω²
R_omega = R_odom[1] + odom_noise_alpha[2]·v² + odom_noise_alpha[3]·ω²
```

- `ekf_localizer` は launch が `odom_covariance_source: message` にして `[0]`・`[35]` を R に使う。
- 時刻（`stamp_mode`）は launch が環境ファイルの `clock_synced` から入れる（`true` → `stamp`、`false` → `receipt`、`auto` → `auto`）。
- `odom_noise_alpha` がすべて 0 なら、固定の `R_odom`（`ekf_localizer` の `R_odom` と同じ値）で今までと同じ。
- `accel_noise_tau` [τ_v, τ_ω] で、加速度に応じた分 (τ·dv/dt)²・(τ·dω/dt)² も R に足せる（既定 0）。
- `odom_scale` [v, omega] をオドメトリの v・omega に掛ける（実効的な車輪間隔・半径が設定と違う分の補正。既定 [1, 1]）。
  Gazebo は omega 0.922（回転中、オドメトリの omega が真値の 1.08〜1.09 倍。実効的な車輪間隔 0.532〜0.535 m）。
- 速さ・加速度に応じた R は効かなかった（加速度に応じた R は、速度が変わる間オドメトリを信じなくなり悪化）。
- オドメトリの遅れの補正は `ekf_localizer` の `odom_delay`（stamp から引いて入れる）で行う。ここにも同じ `odom_delay` があったが、
  2026-10-15 に外した。Gazebo（`gazebo.yaml`）は `odom_delay: 0`：diff_drive_controller の速度の移動平均を 10（0.2 s）から 1（0.02 s）に
  した（Gazebo 側の `controller.yaml` の `velocity_rolling_window_size`、2026-10-12）ので、遅れの補正は要らない
  （0.2 s の移動平均のときは 0.09 s 戻していたが、動き出しが真値より早すぎた）。

### 車輪の角速度からのオドメトリ（`joint_state_odometry`、任意）

環境ファイルに `joint_state_odometry:`（`topic`・`left_wheel_joint`・`right_wheel_joint`・`wheel_radius`・`wheel_separation`）を書くと、
launch が `scripts/joint_state_odometry.py` を起動し、`sensor_msgs/JointState` の車輪の角速度から v = r(wR+wL)/2、
omega = r(wR-wL)/b を計算して `topics.odom` に出す（`topics.odom` は `/localization/wheel_odom` などにする）。
Gazebo で移動平均のかかったオドメトリの代わりに試したもの。実運用（`gazebo.yaml`・`red.yaml` などの実機の環境）では使わない。

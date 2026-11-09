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
| `use_ekf` | （空） | `true` / `false` で ekf_localizer と odom_covariance を起動する / しない（環境ファイルの `nodes.*.enable` より優先。空なら環境ファイルの値）。EKF を起動しないときは NDT が TF を配信する |
| `map_path` | （空） | PCD 地図のパス。環境ファイルの値より優先 |
| `rviz` | （空） | `true` / `false` で RViz を起動する / しない（環境ファイルの `nodes.rviz.enable` より優先。空なら環境ファイルの値）。RViz は `/tf` をそのまま読み、点群をその stamp の時刻の TF（ekf_localizer の予測値の補間。MATLAB・NDT の初期値と同じ）で表示する（2026-10-19〜26 は `/tf` を `tf_ndt_update` に付け替えていた） |

## 構成

```
点群 ─▶ pcl_localization (NDT) ─▶ /ndt_pose ─▶ ekf_localizer ◀─ odom_covariance ◀─ オドメトリ
              ▲                                      │      （倍率・観測ノイズ R・時刻）
              └── TF map→LiDAR（点群の時刻）◀────────┘ TF map→base, /ekf_pose
```

## 環境を追加する

`param/env/<名前>.yaml` を 1 つ置いて `colcon build` すると `env:=<名前>` で使えます
（ビルドせずに試すなら `env:=/path/to/<名前>.yaml`）。項目は [param/env/red.yaml](param/env/red.yaml) のコメントを参照してください。

| 項目 | 内容 |
|---|---|
| `use_sim_time` | Gazebo なら `true` |
| `nodes.<ノード名>.enable` | ノードを起動するか（`whill_odometry`・`joint_state_odometry`・`pcl_localization`・`odom_covariance`・`ekf_localizer`・`rviz`。全部書く）。止めたノードの出力は録画や別の PC から来るものとして、つなぎ方は変えない（EKF を起動しないときは NDT が TF を配信する） |
| `nodes.<ノード名>.topics` | ノードが購読・配信するトピック（ノードの中の名前: 実際の名前）。launch はそのまま remap に渡すので、つなぐときは配信側と購読側を同じ名前にする。書かなかった名前はノードの中の名前のまま。`/tf`・`/tf_static`・`/clock` は変えない。2026-10-22 に `topics`・`launch_odometry`・`joint_state_odometry` から移した（残っていると launch がエラーで止まる） |
| `base_frame_id` / `lidar_frame_id` | EKF の状態・TF の基準フレームと、点群のフレーム |
| `stamp_mode` | EKF の計算に使う、点群とオドメトリの時刻の決め方（両方そろえる。点群のトピック自体の stamp は変えない）。`stamp`（既定）：stamp をそのまま使う / `receipt`：この PC で受信した時刻にする / `auto`：\|受信した時刻 − stamp\| が `clock_offset_threshold`（既定 0.1 s）以上なら受信した時刻、小さければ stamp（1 つずつ判断する）。点群は NDT（`scan_time_source`。`stamp` は `message`、ほかは同じ名前。`/ndt_pose` の stamp と TF の初期値を引く時刻）、オドメトリは `odom_covariance`（`stamp_mode` に同じ値。`/localization/odom` の stamp）が行う。2026-11-09 に `cloud_stamp_corrector` をやめた。2026-10-22 に `clock_synced`（`true` / `false` / `auto`）から名前を変えた |
| `static_tfs` | launch が配信する静的 TF。URDF などが配信するものは書かない |
| `parameters.<ノード名>` | ノードのパラメータの上書き（地図・初期姿勢など）。`whill_odometry`・`joint_state_odometry` も上書きできる。launch がノード名の節の一時ファイルにして渡すので、各パッケージの yaml のノード名の節より優先される |

| 環境 | 内容 |
|---|---|
| `red`（既定）・`blue`・`white` | 実機。車椅子（`nodes.whill_odometry.topics.motor_speed` の `Drp5_whill_<色>`。ラズパイのホスト名）だけが違い、ほかは同じ（変えるときは 3 つとも直す）。取付は launch が静的 TF で配信。whill_odometry を起動。点群・オドメトリの stamp はそのまま使う（`stamp_mode: stamp`。自己位置推定の PC とラズパイは時刻同期している） |
| `gazebo` | シミュレーション。sim time。取付は URDF が配信。基準フレームは `base_footprint`。オドメトリは `/wheelchair/odom`（Gazebo の diff_drive_controller の速度の移動平均を 1 にしたもの）。omega の倍率（`odom_scale`）と R・Q を設定 |

## robot_localization（廃止）

robot_localization（2 段 EKF）に切り替えて比べる機能（`estimator:=robot_localization`）は 2026-10-15 に廃止しました。
Gazebo の比較で ekf_localizer と同じか、その場の回転・曲線では ekf_localizer のほうが良かったためです。
コードは廃止する前の最後の commit（`robot_localization` ブランチの 23767ee）に残っています。

## オドメトリの倍率と観測ノイズ

オドメトリは `odom_covariance` を通して `/localization/odom` に中継され、v・omega に倍率（`odom_scale`）が掛かり、時刻が決まる。
パラメータは環境ファイルの `parameters.odom_covariance`（書かなければ `scripts/odom_covariance.py` の既定値）。

観測ノイズ R は `ekf_localizer` が速さと加速度から計算する（2026-11-09 に `odom_covariance` から移した。メッセージの共分散は使わない）。
パラメータは `ekf_localizer` の `param/ekf.yaml`（全環境で実機 blue の値。環境ごとに変えるなら `parameters.ekf_localizer` に書く。
`parameters.odom_covariance` に R の項目が残っていると launch はエラーで止まる）。

```
R_v     = R_odom[0] + odom_noise_alpha[0]·v² + odom_noise_alpha[1]·ω² + (accel_noise_tau[0]·dv/dt)²
R_omega = R_odom[1] + odom_noise_alpha[2]·v² + odom_noise_alpha[3]·ω² + (accel_noise_tau[1]·dω/dt)²
```

- 時刻（`stamp_mode`）は launch が環境ファイルの `stamp_mode` をそのまま入れる。
- `odom_noise_alpha`（Nav2 AMCL の `odom_alpha` と同じ考え方）と `accel_noise_tau` がすべて 0 なら、固定の `R_odom`。
- `odom_scale` [v, omega] をオドメトリの v・omega に掛ける（実効的な車輪間隔・半径が設定と違う分の補正。既定 [1, 1]）。
  Gazebo は omega 0.922（回転中、オドメトリの omega が真値の 1.08〜1.09 倍。実効的な車輪間隔 0.532〜0.535 m）。
- 速さ・加速度に応じた R は効かなかった（加速度に応じた R は、速度が変わる間オドメトリを信じなくなり悪化）。
- オドメトリの遅れの補正は `ekf_localizer` の `odom_delay`（stamp から引いて入れる）で行う。ここにも同じ `odom_delay` があったが、
  2026-10-15 に外した。Gazebo（`gazebo.yaml`）は `odom_delay: 0`：diff_drive_controller の速度の移動平均を 10（0.2 s）から 1（0.02 s）に
  した（Gazebo 側の `controller.yaml` の `velocity_rolling_window_size`、2026-10-12）ので、遅れの補正は要らない
  （0.2 s の移動平均のときは 0.09 s 戻していたが、動き出しが真値より早すぎた）。

### 車輪の角速度からのオドメトリ（`joint_state_odometry`、任意）

環境ファイルの `nodes.joint_state_odometry.enable` を `true` にすると、launch が `scripts/joint_state_odometry.py` を起動し、
`sensor_msgs/JointState`（`joint_states`）の車輪の角速度から v = r(wR+wL)/2、omega = r(wR-wL)/b を計算して `odom` に出す
（`gazebo.yaml` では `/localization/wheel_odom`。`nodes.odom_covariance.topics.odom` も同じ名前にする）。パラメータ（`left_wheel_joint`・
`right_wheel_joint`・`wheel_radius`・`wheel_separation`）は `parameters.joint_state_odometry`。
Gazebo で移動平均のかかったオドメトリの代わりに試したもの。実運用（`gazebo.yaml`・`red.yaml` などの実機の環境）では使わない。

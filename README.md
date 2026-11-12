# localization_bringup

NDT（[lidar_localization](https://github.com/MaeharaTakumi/lidar_localization)、パッケージ名 `pcl_localization_ros2`）と
EKF（[ekf_localizer](https://github.com/MaeharaTakumi/ekf_localizer)）を、環境ごとの設定で起動します。
実機では [whill_odometry](https://github.com/MaeharaTakumi/whill_odometry) も起動します。
NDT の代わりに SLAM（[lidarslam_ros2](https://github.com/MaeharaTakumi/lidarslam_ros2) の scan_matcher・graph_based_slam）でも起動できます（`lidarslam.launch.py`）。

```bash
ros2 launch localization_bringup localization.launch.py env:=gazebo
ros2 launch localization_bringup localization.launch.py env:=blue map_path:=/path/to/map.pcd    # 実機（車椅子 blue）
ros2 launch localization_bringup localization.launch.py                                         # env なし：NDT と EKF をコードの既定値で起動するだけ
ros2 launch localization_bringup lidarslam.launch.py env:=blue                                   # SLAM（コンテナのコマンド lidarslam）
```

| 引数 | 既定値 | 説明 |
|---|---|---|
| `env` | （空） | 環境の名前（`param/<env>/`。`gazebo`・`red`・`blue`・`white`）か、環境のフォルダのパス。空なら NDT と EKF を、yaml を渡さずにコードの既定値で起動する（取付の TF・地図・sim time が無いので、推定として使うときは env を指定する。コンテナのコマンド `localization` の既定は red） |
| `use_ekf` | （空） | `true` / `false` で ekf_localizer を起動する / しない（`env.yaml` の `nodes` より優先。空なら `env.yaml` の値）。EKF を起動しないときは `pose_to_tf` が NDT・scan_matcher の解から TF map → base を配信する |
| `matcher` | `ndt` | `ndt`（pcl_localization）か `lidarslam`（scan_matcher・graph_based_slam）。`lidarslam.launch.py` はこれを `lidarslam` にして `localization.launch.py` を読むだけ（引数は `env`・`use_ekf`・`rviz`） |
| `map_path` | （空） | PCD 地図のパス。`pcl_localization.yaml` の値より優先 |
| `rviz` | （空） | `true` / `false` で RViz を起動する / しない（`env.yaml` の `nodes` より優先）。RViz は `/tf` をそのまま読み、点群をその stamp の時刻の TF（ekf_localizer の予測値の補間）で表示する。設定は NDT が pcl_localization_ros2 の `localization2.rviz`、lidarslam が lidarslam の `mapping.rviz` |

## 構成

```
点群 /velodyne_points ─▶ pcl_localization (NDT) ─▶ /ndt_pose ─▶ ekf_localizer ◀─ /wheelchair/odom ◀─ whill_odometry（実機）・Gazebo
                               ▲                                     │
                               └── TF map→LiDAR（点群の時刻）◀───────┘ TF map→base, /ekf_pose, /ekf_odom
```

トピックは各ノードの名前のままでつながる。launch が付け替えるのは ekf_localizer の `odom` → `/wheelchair/odom` と、
NDT の `pcl_pose` → `/current_pose`（RViz と、研究室の MATLAB（whill_common_matlab の CR）の実機モードが購読する名前）だけ
（lidarslam のときは、ほかに scan_matcher の `input_cloud` → `/velodyne_points`、ekf_localizer の `ndt_pose` → `/slam_pose`。EKF を起動しないときは
`pose_to_tf` の `pose` → `/ndt_pose` か `/slam_pose`）。

TF map → base は EKF が出す。NDT・scan_matcher は TF を出さず（launch がいつも `publish_tf: false` を入れる）、EKF の TF（点群の時刻の
map → LiDAR）を初期値にする（`use_tf_init_guess`）。EKF を起動しないときは `pose_to_tf`（`scripts/pose_to_tf.py`）が、NDT・scan_matcher の解
（map → LiDAR）に取付の逆を掛けた map → base を解の stamp で出す（フレームの名前は `ekf_localizer.yaml` の値。2026-11-12 から。それまでは NDT が自分で出していた）。

## SLAM（`lidarslam.launch.py`）

```
点群 ─▶ scan_matcher ─▶ /slam_pose ─▶ ekf_localizer ◀─ /wheelchair/odom
           ▲  │                            │
           │  └─ /map_array ─▶ graph_based_slam ◀─ savemap（/map_save → map.pcd・pose_graph.g2o）
           └── TF map→LiDAR（点群の時刻）◀──┘ TF map→base, /ekf_pose
```

- scan_matcher：逐次の位置合わせと地図作り（直近の部分地図 20 個に NDT で合わせ、1.5 m ごとに部分地図を足す）。解は `/slam_pose`
  （NDT の `/ndt_pose` と同じ型・QoS。map → LiDAR、stamp は点群の時刻）と `/current_pose`（PoseStamped。lidarslam のもとの出力）。
  `robot_frame_id` を LiDAR のフレーム（`velodyne`）にして、NDT と同じ LiDAR の姿勢を出す。
- graph_based_slam：ループ閉じ込みと地図の保存（`/modified_map`、サービス `/map_save`）。直した姿勢は scan_matcher と EKF には戻らない。
- 環境は NDT のときと同じ `param/<env>/`（`env.yaml`・`ekf_localizer.yaml`・`whill_odometry.yaml`）。SLAM のパラメータは `param/<env>/lidarslam.yaml`
  （scan_matcher と graph_based_slam の節。初期姿勢は `pcl_localization.yaml` と同じ値）。EKF の `R_ndt` も NDT のときと同じ。
- lidarslam パッケージ単体の launch（`ros2 launch lidarslam lidarslam.launch.py`、パッケージの `param/lidarslam.yaml`）もそのまま使える。
オドメトリの倍率（`odom_scale`）と時刻の決め方（`odom_stamp_mode`）は ekf_localizer が持つ（2026-11-11 に `odom_relay` ノードから移した）。

## 環境（`param/<env>/`）

環境は 1 つのフォルダ。追加するにはフォルダを写して `colcon build` する（ビルドせずに試すなら `env:=/path/to/<フォルダ>`）。

| ファイル | 中身 |
|---|---|
| `env.yaml` | launch が読む（ROS のパラメータファイルではない）：`use_sim_time`、起動するノード `nodes`（`whill_odometry`・`pcl_localization`・`ekf_localizer`・`rviz` を true / false）、launch が配信する静的 TF `static_tfs`（URDF などが配信するものは書かない） |
| `ekf_localizer.yaml` | ekf_localizer のパラメータ（説明は ekf_localizer の README） |
| `pcl_localization.yaml` | NDT のパラメータ（説明は lidar_localization の README）。時刻の決め方 `stamp_mode` もここ |
| `lidarslam.yaml` | SLAM（`lidarslam.launch.py`）の scan_matcher と graph_based_slam のパラメータ（説明は lidarslam_ros2 の README） |
| `whill_odometry.yaml` | whill_odometry のパラメータ（実機だけ。車椅子の `motor_speed_ts` のトピック `input_topic_name` など） |

- `<ノード名>.yaml` は普通の ROS のパラメータファイル（`<ノード名>: ros__parameters:`）なので、`ros2 run ... --ros-args --params-file` にも使える。
- **yaml が無いノード、yaml に書いていない項目は、そのノードのコードの既定値になる**。各パッケージはパラメータファイルを持たない
  （2026-11-11 に `ekf.yaml`・`localization.yaml`・`odometry.yaml` をやめた）。どの環境も、全項目を書いておく
  （launch が入れる下の値は書かなくてよい。書いても launch の値で上書きする。`pcl_localization.yaml` は書いておらず、`lidarslam.yaml` は書いてある）。
- launch が入れる値は `use_sim_time`（`env.yaml` の値。全ノード）、NDT・scan_matcher の `publish_tf`（いつも false）・`use_tf_init_guess`（EKF を起動するときは true）、
  `map_path`（launch 引数を付けたとき）だけ。launch は yaml を読んでこれらを重ね、ノード名の節の一時 yaml にして渡す
  （整数と小数が混ざった数のリストは小数にそろえる。rcl は型の違う要素のリストを読めない）。
- 起動ログに、各ノードがどの yaml を読んだか（無ければ `code defaults`）が出る。効いた値は `ros2 param dump /<ノード名>` で確かめる。

| 環境 | 内容 |
|---|---|
| `red`・`white` | 実機。違うのは車椅子（`whill_odometry.yaml` の `input_topic_name`、`Drp5_whill_<色>`）だけ |
| `blue` | 実機。車椅子のほかに、取付（`env.yaml` の `static_tfs`）とオドメトリの倍率（`ekf_localizer.yaml` の `odom_scale`）も違う |
| `gazebo` | シミュレーション。sim time、取付は URDF（`static_tfs` は空）、基準フレームは `base_footprint`、オドメトリは Gazebo の `/wheelchair/odom`（移動平均 1）。omega の倍率 0.922、Q の速度 3 つは 0 |

実機に共通の設定を変えるときは red・blue・white の 3 つとも直す。

## 時刻

点群とオドメトリの時刻は、それぞれのノードで決める（`stamp`：メッセージの stamp / `receipt`：受け取った時刻 / `auto`：\|受け取った時刻 − stamp\| が閾値以上なら受け取った時刻）。

- 点群：`pcl_localization.yaml` の `stamp_mode`・`stamp_auto_threshold`。NDT が `/ndt_pose` の stamp と、初期値の TF を引く時刻に使う
  （点群のトピック自体の stamp は変えない）。EKF は `/ndt_pose` の stamp をそのまま使う（`/ndt_pose` を受け取った時刻は NDT の計算の遅れを含むので使わない）。
- オドメトリ：`ekf_localizer.yaml` の `odom_stamp_mode`・`odom_stamp_auto_threshold`（EKF が受け取った時刻）。実機の whill_odometry も
  `use_stamped_input`（true でラズパイの stamp）を持つ。
- 今はどの環境も `stamp`（自己位置推定の PC とラズパイは時刻同期している。Gazebo は sim time）。時計がずれている環境では両方を `receipt` か `auto` にする。

## robot_localization（廃止）

robot_localization（2 段 EKF）に切り替えて比べる機能（`estimator:=robot_localization`）は 2026-10-15 に廃止しました。
Gazebo の比較で ekf_localizer と同じか、その場の回転・曲線では ekf_localizer のほうが良かったためです。
コードは廃止する前の最後の commit（`robot_localization` ブランチの 23767ee）に残っています。

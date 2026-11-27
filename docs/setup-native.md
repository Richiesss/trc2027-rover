# ROS 2 Jazzy を直接使う開発手順（JetPack 7 / Ubuntu 24.04）

開発用機 `trc-dev`（JetPack 7.2.1 / Ubuntu 24.04 / ROS 2 Jazzy）で、Docker を使わずに `rover/` のパッケージをビルドして動かす手順です。
Ubuntu 24.04 の PC でも同じ手順で使えます。

> JetPack 6（Ubuntu 22.04）の機体で Jazzy を使う場合だけ、Docker の手順（[docker.md](docker.md)）を使います。

## パッケージ構成

| パッケージ | 役割 |
| --- | --- |
| `trc_gnss` | simpleRTK2B の NMEA を読み、`/gnss/fix`（NavSatFix）を配信。RTK の状態（Fix / Float / 単独）で共分散を切り替える |
| `trc_localization` | robot_localization（2段の EKF + navsat_transform）の設定と、向きの初期化ノード `heading_init`、実機なしで試すための `fake_sensors_node` |
| `trc_description` | センサの取付位置（URDF / TF）。**値は仮なので、機体ができたら実測して書き換える** |
| `trc_logging` | 大会提出用の制御履歴（時刻・位置・方位・制御指令）を CSV に記録する `control_history_node` |
| `trc_perception` | LiDAR の点群から地面を除いて障害物を検出する `obstacle_detector_node`、前方が塞がったら前進を止める `safety_stop_node` |
| `trc_navigation` | 障害物をよけて目標へ向かう経路計画 `local_planner_node`（A* + DWA） |
| `trc_bringup` | 起動用の launch（センサ、位置推定、rosbag の記録） |

```
L2 ─► unilidar_sdk2 ─► /unilidar/cloud, /unilidar/imu
                              │
                              ▼
                    LiDARオドメトリ（KISS-ICP など）─► /lidar/odom ─┐
                                                                    ▼
simpleRTK2B ─► trc_gnss ─► /gnss/fix ─► heading_init ─► navsat_transform ─► ekf_map ─► /odometry/global
                                                                    ▲                      （map 座標の位置）
                                         /unilidar/imu ─► ekf_odom ─┘ ─► /odometry/local
```

## 1. ROS 2 と道具の確認（初回のみ）

```bash
ls /opt/ros                                   # jazzy と出ればOK
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc

sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-serial \
  ros-jazzy-robot-localization ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-rosbag2-storage-mcap
sudo rosdep init 2>/dev/null || true           # 2回目以降はエラーになるが問題ない
rosdep update
```

**通信の範囲を決める（重要）**：何も設定しないと、ROS 2 は同じネットワーク（大学の LAN など）にいる他の機体のノードと自動でつながり、
他人のロボットの `/gnss/fix` などが混ざります（実際に、別の機体の `/gnss_node` が見えて、試験結果がおかしくなったことがある）。
チーム専用のドメイン ID を決め、開発中は同じ機体の中だけで通信するようにします。

```bash
echo 'export ROS_DOMAIN_ID=27' >> ~/.bashrc                      # チーム専用の番号（Docker の設定と同じ）
echo 'export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' >> ~/.bashrc  # 同じ機体の中だけでつながる
source ~/.bashrc
ros2 daemon stop
```

本番用 Jetson など、別の機体と通信させたいときは `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` にし、両方の機体で同じ `ROS_DOMAIN_ID` にします。
`ros2 node list` に身に覚えのないノードが出たら、まずこの設定を確認してください（`ros2 node list --no-daemon` で、その時点で実際に見えているノードだけを表示できる）。

GNSS をつなぐ機体では、シリアルポートの権限も付けておきます（反映には再ログイン）。

```bash
sudo usermod -aG dialout $USER
```

## 2. ビルド

リポジトリのフォルダがそのまま ROS 2 のワークスペースになります（`rover/` の中のパッケージを自動で探します）。

```bash
cd ~/trc2027/trc2027-rover
rosdep install --from-paths rover --ignore-src -y   # 足りない依存を入れる
colcon build --symlink-install
source install/setup.bash
```

毎回 `source` しなくて済むように：

```bash
echo '[ -f ~/trc2027/trc2027-rover/install/setup.bash ] && source ~/trc2027/trc2027-rover/install/setup.bash' >> ~/.bashrc
```

テスト（NMEA の解析と向きの計算）：

```bash
colcon test && colcon test-result --verbose
```

## 3. 実機なしで位置推定を試す

にせのセンサ（`fake_sensors_node`）で、位置推定の一式が動くか確認できます。ターミナルを2つ使います。

```bash
# ターミナル1
ros2 launch trc_bringup localization.launch.py

# ターミナル2（にせのローバーが北向きにスタートし、12秒まっすぐ進んでから曲がる）
ros2 run trc_localization fake_sensors_node
```

ターミナル1に次のように出れば、GNSS とオドメトリから向きが決まっています。

```
[heading_init]: 向きが決まりました: 東から反時計回りに 90.x 度（オドメトリとのずれ 89.x 度）
```

推定位置（`/gps/filtered`）と本当の位置（`/fake/truth`）を見比べられます。

```bash
ros2 topic echo /gps/filtered --field latitude
ros2 topic echo /fake/truth --field latitude
```

スタートの向きを変えて試す：`ros2 run trc_localization fake_sensors_node --ros-args -p heading_deg:=200.0`

### ダミーの点群で、LiDAR オドメトリまで試す

`fake_sensors_node` は、支柱・岩・砂山を置いた仮想フィールドを、ローバーから見た点群（`/unilidar/cloud`）も出せます。
これを KISS-ICP に入れて、「点群 → LiDAR オドメトリ → GNSS との融合」まで一通り試せます。
先に「4. 実機のセンサを動かす」の手順で KISS-ICP をビルドしておきます。

```bash
source ~/trc2027/deps_ws/install/setup.bash          # KISS-ICP
source ~/trc2027/trc2027-rover/install/setup.bash
ros2 launch trc_bringup sim_test.launch.py
```

別のターミナルで、推定位置と本当の位置を見比べます（小数点以下 6 桁目まで揃えば数 cm の誤差）。

```bash
ros2 topic echo /gps/filtered --field latitude
ros2 topic echo /fake/truth --field latitude
```

GNSS が途切れたときに、LiDAR オドメトリだけで位置を保てるかも試せます（25 秒後から 10 秒間 GNSS を止める）。

```bash
ros2 launch trc_bringup sim_test.launch.py gnss_outage_start:=25.0 gnss_outage_duration:=10.0
```

開発環境での結果（2026/11/26）：

| 条件 | 推定位置の誤差（平均 / 最大） |
| --- | --- |
| 通常（GNSS は RTK Fix 相当） | 3.1 cm / 8.3 cm |
| GNSS を 10 秒止めた区間 | 5〜6 cm（LiDAR オドメトリで保てた。GNSS が戻ると元に戻る） |

> **わかったこと**：地面の点が「LiDAR に対していつも同じ位置」に見えると、KISS-ICP はローバーが止まっていると誤認しました（4.9 m 進んだのに 0.7 m と推定）。
> 実際のフィールドも、平らで目印の少ない砂地では LiDAR オドメトリが弱くなるおそれがあります。GNSS と IMU で補う構成にしているのはこのためです。
> 実機でも、開けた場所での LiDAR オドメトリの精度を早めに確認しておくこと。

### シミュレーションを画面で見る（Mac のブラウザから）

`sim_test.launch.py` は `foxglove_bridge`（ポート 8765）も起動します。trc-dev に画面がなくても、Mac のブラウザで 3D 表示できます。

スクリプトを使うと、前回のノードの止め忘れ確認・環境の読み込み・ログ保存（`~/trc2027/logs/sim_<日時>.log`）までまとめて行います。

```bash
# trc-dev
./scripts/sim_view.sh                           # launch の引数もそのまま渡せる（例: gnss_outage_start:=25.0）
./scripts/sim_view.sh --stop                    # 残っているノードを止めるだけ

# Mac（リポジトリを clone した場所で）
./scripts/foxglove_tunnel.sh                    # ポート転送して Foxglove を開く
```

> 前回の `sim_test` などが別のターミナルで動いたままだと、古い TF が混ざって推定が NaN になります。`sim_view.sh` は起動前にこれを見つけて止めます。

手で行う場合：

```bash
# trc-dev（初回のみ）
sudo apt install ros-jazzy-foxglove-bridge

# Mac：ポート 8765 を trc-dev に転送（このターミナルは開いたままにする）
ssh -N -L 8765:localhost:8765 trc-dev
```

Mac のブラウザで [Foxglove](https://app.foxglove.dev)（無料アカウント）を開き、「Open connection」→「Foxglove WebSocket」→ `ws://localhost:8765` に接続します。
（Lichtblick でも同じ手順で接続できるはずですが、foxglove_bridge 3.x の新しい通信方式に未対応の版だとつながりません。つながらなければ Foxglove を使う）

表示の設定：

| パネル | 設定 | 見えるもの |
| --- | --- | --- |
| 3D | Display frame を `map` | 下の全部 |
| 3D | `/fake/world` | 仮想フィールド（支柱・岩・壁・地面）。灰色の点 |
| 3D | `/unilidar/cloud` | LiDAR が今見ている点群（ローバーと一緒に動く） |
| 3D | `/fake/truth_path` | 本当の軌跡 |
| 3D | `/localization/path` | 推定した軌跡（これが本当の軌跡に重なれば OK） |
| 3D | `/gnss/path` | GNSS の測位点（GNSS を止めた区間は途切れる） |
| 3D | TF | `map → odom → base_link → lidar_link` の座標軸 |
| Plot | `/fake/position_error.data` | 推定位置の誤差 [m]。0.1 以下なら 10 cm 以内 |

確認ポイント：

- `/unilidar/cloud` の点が `/fake/world` の支柱や岩にぴったり重なる → LiDAR オドメトリと向きの推定が合っている
- `/localization/path` が `/fake/truth_path` に重なる → 位置推定が合っている
- `gnss_outage_start:=25.0` で起動すると、`/gnss/path` が途切れても `/localization/path` が本当の軌跡に沿って伸び続ける

画面が不要なときは `foxglove:=false` で bridge を起動しません。

### 障害物を見つけて止まる

`sim_test` は障害物検出（`obstacle_detector_node`）と安全停止（`safety_stop_node`）も起動します。
にせのローバーは速度指令（`/cmd_vel`）で動き、指令は次の順に流れます。

```
scripted_driver（決まった道筋：直進 12 秒 → 左に曲がり続ける）
   │ /cmd_vel_nav
   ▼
safety_stop ◀── /obstacles/stop ── obstacle_detector ◀── /unilidar/cloud
   │ /cmd_vel（前方に障害物があれば前進を 0 に。回転・後退は通す）
   ▼
fake_sensors（にせのローバー）
```

```bash
./scripts/sim_view.sh obstacle_ahead:=6.0                 # 6 m 先に岩。手前で止まり、曲がる指令が来たら向きを変えて避ける
./scripts/sim_view.sh obstacle_ahead:=6.0 safety:=false   # 止めずに通り抜ける（比べる用）
```

6 m 先に岩を置いたときの動き（2026/11/26、開発環境）：

| 時刻 | 動き |
| --- | --- |
| 0〜9 秒 | 直進。前方の距離が 0.5 m/s で縮む |
| 約 9 秒 | 岩の 1.5 m 手前（ローバーは 3.7 m 進んだ所）で停止 |
| 12 秒 | 運転手が「曲がれ」を出す → 前進は止めたまま、その場で向きを変える |
| 約 15 秒 | 前方が 1.8 m 以上空いたので前進を再開。岩をよけて走り続ける |

| トピック | 中身 |
| --- | --- |
| `/obstacles/points` | 障害物とみなした点（Foxglove では赤など目立つ色に） |
| `/obstacles/ground` | 地面とみなした点 |
| `/obstacles/corridor` | 前方の帯（幅 1 m・長さ 5 m）。緑：空いている／赤：止まるべき |
| `/obstacles/front_distance` | 帯に入った障害物までの距離 [m]（何もなければ 5.0） |
| `/obstacles/stop` | 1.5 m より近いと true。一度止まったら 1.8 m より空くまで true（ばたつき防止） |
| `/safety/active` | 安全停止が前進を止めているとき true |

地面の見分け方：水平面を 40 cm 四方のマスに分け、周りのマスの一番低い点から地面の高さを求めます（勾配 30 % までは坂として許す）。そこから 15 cm 以上高い点が障害物です。
仮想フィールドでの結果：20 m 以内の障害物の点を 96 % 以上検出、地面を障害物と間違えたのは 0 %。処理は 1 回 数 ms。

安全停止は、障害物の情報が 0.5 秒以上来ないときも前進を止めます（障害物検出が落ちたときに走り続けないように）。

> 実機では、走行制御が `/cmd_vel_nav` に指令を出し、`perception.launch.py safety:=true` で `/cmd_vel`（モータ）へ渡す構成を想定しています。モータ側の担当と、トピック名を合わせること。
> 取付位置（URDF）が合っていないと地面が傾いて見えて誤検出が増えます。本番の砂山や段差でしきい値（`height_threshold`、`max_slope`）の調整も必要です。

### 障害物をよけて目標へ向かう

`sim_test` の運転手は、既定で経路計画（`trc_navigation/local_planner_node`）です。目標（ウェイポイント）へ向かって、障害物をよけながら進みます。

```bash
./scripts/sim_view.sh                                         # 18 m 先（途中に長い壁）へ
./scripts/sim_view.sh 'waypoints:=18,-10;18,0;0,-10;0,0'      # 4 地点を回る
./scripts/sim_view.sh driver:=script obstacle_ahead:=6.0      # これまでの決まった道筋（止まるだけ）
```

ウェイポイントは map 座標です（原点はスタート地点、x はスタート時の前方、y は左）。Foxglove の 3D パネルの「Publish pose」で `/goal_pose` に送ると、その場で目標を変えられます。

2 段構えで動きます。

1. **地図の上で道筋を探す（1 秒ごと）**：見つけた障害物を map 座標の格子地図（25 cm 四方）に書きため、A* で目標までの道筋を探す。障害物はローバーの半径＋余裕（0.7 m）だけ太らせ、近くは通りにくくして真ん中を通る。地面が見えたマスは記録を減らすので、位置のずれで間違って書いた障害物は消えていく
2. **目の前をよけて進む（10 Hz）**：道筋の 2 m 先を目指して、DWA で「速さ × 曲がり方」の候補を 4 秒先まで予測し、障害物に近づく・安全停止に引っかかるものを捨てて、進み具合・向き・余裕・速さの点数で選ぶ

前が塞がっているとき（安全停止がかかったときを含む）は、その場で同じ向きに回り、前が 1.8 m 以上空いた状態が続いてから進み始めます。

| トピック | 中身 |
| --- | --- |
| `/planner/global_path` | 地図の上で探した道筋（map） |
| `/planner/map` | 障害物の地図（OccupancyGrid、map） |
| `/planner/trajectory` | DWA が選んだ動きの予測（base_link） |
| `/planner/waypoints` | 残りのウェイポイント |

trc-dev での結果（2026/11/26）：4 地点（約 60 m）を約 3 分で回り切り、安全停止は 1 回。

> 注意：開発用サンドボックス（2 コア）では KISS-ICP の処理が 1〜2 秒遅れ、その場での回転が多いと `/odometry/global` の向きが大きくずれることがあった（GNSS を外すと起きない）。trc-dev では問題なく走れているが、実機の Jetson でも CPU に余裕があるか確認すること。

### 色々な状況を試す（シナリオ試験）

GNSS の劣化、LiDAR の途切れ、狭いすき間や袋小路などを順に自動で走らせ、結果を表にまとめます。
1 つ 1〜5 分、全部で 20〜30 分かかります（画面は出さない）。

```bash
./scripts/run_scenarios.sh                     # 全部
./scripts/run_scenarios.sh --only gap,narrow   # 一部だけ
./scripts/run_scenarios.sh --list              # 一覧
```

| シナリオ | 内容 | 合格の目安 |
| --- | --- | --- |
| `baseline` | RTK Fix のまま 18 m 先へ（途中に長い壁） | 到着・接触なし・位置誤差 0.5 m 以下 |
| `heading_200` | スタートの向きが南西 | 同上 |
| `gnss_outage_30` | GNSS が 15 秒後から 30 秒途切れる | 位置誤差 1.0 m 以下 |
| `gnss_float` | 10 秒後から 40 秒 RTK Float（誤差 30 cm） | 位置誤差 1.0 m 以下 |
| `gnss_single` | 10 秒後から 30 秒 単独測位（誤差 2.5 m） | 位置誤差 3.0 m 以下 |
| `gnss_jump` | 25 秒後に 1 回だけ 5 m 跳ぶ（RTK Fix のふり＝マルチパス） | 位置誤差 1.0 m 以下 |
| `lidar_drop` | LiDAR が 20 秒後から 3 秒止まる | 到着 |
| `imu_bias` | IMU の回転の速さが 0.01 rad/s ずれている | 到着 |
| `gap` | 幅 2.0 m のすき間を抜ける | 到着・接触なし |
| `narrow` | すき間 1.0 m（通れない）→ 壁の端を回る | 到着・接触なし |
| `dead_end` | まっすぐ行くと袋小路（U 字）→ 回り込む | 到着・接触なし |
| `clutter` | 岩の間を抜ける | 到着・接触なし |
| `square` | 4 地点を回る（約 60 m） | 到着 |

結果は `~/trc2027/logs/scenarios_<日時>/results.md`（表）と `results.csv` に、各シナリオのログと一緒に保存されます。
「接触」は、本当の通り道が障害物にローバーの半径（0.45 m）より近づいたことを表します。

個別のトラブルは `sim_test.launch.py` / `sim_view.sh` の引数でも起こせます（画面で見ながら確かめるとき）。

```bash
./scripts/sim_view.sh layout:=dead_end waypoints:=14,0
./scripts/sim_view.sh gnss_events:="10:40:float; 60:10:outage" gnss_jump:=25:5
./scripts/sim_view.sh lidar_drop:=20:3 imu_bias:=0.01
```

## 4. 実機のセンサを動かす

### GNSS

```bash
ros2 launch trc_bringup sensors.launch.py gnss_port:=/dev/ttyACM0
ros2 topic echo /gnss/quality      # 4 = RTK Fix、5 = RTK Float、1 = 単独測位
```

### LiDAR（ドライバと LiDAR オドメトリ）

外部のパッケージは、別のワークスペース `~/trc2027/deps_ws` にまとめます（自分たちのコードと混ぜないため）。

```bash
mkdir -p ~/trc2027/deps_ws/src && cd ~/trc2027/deps_ws/src
git clone https://github.com/unitreerobotics/unilidar_sdk2.git   # L2 のドライバ（ROS 2 用のフォルダがある）
git clone https://github.com/PRBonn/kiss-icp.git                 # LiDAR オドメトリ
cd ~/trc2027/deps_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

- unilidar_sdk2 の ROS 2 パッケージの場所や、LiDAR の IP の設定方法は、リポジトリの README を確認してください（点群は `/unilidar/cloud`、IMU は `/unilidar/imu` が初期値）。
- KISS-ICP は apt では入らないので、ソースからビルドします（依存ライブラリはビルド中に自動でダウンロードされる。初回は数分かかる）。
- 毎回使うなら `~/.bashrc` に追加しておきます：`echo 'source ~/trc2027/deps_ws/install/setup.bash' >> ~/.bashrc`（trc2027-rover の行より**前**に書く）

KISS-ICP は、位置推定に合う設定（`trc_localization/config/kiss_icp.yaml`）で起動します。

```bash
ros2 launch trc_localization lidar_odometry.launch.py    # 出力は /kiss/odometry
```

この launch ファイルで、次の設定を済ませています（KISS-ICP をそのまま起動すると合わないので注意）。

- 出力の座標系を `odom`、ロボット側を `base_link` にする（`base_link` からの取付位置は `trc_description` の TF を使う）
- **TF（odom→base_link）は配信しない**（EKF が配信するので、二重になるとおかしくなる）
- `use_sim_time` は既定で `false`（KISS-ICP 自体の launch ファイルは既定が `true` なので、実機では使わない）
- 点ごとの時刻による歪み補正（deskew）は切ってある。L2 の点群に時刻が入っているか確認してから試す

### 位置推定

```bash
ros2 launch trc_localization lidar_odometry.launch.py                        # ターミナル1
ros2 launch trc_bringup localization.launch.py lidar_odom_topic:=/kiss/odometry  # ターミナル2
ros2 topic echo /odometry/global   # map 座標の位置と向き
```

スタート直後に RTK Fix の状態で 3 m 以上走ると、向きが決まり、GNSS が地図に入り始めます。

## 5. 制御履歴を記録する（大会提出用）

大会規定では、時刻・位置・方位・制御指令を 1 Hz 以上で記録し、提出・説明する必要があります。
位置推定と一緒に起動すると、`~/trc2027/logs/control_history_日時.csv` に記録されます。

```bash
ros2 launch trc_bringup localization.launch.py log_history:=true                 # 2 Hz（既定）
ros2 launch trc_bringup localization.launch.py log_history:=true history_rate:=5 # 5 Hz
```

| 列 | 内容 |
| --- | --- |
| `time` | 記録した時刻（タイムゾーン付き） |
| `latitude` / `longitude` | 推定位置（`/gps/filtered`） |
| `map_x` / `map_y` | map 座標の位置 [m] |
| `heading_deg` | 方位（北が 0 度、時計回り）。向きが決まる前は空欄 |
| `speed_mps` | 速さ |
| `gnss_quality` | RTK の状態（4 = Fix） |
| `cmd_linear` / `cmd_angular` | 制御指令（`/cmd_vel`） |
| `cmd_age_sec` | 最後に制御指令を受け取ってからの時間 |
| `control_mode` | 運転の種類（`/control_mode` に送られた文字列。auto / command / remote など） |

- 1行ごとにファイルに書き出すので、途中で電源が落ちても、それまでの記録は残ります。
- データが来ていない項目は空欄になります（センサが止まっていた時間が後から分かる）。

## 6. 走行データを記録して、あとで再生する

```bash
# 記録（計測治具を押して歩くときなど）。~/trc2027/bags/sand_01 に保存
ros2 launch trc_bringup record.launch.py name:=sand_01
```

再生して位置推定を試す（何度でもやり直せる）：

```bash
ros2 launch trc_bringup localization.launch.py use_sim_time:=true
ros2 bag play ~/trc2027/bags/sand_01 --clock
```

LiDAR オドメトリも `use_sim_time:=true` で起動してください。

## これからやること（TODO）

- [ ] `trc_description` の取付位置を実測して書き換える
- [ ] 実データで `trc_gnss` の誤差（`sigma_*`）を調整する
- [ ] 坂（登坂ゾーン）を走るなら `two_d_mode` を見直す
- [ ] Jetson で EKF が `Failed to meet update rate` を出す場合は、`ekf.yaml` の `frequency` を 20 などに下げる
- [ ] 点群から障害物・路面を推定するノード

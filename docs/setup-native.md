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

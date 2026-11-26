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
- **LiDAR オドメトリの設定で必ず守ること**
  - 出力（nav_msgs/Odometry）の座標系を `odom`、ロボット側を `base_link` にする
  - **TF（odom→base_link）は配信しない**（EKF が配信するので、二重になるとおかしくなる）
  - 出力トピック名を `lidar_odom_topic` で渡す（例：`ros2 launch trc_bringup localization.launch.py lidar_odom_topic:=/kiss/odometry`）

### 位置推定

```bash
ros2 launch trc_bringup localization.launch.py
ros2 topic echo /odometry/global   # map 座標の位置と向き
```

スタート直後に RTK Fix の状態で 3 m 以上走ると、向きが決まり、GNSS が地図に入り始めます。

## 5. 走行データを記録して、あとで再生する

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
- [ ] 制御履歴（時刻・位置・方位・制御指令を 1 Hz 以上）の記録ノード
- [ ] 点群から障害物・路面を推定するノード

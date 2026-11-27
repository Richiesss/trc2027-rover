# trc2027-rover

鳥取ローバーチャレンジ2027（エントリー部門）に出場するローバーの、Jetsonで動かすソフトウェアです。
タスク・部品・手順書などのプロジェクト管理はNotionで行っています。

シミュレーション環境（開発用PCのDocker：ROS 2 Jazzy + Gazebo）は別リポジトリ
[trc2027-sim](https://github.com/Richiesss/trc2027-sim) にあります。
開発用PC（Ubuntu）で `rover/` をシミュレーションで試す方法は、trc2027-sim の [docs/dev-pc.md](https://github.com/Richiesss/trc2027-sim/blob/main/docs/dev-pc.md) を参照。

## ディレクトリ構成

```
trc2027-rover/
├── rover/      # 本番のコード（ROS 2 パッケージ。リポジトリ直下で colcon build する）
├── compose.yaml # Docker で動かす場合の設定（JetPack 6 の機体向け）
├── docker/     # 同上のイメージ（Dockerfile）
├── logs/       # 走行ログ・制御履歴の保存先（コミットしない）
├── sandbox/    # センサの動作確認用（自由に試す場所。本番には使わない）
│   ├── gnss/   #   GNSS（simpleRTK2B + NEO-D9C）の動作確認スクリプト
│   └── lidar/  #   LiDAR（Unitree L2）の動作確認
└── docs/       # 配線図や設定値など、コードと一緒に管理したい資料
```

| フォルダ | ルール |
| --- | --- |
| `rover/` | 動作確認で使えると分かったコードだけを整理して置く。ROS 2 Jazzy のパッケージにする |
| `sandbox/` | 何を試してもよい。うまくいった手順はNotionの技術メモに残す |
| `docs/` | 配線図、IPアドレスやポートなどの設定値、センサの取付位置など |

## 使い方

ROS 2 Jazzy を直接使います（開発用機 trc-dev：JetPack 7.2.1 / Ubuntu 24.04）。
ビルドと起動の手順は **[docs/setup-native.md](docs/setup-native.md)** を参照。

```bash
mkdir -p ~/trc2027 && cd ~/trc2027
git clone https://github.com/Richiesss/trc2027-rover.git
cd trc2027-rover
rosdep install --from-paths rover --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

| パッケージ（`rover/`） | 役割 |
| --- | --- |
| `trc_gnss` | GNSS 受信機の設定スクリプト（ドライバは大塚さんの [gnss_fix](https://github.com/tikutikuwa/gnss_fix) を使う） |
| `trc_localization` | GNSS と LiDAR オドメトリを融合した位置推定（robot_localization）と向きの初期化 |
| `trc_logging` | 大会提出用の制御履歴（時刻・位置・方位・制御指令）を CSV に記録 |
| `trc_perception` | LiDAR の点群から地面を除いて障害物を検出し、前方が塞がったら前進を止める |
| `trc_navigation` | 障害物をよけて目標へ向かう経路計画（地図 + A* + DWA） |
| `trc_description` | センサの取付位置（TF） |
| `trc_bringup` | 起動用の launch と、rosbag の記録 |

センサ単体の動作確認スクリプト（`sandbox/`）は、venv で動かします。

```bash
python3 -m venv ~/venvs/sandbox
source ~/venvs/sandbox/bin/activate
pip install -r ~/trc2027/trc2027-rover/sandbox/requirements.txt
```

JetPack 6（Ubuntu 22.04）の機体で Jazzy を使う場合は、Docker の手順（[docs/docker.md](docs/docker.md)）を使います。

## ハードウェア構成（予定）

| 役割 | 部品 | 接続 |
| --- | --- | --- |
| 計算機 | Jetson Orin Nano 8GB | — |
| LiDAR | Unitree 4D LiDAR L2 | USB（付属の UART 転送モジュール経由のシリアル）。**Jetson の Ethernet は通信系統用に空けておく** |
| GNSS | ArduSimple simpleRTK2B（ZED-F9P）+ NEO-D9C（みちびきCLAS）+ u-blox ANN-MB2 | USB（/dev/ttyACM0）。ドライバは [gnss_fix](https://github.com/tikutikuwa/gnss_fix) |
| モータードライバ | RoboClaw 2x15A（検討中） | USB / シリアル |
| モーター | goBILDA 5303 Saturn 188:1 ×2（検討中） | RoboClaw経由 |

## 注意

- 開発用機は JetPack 7.2.1（Ubuntu 24.04）に焼き直し、ROS 2 Jazzy を直接入れています（2026/11/26）。シミュレーション環境（trc2027-sim）と同じ Jazzy です。
- Pythonの仮想環境、ビルド結果（`build/` `install/` `log/`）、ログ（CSV・PCD・`logs/`）、`.env`、LiDARのSDKはコミットしないでください（`.gitignore` 参照）。

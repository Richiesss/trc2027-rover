# trc2027-rover

鳥取ローバーチャレンジ2027（エントリー部門）に出場するローバーの、Jetsonで動かすソフトウェアです。
タスク・部品・手順書などのプロジェクト管理はNotionで行っています。

シミュレーション環境（開発用PCのDocker：ROS 2 Jazzy + Gazebo）は別リポジトリ
[trc2027-sim](https://github.com/Richiesss/trc2027-sim) にあります。

## ディレクトリ構成

```
trc2027-rover/
├── rover/      # 本番のコード（ローバーに載せるJetsonで動かす）
├── sandbox/    # センサの動作確認用（自由に試す場所。本番には使わない）
│   ├── gnss/   #   GNSS（simpleRTK2B + NEO-D9C）の動作確認スクリプト
│   └── lidar/  #   LiDAR（Unitree L2）の動作確認
└── docs/       # 配線図や設定値など、コードと一緒に管理したい資料
```

| フォルダ | ルール |
| --- | --- |
| `rover/` | 動作確認で使えると分かったコードだけを整理して置く。ライブラリのバージョンを固定する |
| `sandbox/` | 何を試してもよい。うまくいった手順はNotionの技術メモに残す |
| `docs/` | 配線図、IPアドレスやポートなどの設定値、センサの取付位置など |

## Jetsonでの使い方

Jetsonの共通設定（SSH、LiDAR用の固定IP、シリアル権限など）は、Notionの技術メモ「Jetson環境構築手順」を参照。

```bash
cd ~
git clone https://github.com/Richiesss/trc2027-rover.git

# 動作確認用のPython環境（仮想環境はリポジトリの外に作る）
python3 -m venv ~/venvs/sandbox
source ~/venvs/sandbox/bin/activate
pip install -r ~/trc2027-rover/sandbox/requirements.txt
```

## ハードウェア構成（予定）

| 役割 | 部品 | 接続 |
| --- | --- | --- |
| 計算機 | Jetson Orin Nano 8GB | — |
| LiDAR | Unitree 4D LiDAR L2 | 有線LAN（LiDAR 192.168.1.62 → Jetson 192.168.1.2、UDP 6101→6201） |
| GNSS | ArduSimple simpleRTK2B（ZED-F9P）+ NEO-D9C（みちびきCLAS）+ u-blox ANN-MB2 | USB（/dev/ttyACM0） |
| モータードライバ | RoboClaw 2x15A（検討中） | USB / シリアル |
| モーター | goBILDA 5303 Saturn 188:1 ×2（検討中） | RoboClaw経由 |

## 注意

- Jetson（JetPack 6）は Ubuntu 22.04 ベースです。シミュレーション環境の ROS 2 Jazzy（Ubuntu 24.04）とはバージョンが異なるため、実機でROS 2を使うときはディストリビューションをそろえる方法を決める必要があります。
- Pythonの仮想環境、ログ（CSV・PCD）、LiDARのSDKはコミットしないでください（`.gitignore` 参照）。

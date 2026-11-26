# rover/ — 本番のコード（ROS 2 Jazzy パッケージ）

ローバーで動かす ROS 2 パッケージを置きます。リポジトリのフォルダで `colcon build` すると、ここのパッケージが自動でビルドされます。
手順は [docs/setup-native.md](../docs/setup-native.md) を参照。

| パッケージ | 役割 |
| --- | --- |
| `trc_gnss` | simpleRTK2B の NMEA → `/gnss/fix`（RTK の状態で共分散を切り替え） |
| `trc_localization` | robot_localization の設定（2段の EKF + navsat_transform）、向きの初期化 `heading_init`、試験用の `fake_sensors_node` |
| `trc_logging` | 大会提出用の制御履歴（CSV）を記録 |
| `trc_description` | センサの取付位置（URDF / TF）。値は仮 |
| `trc_bringup` | 起動用の launch（`sensors` / `localization` / `record`） |

- `sandbox/` で動作を確認できたものだけを、整理してここに移します。
- 大会で提出する制御履歴（時刻・位置・方位・制御指令を1Hz以上）を記録できるようにします。

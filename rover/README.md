# rover/ — 本番のコード

ローバーに載せるJetsonで動かすコードを置きます。

- `sandbox/` で動作を確認できたものだけを、整理してここに移します。
- Docker コンテナ（ROS 2 Jazzy）の中で動かします。コンテナ内では `/ws/src/rover` に見えます（[docs/docker.md](../docs/docker.md)）。
- 依存ライブラリは `docker/Dockerfile`（apt）と `docker/requirements.txt`（pip、バージョン固定）に書きます。
- 大会で提出する制御履歴（時刻・位置・方位・制御指令を1Hz以上）を記録できるようにします。

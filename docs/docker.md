# 本番環境（Docker + ROS 2 Jazzy）の構築手順

Jetson の上で、本番用の ROS 2 環境を Docker コンテナとして動かす手順です。

- **本番（`rover/`）は Docker**、**センサの動作確認（`sandbox/`）は venv**、と使い分けます。
- センサ単体の動作確認は、これまで通り Notion の技術メモ（GNSS・LiDARの動作確認）の手順で行います。
- Jetson の共通設定（SSH、LiDAR用の固定IP、シリアル権限など）は、Notion の技術メモ「Jetson環境構築手順」を先に済ませておきます。

## なぜ Docker にするのか

| 理由 | 内容 |
| --- | --- |
| ROS 2 のバージョンをそろえる | Jetson（JetPack 6）は Ubuntu 22.04 なので、そのままだと ROS 2 Humble になる。コンテナなら、シミュレーション（trc2027-sim）と同じ Jazzy を動かせる |
| 壊れても戻せる | 環境がおかしくなっても、イメージから作り直せば元に戻る |
| 大会当日の起動が楽 | `docker compose up -d` の1行で起動でき、自動起動も設定できる |

## 構成

```
trc2027-rover/
├── compose.yaml          # コンテナの起動設定
├── .env.example          # ROS_DOMAIN_ID などの設定例（.env にコピーして使う）
├── docker/
│   ├── Dockerfile        # イメージの中身（ROS 2 Jazzy + 必要なライブラリ）
│   └── requirements.txt  # apt にない Python ライブラリ（バージョン固定）
├── rover/                # → コンテナの /ws/src/rover に見える
└── logs/                 # → コンテナの /ws/logs に見える（走行ログ・制御履歴）
```

コードは Jetson 側のフォルダをコンテナにそのまま見せています（バインドマウント）。
そのため、Mac の VS Code（Remote-SSH）で `~/trc2027-rover/rover/` を編集すれば、すぐコンテナ内に反映されます。
コンテナを消しても、コードとログは消えません。

## 1. Docker を用意する（初回のみ）

Jetson に SSH でつないで確認します。

```bash
docker --version
docker compose version
```

どちらも表示されれば次へ進みます。表示されない場合はインストールします。

```bash
curl -fsSL https://get.docker.com | sh
```

`sudo` なしで `docker` を使えるようにします。設定したら、一度ログアウトして SSH でつなぎ直します。

```bash
sudo usermod -aG docker $USER
```

確認：

```bash
docker run --rm hello-world
```

### 保存先について（推奨）

イメージは数GBあり、SDカードだと読み書きが遅くなります。
M.2 の NVMe SSD を付けた場合は、Docker の保存先を SSD に移すと快適です（例：SSD を `/mnt/ssd` にマウントした場合）。

```bash
sudo mkdir -p /mnt/ssd/docker
echo '{ "data-root": "/mnt/ssd/docker" }' | sudo tee /etc/docker/daemon.json
sudo systemctl restart docker
docker info | grep "Docker Root Dir"   # /mnt/ssd/docker になっていればOK
```

> `/etc/docker/daemon.json` がすでにある場合は、上書きせずに `"data-root"` の行を書き足してください。

## 2. イメージを作る

```bash
cd ~/trc2027-rover
git pull
cp .env.example .env      # 初回のみ
mkdir -p logs             # 初回のみ
docker compose build
```

初回は ROS 2 のイメージのダウンロードとライブラリのインストールで、数十分かかることがあります。
2回目以降は、変更した部分だけが作り直されます。

## 3. 起動して中に入る

```bash
docker compose up -d          # バックグラウンドで起動
docker compose exec rover bash
```

プロンプトが `root@<Jetsonの名前>:/ws#` になれば、コンテナの中です。ROS 2 は自動で読み込まれています。

## 4. 動作確認

コンテナの中で実行します。

```bash
# ROS 2 が動くか
ros2 --help
printenv ROS_DISTRO        # jazzy と表示されればOK
printenv ROS_DOMAIN_ID     # 27（.env で決めた番号）

# GNSS が見えるか（simpleRTK2B を USB で接続しておく）
ls /dev/ttyACM*

# LiDAR に届くか（LiDAR の電源を入れ、LANでつないでおく）
ping -c 3 192.168.1.62

# Python ライブラリが入っているか
python3 -c "import serial, pynmea2, pyubx2, cv2; print('OK', cv2.__version__)"
```

センサのデータの中身まで確かめるのは、これまで通り Jetson 側の venv（`sandbox/`）で行います。
ここでは「コンテナからデバイスとネットワークが見えるか」だけを確認します。

## 5. ROS 2 パッケージをビルドする

`rover/` の中に ROS 2 パッケージを作ったら、コンテナ内でビルドします。

```bash
cd /ws
colcon build --symlink-install
source install/setup.bash
```

ビルド結果（`/ws/build`、`/ws/install`）はボリュームに残るので、コンテナを再起動してもビルドし直す必要はありません。

## 6. 止める・作り直す

```bash
exit                       # コンテナから出る（コンテナは動いたまま）
docker compose down        # コンテナを止めて消す（コード・ログ・ビルド結果は残る）
```

`docker/Dockerfile` や `docker/requirements.txt` を変えたときは、作り直します。

```bash
docker compose build
docker compose up -d
```

ビルド結果もまっさらにしたいときは、ボリュームごと消します。

```bash
docker compose down -v
```

## 7. 開発用PC（trc2027-sim）とつなぐとき

- Jetson と PC の `ROS_DOMAIN_ID` を同じ番号にそろえます（PC側も 27 にする）。
- 同じLAN（同じWi-Fiなど）にいれば、`ros2 topic list` でお互いのトピックが見えます。
- Tailscale 経由では ROS 2 の自動発見（マルチキャスト）が通らないため、そのままでは見えません。必要になったら別途設定します。

## 8. 大会用に自動起動させる（あとで）

`compose.yaml` を次のように変えると、Jetson の電源を入れるだけで本番プログラムが起動します。

```yaml
    restart: "unless-stopped"
    command: ["bash", "-lc", "source /ws/install/setup.bash && ros2 launch <パッケージ名> <launchファイル>"]
```

## よくあるトラブル

| 症状 | 原因と対処 |
| --- | --- |
| `permission denied ... docker.sock` | `docker` グループに入っていない。`sudo usermod -aG docker $USER` のあと、SSH をつなぎ直す |
| コンテナ内で `/dev/ttyACM0` が開けない | `compose.yaml` の `device_cgroup_rules` に、そのデバイスの番号がない。`ls -l /dev/ttyACM0` の「166, 0」のような左の数字を確認して書き足す |
| LiDAR の点群が届かない | Jetson 側の有線LANの固定IP（192.168.1.2）を確認する。コンテナは `network_mode: host` なので、Jetson で届けばコンテナでも届く |
| ビルド中に容量が足りなくなる | `docker system df` で使用量を確認し、`docker system prune` で不要なイメージを消す。根本的には SSD に移す |
| GPU（CUDA）や CSIカメラを使いたい | このイメージは公式の `ros:jazzy-ros-base` なので使えない。必要になったら、`docker/Dockerfile` の `FROM` を [jetson-containers](https://github.com/dusty-nv/jetson-containers) の ROS 2 イメージに差し替え、`compose.yaml` に `runtime: nvidia` を足す |

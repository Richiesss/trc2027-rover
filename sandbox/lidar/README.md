# sandbox/lidar/ — Unitree L2 の動作確認

公式SDK（unilidar_sdk2）はこのフォルダに各自で clone します。外部のリポジトリなのでコミットしません（`.gitignore` 済み）。

```bash
cd ~/trc2027-rover/sandbox/lidar
git clone https://github.com/unitreerobotics/unilidar_sdk2.git
cd unilidar_sdk2/unitree_lidar_sdk
mkdir build && cd build
cmake .. && make -j2
../bin/example_lidar_udp
```

- LiDAR本体のIP: 192.168.1.62 / データの送り先（Jetsonの有線LAN）: 192.168.1.2
- 点群を保存するなど、自作の確認用コードはこのフォルダ直下に置く
- 詳しい手順はNotionの技術メモ「LiDARの動作確認（Jetson）」

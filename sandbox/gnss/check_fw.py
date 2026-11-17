"""ZED-F9P のファームウェアのバージョンを表示する（CLASには 1.32 以上が必要）。

使い方:
    python check_fw.py [ポート名]   # 省略時は /dev/ttyACM0
出力の中の「FWVER=HPG 1.32」などを確認する。gnss_log.py は止めてから実行すること。
"""
import sys

from pyubx2 import POLL, UBXMessage, UBXReader
from serial import Serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"

ser = Serial(PORT, 38400, timeout=3)
ser.write(UBXMessage("MON", "MON-VER", POLL).serialize())
ubr = UBXReader(ser, protfilter=2)  # UBXだけ読む
for _ in range(50):
    _, msg = ubr.read()
    if msg and msg.identity == "MON-VER":
        print(msg)
        break
else:
    print("MON-VER の応答がありませんでした。ポート名と接続を確認してください。")

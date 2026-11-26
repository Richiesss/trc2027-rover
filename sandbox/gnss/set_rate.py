"""ZED-F9P の測位の更新頻度を変える（初期設定は 1 Hz）。

使い方:
    python set_rate.py [Hz] [ポート名]   # 省略時は 5 Hz、/dev/ttyACM0
    python set_rate.py 10
    python set_rate.py 1                 # 元に戻す

layers=7（RAM + BBR + Flash）で書き込むので、電源を切っても設定が残る。
ROS の gnss_node など、同じポートを使うプログラムは止めてから実行すること。

目安:
  - 位置推定には 5〜10 Hz が向いている（ローバーは 0.5 m/s 程度なので、5 Hz で 10 cm ごとに更新）
  - 使う衛星の種類が多いほど、上げられる上限は下がる。10 Hz で RTK が不安定なら 5 Hz に戻す
"""
import sys
import time

from pyubx2 import UBXMessage, UBXReader
from serial import Serial

HZ = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
PORT = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyACM0"

if not 0.5 <= HZ <= 20:
    sys.exit("Hz は 0.5〜20 の範囲で指定してください")

meas_ms = int(round(1000.0 / HZ))

ser = Serial(PORT, 38400, timeout=1)
ubr = UBXReader(ser, protfilter=2)


def wait_for(identity, timeout=3.0):
    end = time.time() + timeout
    while time.time() < end:
        _, msg = ubr.read()
        if msg is not None and msg.identity == identity:
            return msg
    return None


ser.write(UBXMessage.config_set(layers=7, transaction=0, cfgData=[
    ("CFG_RATE_MEAS", meas_ms),   # 測位の間隔 [ms]
    ("CFG_RATE_NAV", 1),          # 測位1回ごとに結果を出す
]).serialize())
ack = wait_for("ACK-ACK")
print(f"書き込み: 測位間隔 {meas_ms} ms（{1000.0 / meas_ms:.1f} Hz） → "
      f"{'受け付けられました' if ack else '応答なし（ACK-NAK か通信エラー）'}")

# 読み出して確認する
ser.write(UBXMessage.config_poll(layer=0, position=0,
                                 keys=["CFG_RATE_MEAS", "CFG_RATE_NAV"]).serialize())
cfg = wait_for("CFG-VALGET")
if cfg is None:
    print("確認のための読み出しに失敗しました")
else:
    meas = getattr(cfg, "CFG_RATE_MEAS", None)
    print(f"現在の設定: 測位間隔 {meas} ms（{1000.0 / meas:.1f} Hz）、NAV={getattr(cfg, 'CFG_RATE_NAV', None)}")

ser.close()

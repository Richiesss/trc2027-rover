"""CLAS（NEO-D9C → ZED-F9P）の補強信号が届いているかを診断する。

使い方:
    python check_clas.py [ポート名] [秒数]   # 省略時は /dev/ttyACM0、20 秒
ROS の gnss_node など、同じポートを使うプログラムは止めてから実行すること。

確認すること:
  1. ZED-F9P の UART2 の設定（9600bps・UBX 入力）が反映されているか
  2. UART2（NEO-D9C がつながっている口）にデータが届いているか（受信バイト数が増えるか）
  3. 届いたデータが補強信号として受け付けられているか（UBX-RXM-COR）
  4. 測位の状態（UBX-NAV-PVT の carrSoln: 0=なし, 1=Float, 2=Fix）

診断のために一時的に出力を増やすが、RAM にだけ書くので電源を切れば元に戻る（終了時にも戻す）。
"""
import sys
import time

from pyubx2 import POLL, UBXMessage, UBXReader
from serial import Serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0

PORT_NAMES = {0x0000: "I2C", 0x0101: "UART1", 0x0201: "UART2", 0x0300: "USB", 0x0400: "SPI"}
COR_PROTOCOL = {0: "不明", 1: "RTCM3", 2: "SPARTN", 29: "UBX-RXM-PMP", 30: "UBX-RXM-QZSSL6（CLAS）"}
COR_USED = {0: "不明", 1: "使われていない", 2: "使われている"}
COR_ERR = {0: "不明", 1: "エラーなし", 2: "エラーあり"}
CARR = {0: "なし（単独/DGNSS）", 1: "RTK Float", 2: "RTK Fix"}

ser = Serial(PORT, 38400, timeout=1)
ubr = UBXReader(ser, protfilter=2)  # UBX だけ読む


def wait_for(identity, timeout=3.0):
    end = time.time() + timeout
    while time.time() < end:
        _, msg = ubr.read()
        if msg is not None and msg.identity == identity:
            return msg
    return None


def uart2_rx(msg):
    """MON-COMMS から UART2 の受信バイト数を取り出す。"""
    for i in range(1, msg.nPorts + 1):
        if getattr(msg, f"portId_{i:02d}", None) == 0x0201:
            return getattr(msg, f"rxBytes_{i:02d}", None)
    return None


def set_outputs(value):
    ser.write(UBXMessage.config_set(layers=1, transaction=0, cfgData=[
        ("CFG_MSGOUT_UBX_RXM_COR_USB", value),
        ("CFG_MSGOUT_UBX_NAV_PVT_USB", value),
    ]).serialize())


# 1. UART2 の設定
ser.write(UBXMessage.config_poll(layer=0, position=0,
                                 keys=["CFG_UART2_BAUDRATE", "CFG_UART2INPROT_UBX"]).serialize())
cfg = wait_for("CFG-VALGET")
if cfg is None:
    print("[1] 設定を読めませんでした")
else:
    baud = getattr(cfg, "CFG_UART2_BAUDRATE", None)
    ubx_in = getattr(cfg, "CFG_UART2INPROT_UBX", None)
    ok = baud == 9600 and ubx_in == 1
    print(f"[1] UART2: {baud} bps, UBX入力={ubx_in}  → {'OK' if ok else 'NG（set_clas.py を実行する）'}")

# 2〜4. 一定時間ようすを見る
set_outputs(1)
ser.write(UBXMessage("MON", "MON-COMMS", POLL).serialize())
first = wait_for("MON-COMMS")
rx_start = uart2_rx(first) if first else None

print(f"\n{DURATION:.0f} 秒間ようすを見ます...")
cor_count, cor_last, pvt_last = 0, None, None
end = time.time() + DURATION
while time.time() < end:
    _, msg = ubr.read()
    if msg is None:
        continue
    if msg.identity == "RXM-COR":
        cor_count += 1
        cor_last = msg
    elif msg.identity == "NAV-PVT":
        pvt_last = msg

ser.write(UBXMessage("MON", "MON-COMMS", POLL).serialize())
last = wait_for("MON-COMMS")
rx_end = uart2_rx(last) if last else None
set_outputs(0)

if rx_start is None or rx_end is None:
    print("[2] UART2 の受信量を読めませんでした")
else:
    delta = rx_end - rx_start
    verdict = "届いている" if delta > 0 else "何も届いていない（NEO-D9C の挿さり方・電源・アンテナを確認）"
    print(f"[2] UART2 の受信: {DURATION:.0f} 秒で {delta} バイト → {verdict}")

if cor_last is None:
    print("[3] 補強信号の受付（RXM-COR）: なし → ZED-F9P は補強信号を受け取っていない")
else:
    p = getattr(cor_last, "protocol", None)
    print(f"[3] 補強信号の受付（RXM-COR）: {cor_count} 件。最後の1件 → "
          f"種類={COR_PROTOCOL.get(p, p)}、"
          f"誤り={COR_ERR.get(getattr(cor_last, 'errStatus', None), '?')}、"
          f"利用={COR_USED.get(getattr(cor_last, 'msgUsed', None), '?')}")

if pvt_last is None:
    print("[4] 測位の状態（NAV-PVT）を読めませんでした")
else:
    carr = getattr(pvt_last, "carrSoln", None)
    print(f"[4] 測位: fixType={pvt_last.fixType}、衛星 {pvt_last.numSV} 機、"
          f"RTK={CARR.get(carr, carr)}")

ser.close()

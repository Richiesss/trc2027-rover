"""NEO-D9C（CLAS補正）を使うために、ZED-F9P の UART2 を UBX入力・9600bps に設定する。

使い方:
    python set_clas.py [ポート名]   # 省略時は /dev/ttyACM0
layers=7（RAM + BBR + Flash）で書き込むので、電源を切っても設定が残る。
gnss_log.py は止めてから実行すること。
"""
import sys

from pyubx2 import UBXMessage
from serial import Serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"

ser = Serial(PORT, 38400, timeout=3)
msg = UBXMessage.config_set(layers=7, transaction=0, cfgData=[
    ("CFG_UART2_BAUDRATE", 9600),
    ("CFG_UART2INPROT_UBX", 1),
])
ser.write(msg.serialize())
print("sent: UART2 = 9600bps, UBX input enabled")

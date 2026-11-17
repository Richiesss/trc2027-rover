"""GNSS受信機（simpleRTK2B）から位置を読み、画面表示しながらCSVに記録する。

使い方:
    python gnss_log.py [ポート名]   # 省略時は /dev/ttyACM0

fix の値: 0=測位なし, 1=単独測位, 2=DGPS, 4=RTK Fix（cm級）, 5=RTK Float
"""
import csv
import datetime
import sys

import pynmea2
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"


def main():
    ser = serial.Serial(PORT, 38400, timeout=1)
    fname = datetime.datetime.now().strftime("gnss_%Y%m%d_%H%M%S.csv")
    print(f"recording to {fname} (Ctrl-C で終了)")

    with open(fname, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["pc_time", "utc", "lat", "lon", "alt", "fix", "sats", "hdop"])
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if not line.startswith("$GNGGA"):
                continue
            try:
                m = pynmea2.parse(line)
            except pynmea2.ParseError:
                continue
            row = [datetime.datetime.now().isoformat(), m.timestamp, m.latitude,
                   m.longitude, m.altitude, m.gps_qual, m.num_sats, m.horizontal_dil]
            w.writerow(row)
            f.flush()
            print(row)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass

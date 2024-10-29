import sys
import csv
from datetime import datetime,timedelta

if len(sys.argv) != 2:
    print("Usage: {} <log-file>".format(sys.argv[0]))
    sys.exit(1)

lines = [x for x in open(sys.argv[1]).read().splitlines() if "LOG:" in x]
w = csv.writer(sys.stdout)
w.writerow([
    "logtime",
    "rxtime",
    "node_id",
    "lat",
    "lng",
    "vbat",
    "quality",
    "satellites",
    "raw_temp",
    "last",
    "rssi",
])

add7hours = timedelta(hours=7)
for line in lines:
    items = line.split(" ")
    try:
        rxtime = datetime.strptime(" ".join(items[0:2]),'%Y-%m-%d %H:%M:%S.%f')
        logtime = datetime.strptime(" ".join(items[4:6]),'%Y-%m-%d %H:%M:%S')
    except ValueError:
        continue
    logtime += add7hours
    row = [
        logtime.strftime("%Y-%m-%d %H:%M:%S"),
        rxtime.strftime("%Y-%m-%d %H:%M:%S"),
        items[3],              # node_id
        float(items[6])/1e7,   # lat
        float(items[7])/1e7,   # lng
        float(items[8])/1e3,   # vbat
        int(items[9]),         # quality
        int(items[10]),        # satellites
        int(items[11]),        # raw temperature
        int(items[12])/1000,   # last heard from gateway
        int(items[13]),        # rssi
    ]
    w.writerow(row)

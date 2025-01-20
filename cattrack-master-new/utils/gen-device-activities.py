import sys
import re
from datetime import datetime, timedelta

TS_RE = re.compile(r"(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}) (.*)")

if len(sys.argv) != 3:
    print(f"Usage: {sys.argv[0]} <node-id> <log-file>")
    exit(1)

mcu_sleep = radio_time = gps_time = 0
first_ts = last_ts = None
radio_start = None
gps_on_count = 0
gps_success_count = 0
ts_mcu_sleep = None

node_id = sys.argv[1]
lines = open(sys.argv[2]).readlines()
for line in lines:
    m = TS_RE.match(line)
    if not m:
        continue
    try:
        ts = datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S")
    except ValueError:
        continue
    msg = m.group(2)

    # get rid of all entries with year=2000 (time not yet synchronized)
    if ts.year == 2000:
        continue

    # make timestamp GMT+7
    #ts += timedelta(hours=7)

    # adjust date that moved back during GPS bug
    if last_ts is not None:
        while ts + timedelta(minutes=1) < last_ts:
            ts += timedelta(days=1)

    # record the first timestamp and the lastest known timestamp
    if first_ts is None:
        first_ts = ts
        ts_wake = ts
    last_ts = ts

    # consider anything after sleep 'waking up'
    if ts_mcu_sleep is not None:
        diff = (ts - ts_mcu_sleep).total_seconds()
        mcu_sleep += diff
        ts_mcu_sleep = None
        ts_wake = ts

    if msg.startswith("Going to sleep"):
        timediff = (ts-ts_wake).total_seconds()
        print(f"{node_id},cpu,{ts_wake},{ts},{timediff}")
        ts_mcu_sleep = ts
    elif msg.startswith("Initializing GPS"):
        gps_start = ts
        gps_on_count += 1
    elif msg.startswith("Cannot acquire a fix"):
        timediff = (ts-gps_start).total_seconds()
        gps_time += timediff
        print(f"{node_id},gps-nofix,{gps_start},{ts},{timediff}")
    elif msg.startswith("Time and location acquired"):
        timediff = (ts-gps_start).total_seconds()
        gps_time += timediff
        gps_success_count += 1
        print(f"{node_id},gps-fix,{gps_start},{ts},{timediff}")
    elif msg.startswith("TX: REPORT"):
        if radio_start is None:
            radio_start = ts
    elif msg.startswith("Turn off radio"):
        timediff = (ts - radio_start).total_seconds()
        radio_time += timediff
        print(f"{node_id},radio,{radio_start},{ts},{timediff}")
        radio_start = None

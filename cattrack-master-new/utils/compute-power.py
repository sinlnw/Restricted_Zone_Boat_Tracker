import sys
import re
from datetime import datetime, timedelta

BASE_CURRENT = 0.9
MCU_CURRENT = 9.4
GPS_CURRENT = 23.6
RADIO_CURRENT = 111.1

TS_RE = re.compile(r"(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}) (.*)")

if len(sys.argv) != 3:
    print(f"Usage: {sys.argv[0]} <battery-capacity> <log-file>")
    exit(1)

batt_cap = float(sys.argv[1])
mcu_sleep = radio_time = gps_time = 0
first_ts = last_ts = None
radio_start = None
gps_on_count = 0
gps_success_count = 0
ts_mcu_sleep = None

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

    # adjust date that moved back during GPS bug
    if last_ts is not None:
        while ts + timedelta(minutes=1) < last_ts:
            ts += timedelta(days=1)

    # record the first timestamp and the lastest known timestamp
    if first_ts is None:
        first_ts = ts
    last_ts = ts

    # consider anything after sleep 'waking up'
    if ts_mcu_sleep is not None:
        diff = (ts - ts_mcu_sleep).total_seconds()
        mcu_sleep += diff
        ts_mcu_sleep = None

    if "Going to sleep" in msg:
        ts_mcu_sleep = ts
    elif msg.startswith("Initializing GPS"):
        gps_start = ts
        gps_on_count += 1
    elif msg.startswith("Cannot acquire a fix"):
        gps_time += (ts - gps_start).total_seconds()
    elif msg.startswith("Time and location acquired"):
        gps_time += (ts - gps_start).total_seconds()
        gps_success_count += 1
    elif msg.startswith("TX: REPORT"):
        if radio_start is None:
            radio_start = ts
    elif msg.startswith("Turn off radio"):
        radio_time += (ts - radio_start).total_seconds()
        radio_start = None

total_time = (last_ts - first_ts).total_seconds()
mcu_time = total_time - mcu_sleep
sleep_time = total_time - mcu_time - gps_time - radio_time
print("----------------------------------------------------------")
print("First timestamp = {}".format(first_ts + timedelta(hours=7)))
print("Last timestamp = {}".format(last_ts + timedelta(hours=7)))
print("----------------------------------------------------------")
print("Total time = {:.0f} seconds".format(total_time))
print("MCU time = {:.0f} seconds".format(mcu_time))
print("GPS time = {:.0f} seconds".format(gps_time))
print("Radio TX time = {:.0f} seconds".format(radio_time))
print("----------------------------------------------------------")
print("GPS turned on {} times".format(gps_on_count))
print("GPS acquired fix {} times".format(gps_success_count))
print("----------------------------------------------------------")
avg_current = (
        total_time*BASE_CURRENT + 
        mcu_time*MCU_CURRENT + 
        gps_time*GPS_CURRENT +
        radio_time*RADIO_CURRENT
    ) / total_time
print("Average current = {:.2f} mA".format(avg_current))
life_time = batt_cap/avg_current
print("Battery life ({:.0f} mAh) = {:.2f} hours, or {:.2f} days".format(
    batt_cap,life_time,life_time/24))
print("----------------------------------------------------------")

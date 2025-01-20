import sys
import re
from datetime import datetime, timedelta

TX_DURATION = 2

TS_RE = re.compile(r"(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})(\.\d{6}) LOG: (\d{1,2}) .*")

if len(sys.argv) != 2:
    print(f"Usage: {sys.argv[0]} <log-file>")
    exit(1)

lines = open(sys.argv[1]).readlines()
for line in lines:
    m = TS_RE.match(line)
    if not m:
        continue
    try:
        ts = datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S")
        if float(m.group(2)) >= 0.5:
            ts += timedelta(seconds=1)
    except ValueError:
        continue
    node_id = m.group(3)
    ts_start = ts - timedelta(seconds=TX_DURATION)
    print(f"{node_id},radio,{ts_start},{ts},{TX_DURATION}")

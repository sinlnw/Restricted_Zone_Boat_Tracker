#!/usr/bin/env python
import sys
import struct

MAGIC = "CATTRACK-LOGGER"

if len(sys.argv) != 3:
    print("Usage: {} log-partition last|-".format(sys.argv[0]))
    sys.exit(1)

cursor_struct = struct.Struct("<Q")
if sys.argv[2] != "-":  # start from the very beginning of the logs
    last = int(sys.argv[2])
else:
    last = -1
with open(sys.argv[1],"rb") as device:
    ctrl_block = device.read(512)

    if ctrl_block[-len(MAGIC):] != MAGIC.encode('ascii'):
        print("This does not seem to be a log partition.")
        sys.exit(1)

    cursor, = cursor_struct.unpack(ctrl_block[0:cursor_struct.size])
    print("Current cursor:",cursor)
    print()
    print("Logged messages:")
    print("----------------")

    if last == -1 or last > cursor:
        device.seek(512)
    else:
        device.seek(512+cursor-last)
        cursor = last

    while cursor > 0:
        num = min(cursor,1024)
        cursor -= num
        logs = device.read(num)
        cleaned_logs = bytearray([x for x in logs if 0 <= x <= 127])
        print(cleaned_logs.decode('ascii'),end='')

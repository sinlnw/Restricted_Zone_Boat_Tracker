#!/usr/bin/env python
import sys
import struct

MAGIC = "CATTRACK"

if len(sys.argv) != 2:
    print("Usage: {} <data-partition>".format(sys.argv[0]))
    sys.exit(1)

state = struct.Struct("<QQ")
report = struct.Struct("<BBBBBBHllBBHL")

with open(sys.argv[1],"rb") as device:
    ctrl_block = device.read(512)
    if ctrl_block[-len(MAGIC):] != MAGIC.encode('ascii'):
        print("This does not seem to be a data partition.")
        sys.exit(1)
    print("Report size =",report.size)
    first,used = state.unpack(ctrl_block[0:state.size])
    print("first={} used={}".format(first,used))
    block = 1
    print()
    print("Logged data:")
    print("============")
    for i in range(first+used):
        device.seek((i+1)*512)
        data = device.read(report.size)
        (year,month,day,hour,minute,second,
         vbat,lat,lng,quality,satellites,temp,last_heard) = \
                 report.unpack(data)
        lat = lat/100000
        lng = lng/100000
        vbat = vbat/1000
        year += 2000
        last_heard = last_heard/1000
        print("timestamp={}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}"
              " lat={:.7f} long={:.7f} vbat={} quality={} satellites={}"
              " raw_temp={} last_heard={:.3f}".format(
                  year,month,day,hour,minute,second,
                  lat,lng,vbat,
                  quality,satellites,temp,last_heard,
            )
        )

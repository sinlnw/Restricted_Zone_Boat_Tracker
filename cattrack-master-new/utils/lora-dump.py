import sys
from serial import Serial

if len(sys.argv) != 2:
    print(f'Usage: {sys.argv[0]} <serial-dev>')
    exit(1)

ser = Serial(sys.argv[1], 115200)
while True:
    while ser.read(1) != b'\x7e':
        pass
    src = ord(ser.read(1))
    rssi = ord(ser.read(1))
    len = ord(ser.read(1))
    payload = ser.read(len)
    cs = ser.read(1)
    if rssi > 100:
        rssi = rssi - 256
    payload_hex = ' '.join(f'{x:02x}' for x in payload)
    payload_asc = ''.join(chr(x) if 32 <= x <= 126 else '.' for x in payload)
    print('--------------------------------')
    print(f'SRC={src} RSSI={rssi}')
    print(f'PAYLOAD HEX: {payload_hex}')
    print(f'PAYLOAD ASC: {payload_asc}')


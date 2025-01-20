import os
import sys
import struct
import xml.etree.ElementTree as ET
import tkinter as tk
from tkinter import filedialog, messagebox

MAGIC = "CATTRACK"

root = tk.Tk()
root.withdraw()

def sig_verified(disk):
    disk.seek(0)
    mbr = disk.read(512)
    return (mbr[446+4],mbr[462+4],mbr[478+4]) == (0x0E,0x33,0x34)

def find_card():
    if sys.platform == 'darwin':
        plist = os.popen('diskutil list -plist').read()
        tree = ET.fromstring(plist)
        contents = [x for x in tree.find('dict')]
        keys,values = contents[::2],contents[1::2]
        plist_dict = {k.text:v for k,v in zip(keys,values)}
        volume_paths = [x.text for x in plist_dict['WholeDisks']]
        for path in volume_paths:
            devname = f'/dev/r{path}'
            try:
                disk = open(devname,'rb')
            except PermissionError:
                disk = None
                continue
            if sig_verified(disk):
                break
            disk.close()
        else:
            return None,None,None
    elif sys.platform == 'win32':
        idx = 1
        while True:
            devname = rf'\\.\PHYSICALDRIVE{idx}'
            try:
                disk = open(devname,'rb')
            except FileNotFoundError:
                disk = None
                break
            if sig_verified(disk):
                break
            disk.close()
            disk = None
            idx += 1
        if disk is None:
            return None,None,None
    else:
        print(f'Platform "{sys.platform}" not yet supported')

    disk.seek(0)
    mbr = disk.read(512)
    part = mbr[462:478]
    lba, = struct.unpack('<I',part[8:12])
    disk.seek(lba*512)
    return devname,disk,lba

card = None
try:
    dev,card,lba = find_card()
    if card is None:
        raise Exception('Tracker SD card not found')
    print(f"Found tracker SD on '{dev}'")

    ctrl_block = card.read(512)
    if ctrl_block[-len(MAGIC):] != MAGIC.encode('ascii'):
        raise Exception('Invalid data partition.')
    state = struct.Struct("<QQ")
    report = struct.Struct("<BBBBBBHllBBHL")
    first,used = state.unpack(ctrl_block[0:state.size])
    print(f'Found {used} data records')
    file_path = filedialog.asksaveasfilename()
    if not file_path:
        raise Exception('Data downloading canceled')
    print(f'Writing data to {file_path}')
    with open(file_path,'w') as f:
        print('date,time_utc,lat,lon,vbat,quality,satellites',file=f)
        block = 1
        for i in range(first+used):
            card.seek((lba+i+1)*512)
            data = card.read(report.size)
            (year,month,day,hour,minute,second,
             vbat,lat,lng,quality,satellites,temp,last_heard) = \
                     report.unpack(data)
            lat = lat/100000
            lng = lng/100000
            vbat = vbat/1000
            year += 2000
            last_heard = last_heard/1000
            print('{}-{:02d}-{:02d},{:02d}:{:02d}:{:02d},{:.7f},{:.7f},{:.3f},{},{}'.format(
                      year,month,day,hour,minute,second,
                      lat,lng,vbat,
                      quality,satellites),
                  file=f)
    messagebox.showinfo(message=f"Data successfully saved to '{file_path}'")
except Exception as e:
    messagebox.showerror(title='Error',message=str(e))
finally:
    if card is not None:
        card.close()

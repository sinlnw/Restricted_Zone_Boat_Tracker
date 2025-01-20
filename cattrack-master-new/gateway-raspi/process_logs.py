import sys
import os
import re
import pickle
from datetime import datetime, timedelta
from pathlib import Path
import requests

import settings


CHECKPOINT = Path('checkpoint.pickle')
DATA = Path('data.csv')
TZ_OFFSET = timedelta(hours=7)
HEADER = {'Authorization': 'Bearer ' + settings.API_TOKEN}


def load_checkpoint():
    if CHECKPOINT.exists():
        with open(CHECKPOINT, 'rb') as f:
            checkpoint = pickle.load(f)
    else:
        checkpoint = {'file': '', 'line': 0}
    return checkpoint


def get_logs(checkpoint, log_dir):
    listdir = [str(x) for x in log_dir.glob('gateway.*.*.log')]
    logs = [x for x in listdir if x >= checkpoint['file']]
    logs.sort()
    return logs


def process_log(datafile, logfile, start_line):
    count = 0
    with open(logfile) as f:
        lines = f.readlines()
    for line in lines[start_line-1:]:
        line = line.strip()
        if 'LOG: ' not in line:
            continue
        cols = line.split(' ')
        if cols[10] == '65535': # boot message
            continue
        rx_at = datetime.strptime(
                    ' '.join(cols[0:2]),
                    '%Y-%m-%d %H:%M:%S.%f'
                )
        node_id = int(cols[5])
        created_at = datetime.strptime(
                    ' '.join(cols[6:8]),
                    '%Y-%m-%d %H:%M:%S'
                ) + TZ_OFFSET
        lat = int(cols[8])/1e5
        lon = int(cols[9])/1e5
        vbat = int(cols[10])/1e3
        siv = int(cols[12])
        temp = int(cols[13])
        ttf = int(cols[14])/1e3
        rssi = int(cols[15])
        print(f'{created_at},{rx_at},{node_id},{lat:.5f},'
              f'{lon:.5f},{vbat:.3f},{siv},{ttf},{rssi}',
              file=datafile)
        message = {
            'created_at': int(created_at.timestamp()*1e3),
            'rx_at': int(rx_at.timestamp()*1e3),
            'lat': lat,
            'lng': lon,
            'temperature': temp,
            'vBat': vbat,
            'quality': 1,
            'satellites': siv,
            'rssi': rssi,
            'nodeId': node_id,
            'gatewayId': 3,
        }

        print('Connecting to API server')
        response = requests.post(settings.API_URL, json=message, headers=HEADER)
        print('Server response:', response.text)
        count += 1
    return count, len(lines)


def open_data_file():
    if DATA.exists():
        return open(DATA, 'a')
    f = open(DATA, 'a')
    f.write('created_at,rx_at,id,lat,lon,vbat,siv,ttf,rssi\n')
    return f


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Usage: {} log-dir'.format(sys.argv[0]))
        sys.exit(1)
    log_dir = Path(sys.argv[1])
    if not log_dir.is_dir():
        print(f'{log_dir} is not a directory')
        sys.exit(2)
    checkpoint = load_checkpoint()
    logs = get_logs(checkpoint, log_dir)
    with open_data_file() as data_file:
        for log in logs:
            if log == checkpoint['file']:
                start_line = checkpoint['line'] + 1
            else:
                start_line = 1
            print(f'Extracting {log} starting from line {start_line}')
            count, last_line = process_log(data_file, log, start_line)
            print(f'Recorded {count} new entries')
            checkpoint['file'] = log
            checkpoint['line'] = last_line
    with open(CHECKPOINT, 'wb') as f:
        pickle.dump(checkpoint, f)

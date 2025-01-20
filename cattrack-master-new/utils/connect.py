import sys
import os
import serial
import time
import requests
import json
from datetime import datetime, timedelta

try:
    from config import *
except:
    API_USER = None
    pass

#############################
def extract_data(line):
    if "LOG:" not in line:
        return None
    line = str(datetime.now()) + " " + line
    items = line.split(" ")
    try:
        rxtime = datetime.strptime(" ".join(items[0:2]),'%Y-%m-%d %H:%M:%S.%f')
        logtime = datetime.strptime(" ".join(items[4:6]),'%Y-%m-%d %H:%M:%S')
    except ValueError:
        return None
    logtime += timedelta(hours=7)
    return {
        "recorded_time"      : logtime.strftime("%Y-%m-%d %H:%M:%S"),
        "received_time"      : rxtime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "device"             : int(items[3]),
        "lat"                : float(items[6])/1e5,
        "lon"                : float(items[7])/1e5,
        "vbat"               : float(items[8])/1e3,
        "quality"            : int(items[9]),      
        "satellites"         : int(items[10]),     
        "temperature"        : int(items[11]),     
        "last_heard_from_gw" : int(items[12])/1000,
        "rssi"               : int(items[13]),     
    }

#############################
def log(out,line):
    print(datetime.now(),line,file=out)
    print(datetime.now(),line)
    out.flush()

#############################
def get_token(user,passwd):
    requests.packages.urllib3.disable_warnings()

    headers = {
        "content-type": "application/json"
    }

    body_json = {
        "username": user,
        "password": passwd,
    }

    resp=requests.post(API_AUTH_TOKEN,json.dumps(body_json),headers=headers,verify=False)
    response_json = resp.json()
    receivedToken = response_json["token"]

    return receivedToken

#############################
def upload_location(token,data):
    headers = {
            "content-type": "application/json",
            "Authorization": "Bearer "+ token,
        }
    resp=requests.post(API_LOCATION,json.dumps(data),headers=headers,verify=False)
    return resp.status_code

#############################
def port_generator(device_prefix):
    while True:
        fp = os.popen("ls {}* 2> /dev/null".format(device_prefix))
        ports = fp.read().splitlines()
        if fp.close() is not None:
            yield None
            continue
        for p in ports:
            yield p

#############################
if len(sys.argv) != 3:
    print("Usage: {} <device-prefix> <log>".format(sys.argv[0]))
    sys.exit(1)

device_prefix = sys.argv[1]
gateway = None
ports = port_generator(device_prefix)

with open(sys.argv[2],"a") as out:
    # attempt to connect to CatTrack web service

    while True:
        try:
            if gateway is None:
                while True:
                    # scan for the next port with the specified prefix
                    port = next(ports)
                    if port is None:
                        log(out,
                            "No port '{}*' is available; "
                            "try again in {} seconds..."
                            .format(device_prefix,RECONNECT_TIME))
                        time.sleep(RECONNECT_TIME)
                    else:
                        break
                log(out,"Connecting to {}".format(port))
                gateway = serial.Serial(port) 

            # Serial.readline() returns a byte array, so use decode() to
            # convert to string
            line = gateway.readline().decode().strip()
            log(out,line)
            data = extract_data(line)
            if data:
                try: # attempt to connect to CatTrack web service
                    api_token = get_token(API_USER,API_PASSWD)
                    upload_location(api_token,data)
                    log(out,"Data uploaded to web service")
                except:
                    log(out,"Failed to upload data to web service")
                    pass

        except serial.SerialException:
            log(out, 
                "Cannot read from {}; "
                "try to reconnect in {} seconds".format(port,RECONNECT_TIME))
            gateway = None
            time.sleep(RECONNECT_TIME)

        except Exception as e:
            # Unknown exception
            print(str(e))
            print(e.__class__.__name__)
            break


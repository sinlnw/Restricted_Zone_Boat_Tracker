import sys
import time
import ctypes
from datetime import datetime, timedelta
import logging
import json
import asyncio
import requests

import RPi.GPIO as GPIO
from SX127x.constants import MODE
from lora import LoRa
import boards
from packets import (
    PacketBoot,
    PacketReport,
    PacketAck,
)

try:
    import settings
except ModuleNotFoundError:
    print("Cannot find settings.py")
    sys.exit(1)

logger = logging.getLogger(__name__)
logger.setLevel(settings.LOG_LEVEL)
log_formatter = logging.Formatter(fmt="%(asctime)s.%(msecs)03d %(levelname)s: %(message)s",
                                  datefmt="%Y-%m-%d %H:%M:%S")
log_console_handler = logging.StreamHandler()
log_console_handler.setFormatter(log_formatter)
logger.addHandler(log_console_handler)
if settings.LOG_FILE is not None:
    log_file_handler = logging.FileHandler(settings.LOG_FILE)
    log_file_handler.setFormatter(log_formatter)
    logger.addHandler(log_file_handler)

GPIO.setmode(GPIO.BCM)

###########################################################
def upload_location(token,data):
    headers = {
            "content-type": "application/json",
            "Authorization": "Bearer "+ token,
        }
    resp=requests.post(API_LOCATION,json.dumps(data),headers=headers,verify=False)
    return resp.status_code

###########################################################
def upload(data):
    to_upload = {
        'nodeId':      data['device'],
        'created_at':  datetime.timestamp(data['recorded_time'])*1e3,
        'rx_at':       datetime.timestamp(data['received_time'])*1e3,
        'lat':         data['lat'],
        'lng':         data['lon'],
        'temperature': data['temperature'],
        'vBat':        data['vbat'],
        'quality':     data['quality'],
        'satellites':  data['satellites'],
        'rssi':        data['rssi'],
        'ttf':         data['ttf'],
    }
    response = requests.post(API_URL, json=to_upload, headers=head)
    print("Server response:", response.text)


###########################################################
class LoraWorker(LoRa):

    def __init__(self, board, address, use_ack=False, verbose=False):
        super().__init__(board, verbose=verbose)
        self.board = board
        self.address = address
        self.use_ack = use_ack
        self.promiscuous = False
        self.set_mode(MODE.SLEEP)
        self.rx_avail = False  # rx packet available
        self.tx_ready = True   # available for packet tx
        self.last_tx = 0.0

    @property
    def name(self):
        return self.board.__name__

    def on_tx_done(self):
        self.clear_irq_flags(TxDone=1)
        self.tx_ready = True

    def on_rx_done(self):
        self.rx_avail = True

    def enter_rx_mode(self):
        self.set_dio_mapping([0, 0, 0, 0, 0, 0])
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)

    def enter_tx_mode(self):
        self.set_dio_mapping([1, 0, 0, 0, 0, 0])
        self.set_mode(MODE.TX)

    async def transmit(self, dst, payload):
        if not isinstance(payload, bytes):
            raise TypeError('payload must be of type bytes')
        while not self.tx_ready:
            await asyncio.sleep(0.001)
        self.tx_ready = False
        self.write_payload([
            dst,  # receiver (0xff for broadcast)
            self.address, # sender
            0x00, # id
            0x00, # flags
        ] + list(payload))
        self.enter_tx_mode()
        self.last_tx = time.time()

        # wait until tx is done
        while not self.tx_ready:
            await asyncio.sleep(0.1)
        self.enter_rx_mode()

    async def process_pkt_boot(self, src, pkt, rssi):
        boot = PacketBoot()
        boot.unpack(pkt)
        my_addr = boot.config.radio_device_address
        gw_addr = boot.config.radio_gateway_address
        logger.info(f'[{self.name}] Device booting reported from {src}, RSSI={rssi}, '
                    f'with parameters:')
        logger.info(f' * Firmware version: {boot.firmware}')
        logger.info(f' * Device model: 0x{boot.device_model:02x}')
        logger.info(f' * Reset flags: 0x{boot.reset_flags:02x}')
        logger.info(f' * radio_freq = {boot.config.radio_freq:.2f} MHz')
        logger.info(f' * radio_tx_power = {boot.config.radio_tx_power} dBm')
        logger.info(f' * radio_device_address = {my_addr} (0x{my_addr:02X})')
        logger.info(f' * radio_gateway_address = {gw_addr} (0x{gw_addr:02X})')
        logger.info(f' * collect_interval_day = {boot.config.collect_interval_day} sec')
        logger.info(f' * collect_interval_night = {boot.config.collect_interval_night} sec')
        logger.info(f' * day_start_hour = {boot.config.day_start_hour}')
        logger.info(f' * day_end_hour = {boot.config.day_end_hour}')
        logger.info(f' * time_zone = {boot.config.time_zone} hours')
        logger.info(f' * use_ack = {boot.config.use_ack}')
        logger.info(f' * ack_timeout = {boot.config.ack_timeout} sec')
        logger.info(f' * long_range = {boot.config.long_range}')
        logger.info(f' * tx_repeat = {boot.config.tx_repeat}')
        logger.info(f' * gps_max_wait_for_fix = {boot.config.gps_max_wait_for_fix} sec')
        logger.info(f' * next_collect_no_fix = {boot.config.next_collect_no_fix} sec')
        logger.info(f' * total_slots = {boot.config.total_slots}')
        logger.info(f' * slot_interval = {boot.config.slot_interval} sec')

    async def process_pkt_report(self, src, pkt, rssi):
        report_pkt = PacketReport()
        report_pkt.unpack(pkt)
        report = report_pkt.report

        logger.info('[{}] '
            'LOG: {} 20{:02}-{:02}-{:02} {:02}:{:02}:{:02} {} {} {} {} {} {} {} {}'.format(
                self.name,
                src,
                report.year,
                report.month,
                report.day,
                report.hour,
                report.minute,
                report.second,
                report.latitude,
                report.longitude,
                report.vbat,
                report.quality,
                report.satellites,
                report.temperature,
                report.ttf,
                rssi,
            )
        )
        if self.use_ack:
            # TODO Investigate - without this delay the receiver will not receive the ack
            await asyncio.sleep(0.25)
            ack = PacketAck(seq=report_pkt.seq)
            logger.info(f'[{self.name}] TX: ACK to={src} seq={ack.seq}')
            await self.transmit(src, ack.bytes)
        try:
            rxtime = datetime.now()
            logtime = datetime(2000+report.year,
                               report.month,
                               report.day,
                               report.hour,
                               report.minute,
                               report.second)
            logtime += timedelta(hours=7)
        except ValueError:
            return
        data = {
            "recorded_time" : logtime,
            "received_time" : rxtime.now(),
            "device"        : src,
            "lat"           : report.latitude/1e5,
            "lon"           : report.longitude/1e5,
            "vbat"          : report.vbat/1e3,
            "quality"       : report.quality,
            "satellites"    : report.satellites,
            "temperature"   : report.temperature,
            "ttf"           : report.ttf/1000,
            "rssi"          : rssi,
        }
        #try: # attempt to connect to CatTrack web service
        #    upload(data)
        #    logger.info(f"[{self.name}] Data uploaded to web service")
        #except:
        #    logger.warning(f"[{self.name}] Failed to upload data to web service")
        #    pass

    async def dispatch_packet(self, pkt, src, rssi):
        logger.debug(f'[{self.name}] Received packet '
                     f'(type={pkt[0]}, size={len(pkt)}, rssi={rssi})')
        if PacketBoot.probe(pkt):
            await self.process_pkt_boot(src, pkt, rssi)
        elif PacketReport.probe(pkt):
            await self.process_pkt_report(src, pkt, rssi)
        else:
            logger.warning(f'[{self.name}] Unsupported packet (type={pkt[0]}, size={len(pkt)}) '
                           f'received from 0x{src:02X}')

    async def start_upload(self):
        logger.info(f"[{self.name}] Upload task started")
        while True:
            await asyncio.sleep(1)

    async def start(self):
        if self.use_ack:
            logger.info(f"[{self.name}] START with ack mode")
        else:
            logger.info(f"[{self.name}] START with non-ack mode")
        self.enter_rx_mode()
        if settings.API_URL is not None:
            asyncio.create_task(self.start_upload())
        while True:
            # wait for incoming packets
            while not self.rx_avail:
                await asyncio.sleep(0.001)
            self.rx_avail = False
            rssi = self.get_pkt_rssi_value()
            payload = self.read_payload(nocheck=True) # mean "do not check CRC!"
            pkt_error = not self.rx_is_good()
            self.clear_irq_flags(RxDone=1, PayloadCrcError=1)
            if pkt_error:
                logger.warning(f'[{self.name}] CRC ERROR (no data written)')
                continue
            src = payload[1]
            dst = payload[0]
            if dst in [0xff, self.address] or self.promiscuous:
                pkt = bytes(bytearray(payload[4:]))
                self.board.led_on()
                await self.dispatch_packet(pkt, src, rssi)
                await asyncio.sleep(0.01)
                self.board.led_off()


class Uploader():

    def __init__(self, url):

        self.url = url
        self.name = "Uploader"

    async def start(self):
        #self.printT(f"Start Task Uploader Period {UPLOAD_INT}")
        while True:
            await asyncio.sleep(UPLOAD_INT)
            #self.printT("Start Upload")
            json_datas['timestamp'] = TimeStamp()[2:]
            #self.printT(f"Send Request {json_datas}")
            result = requests.post(API_LOCATION,json.dumps(json_datas),headers={'content-type': 'application/json'})
            if result.status_code == 200:
                #self.printT(f"Send Success")
                c_tempLog = copy.deepcopy(tempLog)
                json_datas['log_data'] = c_tempLog
            else:
                logger.warning(f'[Uploader] Send failure')


###########################################################
if __name__ == "__main__":
    # All boards' I/O pins must be configured, no matter they are used or not
    for board in [boards.Board0, 
                  boards.Board1,
                  boards.Board2,
                  boards.Board3]:
        GPIO.setup(board.SPI_CS, GPIO.OUT)
        GPIO.output(board.SPI_CS, GPIO.HIGH)
        board.setup()
        board.reset()

    # Start a worker for each configured board
    workers = []
    for entry in settings.BOARDS:
        board = entry['board']
        worker = LoraWorker(board, address=entry['addr'], use_ack=entry['useack'])
        worker.set_pa_config(pa_select=1, max_power=21, output_power=entry['txpower'])
        worker.set_bw(entry['bw'])
        worker.set_freq(entry['freq'])
        worker.set_coding_rate(entry['cr'])
        worker.set_spreading_factor(entry['sf'])
        worker.set_rx_crc(True)
        worker.set_low_data_rate_optim(True)
        assert(worker.get_agc_auto_on() == 1)
        workers.append(worker)

    loop = asyncio.get_event_loop()
    try:
        for worker in workers:
            loop.create_task(worker.start())
        loop.run_forever()
    except KeyboardInterrupt:
        logger.warning("Keyboard interrupt; exiting")
    finally:
        for worker in workers:
            worker.set_mode(MODE.SLEEP)
            worker.board.teardown()
        logger.info("Cleaning up GPIO")
        GPIO.cleanup()  

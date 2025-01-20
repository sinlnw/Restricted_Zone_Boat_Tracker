#include <SPI.h>
#include <SD.h>
#include <Adafruit_SleepyDog.h>
#include <Adafruit_GPS.h>
#include "version.h"

#define DEBUG
#define SD_LOADER
//#define DEBUG_SD_LOADER
#define WATCHDOG_TIMEOUT   4000

#include "pt/pt.h"
#include "printf.h"
#include "radio.h"
#include "config.h"
#include "io.h"
#include "card-utils.h"
#include "debug.h"

#ifdef SD_LOADER
__attribute__ ((section(".sketch_boot")))
unsigned char sduBoot[0xC000] = {
#include "boot/feather_m0.h"
};
#endif

#define TIMER_START(ts)       ts = millis();
#define TIMER_EXPIRED(ts,timeout)  (millis()-ts > timeout)
#define PT_DELAY(pt,ms,tsVar) \
  tsVar = millis(); \
  PT_WAIT_UNTIL(pt, millis()-tsVar >= (ms));

#define PT_WAIT_RADIO_READY(pt,tsVar) \
    PT_WAIT_UNTIL(pt,radio.send_done() || (millis() - tsVar > 5000)); \
    if (!radio.send_done()) { \
      DEBUG_PRINTF("Radio stuck in TX state for too long.\r\n"); \
      force_reset(); \
    }

Sd2Card card;
Config config;
Radio radio;
uint32_t last_report_rx_time = 0;
struct pt ptBatProbe;
struct pt ptCollect;
struct pt ptSerial;

/***********************************************
 * 
 */
void error_blink_loop() {
  for (;;) {
    LED_ON();
    delay(200);
    LED_OFF();
    delay(200);
  }
}

/*********************************************************************
 * Force watchdog reset
 */
void force_reset() {
  DEBUG_PRINTF("Force reset\r\n");
  Watchdog.enable(1000);
  while (true)
    ;
}

/*********************************************************************
 * Custom watchdog reset to prevent very slow synchronization of SAMD chip
 */
void watchdog_reset() {
  static uint32_t last = 0;
  if ( (millis() - last) > WATCHDOG_TIMEOUT/2 ) {
    //DEBUG_PRINTLN("Reset watchdog");
    Watchdog.reset();
    last = millis();
  }
}

/***********************************************
 * 
 */
void init_radio() {
  DEBUG_PRINT("Initializing radio...");
  if (!radio.init(
      config.radio_gateway_address,
      config.radio_freq,
      config.radio_tx_power,
      config.long_range)) 
  {
    DEBUG_PRINTLN("failed!");
    error_blink_loop();
  }
  DEBUG_PRINTLN("done.");
}

/***********************************************
 * 
 */
PT_THREAD(taskCollect(struct pt* pt)) {

  static uint32_t ts = 0;
  static uint8_t buf[64];
  static PacketReport* pkt;
  static Address from;

  PT_BEGIN(pt);
  for (;;) {
    PT_WAIT_UNTIL(pt,radio.available());
    uint8_t len = sizeof(buf);
    radio.recv(&from,buf,&len);
    DEBUG_PRINTF("RX: from=%d len=%d\r\n",from,len);
    if (len == sizeof(PacketReport) && (buf[0] == PKT_TYPE_REPORT)) {
      last_report_rx_time = millis();
      pkt = (PacketReport*)buf;
      DEBUG_PRINTF("RX: REPORT seq=%d\r\n",pkt->seq);
      printf(Serial,"LOG: %d 20%02u-%02u-%02u %02u:%02u:%02u %ld %ld %u %u %u %u %lu %d\r\n",
        from,
        pkt->report.year,
        pkt->report.month,
        pkt->report.day,
        pkt->report.hour,
        pkt->report.minute,
        pkt->report.second,
        pkt->report.latitude,
        pkt->report.longitude,
        pkt->report.vbat,
        pkt->report.quality,
        pkt->report.satellites,
        pkt->report.temperature,
        pkt->report.ttf,
        radio.last_rssi
      );
      if (config.use_ack) {
        // TODO Investigate - without this delay the receiver will not receive the ack
        PT_DELAY(pt,250,ts);

        // some other task might use the radio
        PT_WAIT_RADIO_READY(pt,ts);
        DEBUG_PRINTF("TX: ACK to=%d seq=%d\r\n",from,pkt->seq);
        PacketAck ack;
        ack.seq = pkt->seq;
        radio.send(from,(uint8_t*)&ack,sizeof(ack));
      }
    }
    else if (len == sizeof(PacketBoot) && (buf[0] == PKT_TYPE_BOOT)) {
      PacketBoot* boot = (PacketBoot*)buf;
      printf(Serial, "Device booting reported with parameters:\r\n");
      printf(Serial, " * Firmware version: %d\r\n",
        boot->firmware);
      printf(Serial, " * Device model: 0x%02x\r\n",
        boot->device_model);
      printf(Serial, " * Reset flags: 0x%02x\r\n",
        boot->reset_flags);
      printf(Serial, " * radio_freq = %d.%02d MHz\r\n",
        (int)boot->config.radio_freq,
        (int)(boot->config.radio_freq*100) % 100);
      printf(Serial, " * radio_tx_power = %d dBm\r\n",
        boot->config.radio_tx_power);
      printf(Serial, " * radio_device_address = %d (0x%02X)\r\n",
        boot->config.radio_device_address,
        boot->config.radio_device_address);
      printf(Serial, " * radio_gateway_address = %d (0x%02X)\r\n",
        boot->config.radio_gateway_address,
        boot->config.radio_gateway_address);
      printf(Serial, " * collect_interval_day = %d sec\r\n",
        boot->config.collect_interval_day);
      printf(Serial, " * collect_interval_night = %d sec\r\n",
        boot->config.collect_interval_night);
      printf(Serial, " * day_start_hour = %d\r\n",
        boot->config.day_start_hour);
      printf(Serial, " * day_end_hour = %d\r\n",
        boot->config.day_end_hour);
      printf(Serial, " * time_zone = %d hours\r\n",
        boot->config.time_zone);
      printf(Serial, " * use_ack = %d\r\n",
        boot->config.use_ack);
      printf(Serial, " * ack_timeout = %d sec\r\n",
        boot->config.ack_timeout);
      printf(Serial, " * long_range = %d\r\n",
        boot->config.long_range);
      printf(Serial, " * tx_repeat = %d\r\n",
        boot->config.tx_repeat);
      printf(Serial, " * gps_max_wait_for_fix = %d sec\r\n",
        boot->config.gps_max_wait_for_fix);
      printf(Serial, " * next_collect_no_fix = %d sec\r\n",
        boot->config.next_collect_no_fix);
      printf(Serial, " * total_slots = %d\r\n",
        boot->config.total_slots);
      printf(Serial, " * slot_interval = %d sec\r\n",
        boot->config.slot_interval);
    }
  }
  PT_END(pt);
}

/***********************************************
 * 
 */
PT_THREAD(taskBatProbe(struct pt* pt)) {

  static uint32_t ts = 0;
  static uint16_t sum;
  static int i;

  PT_BEGIN(pt);
  for (;;) {
    sum = 0;
    for (i = 0; i < 5; i++) { // take 5 voltage samples
      sum += analogRead(A0);
      PT_DELAY(pt,100,ts);
    }
    uint16_t vbat_mv = sum*3300*11/1023/5;
    printf(Serial,"BATT: %d\r\n", vbat_mv);
    PT_DELAY(pt,60000,ts);
  }
  PT_END(pt);
}

/***********************************************
 * 
 */
PT_THREAD(taskSerial(struct pt* pt)) {

  static uint32_t ts = 0;

  PT_BEGIN(pt);
  for (;;) {
    PT_WAIT_UNTIL(pt,Serial.available());
    char c = Serial.read();
  }
  PT_END(pt);
}

/***********************************************
 * 
 */
void init_card() {
  DEBUG_PRINT("Initializing SD card...");
  if (!card.init(SPI_HALF_SPEED,PIN_SD_SS)) {
    DEBUG_PRINTLN("failed!");
    error_blink_loop();
  }
  DEBUG_PRINTLN("done.");

  /******************************
   * Process configuration on SD
   ******************************/
  uint8_t config_part_idx;

  get_data_and_log_partitions(
    card, &config_part_idx, nullptr, nullptr, nullptr, nullptr);
  if (config_part_idx == 0) {
    DEBUG_PRINTF("Cannot find configuration partition id 0x%02x\r\n",
      CONFIG_PARTITION_ID);
    error_blink_loop();
  }
  DEBUG_PRINTF("Found configuration partition index %d\r\n", config_part_idx);
  SdVolume volume;
  if (!volume.init(card,config_part_idx)) {
    DEBUG_PRINTF("Cannot open configuration volume\r\n");
    error_blink_loop();
  }
  DEBUG_PRINTF("Reading configuration file...");
  if (!config.read_from_volume(volume)) {
    DEBUG_PRINTF("failed!\r\n");
    error_blink_loop();
  }
  DEBUG_PRINTF("successful.\r\n");

}

/***********************************************
 * 
 */
void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(PIN_RFM95_CS,OUTPUT);
  digitalWrite(PIN_RFM95_CS,HIGH);

  LED_OFF();
  Serial.begin(9600);
  delay(2000);
  while (!Serial)
    ; // wait for serial port to connect
  init_card();
  init_radio();
  DEBUG_PRINTF("Gateway initialized with parameters:\r\n");
  DEBUG_PRINTF(" * Firmware version: %d\r\n", FIRMWARE_VERSION);
  DEBUG_PRINTF(" * radio_freq = %d.%02d MHz\r\n",
    (int)config.radio_freq,
    (int)(config.radio_freq*100) % 100);
  DEBUG_PRINTF(" * radio_tx_power = %d dBm\r\n", config.radio_tx_power);
  DEBUG_PRINTF(" * radio_gateway_address = %d (0x%02X)\r\n",
    config.radio_gateway_address,
    config.radio_gateway_address);
  DEBUG_PRINTF(" * use_ack = %d\r\n", config.use_ack);
  DEBUG_PRINTF(" * long_range = %d\r\n", config.long_range);
  DEBUG_PRINTF(" * report size = %d bytes\r\n", sizeof(ReportItem));

  PT_INIT(&ptBatProbe);
  PT_INIT(&ptCollect);
  PT_INIT(&ptSerial);
  LED_ON();
  Watchdog.enable(WATCHDOG_TIMEOUT);
}

/***********************************************
 * 
 */
void loop() {
  // operate only when serial is connected; reset in 5 seconds if disconnected
  if (Serial) {
    watchdog_reset();
    taskCollect(&ptCollect);
    taskBatProbe(&ptBatProbe);
    taskSerial(&ptSerial);
  }
}

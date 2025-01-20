/*********************************************************************
 * This sketch is supposed to be used with a power switch attached
 */
#include <Wire.h>

#include "settings.h"
#include "debug.h"

//#define DEBUG_STORAGE
#include "storage.h"
#include "at24eeprom.h"

#include "time-utils.h"
#include "logger.h"
#include "report.h"
#include "io.h"
#include "card-utils.h"
#include "sys-utils.h"
#include "uploader.h"
#include "tracker.h"
#include "config.h"

#define TASK "MAIN"

AT24EEPROM eeprom;
Storage<ReportItem> storage(eeprom);

#ifdef LOGGING
Sd2Card card;
Logger logger(card);
#endif


#ifdef LOGGING
/***********************************************
 * Prepare SD logging service 
 */
void init_card() {
  // logger still not available
  DEBUG_PRINT("Initializing SD card...");
  if (!card.init(SPI_HALF_SPEED, PIN_SD_SS)) {
    DEBUG_PRINTLN("failed!");
    error_blink_loop(1, false);
  }
  DEBUG_PRINTLN("done.");

  uint8_t  config_part_idx;
  uint32_t data_start, data_size;
  uint32_t log_start, log_size;

  get_data_and_log_partitions(
    card,
    &config_part_idx,
    &data_start, &data_size,
    &log_start, &log_size);

  if (log_start == 0) {
    DEBUG_PRINTF("Cannot find log partition id 0x%02x\r\n",
      LOG_PARTITION_ID);
    error_blink_loop(2, false);
  }
  DEBUG_PRINTF("Setting up log partition at block #%d, total %d bytes...",
    log_start, ui64toa(log_size*512ULL));
  if (!logger.setup(log_start,log_size,false)) {
    DEBUG_PRINTLN("failed!");
    error_blink_loop(3, false);
  }
  DEBUG_PRINTLN("done.");
  LOG_TS("==== Logger started ====\r\n");
}
#endif


/*********************************************************************
 * Record special 'wakeup' message for reset event indication
 */
void record_wakeup_message(uint8_t reset_flags) {
  ReportItem report;
  report.type = REPORT_TYPE_WAKEUP;
  report.year = 0;
  report.month = 0;
  report.day = 0;
  report.hour = 0;
  report.minute = 0;
  report.second = reset_flags;
  report.vbat = read_vbat_mv();
  report.latitude = 0;
  report.longitude = 0;
  report.siv = 0;
  report.temperature = read_temperature();
  report.time_to_fix = 0;
  storage.push(report);
}

/*********************************************************************
 * Trigger power-off signal
 */
void power_off(uint16_t seconds) {
  LOG_TS(TASK ": Power off for %d seconds\r\n", seconds);
  for (int i=0; i<5; i++) { // repeat 5 times just in case
    Wire.beginTransmission(POWER_SWITCH_ADDR);
    Wire.write(POWER_SWITCH_OFF_CMD);
    Wire.write(seconds & 0xff);
    Wire.write(seconds >> 8);
    Wire.endTransmission();
    delay(500);
  }
  LOG_TS(TASK ": Shouldn't be here\r\n", seconds);
}

/*********************************************************************
 * Return number of wakeup messages in the storage
 */
int wakeup_count() {
  int count = 0;
  ReportItem report;
  for (int i=0; i<storage.count(); i++) {
    storage.peek(&report, i);
    if (report.type == REPORT_TYPE_WAKEUP)
      count++;
  }
  return count;
}

/*********************************************************************
 * 
 */
void setup() {
  Wire.begin();
  uint8_t reset_flags = REG_PM_RCAUSE;

  pinMode(LED_BUILTIN, OUTPUT);
  LED_ON();

#ifdef DEBUG
  Serial.begin(115200);
  while (!Serial)
    ;
#endif

  init_time();

#ifdef LOGGING
  init_card();
#endif

  LOG_TS(TASK ": Booting with reset flags: 0x%02X\r\n", reset_flags);
  LOG_TS(TASK ": Initializing device\r\n");
  eeprom.setup(0x50);
  storage.setup(TRACKER_START_EEPROM_ADDR, TRACKER_MAX_OFFLINE_RECORDS, false);

#if(SAVE_DEFAULT_CONFIG)
  config.save(&eeprom);
#endif

#if(!SKIP_CONFIG_LOADING)
  config.load(&eeprom);
#endif

#ifndef DEBUG
  USBDevice.detach();
#endif
  LOG_TS(TASK ": Initializing GNSS\r\n");
  Serial1.begin(9600);
  if (!tracker.begin(Serial1, &storage)) {
    error_blink_loop(4, true);
  }
  LOG_TS(TASK ": GNSS initialized\r\n");

  watchdog_start();

  record_wakeup_message(reset_flags);
  LOG_TS(TASK ": Wakeup message recorded: record count = %lu\r\n", (uint32_t)storage.count());

  LED_OFF();
}

/*********************************************************************
 * 
 */
void loop() {
  tracker.track();
  int count = wakeup_count();
  LOG_TS(TASK ": Found %d wakeup messages\r\n", count);
  if (wakeup_count() >= config.boot_count) {
    LOG_TS(TASK ": Trigger upload\r\n", count);
    uploader.begin(&storage);
    uploader.upload();
  }

  LOG_TS(TASK ": Trigger power-off\r\n");

  // compensate the elapsed time since boot to the duration
  if (millis()/1000 < config.off_duration) {
    power_off(config.off_duration - millis()/1000);
  }
  else {
    power_off(CONF_POWER_OFF_DURATION_MIN);
  }

  // module should be off now; the following code should be unreachable except
  // during debugging
  sleeping_wait(60);
}

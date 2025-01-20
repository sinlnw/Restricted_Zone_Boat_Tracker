#include <Adafruit_SleepyDog.h>
#include "version.h"

#define LOGGING
//#define SD_LOADER
#define WATCHDOG_TIMEOUT   10000
#define MAX_TASKS   3
#define GPS_SERIAL  Serial1

// max ack timeouts before giving up
#define MAX_ACK_TIMEOUT_COUNT 3
#define ADAFRUIT_FEATHER_M0 // rachata
//#define ADALOGGER //rachata
#define DEBUG
#include "debug.h"
//#define STORAGE_DEBUG
#include "storage.h"

#include "gps.h"
#include "logger.h"
#include "radio.h"
#include "config.h"
#include "io.h"
#include "time-utils.h"
#include "card-utils.h"
#include "task.h"

//rachata start
JsonDocument all_areas_doc;
JsonArray all_areas;
uint32_t buzzer_starttime = 0;
bool buzzer_on = false;
bool is_area_file_exist = false;
#define AREA_FILE_NAME "/AREA.txt"
#define BUZZER_PIN A1
#define BUZZER_DURATION 1 // in seconds
#define IN_AREA_INTERVAL 10 // in seconds
// #define ADAFRUIT_FEATHER_M0
//rachata end


#if defined(SD_LOADER) && defined(ADAFRUIT_FEATHER_M0)
__attribute__ ((section(".sketch_boot")))
unsigned char sduBoot[0xC000] = {
#include "boot/feather_m0.h"
};
#endif

Sd2Card card;
Config config;
Storage<ReportItem> storage(card);
GPSClass GPS(GPS_SERIAL);
Radio radio;
Task* tasks[MAX_TASKS];
uint8_t num_tasks;
volatile bool pps_detected = false;
#ifdef DEBUG
volatile bool wake_now = false;
#endif

#ifdef LOGGING
Logger logger(card);
#endif

#define TX_SLOT_ENDED  (time_wait_for_tx() > 0)

/***********************************************
 * 
 */
void init_card() {
  // logger still not available
  DEBUG_PRINT("Initializing SD card...");
  if (!card.init(SPI_HALF_SPEED, PIN_SD_SS)) {
    DEBUG_PRINTLN("failed!");
    error_blink_loop(1);
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

#ifdef LOGGING
  /******************************
   * Prepare logging service 
   ******************************/
  if (log_start == 0) {
    DEBUG_PRINTF("Cannot find log partition id 0x%02x\r\n",
      LOG_PARTITION_ID);
    error_blink_loop(2);
  }
  DEBUG_PRINTF("Setting up log partition at block #%d, total %d bytes...",
    log_start, ui64toa(log_size*512ULL));
  if (!logger.setup(log_start, log_size, false)) {
    DEBUG_PRINTLN("failed!");
    error_blink_loop(3);
  }
  DEBUG_PRINTLN("done.");
  LOG_TS("==== Logger started ====\r\n");
#endif

  /******************************
   * Process configuration on SD
   ******************************/
  if (config_part_idx == 0) {
    LOG_TS("Cannot find configuration partition id 0x%02x\r\n",
      CONFIG_PARTITION_ID);
    error_blink_loop(4);
  }
  LOG_TS("Found configuration partition index %d\r\n", config_part_idx);
  SdVolume volume;
  if (!volume.init(card, config_part_idx)) {
    LOG_TS("Cannot open configuration volume\r\n");
    error_blink_loop(5);
  }
  LOG_TS("Reading configuration file...");
  if (!config.read_from_volume(volume)) {
    LOG("failed!\r\n");
    error_blink_loop(6);
  }
  LOG("successful.\r\n");

  /******************************
   * Prepare data storage service 
   ******************************/
  if (data_start == 0) {
    LOG_TS("Cannot find data partition id 0x%02x\r\n",
      STORAGE_PARTITION_ID);
    error_blink_loop(7);
  }
  LOG_TS("Setting up data partition at block #%d, total %d blocks..",
    data_start, data_size);
  if (!storage.setup(data_start, data_size, false)) {
    LOG("failed.\r\n");
    error_blink_loop(8);
  }
  LOG("successful.\r\n");
}


/***********************************************
 * 
 */
void init_radio() {
  LOG_TS("Initializing radio...");
  if (!radio.init(
      config.radio_device_address,
      config.radio_freq,
      config.radio_tx_power,
      config.long_range))
  {
    LOG("failed.\r\n");
    error_blink_loop(9);
  }
  LOG("successful; NodeId = 0x%02X\r\n", config.radio_device_address);
}

/***********************************************
 * 
 */
void error_blink_loop(uint8_t code) {
  LOG_TS("ERROR: code %d\n", code);
  for (;;) {
    LED_ON();
    delay(200);
    LED_OFF();
    delay(200);
  }
}

#if defined(ADAFRUIT_FEATHER_M0)
/***********************************************
 * temperature reading
 * see: http://ww1.microchip.com/downloads/en/AppNotes/Atmel-42645-ADC-Configurations-with-Examples_ApplicationNote_AT11481.pdf
 */
static __inline__ void syncADC() __attribute__((always_inline, unused));
static void syncADC() {
  while (ADC->STATUS.bit.SYNCBUSY == 1)
    ;
}

uint16_t read_temperature() {
  uint32_t valueRead = 0;

  SYSCTRL->VREF.bit.TSEN = 1;  // Enable temperature sensor
  ADC->INPUTCTRL.bit.MUXPOS = 0x18;  // Temperature reference
  ADC->CTRLA.bit.ENABLE = 0x01; // Enable ADC

  // Start conversion
  syncADC();
  ADC->SWTRIG.bit.START = 1;

  // Clear the Data Ready flag
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  // Start conversion again, since The first conversion after the reference is changed must not be used.
  syncADC();
  ADC->SWTRIG.bit.START = 1;

  // Store the value
  while (ADC->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete
  valueRead = ADC->RESULT.reg;

  syncADC();
  ADC->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
  syncADC();

  return valueRead;
}
#endif

#if defined(ARDUINO_AVR_FEATHER32U4)
uint16_t read_temperature() {
  unsigned int wADC;

  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.

  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC

  delay(20);            // wait for voltages to become stable.

  ADCSRA |= _BV(ADSC);  // Start the ADC

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA, ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;

  return wADC;
}
#endif

/*********************************************************************
 * Force watchdog reset
 */
void force_reset() {
  LOG_TS("Something went wrong; force reset\r\n");
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

/*********************************************************************
 * Put the device into sleep mode for the specified number of seconds
 */
void sleeping_wait(uint16_t seconds) {
  if (seconds == 0) return;
  LOG_TS("Going to sleep for %d second(s)\r\n", seconds);
  LED_OFF();
#if defined(DEBUG)
  // Cannot sleep for real in debug mode
  uint32_t ts = millis();
  wake_now = false;
  while (millis() - ts < seconds*1000) {
    watchdog_reset();
    if (wake_now) break;
  }
  wake_now = false;
#else
  uint32_t until = get_total_seconds() + seconds;
  uint32_t remain;
  while ((remain = until - get_total_seconds()) > 0) {
    if (remain > 8)
      adjustTime(Watchdog.sleep(8000)/1000);
    else if (remain > 4)
      adjustTime(Watchdog.sleep(4000)/1000);
    else if (remain > 2)
      adjustTime(Watchdog.sleep(2000)/1000);
    else if (remain > 1)
      adjustTime(Watchdog.sleep(1000)/1000);
  }
#endif
  LOG_TS("Waking up\r\n");
  LED_ON();
}

/***********************************************
 * 
 */
bool is_day() {
  if (get_year() == 2000)
    // time is not yet known, assume daytime
    return true;

  uint8_t hour_val = (get_hours() + config.time_zone) % 24;
  if ( (hour_val >= config.day_start_hour) && 
       (hour_val < config.day_end_hour))
    return true;

  return false;
}

/***********************************************
 * 
 */
class TaskReport:public virtual Task {
private:
  uint32_t _ts;
  PacketReport _pkt;
  uint8_t _send_seq;
  uint8_t _buf[64];
  uint8_t _len;
  Address _from;
  bool _ack_received;
  uint8_t _ack_timeout_count;

public:
  virtual const char * get_name() {
    return "Report";
  }

  virtual TASK(run()) {

    TASK_BEGIN();

    _ts = 0;
    _send_seq = 0;
    for (;;) { // main task loop
      // make sure clock is synchronized
      while (get_year() == 2000) {
        LOG_TS("REPORT: Clock not synchronized; giving up this tx time slot\r\n");
        set_wakeup_time(get_total_seconds() + time_wait_for_next_tx());
        TASK_WAIT_UNTIL(get_total_seconds() >= get_wakeup_time());
        reset_wakeup_time();
      }

      // wait for my own time slot
      uint32_t time_to_wait = time_wait_for_tx();
      if (time_to_wait > 0) {
        LOG_TS("REPORT: Wait for time slot: %d seconds\r\n", time_to_wait);
        LOG_TS("REPORT: Turn off radio\r\n");
        RADIO_OFF();
        set_wakeup_time(get_total_seconds() + time_to_wait);
        TASK_WAIT_UNTIL(get_total_seconds() >= get_wakeup_time());
        reset_wakeup_time();
      }

      for (;;) { // retrieve stored records
        while (storage.empty()) {
          LOG_TS("REPORT: No records found; giving up this tx time slot\r\n");
          set_wakeup_time(get_total_seconds() + time_wait_for_next_tx());
          TASK_WAIT_UNTIL(get_total_seconds() >= get_wakeup_time());
          reset_wakeup_time();
        }
        if (TX_SLOT_ENDED) break;

        LOG_TS("REPORT: Record count = %d\r\n", storage.count());

        storage.top(&_pkt.report);
        _send_seq++;
        _pkt.seq = _send_seq;

        _ack_received = false;
        _ack_timeout_count = 0;
        while (!_ack_received) { // (re)transmit until acked
          if (TX_SLOT_ENDED) break; // tx slot already ended
          //TASK_DELAY(2000, _ts); // XXX testing
          LOG_TS("REPORT: TX seq=%d\r\n", _pkt.seq);
          _ts = millis();
          radio.send(config.radio_gateway_address, (uint8_t*)&_pkt, sizeof(_pkt));
          TASK_WAIT_UNTIL(radio.send_done());
          LOG_TS("REPORT: TX done (%d ms); awaiting ACK\r\n", millis()-_ts);
          TASK_SHORT_BLINK(_ts, 10);

          while (!_ack_received) { // wait for ack
            TASK_WAIT_UNTIL(radio.available() || (millis()-_ts > config.ack_timeout*1000));
            if (!radio.available()) {
              LOG_TS("REPORT: Timed out\r\n");
              _ack_timeout_count++;
              break;
            }
            _len = sizeof(_buf);
            radio.recv(&_from, _buf, &_len);
            // make sure ACK came from a designated gateway, or 0xff to accept
            // any gateway
            if ((_len == sizeof(PacketAck) && 
                (_buf[0] == PKT_TYPE_ACK) &&
                ((config.radio_gateway_address == 0xff) || 
                 (_from == config.radio_gateway_address))))
            {
              PacketAck* ack = (PacketAck*)_buf;
              LOG_TS("REPORT: RX ACK src=0x%02x seq=%d\r\n", _from, ack->seq);
              if (_send_seq == ack->seq) {
                LOG_TS("REPORT: ACK received\r\n");
                // it is now safe to remove the acked item
                storage.pop();
                LOG_TS("REPORT: Remove record\r\n");
                _ack_received = true;
                break;
              }
            }
          } // wait for ack

          if (_ack_timeout_count >= MAX_ACK_TIMEOUT_COUNT) {
            LOG_TS("REPORT: Ack timeout limit exceeded; giving up this tx time slot\r\n");
            RADIO_OFF();
            set_wakeup_time(get_total_seconds() + time_wait_for_next_tx());
            TASK_WAIT_UNTIL(get_total_seconds() >= get_wakeup_time());
            reset_wakeup_time();
            break;
          }

        } // (re)transmit until acked
      } // retrieve stored records
    } // main task loop

    TASK_END();
  }
} taskReport;

/***********************************************
 * 
 */
class TaskLogger:public virtual Task {
private:
  uint32_t _ts;
  uint32_t _last_collected;
  uint32_t _gps_on_time;
  uint32_t _next_collection;
  uint8_t _send_seq;
  PacketReport _pkt;
  bool    _first_record;

public:
  virtual const char * get_name() {
    return "Logger";
  }

  virtual TASK(run()) {

    TASK_BEGIN();

    _ts = 0;
    _next_collection = 0;
    _last_collected = 0;
    _send_seq = 0;
    _first_record = true;
    for (;;) {
      GPS.reset_buffer(true);
      GPS_ON();
      GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
      GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
      _gps_on_time = millis();

      LOG_TS("LOGGER: Wait for fix\r\n");

      /* yield while waiting for PPS to wake MCU up */
      pps_detected = false;
      set_wakeup_time(get_total_seconds() + config.gps_max_wait_for_fix);
#if defined(ARDUINO_AVR_FEATHER32U4)
      TASK_WAIT_UNTIL(pps_detected ||
                      (get_total_seconds() >= get_wakeup_time()));
#elif defined(ADAFRUIT_FEATHER_M0)
      attachInterrupt(digitalPinToInterrupt(PIN_1PPS), pps_handler, HIGH);
      TASK_WAIT_UNTIL(pps_detected ||
                      (get_total_seconds() >= get_wakeup_time()));
      detachInterrupt(digitalPinToInterrupt(PIN_1PPS));
#endif
      reset_wakeup_time();

      if (!pps_detected) {
        LOG_TS("LOGGER: PPS not detected\r\n");
        GPS_OFF();
        // schedule time to try again
        _next_collection = get_total_seconds() + config.next_collect_no_fix;
        set_wakeup_time(_next_collection);
        TASK_WAIT_UNTIL(get_total_seconds() >= get_wakeup_time());
        continue;
      }

      // PPS signal detected; start reading and parsing NMEA
      LOG_TS("LOGGER: PPS detected\r\n");

      // reset GPS reading and flush serial buffer
      GPS.reset_buffer(true);

      // record timestamp for timeout checking
      _ts = millis();

      // keep reading GPS until all required information has been received 
      // or timed out
      while (!GPS.read()) {
        if ((millis() - _ts) > ((uint32_t)config.gps_max_wait_for_fix*1000) ) {
          break; // timed out
        }
        TASK_YIELD();
      }

      GPS_OFF();

      if (!GPS.read()) {
        LOG_TS("LOGGER: Cannot acquire a fix\r\n");
        // schedule time to try again
        _next_collection = get_total_seconds() + config.next_collect_no_fix;
        set_wakeup_time(_next_collection);
        TASK_WAIT_UNTIL(get_total_seconds() >= get_wakeup_time());
        reset_wakeup_time();
        continue;
      }

      set_time(GPS.year+2000, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds);
      LOG_TS("LOGGER: Time/loc acquired\r\n");

      // if time is messed up, force reset immediately;
      // otherwise CPU may not wake up after sleep
      if (get_day() == 0 || get_month() == 0) {
        force_reset();
      }

      // to make sure every device collects location data at the same points in
      // time, the next collection time is the number of seconds since midnight
      // (UTC) that is multiple of the configured collect interval
      uint32_t second_now = GPS.hour*3600 + GPS.minute*60 + GPS.seconds;
      uint32_t time_to_fix = (millis() - _gps_on_time);
      uint16_t col_interval = is_day() ?
        config.collect_interval_day : config.collect_interval_night;
      _next_collection = get_total_seconds() + col_interval - (second_now%col_interval);

      _pkt.report.year = GPS.year;
      _pkt.report.month = GPS.month;
      _pkt.report.day = GPS.day;
      _pkt.report.hour = GPS.hour;
      _pkt.report.minute = GPS.minute;
      _pkt.report.second = GPS.seconds;

      // if this is the first record, insert a special "boot" record first
      if (_first_record) {
        _first_record = false;
        _pkt.report.latitude = (1<<31)-1;
        _pkt.report.longitude = (1<<31)-1;
        _pkt.report.vbat = 65535;
        _pkt.report.quality = 0;
        _pkt.report.satellites = 0;
        _pkt.report.temperature = 0;
        _pkt.report.ttf = millis();  // will be used to derive boot time
        storage.push(_pkt.report);
        LOG_TS("LOGGER: Record pushed to storage; record count = %lu\r\n", (uint32_t)storage.count());
      }
      _pkt.report.latitude = GPS.latitude;
      _pkt.report.longitude = GPS.longitude;
      _pkt.report.vbat = ((uint32_t)analogRead(PIN_VBAT))*2*3300/1024;
      _pkt.report.quality = GPS.fixquality;
      _pkt.report.satellites = GPS.satellites;
      _pkt.report.temperature = read_temperature();
      _pkt.report.ttf = time_to_fix;
      storage.push(_pkt.report);
      LOG_TS("LOGGER: Record pushed to storage; record count = %lu\r\n", (uint32_t)storage.count());

      // -----------------
      // TODO: investigate
      // -----------------
      // Why is this line needed?  This line can reduce the power consumption to
      // 0.9 mA.  Without it the power consumption is around 2-3 mA.
      //
      // SD library always pull CS high after every read/write operation anyway.
      digitalWrite(PIN_SD_SS, HIGH);

#if !defined(ADALOGGER)
      // in non-ack mode, send logged data to gateway right away in the next
      // tx slot
      if (!config.use_ack) {
        _send_seq++;
        _pkt.seq = _send_seq;

        static uint32_t wait_time;
        wait_time = time_wait_for_tx();
        if (wait_time < 0) {
          // transmission in non-ack mode may take a long time, especially with
          // long range enabled and multiple tx repeats, so the node should make
          // sure it has a whole time slot available
          wait_time += config.slot_interval;
        }
        set_wakeup_time(get_total_seconds() + time_wait_for_tx());
        TASK_WAIT_UNTIL(get_total_seconds() >= get_wakeup_time());
        reset_wakeup_time();

        for (uint8_t i=0; i<config.tx_repeat; i++) {
          watchdog_reset();
          LOG_TS("LOGGER: TX seq=%d\r\n", _pkt.seq);
          _ts = millis();
          radio.send(config.radio_gateway_address, (uint8_t*)&_pkt, sizeof(_pkt));
          TASK_WAIT_UNTIL(radio.send_done());
          LOG_TS("LOGGER: TX done (%d ms); awaiting ACK\r\n", millis()-_ts);
          SHORT_BLINK(50, 100);
        }
        RADIO_OFF();
      }
#endif

      // wake up earlier to compensate more time to wait for fix, using the
      // current round's wait time as a prediction
      _next_collection -= time_to_fix/1000;
      set_wakeup_time(_next_collection);
      LOG_TS("LOGGER: Time to wait until next round: %ld\r\n", _next_collection - get_total_seconds());
      TASK_WAIT_UNTIL(get_total_seconds() >= get_wakeup_time());
      reset_wakeup_time();
    }

    TASK_END();
  }
} taskLogger;

/***********************************************
 * 
 */
#if defined(ARDUINO_AVR_FEATHER32U4)
ISR (PCINT0_vect) { // handle pin change interrupt for D8 to D13
#elif defined(ADAFRUIT_FEATHER_M0)
void pps_handler() {
#endif
  if (digitalRead(PIN_1PPS) == HIGH) {
    pps_detected = true;
#ifdef DEBUG
    wake_now = true;
#endif
  }
}

/***********************************************
 * 
 */
void setup() {
#if defined(ADAFRUIT_FEATHER_M0)
  uint8_t device_model = 0x01;
  uint8_t reset_flags = REG_PM_RCAUSE;
#elif defined(ARDUINO_AVR_FEATHER32U4)
  uint8_t device_model = 0x02;
  uint8_t reset_flags = MCUSR;
#endif
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_1PPS, INPUT_PULLUP);
  pinMode(PIN_GPS_EN, OUTPUT);
  pinMode(PIN_RFM95_CS, OUTPUT);
  digitalWrite(PIN_RFM95_CS, HIGH);
  GPS_SERIAL.begin(9600);

  LED_OFF(); GPS_OFF();
#ifdef DEBUG
  Serial.begin(9600);
  while (!Serial)
    ; // wait for serial port to connect. Needed for native USB port only
#endif
  LED_ON();
  init_time();
  init_card();
  init_radio();

  LOG_TS("Device initialized with parameters:\r\n");
  LOG(" * Firmware version: %d\r\n", FIRMWARE_VERSION);
  LOG(" * Device model: 0x%02x\r\n", device_model);
  LOG(" * Reset flags: 0x%02x\r\n", reset_flags);
  LOG(" * radio_freq = %d.%02d MHz\r\n",
    (int)config.radio_freq,
    (int)(config.radio_freq*100) % 100);
  LOG(" * radio_tx_power = %d dBm\r\n", config.radio_tx_power);
  LOG(" * radio_device_address = %d (0x%02X)\r\n",
    config.radio_device_address,
    config.radio_device_address);
  LOG(" * radio_gateway_address = %d (0x%02X)\r\n",
    config.radio_gateway_address,
    config.radio_gateway_address);
  LOG(" * collect_interval_day = %d sec\r\n", config.collect_interval_day);
  LOG(" * collect_interval_night = %d sec\r\n", config.collect_interval_night);
  LOG(" * day_start_hour = %d\r\n", config.day_start_hour);
  LOG(" * day_end_hour = %d\r\n", config.day_end_hour);
  LOG(" * time_zone = %d hours\r\n", config.time_zone);
  LOG(" * use_ack = %d\r\n", config.use_ack);
  LOG(" * ack_timeout = %d sec\r\n", config.ack_timeout);
  LOG(" * long_range = %d\r\n", config.long_range);
  LOG(" * tx_repeat = %d\r\n", config.tx_repeat);
  LOG(" * gps_max_wait_for_fix = %d sec\r\n", config.gps_max_wait_for_fix);
  LOG(" * next_collect_no_fix = %d sec\r\n", config.next_collect_no_fix);
  LOG(" * total_slots = %d\r\n", config.total_slots);
  LOG(" * slot_interval = %d sec\r\n", config.slot_interval);
  LOG(" * report size = %d bytes\r\n", sizeof(ReportItem));

#ifndef DEBUG
  USBDevice.detach();
#endif

  Watchdog.enable(WATCHDOG_TIMEOUT);

  // send boot report right away
  PacketBoot pkt;
  pkt.firmware = FIRMWARE_VERSION;
  pkt.device_model = device_model;
  pkt.reset_flags = reset_flags;
  memcpy(&pkt.config, &config, sizeof(config));
  radio.send(config.radio_gateway_address, (uint8_t*)&pkt, sizeof(pkt));
  while (!radio.send_done()) {
    watchdog_reset();
  }

  GPS_OFF();
  RADIO_OFF();

  // turn off LED for a moment to indicate that the boot process is done
  LED_OFF();
  delay(1000);
  LED_ON();

  if (config.use_ack) {
    num_tasks = 2;
    tasks[0] = &taskLogger;
    tasks[1] = &taskReport;
  }
  else {
    num_tasks = 1;
    tasks[0] = &taskLogger;
  }
}

/***********************************************
 * 
 */
void loop() {
  uint32_t min_wakeup_time = (uint32_t)-1;
  uint32_t wakeup_time;
  uint32_t current;
  for (uint8_t i=0; i<num_tasks; i++) {
    // always run a task no matter it is sleeping or not; in case it needs
    // to be woken up by some other source
    tasks[i]->run();
    // determine the earliest wakeup time among all the tasks
    wakeup_time = tasks[i]->get_wakeup_time();
    if (wakeup_time < min_wakeup_time) {
      min_wakeup_time = wakeup_time;
    }
  }
  // if there is some time before the earliest wakeup time, put the CPU to sleep
  current = get_total_seconds();
  if (min_wakeup_time >= current) {
    sleeping_wait(min_wakeup_time - current);
  }
  watchdog_reset();
}

/***********************************************
 * Return the number of seconds to wait for the allowed transmission slot.
 * A negative value indicates the number of seconds elapsed since the
 * beginning of the current slot.
 *
 * Currently it is based on a deterministic time point calculated from the
 * node address.
 */
int16_t time_wait_for_tx() {
  uint32_t current_time = get_total_seconds();
  uint32_t current_slot = current_time/config.slot_interval;
  uint16_t my_slot = config.radio_device_address % config.total_slots;
  int16_t slots_to_wait = 
    (my_slot-current_slot+config.total_slots) % config.total_slots;
  int16_t time_to_wait =
    (slots_to_wait+current_slot)*config.slot_interval - current_time;
  return time_to_wait;
}

/***********************************************
 * Return the number of seconds to wait for the NEXT allowed transmission
 * slot.
 */
int16_t time_wait_for_next_tx() {
  if (time_wait_for_tx() <= 0) {
    uint32_t round_time = config.total_slots*config.slot_interval;
    return round_time + time_wait_for_tx();
  }
  else {
    return time_wait_for_tx();
  }
}

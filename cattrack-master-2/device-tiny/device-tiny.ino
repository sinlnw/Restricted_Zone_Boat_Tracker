/* CAUTIONS
 * ========
 * (1)
 * The SleepyDog library requires ASFcore.  However, both RadioHead and
 * ASFcore define TC3_Handler, which causes a conflict.  Edit ASFcore's
 * tc_interrupt.c and look for:
 *
 *   #define _TC_INTERRUPT_HANDLER(n, m) \
 *
 * then add the following line BEFORE the line above:
 *
 *   #define TC3_Handler TC3_Handler_not_used
 *
 * (2)
 * "Time" library used by RTCZero (for M0) has a header Time.h, which is
 * mixed up with the system's time.h header.
 *  - Rename Time.h inside <libraries>/Time/ to something else, e.g., TimeX.h
 *  - Modify all *.cpp files in <libraries>/Time/ so that they include the new
 *    name instead.
 */
#include <Adafruit_SleepyDog.h>
#include <TimeLib.h>

// Define this macro when using Adalogger model instead of Lora model
//#define ADALOGGER

// Partition info may have to be hard-coded due to insufficient flash and RAM
#define HARDCODED_PART_INFO

#include "version.h"
#include "radio.h"
#include "config.h"
#include "gps.h"
#include "io.h"
#include "card-utils.h"
#include "time-utils.h"
#include "storage.h"

//#define DEBUG
//#define SD_LOADER
//#define DEBUG_SD_LOADER

#define WATCHDOG_TIMEOUT   10000
#define GPS_SERIAL         Serial1

#ifdef DEBUG
#define DEBUG_PRINT(x...) Serial.print(x)
#define DEBUG_PRINTLN(x...) Serial.println(x)
#else
#define DEBUG_PRINT(x...)
#define DEBUG_PRINTLN(x...)
#endif

#if defined(SD_LOADER) && defined(ADAFRUIT_FEATHER_M0)
__attribute__ ((section(".sketch_boot")))
#ifdef DEBUG_SD_LOADER
unsigned char sduBoot[0xC000] = {
#else
unsigned char sduBoot[0xA000] = {
#endif
#include "boot/feather_m0.h"
};
#endif

Sd2Card card;
Config config;
Radio radio;
Storage<ReportItem> storage(card);
time_t next_collection = 0;
uint8_t sent_seq = 0;
volatile bool pps_detected = false;
volatile bool wake_now = false;

/***********************************************
 * 
 */
void error_blink_loop(uint8_t code) {
  DEBUG_PRINT(F("ERROR: code "));
  DEBUG_PRINTLN(code);
  for (;;) {
    LED_ON();
    delay(200);
    LED_OFF();
    delay(200);
  }
}

#if defined(ARDUINO_AVR_FEATHER32U4)
/***********************************************
 * From https://playground.arduino.cc/Main/PinChangeInterrupt/
 */
void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}
#endif

/************************************
 *
 */
void init_card() {
  SdVolume volume;
  if (!card.init(SPI_HALF_SPEED,PIN_SD_SS)) {
    //DEBUG_PRINT(F("Cannot init card"));
    error_blink_loop(1);
  }

  uint8_t  config_part_idx;
  uint32_t data_start, data_size;
  uint32_t log_start, log_size;

#ifdef HARDCODED_PART_INFO
  config_part_idx = 1;
  data_start = 34816;
  data_size = 8388608;
  log_start = 0;
  log_size = 0;
#else
  get_data_and_log_partitions(
    card,
    &config_part_idx,
    &data_start, &data_size,
    &log_start, &log_size);
#endif

  /******************************
   * Process configuration on SD
   ******************************/
  if (config_part_idx == 0) {
    //DEBUG_PRINT(F("Cannot find configuration partition id 0x"));
    //DEBUG_PRINTLN(CONFIG_PARTITION_ID,HEX);
    error_blink_loop(2);
  }
  if (!volume.init(card,config_part_idx)) {
    //DEBUG_PRINTLN(F("Cannot open configuration volume"));
    error_blink_loop(3);
  }
  if (!config.read_from_volume(volume)) {
    //DEBUG_PRINTLN(F("failed to read config file"));
    error_blink_loop(4);
  }

  /******************************
   * Prepare data storage service 
   ******************************/
  if (data_start == 0) {
    //DEBUG_PRINT(F("Cannot find data partition id 0x"));
    //DEBUG_PRINTLN(STORAGE_PARTITION_ID,HEX);
    error_blink_loop(5);
  }
  DEBUG_PRINT(F("Setting up data partition at block #"));
  DEBUG_PRINT(data_start);
  DEBUG_PRINT(F(" total "));
  DEBUG_PRINT(data_size);
  DEBUG_PRINT(F(" blocks.."));
  if (!storage.setup(data_start,data_size,false)) {
    //DEBUG_PRINTLN(F("failed."));
    error_blink_loop(6);
  }
  DEBUG_PRINTLN(F("successful."));
}

/************************************
 *
 */
void init_radio() {
  DEBUG_PRINTLN(F("Init radio."));
  if (!radio.init(
      config.radio_device_address,
      config.radio_freq,
      config.radio_tx_power,
      config.long_range))
  {
    //DEBUG_PRINTLN(F("failed."));
    error_blink_loop(7);
  }
  //DEBUG_PRINTLN(F("successful."));
}

/*********************************************************************
 * Force watchdog reset
 */
void force_reset() {
  //DEBUG_PRINTLN(F("Something went wrong; force reset"));
  Watchdog.enable(1000);
  while (true)
    ;
}

/*********************************************************************
 * Put the device into sleep mode for the specified number of seconds
 */
void sleeping_wait(uint16_t seconds) {
  if (seconds == 0) return;
#if defined(DEBUG)
  // Cannot really sleep in debug mode
  uint32_t ts = millis();
  wake_now = false;
  while ((uint32_t)(millis() - ts) < (uint32_t)seconds*1000) {
    Watchdog.reset();
    if (wake_now) break;
  }
  wake_now = false;
#elif defined(ARDUINO_AVR_FEATHER32U4)
  time_t until = get_total_seconds()+seconds;
  time_t remain;
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
#elif defined(ADAFRUIT_FEATHER_M0)
  time_t end_time = rtc.getEpoch() + seconds;
  while (true) {

    // wake up every nap_time to prevent dying during sleep
    uint16_t nap_time = WATCHDOG_TIMEOUT/1000/2 < seconds ?
      WATCHDOG_TIMEOUT/1000/2 : seconds;

    // set alarm for the next nap_time
    time_t current_time = rtc.getEpoch();
    rtc.setAlarmEpoch(current_time + nap_time);
    rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);

    // ready to fall asleep
    rtc.standbyMode();
    // now waked up
    Watchdog.reset();

    current_time = rtc.getEpoch();

    // if enough sleep; break out of the loop
    if (current_time >= end_time)
      break;

    // fell asleep but remaining time got longer
    // this means something went wrong; reset right away
    if (end_time - current_time > seconds) {
      force_reset();
    }

    seconds = end_time - current_time;
  }
#endif
}

#if defined(ARDUINO_AVR_FEATHER32U4)
/*********************************************************************
 *
 */
uint16_t read_temperature() {
  uint16_t wADC;

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
  while (bit_is_set(ADCSRA,ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;

  return wADC;
}

#elif defined(ADAFRUIT_FEATHER_M0)
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

/*********************************************************************
 *
 */
void log_and_report() {
  GPSClass GPS(GPS_SERIAL);
  time_t ts;
  time_t gps_on_time = millis();

  // turn on GPS and wait for a fix
  //LED_ON();
  GPS_ON();
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  DEBUG_PRINTLN(F("Wait for fix"));

  /* put MCU to sleep and wait for PPS to wake it up */
  pps_detected = false;
#if defined(ARDUINO_AVR_FEATHER32U4)
  sleeping_wait(config.gps_max_wait_for_fix);
#elif defined(ADAFRUIT_FEATHER_M0)
  attachInterrupt(digitalPinToInterrupt(PIN_1PPS), pps_handler, HIGH);
  sleeping_wait(config.gps_max_wait_for_fix);
  detachInterrupt(digitalPinToInterrupt(PIN_1PPS));
#endif

  if (!pps_detected) {
    DEBUG_PRINTLN(F("PPS not detected"));
    LED_OFF(); GPS_OFF();
    // schedule time to try again
    next_collection = get_total_seconds() + config.next_collect_no_fix;
    return;
  }

  // PPS signal detected; start reading and parsing NMEA
  DEBUG_PRINTLN(F("PPS detected"));

  // reset GPS reading and flush serial buffer
  GPS.reset_buffer(true);

  // record timestamp for timeout checking
  ts = millis();

  // keep reading GPS until all required information has been received 
  // or timed out
  while (!GPS.read()) {
    Watchdog.reset();
    if ( (millis() - ts) > ((uint32_t)config.gps_max_wait_for_fix*1000) ) {
      //DEBUG_PRINTLN(F("Cannot acquire a fix; turn off GPS"));
      LED_OFF(); GPS_OFF();
      // schedule time to try again
      next_collection = get_total_seconds() + config.next_collect_no_fix;
      return;
    }
  }

  DEBUG_PRINTLN(F("Time/loc acquired"));
  LED_OFF(); GPS_OFF();

  set_time(GPS.year+2000, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds);

  // if time is messed up, force reset immediately;
  // otherwise CPU may not wake up after sleep
  if (get_day() == 0 || get_month() == 0) {
    force_reset();
  }

  // to make sure every device collects location data at the same points in
  // time, the next collection time is the number of seconds since midnight
  // (UTC) that is multiple of the configured collect interval
  uint32_t second_now = GPS.hour*3600 + GPS.minute*60 + GPS.seconds;
  uint32_t time_to_fix = (millis() - gps_on_time);
  uint16_t col_interval = is_day() ?
    config.collect_interval_day : config.collect_interval_night;
  next_collection = get_total_seconds() + col_interval - (second_now%col_interval);

  // wake up earlier to compensate more time to wait for fix, using the
  // current round's wait time as a prediction
  next_collection -= time_to_fix/1000;

  // log data and send to gateway
  PacketReport pkt;
  pkt.report.year = GPS.year;
  pkt.report.month = GPS.month;
  pkt.report.day = GPS.day;
  pkt.report.hour = GPS.hour;
  pkt.report.minute = GPS.minute;
  pkt.report.second = GPS.seconds;
  pkt.report.latitude = GPS.latitude;
  pkt.report.longitude = GPS.longitude;
  pkt.report.vbat = ((uint32_t)analogRead(PIN_VBAT))*2*3300/1024;
  pkt.report.quality = GPS.fixquality;
  pkt.report.satellites = GPS.satellites;
  pkt.report.temperature = read_temperature();
  pkt.report.last_heard_from_gw = time_to_fix; // this field is unused in non-ack mode
  storage.push(pkt.report);
  //DEBUG_PRINT("year: "); DEBUG_PRINTLN(GPS.year);
  //DEBUG_PRINT("month: "); DEBUG_PRINTLN(GPS.month);
  //DEBUG_PRINT("day: "); DEBUG_PRINTLN(GPS.day);
  //DEBUG_PRINT("hour: "); DEBUG_PRINTLN(GPS.hour);
  //DEBUG_PRINT("minute: "); DEBUG_PRINTLN(GPS.minute);
  //DEBUG_PRINT("second: "); DEBUG_PRINTLN(GPS.seconds);
  //DEBUG_PRINT("lat: "); DEBUG_PRINTLN(GPS.latitude);
  //DEBUG_PRINT("long: "); DEBUG_PRINTLN(GPS.longitude);
  DEBUG_PRINT(F("Record pushed to storage; "));
  DEBUG_PRINT(F("record count = "));
  DEBUG_PRINTLN((uint32_t)storage.count());

  // -----------------
  // TODO: investigate
  // -----------------
  // Why is this line needed?  This line can reduce the power consumption to
  // 0.9 mA.  Without it the power consumption is around 2-3 mA.
  //
  // SD library always pull CS high after every read/write operation anyway.
  digitalWrite(PIN_SD_SS,HIGH);

#if !defined(ADALOGGER)
  sent_seq++;
  pkt.seq = sent_seq;

  sleeping_wait(time_wait_for_tx());

  for (uint8_t i=0; i<config.tx_repeat; i++) {
    Watchdog.reset();
    DEBUG_PRINTLN(F("Send to gateway"));
    radio.send(config.radio_gateway_address,(uint8_t*)&pkt,sizeof(pkt));
    while (!radio.send_done()) {
      Watchdog.reset();
    }
    //DEBUG_PRINTLN(F("Data sent to gateway"));
    SHORT_BLINK(50,100);
  }
  RADIO_OFF();
#endif
}

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
bool is_day() {
  if (get_year() == 2000)
    // time is not yet known, assume daytime
    return true;

  uint8_t hours = (get_hours() + config.time_zone) % 24;
  if ( (hours >= config.day_start_hour) && 
       (hours < config.day_end_hour))
    return true;

  return false;
}

/*********************************************************************
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
#if defined(ADALOGGER)
  pinMode(PIN_SD_LED,OUTPUT);
  digitalWrite(PIN_SD_LED,LOW);
#else
  pinMode(PIN_RFM95_CS,OUTPUT);
  digitalWrite(PIN_RFM95_CS,HIGH);
#endif
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(PIN_1PPS,INPUT_PULLUP);
  pinMode(PIN_GPS_EN,OUTPUT);
  pinMode(PIN_SD_SS,OUTPUT);
  digitalWrite(PIN_SD_SS,HIGH);
  LED_ON(); GPS_OFF();

#ifdef DEBUG
  Serial.begin(9600);
  while (!Serial)
    ;
  DEBUG_PRINTLN(F("device starting up..."));
#endif
  GPS_SERIAL.begin(9600);

  init_time();
  SHORT_BLINK(50,100);
  init_card();
  SHORT_BLINK(50,100);
#if !defined(ADALOGGER)
  init_radio();
  //SHORT_BLINK(50,100);
#endif

#ifdef DEBUG
  //DEBUG_PRINTLN(F("Device initialized with parameters:"));
  //DEBUG_PRINT(F(" * Firmware version: ")); DEBUG_PRINTLN(FIRMWARE_VERSION);
  //DEBUG_PRINT(F(" * Device model: 0x")); DEBUG_PRINTLN(device_model,HEX);
  //DEBUG_PRINT(F(" * Reset flags:")); DEBUG_PRINTLN(reset_flags);
  //DEBUG_PRINT(F(" * radio_freq (MHz) = "));
  //  DEBUG_PRINT((uint16_t)config.radio_freq);
  //  DEBUG_PRINT('.');
  //  DEBUG_PRINTLN((uint16_t)(config.radio_freq*100)%100);
  //DEBUG_PRINT(F(" * radio_tx_power (dBm) = ")); DEBUG_PRINTLN(config.radio_tx_power);
  //DEBUG_PRINT(F(" * radio_device_address = 0x")); DEBUG_PRINTLN(config.radio_device_address,HEX);
  //DEBUG_PRINT(F(" * radio_gateway_address = 0x")); DEBUG_PRINTLN(config.radio_gateway_address,HEX);
  //DEBUG_PRINT(F(" * collect_interval_day (sec) = ")); DEBUG_PRINTLN(config.collect_interval_day);
  //DEBUG_PRINT(F(" * collect_interval_night (sec) = ")); DEBUG_PRINTLN(config.collect_interval_night);
  //DEBUG_PRINT(F(" * day_start_hour = ")); DEBUG_PRINTLN(config.day_start_hour);
  //DEBUG_PRINT(F(" * day_end_hour = ")); DEBUG_PRINTLN(config.day_end_hour);
  //DEBUG_PRINT(F(" * time_zone = ")); DEBUG_PRINTLN(config.time_zone);
  //DEBUG_PRINT(F(" * long_range = ")); DEBUG_PRINTLN(config.long_range);
  //DEBUG_PRINT(F(" * tx_repeat = ")); DEBUG_PRINTLN(config.tx_repeat);
  //DEBUG_PRINT(F(" * gps_max_wait_for_fix (sec) = ")); DEBUG_PRINTLN(config.gps_max_wait_for_fix);
  //DEBUG_PRINT(F(" * next_collect_no_fix (sec) = ")); DEBUG_PRINTLN(config.next_collect_no_fix);
  //DEBUG_PRINT(F(" * total_slots = ")); DEBUG_PRINTLN(config.total_slots);
  //DEBUG_PRINT(F(" * slot_interval (sec) = ")); DEBUG_PRINTLN(config.slot_interval);
  //DEBUG_PRINT(F(" * report size (bytes) = ")); DEBUG_PRINTLN(sizeof(ReportItem));
#else
  USBDevice.detach();
#endif

  Watchdog.enable(WATCHDOG_TIMEOUT);
#if defined(ARDUINO_AVR_FEATHER32U4)
  pciSetup(PIN_1PPS);
#endif

#if !defined(ADALOGGER)
  // send boot report right away
  PacketBoot pkt;
  pkt.firmware = FIRMWARE_VERSION;
  pkt.device_model = device_model;
  pkt.reset_flags = reset_flags;
  memcpy(&pkt.config, &config, sizeof(config));
  radio.send(config.radio_gateway_address,(uint8_t*)&pkt,sizeof(pkt));
  while (!radio.send_done()) {
    Watchdog.reset();
  }
#endif

  next_collection = 0;
  RADIO_OFF();
  LED_OFF();
}

/***********************************************
 * 
 */
void loop() {
  time_t current = get_total_seconds();
  if (next_collection >= current) {
    sleeping_wait(next_collection - current);
  }
  log_and_report();
}

/***********************************************
 * Compute the wait time in seconds before beginning transmission
 *
 * Currently it is based on a deterministic time point calculated from the
 * node address.
 */
uint16_t time_wait_for_tx() {
  uint32_t current_time = get_total_seconds();
  uint32_t current_slot = current_time/config.slot_interval;
  uint32_t my_slot = config.radio_device_address % config.total_slots;
  uint32_t slots_to_wait = 
    (my_slot-current_slot+config.total_slots) % config.total_slots + 1;
  uint32_t time_to_wait =
    (slots_to_wait+current_slot)*config.slot_interval - current_time;
  return time_to_wait;
}

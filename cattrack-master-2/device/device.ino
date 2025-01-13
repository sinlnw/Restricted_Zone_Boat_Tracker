/* CAUTION
 * =======
 * The SleepyDog library requires ASFcore.  However, both RadioHead and
 * ASFcore define TC3_Handler, which causes a conflict.  Edit ASFcore's
 * tc_interrupt.c and look for:
 *
 *   #define _TC_INTERRUPT_HANDLER(n, m) \
 *
 * then add the following line BEFORE the line above:
 *
 *   #define TC3_Handler TC3_Handler_not_used
 */
#include <Adafruit_SleepyDog.h>
#include <ArduinoJson.h>
#include "version.h"

#define DEBUG
//#define STORAGE_DEBUG
//#define LOGGING
//#define SD_LOADER

#define WATCHDOG_TIMEOUT   10000
#define GPS_SERIAL  Serial1

#include "pt/pt.h"
#include "gps.h"
#include "logger.h"
#include "radio.h"
#include "config.h"
#include "io.h"
#include "time-utils.h"
#include "card-utils.h"

#include "debug.h"
#include "storage.h"

#if defined(SD_LOADER) && defined(ADAFRUIT_FEATHER_M0)
__attribute__ ((section(".sketch_boot")))
unsigned char sduBoot[0xA000] = {
#include "boot/feather_m0.h"
};
#endif

Sd2Card card;
Config config;
Storage<ReportItem> storage(card);
Radio radio;
GPSClass GPS(GPS_SERIAL);
time_t next_collection = 0;
uint32_t last_heard_from_gw = 0;
uint8_t sent_seq = 0;
uint8_t acked_seq = 0;
struct pt ptRadio;
struct pt ptReport;
struct pt ptLogger;

//rachata start
JsonDocument all_areas_doc;
JsonArray all_areas;
uint32_t buzzer_starttime = 0;
bool buzzer_on = false;
#define AREA_FILE_NAME "/AREA.txt"
#define BUZZER_PIN 6
#define BUZZER_DURATION 1 // in seconds
#define IN_AREA_INTERVAL 10 // in seconds
// #define ADAFRUIT_FEATHER_M0
//rachata end

volatile bool pps_detected = false;
#ifdef DEBUG
volatile bool wake_now = false;
#endif

#ifdef LOGGING
Logger logger(card);
#endif

#define TIMER_START(ts)       ts = millis();
#define TIMER_EXPIRED(ts,timeout)  (millis()-ts > timeout)
#define PT_DELAY(pt,ms,tsVar) \
  tsVar = millis(); \
  PT_WAIT_UNTIL(pt, millis()-tsVar >= (ms));

/***********************************************
 * 
 */
void init_gps() {
  LOG_TS("Initializing GPS...\r\n");
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  //delay(500);
  LOG_TS("done.\r\n");
}

/***********************************************
 * 
 */
void init_card() {
  // logger still not available
  DEBUG_PRINT("Initializing SD card...");
  if (!card.init(SPI_HALF_SPEED,PIN_SD_SS)) {
    DEBUG_PRINTLN("failed!");
    error_blink_loop();
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
    error_blink_loop();
  }
  DEBUG_PRINTF("Setting up log partition at block #%d, total %d bytes...",
    log_start, ui64toa(log_size*512ULL));
  if (!logger.setup(log_start,log_size,false)) {
    DEBUG_PRINTLN("failed!");
    error_blink_loop();
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
    error_blink_loop();
  }
  LOG_TS("Found configuration partition index %d\r\n", config_part_idx);
  SdVolume volume;
  if (!volume.init(card,config_part_idx)) {
    LOG_TS("Cannot open configuration volume\r\n");
    error_blink_loop();
  }
  LOG_TS("Reading configuration file...");
  if (!config.read_from_volume(volume)) {
    LOG("failed!\r\n");
    error_blink_loop();
  }
  LOG("successful.\r\n");

  /******************************
   * Prepare data storage service 
   ******************************/
  if (data_start == 0) {
    LOG_TS("Cannot find data partition id 0x%02x\r\n",
      STORAGE_PARTITION_ID);
    error_blink_loop();
  }
  LOG_TS("Setting up data partition at block #%d, total %d blocks..",
    data_start, data_size);
  if (!storage.setup(data_start,data_size,false)) {
    LOG("failed.\r\n");
    error_blink_loop();
  }
  LOG("successful.\r\n");


  ////rachata start
  
  // Read the area file

  /* #define AREA_FILE_NAME2 "AREA.txt"
  SdFile area_file;
  LOG_TS("Reading area file...");
  if (!open_file_in_root(area_file,volume,AREA_FILE_NAME2,O_READ)) {
    DEBUG_PRINTLN("Cannot open area file");
  }

  // Read the file until there's nothing else in it
  while (area_file.available()) {
    Serial.write(area_file.read());
  }
  area_file.close(); */
  LOG_TS("Initializing SD card for reading area file...");

  if (!SD.begin(PIN_SD_SS)) {
    LOG_TS("initialization failed!");
    return;
  }
  LOG("done.\r\n");

  // Open the file for reading
  File area_file = SD.open(AREA_FILE_NAME);
  if (!area_file) {
    LOG_TS("Error: Could not open area file.\r\n");
    return;
  }

  LOG_TS("Reading area file contents:...");
  

  DeserializationError error = deserializeJson(all_areas_doc, area_file);
  if (error) {
    LOG_TS("Failed to read file, error: %s\r\n", error.c_str());
  }
  all_areas = all_areas_doc.as<JsonArray>();
  int area_count = 0;
  for (JsonObject myarea : all_areas){
    area_count++;
    LOG("Area %d\r\n", area_count);
    JsonArray all_polygon = myarea["all_drawings"].as<JsonArray>(); // read all polygons of each area
    

    int polygon_count = 0;
    for(JsonObject mypolygon : all_polygon){
      polygon_count++;
      LOG("Polygon %d\r\n", polygon_count);
      const char* geometry_type = mypolygon["geometry"]["type"];
      LOG(geometry_type);
      LOG("\r\n");
      // Access the coordinates array
      JsonArray coordinates = mypolygon["geometry"]["coordinates"][0];
  
      for (JsonArray coordinate : coordinates) {
        float longitude = coordinate[0].as<float>();
        float latitude = coordinate[1].as<float>();
        LOG("Longitude: %f, Latitude: %f\r\n", longitude, latitude);
      }
    }
  }
  
  
  
  // Close the file
  area_file.close();
  LOG("Open area file successful\r\n"); 
  ////rachata end
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
    error_blink_loop();
  }
  LOG("successful; NodeId = 0x%02X\r\n", config.radio_device_address);
}

/***********************************************
 * 
 */
void error_blink_loop() {
  LOG_TS("ERROR! Halt all further processing\r\n");
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
  while (bit_is_set(ADCSRA,ADSC));

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
 * Put the device into sleep mode for the specified number of seconds
 */
void sleeping_wait(uint16_t seconds) {
  if (seconds == 0) return;
  LOG_TS("Going to sleep for %d second(s)\r\n", seconds);
#if defined(DEBUG)
  // Cannot sleep for real in debug mode
  uint32_t ts = millis();
  wake_now = false;
  while (millis() - ts < seconds*1000) {
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
  LOG_TS("Waking up\r\n");
}

/***********************************************
 * 
 */
bool is_day() {
  if (get_year() == 2000)
    // time is not yet known, assume daytime
    return true;

  uint8_t hour = (get_hours() + config.time_zone) % 24;
  if ( (hour >= config.day_start_hour) && 
       (hour < config.day_end_hour))
    return true;

  return false;
}

/***********************************************
 * 
 */
PT_THREAD(taskRadio(struct pt* pt)) {

  static uint32_t ts = 0;

  PT_BEGIN(pt);

  for (;;) {
    PT_WAIT_UNTIL(pt,radio.available());
    uint8_t buf[64];
    Address from;
    uint8_t len = sizeof(buf);
    radio.recv(&from,buf,&len);
    if (!len || from != 0x00) continue;  // listen to gateway only
    last_heard_from_gw = millis();
    if (len == sizeof(PacketAdvertise) && buf[0] == PKT_TYPE_ADVERTISE) {
      PacketAdvertise* adv = (PacketAdvertise*)buf;
      LOG_TS("RX: ADV src=0x%02x seq=%d\r\n",from,adv->seq);
    }
    else if (len == sizeof(PacketAck) && buf[0] == PKT_TYPE_ACK) {
      PacketAck* ack = (PacketAck*)buf;
      acked_seq = ack->seq;
      LOG_TS("RX: ACK src=0x%02x seq=%d\r\n",from,ack->seq);
    }
  }

  PT_END(pt);
}

/***********************************************
 * 
 */
PT_THREAD(taskReport(struct pt* pt)) {
  static uint32_t ts = 0;
  static PacketReport pkt;

  PT_BEGIN(pt);

  for (;;) {
    PT_WAIT_UNTIL(pt,!storage.empty());

    LOG_TS("REPORT: record count = %d\r\n", storage.count());

    storage.top(&pkt.report);
    sent_seq++;
    pkt.seq = sent_seq;

    // try sending when gateway was recently heard, until ack is received
    while (acked_seq != sent_seq) {
      LOG_TS("REPORT: awaiting gateway\r\n");
      PT_WAIT_UNTIL(pt,
        last_heard_from_gw != 0 &&
        millis()-last_heard_from_gw < 2*(config.advertise_interval*1000));
      LOG_TS("REPORT: TX seq=%d\r\n",pkt.seq);
      radio.send(config.radio_gateway_address,(uint8_t*)&pkt,sizeof(pkt));
      PT_WAIT_UNTIL(pt,radio.send_done());
      LOG_TS("REPORT: done; awaiting ACK\r\n");
      PT_SHORT_BLINK(pt,ts,10);
      PT_DELAY(pt,config.ack_timeout*1000,ts);
    }

    LOG_TS("REPORT: ACK received\r\n");

    // it is now safe to remove the acked item
    storage.pop();
    LOG_TS("REPORT: remove record\r\n");
  }

  PT_END(pt);
}

// Rachata start
bool isPointInAreas(float test_long, float test_lat, int test_day, int test_month)
{
  // check if point is in any of area polygons

  bool c = false; // the counter of the crossing , true if the point is in the area
  LOG("test isPointInAreas test Longitude: %f, Latitude: %f\r\n", test_long, test_lat);
  
  // Loop through all areas
  for (JsonObject myarea : all_areas){

    //if today is not in the day_month_range, skip the area
    int start_day = myarea["start_day"].as<int>();
    int start_month = myarea["start_month"].as<int>();
    int end_day = myarea["end_day"].as<int>();
    int end_month = myarea["end_month"].as<int>();
    if (!is_date_in_day_month_range(test_day, test_month, start_day, start_month, end_day, end_month)){
      LOG("skip area with %d/%d - %d/%d\r\n", start_day, start_month, end_day, end_month);
      continue;
    }
    JsonArray all_polygon = myarea["all_drawings"].as<JsonArray>();
    for(JsonObject mypolygon : all_polygon){
        // Access the coordinates array
        JsonArray coordinates = mypolygon["geometry"]["coordinates"][0];

        /* // Ensure the polygon has at least 3 vertices
        if (coordinates.size() < 3) {
          continue;
        } */

        float prev_longitude = 0.0;
        float prev_latitude = 0.0;
        bool first_coordinate = true;

        for (JsonArray coordinate : coordinates) {

          float longitude = coordinate[0].as<float>();
          float latitude = coordinate[1].as<float>();
          if (longitude < -180){
            longitude = 360 + longitude;
          }
          // Check if the test point is inside the area polygon using the ray-casting algorithm
          if (!first_coordinate){
            if ( ((latitude>test_lat) != (prev_latitude>test_lat)) &&
              (test_long < (prev_longitude-longitude) * (test_lat-latitude) / (prev_latitude-latitude) + longitude) )
              c = !c;
          }
          first_coordinate = false;
          LOG("test isPointInAreas prev Longitude: %f, Latitude: %f\r\n", prev_longitude, prev_latitude);
          LOG("test isPointInAreas Longitude: %f, Latitude: %f\r\n", longitude, latitude);

          // Update the previous coordinate
          prev_longitude = longitude;
          prev_latitude = latitude;

        }
      }
  }

  return c;
}

bool is_date_in_day_month_range(int test_day, int test_month, int start_day, int start_month, int end_day, int end_month) {
    if (start_month < 1 || start_month > 12 || end_month < 1 || end_month > 12 ||
        start_day < 1 || start_day > 31 || end_day < 1 || end_day > 31 ||
        test_month < 1 || test_month > 12 || test_day < 1 || test_day > 31) {
        // invalid date range
        return false;
    }

    if (start_month > end_month || (start_month == end_month && start_day > end_day)) {
        // day_month_range cross year
        return is_date_in_day_month_range(test_day, test_month, start_day, start_month, 31, 12) ||
               is_date_in_day_month_range(test_day, test_month, 1, 1, end_day, end_month);
    } else {
        // day_month_range same year
        return (start_month <= test_month && test_month <= end_month) &&
               ((test_month == start_month && test_day >= start_day) ||
                (test_month == end_month && test_day <= end_day) ||
                (start_month < test_month && test_month < end_month));
    }
}
// Rachata end
/***********************************************
 * 
 */
PT_THREAD(taskLogger(struct pt* pt)) {

  static uint32_t ts = 0;
  static uint32_t last_collected = 0;
  static bool coord_in_area = false;

  PT_BEGIN(pt);
  LOG("test logger begin \r\n");
  for (;;) {
    LOG("\ntest loop start \r\n");
    GPS.reset_buffer(true);  // reset GPS reading and flush serial buffer
    LOG("test loop start reading gps\r\n");
    PT_WAIT_UNTIL(pt,GPS.read()); // start over from the newest report
    LOG("test gps finished read\r\n");
    uint16_t collect_interval = 
      is_day() ? config.collect_interval_day : config.collect_interval_night;
    collect_interval = 30; //rachata override for testing
    //rachata start //OR: make buzzer ON OFF using PT_DELAY
    if (coord_in_area){
      LOG("Point is in the area active buzzer\r\n");
      collect_interval = IN_AREA_INTERVAL; // if in the area, collect data more frequently
      //turn buzzer on and off
        if(!buzzer_on && (millis() - buzzer_starttime > (IN_AREA_INTERVAL - BUZZER_DURATION)*1000)){
          LOG("Buzzer on\r\n");
          digitalWrite(BUZZER_PIN, HIGH);
          digitalWrite(LED_BUILTIN,HIGH);
          buzzer_starttime = millis();
          buzzer_on = true;
        }
        else if(buzzer_on && millis() - buzzer_starttime > BUZZER_DURATION*1000){
          LOG("Buzzer off\r\n");
          digitalWrite(BUZZER_PIN, LOW);
          digitalWrite(LED_BUILTIN,LOW);
          buzzer_starttime = millis();
          buzzer_on = false;
        }
    } else if (buzzer_on) {
      LOG("not in area deactivate Buzzer\r\n");
      // not in area , turn off the buzzer if it is on
        digitalWrite(BUZZER_PIN, LOW);
        buzzer_on = false;
    }
    //rachata end

    if (last_collected && (millis() - last_collected < collect_interval*1000)){
      LOG("wait for more time to collect gps data \r\n\n"); // rachata for testing
      continue;}
    LOG_TS("Synchronizing time...\r\n");
    set_time(GPS.year+2000, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds);
    LOG_TS("Time synchronized\r\n");
    last_collected = millis();
    ReportItem report;
    report.year = GPS.year;
    report.month = GPS.month;
    report.day = GPS.day;
    report.hour = GPS.hour;
    report.minute = GPS.minute;
    report.second = GPS.seconds;
    report.latitude = GPS.latitude;
    report.longitude = GPS.longitude;
    report.vbat = ((uint32_t)analogRead(PIN_VBAT))*2*3300/1024;
    report.quality = GPS.fixquality;
    report.satellites = GPS.satellites;
    report.temperature = read_temperature();
    report.last_heard_from_gw = millis()-last_heard_from_gw;

    //rachata start 
    //check if the point is in the area polygons
    coord_in_area = isPointInAreas(GPS.longitude/10000000.0, GPS.latitude/10000000.0,report.day,report.month);// TODO: deal with the unit of the location
    LOG("Coord is %d\r\n", coord_in_area);
    SHORT_BLINK(50,100);
    //rachata end
    storage.push(report);
    LOG_TS("Record pushed to storage; record count = %d\r\n",storage.count());
    
  }

  PT_END(pt);
}

/************************************************************************
 * Turn on GPS and try to acquire a fix; then record data on SD card and
 * report to gateway in a best-effort manner
 */
void log_and_report() {
  time_t ts;

  // turn on GPS and wait for a fix
  //LED_ON();
  GPS_ON();
  init_gps();

  LOG_TS("Waiting for fix...\r\n");

  /* put MCU to sleep and wait for PPS to wake it up */
  pps_detected = false;
  attachInterrupt(digitalPinToInterrupt(PIN_1PPS), pps_handler, HIGH);
  sleeping_wait(config.gps_max_wait_for_fix);
  detachInterrupt(digitalPinToInterrupt(PIN_1PPS));

  if (!pps_detected) {
    LOG_TS("Cannot detect PPS; turn off GPS\r\n");
    LED_OFF(); GPS_OFF();
    // schedule time to try again
    next_collection = get_total_seconds() + config.next_collect_no_fix;
    return;
  }

  // PPS signal detected; start reading and parsing NMEA
  LOG_TS("PPS detected\r\n");

  // reset GPS reading and flush serial buffer
  GPS.reset_buffer(true);

  // record timestamp for timeout checking
  ts = millis();

  // keep reading GPS until all required information has been received 
  // or timed out
  while (!GPS.read()) {
    Watchdog.reset();
    if ( (millis() - ts) > (config.gps_max_wait_for_fix*1000) ) {
      LOG_TS("Cannot acquire a fix; turn off GPS\r\n");
      LED_OFF(); GPS_OFF();
      // schedule time to try again
      next_collection = get_total_seconds() + config.next_collect_no_fix;
      return;
    }
  }

  // time and location successfully acquired; turn GPS off
  LOG_TS("Time and location acquired: %d-%02d-%02d %02d:%02d:%02d (%d.%07d,%d.%07d)\r\n",
     GPS.year+2000,
     GPS.month,
     GPS.day,
     GPS.hour,
     GPS.minute,
     GPS.seconds,
     GPS.latitude/10000000,
     GPS.latitude%10000000,
     GPS.longitude/10000000,
     GPS.longitude%10000000
  );
  LED_OFF(); GPS_OFF();

  LOG_TS("Synchronizing time...\r\n");
  set_time(GPS.year+2000, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds);
  LOG_TS("Time synchronized\r\n");

  // if time is messed up, force reset immediately;
  // otherwise CPU may not wake up after sleep
  if (get_day() == 0 || get_month() == 0) {
    force_reset();
  }

  next_collection = get_total_seconds() + 
    (is_day() ? config.collect_interval_day : config.collect_interval_night);

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
  pkt.report.last_heard_from_gw = 0;
  storage.push(pkt.report);
  LOG_TS("Record pushed to storage; record count = %d\r\n",storage.count());

  sent_seq++;
  pkt.seq = sent_seq;

  sleeping_wait(time_wait_for_tx());

  for (int i=0; i<config.tx_repeat; i++) {
    Watchdog.reset();
    LOG_TS("TX: REPORT seq=%d\r\n",pkt.seq);
    radio.send(config.radio_gateway_address,(uint8_t*)&pkt,sizeof(pkt));
    while (!radio.send_done()) {
      Watchdog.reset();
    }
    SHORT_BLINK(50,100);
  }
  LOG_TS("Turn off radio\r\n");
  RADIO_OFF();
}

/***********************************************
 * 
 */
void pps_handler() {
  pps_detected = true;
#ifdef DEBUG
  wake_now = true;
#endif
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
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(PIN_1PPS,INPUT_PULLUP);
  pinMode(PIN_GPS_EN,OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT); //rachata
  pinMode(PIN_RFM95_CS,OUTPUT);
  digitalWrite(PIN_RFM95_CS,HIGH);
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
  radio.send(config.radio_gateway_address,(uint8_t*)&pkt,sizeof(pkt));
  while (!radio.send_done()) {
    Watchdog.reset();
  }

  if (config.use_ack) {
    PT_INIT(&ptRadio);
    PT_INIT(&ptReport);
    PT_INIT(&ptLogger);
    GPS_ON();
  }
  else {
    next_collection = 0;
    RADIO_OFF();
  }

  LED_OFF();
}

/***********************************************
 * 
 */
void loop() {
  if (config.use_ack) {
    taskRadio(&ptRadio);
    taskReport(&ptReport);
    taskLogger(&ptLogger);
    Watchdog.reset();
  }
  else {
    time_t current = get_total_seconds();
    if (next_collection >= current) {
      sleeping_wait(next_collection - current);
    }
    log_and_report();
  }


  
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

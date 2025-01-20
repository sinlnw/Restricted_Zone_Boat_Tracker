#include "settings.h"
#include "debug.h"
#include "logger.h"
#include "io.h"
#include "timer.h"
#include "sys-utils.h"
#include "report.h"
#include "tracker.h"
#include "config.h"

#define TASK "TRACKER"

SFE_UBLOX_GNSS Tracker::_gnss;
bool Tracker::_fix_acquired, Tracker::_datetime_valid;
UBX_NAV_PVT_data_t Tracker::_gnss_data;
Storage<ReportItem>* Tracker::storage;

Tracker Tracker;

bool Tracker::begin(HardwareSerial& serial, Storage<ReportItem>* storage) {
  if (_gnss.begin(serial)) {
    //gnss.softwareResetGNSSOnly();
    _gnss.setUART1Output(COM_TYPE_UBX); // Set the UART port to output UBX only
    _gnss.setI2COutput(0); // No output over I2C
  }
  else {
    return false;
  }
  Tracker::storage = storage;
  return true;
}

/*********************************************************************
 * Callback when receiving GNSS data
 */
void Tracker::_pvt_callback(UBX_NAV_PVT_data_t ubxDataStruct) {
  memcpy(&_gnss_data, &ubxDataStruct, sizeof(_gnss_data));
  LOG_TS("DEBUG: gnssFixOK=%d validDate=%d validTime=%d fullyResolved=%d\r\n",
    _gnss_data.flags.bits.gnssFixOK,
    _gnss_data.valid.bits.validDate,
    _gnss_data.valid.bits.validTime,
    _gnss_data.valid.bits.fullyResolved);
  _fix_acquired = _gnss_data.flags.bits.gnssFixOK;
  _datetime_valid = _gnss_data.valid.bits.validDate
                 && _gnss_data.valid.bits.validTime;
}


/*********************************************************************
 * 
 */
void Tracker::track() {
  uint32_t ts, ts2;
  uint32_t gps_on_time;

  ts = 0;
  _gnss.setNavigationFrequency(2);
  _gnss.setAutoPVTcallback(&Tracker::_pvt_callback);

  _fix_acquired = false;
  gps_on_time = millis();

  LOG_TS(TASK ": Wait for fix\r\n");

  // wait for a fix
  TIMER_START(ts);
  ts2 = millis();
  while (!_fix_acquired && !TIMED_OUT(ts, config.fix_wait*1000)) {
    if (millis() - ts2 > 1000) { // short blink every second
      SHORT_BLINK(5, 0);
      ts2 = millis();
    }
    _gnss.checkUblox(); // Check for the arrival of new data and process it.
    _gnss.checkCallbacks(); // Check if any callbacks are waiting to be processed.
    watchdog_reset();
  }

  if (!_fix_acquired) {
    LOG_TS(TASK ": Cannot acquire a fix\r\n");
  }
  else {
    LOG_TS(TASK ": Fix acquired\r\n");
    set_time(_gnss_data.year,
             _gnss_data.month,
             _gnss_data.day,
             _gnss_data.hour,
             _gnss_data.min,
             _gnss_data.sec);

    // record timestamp for computing time-to-fix
    ts = millis();

    ReportItem report;
    report.type = REPORT_TYPE_DATA;
    report.year = get_year() - 2000;
    report.month = get_month();
    report.day = get_day();
    report.hour = get_hours();
    report.minute = get_minutes();
    report.second = get_seconds();
    report.vbat = read_vbat_mv();
    report.latitude = _gnss_data.lat;
    report.longitude = _gnss_data.lon;
    report.siv = _gnss_data.numSV;
    report.temperature = read_temperature();
    report.time_to_fix = ts - gps_on_time;

    LOG_TS(TASK ": Recording {\"type\":%d,"
             "\"ts\":\"%04d-%02d-%02d %02d:%02d:%02d\","
             "\"vbat\":%d,"
             "\"lat\":%d,\"lon\":%d,"
             "\"siv\":%d,\"temp\":%d,\"ttf\":%d}\r\n",
      report.type,
      report.year+2000, report.month, report.day,
      report.hour, report.minute, report.second,
      report.vbat,
      report.latitude, report.longitude,
      report.siv, report.temperature, report.time_to_fix);

    storage->push(report);
    LOG_TS(TASK ": Record pushed to storage; record count = %lu\r\n",
      (uint32_t)storage->count());
  }

  LOG_TS(TASK ": Turning off GNSS\r\n");
  _gnss.powerOff(0);
}


#include <stdint.h>
#include <Adafruit_SleepyDog.h>

#define DEBUG
//#define LOGGING

#include "minmea.h"
#include "gps.h"
#include "logger.h"

/***********************************************
 * Borrowed from Adafruit's GPS library
 */
static uint8_t parseHex(char c) {
    if (c < '0')
      return 0;
    if (c <= '9')
      return c - '0';
    if (c < 'A')
       return 0;
    if (c <= 'F')
       return (c - 'A')+10;
    // if (c > 'F')
    return 0;
}

/***********************************************
 * 
 */
GPSClass::GPSClass(Stream& serial):
  _serial(serial)
{
  reset_buffer(false);
}

/***********************************************
 * 
 */
void GPSClass::sendCommand(const char *cmd) {
  _serial.println(cmd);
}

/***********************************************
 * Try to parse whatever is stored in the NMEA buffer, then set _rmc_received
 * or _gga_received flags if corresponding info has been obtained
 */
void GPSClass::_parse() {
  // checksum check, obtained from Adafruit's GPS library

  // first look if we even have one
  uint16_t slen = strlen((char*)_nmea_buf);
  if (_nmea_buf[slen-3] == '*') {
    uint16_t sum = parseHex(_nmea_buf[slen-2]) * 16;
    sum += parseHex(_nmea_buf[slen-1]);
    
    // check checksum 
    for (uint8_t i=1; i < slen-3; i++) {
      sum ^= _nmea_buf[i];
    }
    if (sum != 0) {
      // bad checksum :(
      LOG_TS("bad checksum\r\n");
      return;
    }
  }
  else {
    LOG_TS("invalid sentence (no checksum)\r\n");
    return;
  }

  switch (minmea_sentence_id((char*)_nmea_buf, false)) {
    case MINMEA_SENTENCE_RMC: 
      {
        struct minmea_sentence_rmc frame;
        if (minmea_parse_rmc(&frame, (char*)_nmea_buf)) {
          if (frame.valid) { 
            latitude = (int32_t)(minmea_tocoord(&frame.latitude)*10000000);
            longitude = (int32_t)(minmea_tocoord(&frame.longitude)*10000000);
            year = frame.date.year;
            month = frame.date.month;
            day = frame.date.day;
            hour = frame.time.hours;
            minute = frame.time.minutes;
            seconds = frame.time.seconds;
            _rmc_received = true;
          }
        }
      }
      break;

    case MINMEA_SENTENCE_GGA:
      {
        struct minmea_sentence_gga frame;
        if (minmea_parse_gga(&frame, (char*)_nmea_buf)) {
          if (frame.fix_quality > 0) {
            latitude = (int32_t)(minmea_tocoord(&frame.latitude)*10000000);
            longitude = (int32_t)(minmea_tocoord(&frame.longitude)*10000000);
            hour = frame.time.hours;
            minute = frame.time.minutes;
            seconds = frame.time.seconds;
            fixquality = frame.fix_quality;
            satellites = frame.satellites_tracked;
            _gga_received = true;
          }
        }
      }
      break;
  }
}

/***********************************************
 * Read and parse NMEA sentence, then return true when all required
 * information has been received
 */
bool GPSClass::read() {
  if (_gga_received && _rmc_received) 
    return true;

  if (!_serial.available())
    return false;

  char c = _serial.read();

  if (c == '$') {
    // got start of NMEA sentence; start over
    _nmea_ptr = 0;
    _nmea_buf[_nmea_ptr++] = c;
    return false;
  }
  else if (c == '\n' || c == '\r') {
    // EOL found; terminate and try to parse
    _nmea_buf[_nmea_ptr] = '\0';
    LOG_TS("--%s--\r\n", _nmea_buf);
    _parse();
    LOG_TS("parse done. %d %d\r\n", _gga_received, _rmc_received);
    return _gga_received && _rmc_received;
  }

  _nmea_buf[_nmea_ptr++] = c;
  if (_nmea_ptr == MAX_NMEA_LENGTH) { // unusual NMEA length; start over
    LOG_TS("NMEA is unusally long...start over\r\n");
    _nmea_ptr = 0;
  }

  return false;
}

/***********************************************
 * 
 */
void GPSClass::reset_buffer(bool flush_serial) {
  _gga_received = false;
  _rmc_received = false;
  _nmea_ptr = 0;
  _nmea_buf[0] = '\0';

  if (flush_serial) {
    while (_serial.available()) {
      Watchdog.reset();
      _serial.read();
    }
  }
}


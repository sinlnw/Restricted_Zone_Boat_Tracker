#ifdef ARDUINO_AVR_FEATHER32U4
#include <TimeLib.h>
#endif

#ifdef ADAFRUIT_FEATHER_M0
#include <RTCZero.h>
RTCZero rtc;
#endif

#include <stdio.h>
#include "time-utils.h"

char _datetime_string[30];

/************************************
 *
 */
void init_time() {
#ifdef ADAFRUIT_FEATHER_M0
  rtc.begin();
#endif
  set_time(2000,1,1,0,0,0);
}

/************************************
 *
 */
void set_time(uint16_t year,
              uint8_t  month,
              uint8_t  day,
              uint8_t  hour,
              uint8_t  minute,
              uint8_t  second) {
#ifdef ARDUINO_AVR_FEATHER32U4
  setTime(hour,minute,second,day,month,year);
#endif

#ifdef ADAFRUIT_FEATHER_M0
  rtc.setTime(hour,minute,second);
  rtc.setDate(day,month,year-2000);
#endif
}

/************************************
 *
 */
time_t get_total_seconds() {
#ifdef ARDUINO_AVR_FEATHER32U4
  return now();
#endif

#ifdef ADAFRUIT_FEATHER_M0
  return rtc.getEpoch();
#endif
}

/************************************
 *
 */
char* get_current_time_str() {
  sprintf(_datetime_string, "%d-%02d-%02d %02d:%02d:%02d",
    get_year(),
    get_month(),
    get_day(),
    get_hours(),
    get_minutes(),
    get_seconds());
  return _datetime_string;
}

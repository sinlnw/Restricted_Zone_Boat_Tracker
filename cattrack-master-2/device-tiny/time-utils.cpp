#if defined(ARDUINO_AVR_FEATHER32U4)
#include <TimeLib.h>

#elif defined(ADAFRUIT_FEATHER_M0)
//#include <RTCZero.h>
//RTCZero rtc;
#endif

#include "time-utils.h"

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
#if defined(ARDUINO_AVR_FEATHER32U4)
  setTime(hour,minute,second,day,month,year);
#elif defined(ADAFRUIT_FEATHER_M0)
  rtc.setTime(hour,minute,second);
  rtc.setDate(day,month,year-2000);
#endif
}

/************************************
 *
 */
time_t get_total_seconds() {
#if defined(ARDUINO_AVR_FEATHER32U4)
  return now();
#elif defined(ADAFRUIT_FEATHER_M0)
  return rtc.getEpoch();
#endif
}

#ifndef __TIME_UTILS_H__
#define __TIME_UTILS_H__

#include <time.h>
#include <stdint.h>

void init_time();
void set_time(uint16_t year,
              uint8_t  month,
              uint8_t  day,
              uint8_t  hour,
              uint8_t  minute,
              uint8_t  second);
time_t get_total_seconds();

#if defined(ARDUINO_AVR_FEATHER32U4)
#define get_hours()      hour()
#define get_minutes()    minute()
#define get_seconds()    second()
#define get_year()       year()
#define get_month()      month()
#define get_day()        day()

#elif defined(ADAFRUIT_FEATHER_M0)
//#include <RTCZero.h>
extern RTCZero rtc;
#define get_hours()      rtc.getHours()
#define get_minutes()    rtc.getMinutes()
#define get_seconds()    rtc.getSeconds()
#define get_year()       (rtc.getYear()+2000)
#define get_month()      rtc.getMonth()
#define get_day()        rtc.getDay()

#endif

#endif

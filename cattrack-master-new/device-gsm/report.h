#ifndef __REPORT_H__
#define __REPORT_H__

#include <stdint.h>

#define REPORT_TYPE_DATA       0
#define REPORT_TYPE_RESET      1
#define REPORT_TYPE_BOOT       2
#define REPORT_TYPE_HEARTBEAT  3
#define REPORT_TYPE_WAKEUP     4

struct ReportItem {
  uint8_t  year, month, day;
  uint8_t  hour, minute, second;
  uint8_t  siv, type;
  uint16_t vbat;  // in mV
  int32_t  latitude, longitude; // unit of 1/100000 degrees
  uint32_t time_to_fix;  // ms
  uint16_t temperature;
} __attribute__((packed));

#endif

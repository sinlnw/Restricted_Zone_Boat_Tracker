#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <Arduino.h>
#include "printf.h"

#ifdef DEBUG
#define DEBUG_PRINT(x)    Serial.print(x)
#define DEBUG_PRINTLN(x)  Serial.println(x)
#define DEBUG_PRINTF(...) ::printf(Serial,__VA_ARGS__)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(...)
#endif

#define DEBUG_TS(args...)  \
  do { \
    DEBUG_PRINTF("%s ", get_current_time_str()); \
    DEBUG_PRINTF(args); \
  } while (0)

#endif

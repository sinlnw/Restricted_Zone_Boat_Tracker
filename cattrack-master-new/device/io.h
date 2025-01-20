#ifndef __IO_H__
#define __IO_H__

#if defined(ADAFRUIT_FEATHER_M0)
#define PIN_VBAT A7
#define PIN_RFM95_CS  8
#define PIN_RFM95_RST 4
#define PIN_RFM95_INT 3

#elif defined(ARDUINO_AVR_FEATHER32U4)
#define PIN_VBAT A9
#define PIN_RFM95_CS  8
#define PIN_RFM95_RST 4
#define PIN_RFM95_INT 7

#else
#error("Unsupported board")
#endif

#if defined(ADALOGGER)
#define PIN_SD_SS     4
#define PIN_SD_LED    8
#define RADIO_ON()
#define RADIO_OFF()
#else
#define PIN_SD_SS     10
#define RADIO_ON()    do { LOG_TS("RADIO ON\r\n"); radio.idle(); } while(0)
#define RADIO_OFF()   do { LOG_TS("RADIO OFF\r\n"); radio.sleep(); } while(0)
#endif

#define PIN_1PPS      11
#define PIN_GPS_EN    12

#define LED_ON()      digitalWrite(LED_BUILTIN,HIGH)
#define LED_OFF()     digitalWrite(LED_BUILTIN,LOW)
#define GPS_ON()      do { LOG_TS("GPS ON\r\n"); digitalWrite(PIN_GPS_EN,LOW); } while(0)
#define GPS_OFF()     do { LOG_TS("GPS OFF\r\n"); digitalWrite(PIN_GPS_EN,HIGH); } while(0)

#define SHORT_BLINK(on,off) \
        do { \
          digitalWrite(LED_BUILTIN,HIGH); \
          delay(on); \
          digitalWrite(LED_BUILTIN,LOW); \
          delay(off); \
        } while (0);

#endif

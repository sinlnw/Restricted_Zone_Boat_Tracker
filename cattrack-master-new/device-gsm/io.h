#ifndef __IO_H__
#define __IO_H__

#define PIN_SD_SS     4

#define LED_ON()      digitalWrite(LED_BUILTIN, HIGH)
#define LED_OFF()     digitalWrite(LED_BUILTIN, LOW)

#define SHORT_BLINK(on, off) \
        do { \
          digitalWrite(LED_BUILTIN, HIGH); \
          delay(on); \
          digitalWrite(LED_BUILTIN, LOW); \
          delay(off); \
        } while (0);

#endif

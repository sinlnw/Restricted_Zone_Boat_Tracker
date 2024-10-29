#include <SPI.h>
#include <Adafruit_SleepyDog.h>

//#define DEBUG
//#define SD_LOADER
//#define DEBUG_SD_LOADER

#define WATCHDOG_TIMEOUT   4000
#define RADIO_DEVICE_ID    0x00
#define RADIO_FREQ         447.5
#define RADIO_TX_POWER     23
#define RADIO_LONG_RANGE   1
#define SFD                0x7E

#include "pt/pt.h"
#include "printf.h"
#include "radio.h"
#include "io.h"
#include "debug.h"

#define TIMER_START(ts)       ts = millis();
#define TIMER_EXPIRED(ts,timeout)  (millis()-ts > timeout)
#define PT_DELAY(pt,ms,tsVar) \
  tsVar = millis(); \
  PT_WAIT_UNTIL(pt, millis()-tsVar >= (ms));

Radio radio;
struct pt ptCollect;

/***********************************************
 * 
 */
void error_blink_loop() {
  for (;;) {
    LED_ON();
    delay(200);
    LED_OFF();
    delay(200);
  }
}

/***********************************************
 * 
 */
void init_radio() {
  DEBUG_PRINT("Initializing radio...");
  if (!radio.init(RADIO_DEVICE_ID, RADIO_FREQ, RADIO_TX_POWER, RADIO_LONG_RANGE)) 
  {
    DEBUG_PRINTLN("failed!");
    error_blink_loop();
  }
  DEBUG_PRINTLN("done.");
}

/**************************************************************************
 * Receives a packet from LoRa and sends the binary content to serial port.
 *
 * The serial packet format is of the form:
 *
 *  +------+------+------+-----+-----------+----------+
 *  | 0x7E | from | rssi | len |  payload  | checksum |
 *  +------+------+------+-----+-----------+----------+
 *     1       1     1       n            1
 *
 *  Checksum is computed by Xoring all bytes from src up to the payload's last
 *  byte.
 */
PT_THREAD(taskCollect(struct pt* pt)) {

  static uint32_t ts = 0;
  static uint8_t buf[64];
  static Address from;
  static uint8_t cs;

  PT_BEGIN(pt);
  for (;;) {
    PT_WAIT_UNTIL(pt,radio.available());
    LED_ON();
    uint8_t len = sizeof(buf);
    radio.recv(&from,buf,&len);
    cs = 0;
    Serial.write(SFD);
    Serial.write(&from, sizeof(from));
    for (uint8_t i = 0; i < sizeof(from); i++)
      cs ^= ((uint8_t*)(&from))[i];
    Serial.write(radio.last_rssi);
    cs ^= radio.last_rssi;
    Serial.write(&len, sizeof(len));
    for (uint8_t i = 0; i < sizeof(len); i++)
      cs ^= ((uint8_t*)(&len))[i];
    Serial.write(buf, len);
    for (uint8_t i = 0; i < len; i++)
      cs ^= buf[i];
    Serial.write(cs);
    LED_OFF();
  }
  PT_END(pt);
}

/***********************************************
 * 
 */
void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(PIN_RFM95_CS,OUTPUT);
  digitalWrite(PIN_RFM95_CS,HIGH);

  LED_OFF();
  Serial.begin(115200);
  delay(2000);
  while (!Serial)
    ; // wait for serial port to connect
  init_radio();

  PT_INIT(&ptCollect);
  LED_ON();
  Watchdog.enable(WATCHDOG_TIMEOUT);
}

/***********************************************
 * 
 */
void loop() {
  // operate only when serial is connected; reset in 5 seconds if disconnected
  if (Serial) {
    Watchdog.reset();
    taskCollect(&ptCollect);
  }
}

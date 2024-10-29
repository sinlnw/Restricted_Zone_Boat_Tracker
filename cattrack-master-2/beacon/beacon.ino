#include <Adafruit_SleepyDog.h>
#include "radio.h"
#include "io.h"

#define WATCHDOG_TIMEOUT   10000
#define RADIO_DEVICE_ID    0x55
#define RADIO_FREQ         434.0
#define RADIO_TX_POWER     23
#define RADIO_LONG_RANGE   1
#define RADIO_GATEWAY_ADDR 0x00

#ifdef DEBUG
#define DEBUG_PRINT(x...) Serial.print(x)
#define DEBUG_PRINTLN(x...) Serial.println(x)
#else
#define DEBUG_PRINT(x...)
#define DEBUG_PRINTLN(x...)
#endif

Radio radio;
uint8_t sent_seq = 0;

/***********************************************
 * 
 */
void error_blink_loop(uint8_t code) {
  DEBUG_PRINT(F("ERROR: code "));
  DEBUG_PRINTLN(code);
  for (;;) {
    LED_ON();
    delay(200);
    LED_OFF();
    delay(200);
  }
}

/************************************
 *
 */
void init_radio() {
  DEBUG_PRINTLN(F("Init radio."));
  if (!radio.init(RADIO_DEVICE_ID, RADIO_FREQ, RADIO_TX_POWER, RADIO_LONG_RANGE)) 
  {
    //DEBUG_PRINTLN(F("failed."));
    error_blink_loop(7);
  }
  //DEBUG_PRINTLN(F("successful."));
}

/*********************************************************************
 * Force watchdog reset
 */
void force_reset() {
  //DEBUG_PRINTLN(F("Something went wrong; force reset"));
  Watchdog.enable(1000);
  while (true)
    ;
}

/*********************************************************************
 *
 */
void setup() {
  pinMode(PIN_RFM95_CS,OUTPUT);
  digitalWrite(PIN_RFM95_CS,HIGH);
  pinMode(LED_BUILTIN,OUTPUT);
  LED_ON();

  Serial.begin(9600);

  init_radio();
  Watchdog.enable(WATCHDOG_TIMEOUT);
  LED_OFF();

  sent_seq = 0;
}

/***********************************************
 * 
 */
void loop() {
  uint8_t pkt[50];
  uint8_t len;

  len = sprintf((char*)pkt,"%d_0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ", sent_seq);

  Serial.print("Sending ");
  Serial.println(sent_seq);
  sent_seq++;
  radio.send(RADIO_GATEWAY_ADDR,pkt,len);
  SHORT_BLINK(50,100);
  while (!radio.send_done()) {
    Watchdog.reset();
  }
  delay(5000);
}

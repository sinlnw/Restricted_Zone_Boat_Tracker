/** Feather 32U4 **/
#ifdef ARDUINO_AVR_FEATHER32U4
#include <TimeLib.h>
#include <Adafruit_SleepyDog.h>
#define INIT_TIME()
#define GET_SECONDS()  now()
#endif

/** Feather M0 **/
#ifdef ARDUINO_SAMD_FEATHER_M0
#include <RTCZero.h>
RTCZero rtc;
#define INIT_TIME()    rtc.begin()
#define GET_SECONDS()  rtc.getEpoch()
#endif

#include <Adafruit_GPS.h>

//#define DEBUG

#include "pt/pt.h"
#include "storage.h"
#include "printf.h"
#include "radio.h"
#include "config.h"
#include "io.h"
#include "debug.h"

#define RADIO_MY_ADDRESS  0x01

Sd2Card card;
Storage<ReportItem,STORAGE_MAX_RECORDS> storage(card);
Adafruit_GPS GPS(&Serial1);
Radio radio;
time_t next_collection = 0;
uint8_t sent_seq = 0;

/***********************************************
 * 
 */
void init_gps() {
  DEBUG_PRINT("Initializing GPS...");
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ);
  delay(500);
  DEBUG_PRINTLN("done.");
}

/***********************************************
 * 
 */
void init_storage() {
  DEBUG_PRINT("Initializing storage...");
  if (!storage.setup(STORAGE_DATA_START_BLOCK,false)) {
    DEBUG_PRINTLN("failed!");
    error_blink_loop();
  }
  DEBUG_PRINTLN("done.");
}

/***********************************************
 * 
 */
void init_radio() {
  DEBUG_PRINT("Initializing radio...");
  if (!radio.init(RADIO_MY_ADDRESS,RADIO_FREQ,RADIO_TX_POWER)) {
    DEBUG_PRINTLN("failed!");
    error_blink_loop();
  }
  DEBUG_PRINTLN("done.");
}

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

/*********************************************************************
 * Put the device into sleep mode for the specified number of seconds
 */
#ifdef DEBUG
void sleeping_wait(uint16_t seconds) {
  // Cannot sleep for real in debug mode
  delay(seconds*1000);
  return;
}
#else
#ifdef ARDUINO_AVR_FEATHER32U4
void sleeping_wait(uint16_t seconds) {
  time_t until = GET_SECONDS()+seconds;
  time_t remain;
  while ((remain = until - GET_SECONDS()) > 0) {
    if (remain > 8)
      adjustTime(Watchdog.sleep(8000)/1000);
    else if (remain > 4)
      adjustTime(Watchdog.sleep(4000)/1000);
    else if (remain > 2)
      adjustTime(Watchdog.sleep(2000)/1000);
    else if (remain > 1)
      adjustTime(Watchdog.sleep(1000)/1000);
  }
}
#endif  // ARDUINO_AVR_FEATHER32U4
#ifdef ARDUINO_SAMD_FEATHER_M0
void sleeping_wait(uint16_t seconds) {
  rtc.setTime(0,0,0);  // hh,mm,ss
  rtc.setAlarmTime(0,seconds/60,seconds%60);
  rtc.enableAlarm(rtc.MATCH_HHMMSS);
  rtc.standbyMode();
}
#endif  // ARDUINO_SAMD_FEATHER_M0
#endif  // DEBUG

/***********************************************
 * 
 */
void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(PIN_1PPS,INPUT_PULLUP);
  pinMode(PIN_GPS_EN,OUTPUT);

  LED_ON();
#ifdef DEBUG
  Serial.begin(9600);
  while (!Serial)
    ; // wait for serial port to connect. Needed for native USB port only
#endif
  INIT_TIME();
  init_storage();
  init_radio();
  init_gps();

  /* Taken from
   *
   *    https://github.com/arduino/ArduinoCore-samd/issues/142 
   *
   * to allow change/rising/falling interrupt.   However, this seems to raise
   * the power consumption by 3.4 mA. :( */

  // Configure the regulator to run in normal mode when in standby mode
  // Otherwise it defaults to low power mode and can only supply 50 uA
  SYSCTRL->VREG.bit.RUNSTDBY = 1;

  // Enable the DFLL48M clock in standby mode
  SYSCTRL->DFLLCTRL.bit.RUNSTDBY = 1;

  // Disable the USB device, this avoids USB interrupts
  // mainly the SOF every 1ms.
  // Note: you'll have to double tap the reset button to load new sketches
  USBDevice.detach();

  LED_OFF();
  delay(5000);
}

/***********************************************
 * 
 */
void loop() {
  // MCU:on LED:on GPS:on Radio:idle
  LED_ON(); GPS_ON(); RADIO_ON(); delay(10000);
  SHORT_BLINK(50,50); SHORT_BLINK(50,50); SHORT_BLINK(50,50);

  // MCU:on LED:on GPS:on Radio:tx
  LED_ON();
  for (int i=0; i<5; i++) {
    radio.send(0x00,(uint8_t*)"123456789012345678901234567890",30);
    while (!radio.send_done())
      ;
  }
  SHORT_BLINK(50,50); SHORT_BLINK(50,50); SHORT_BLINK(50,50);

  // MCU:on LED:off GPS:on Radio:idle
  LED_OFF(); delay(10000);
  SHORT_BLINK(50,50); SHORT_BLINK(50,50); SHORT_BLINK(50,50);

  // MCU:on LED:off GPS:off Radio:idle
  GPS_OFF(); delay(10000);
  SHORT_BLINK(50,50); SHORT_BLINK(50,50); SHORT_BLINK(50,50);

  // MCU:on LED:off GPS:off Radio:sleep
  RADIO_OFF(); delay(10000);
  SHORT_BLINK(50,50); SHORT_BLINK(50,50); SHORT_BLINK(50,50);

  // MCU:off LED:off GPS:off Radio:sleep
  sleeping_wait(10);
  SHORT_BLINK(50,50); SHORT_BLINK(50,50); SHORT_BLINK(50,50);
}

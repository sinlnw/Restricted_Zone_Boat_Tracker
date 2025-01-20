#include <Adafruit_SleepyDog.h>
#include "settings.h"
#include "debug.h"
#include "logger.h"
#include "io.h"
#include "sys-utils.h"

#define TASK  "SYS"

#ifdef DEBUG
volatile bool wake_now = false;
#endif

/*********************************************************************
 * Blink indefinitely to indicate error
 */
void error_blink_loop(uint8_t code, bool logging) {
  if (logging) {
    LOG_TS(TASK ": Error with code %d\n", code);
  }
  else {
    DEBUG_TS(TASK ": Error with code %d\n", code);
  }
  for (;;) {
    LED_ON();
    delay(200);
    LED_OFF();
    delay(200);
  }
}

/*********************************************************************
 * Return current battery voltage in mV
 */
uint16_t read_vbat_mv() {
  return analogRead(ADC_BATTERY)*3300LL*(1200000LL+330000LL)/1200000LL/1024LL;
}

/*********************************************************************
 * temperature reading
 * see: http://ww1.microchip.com/downloads/en/AppNotes/Atmel-42645-ADC-Configurations-with-Examples_ApplicationNote_AT11481.pdf
 */
static __inline__ void syncADC() __attribute__((always_inline, unused));
static void syncADC() {
  while (ADC->STATUS.bit.SYNCBUSY == 1)
    ;
}

uint16_t read_temperature() {
  uint32_t valueRead = 0;

  SYSCTRL->VREF.bit.TSEN = 1;  // Enable temperature sensor
  ADC->INPUTCTRL.bit.MUXPOS = 0x18;  // Temperature reference
  ADC->CTRLA.bit.ENABLE = 0x01; // Enable ADC

  // Start conversion
  syncADC();
  ADC->SWTRIG.bit.START = 1;

  // Clear the Data Ready flag
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  // Start conversion again, since The first conversion after the reference is changed must not be used.
  syncADC();
  ADC->SWTRIG.bit.START = 1;

  // Store the value
  while (ADC->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete
  valueRead = ADC->RESULT.reg;

  syncADC();
  ADC->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
  syncADC();

  return valueRead;
}

/*********************************************************************
 * Force watchdog reset
 */
void force_reset() {
  LOG_TS(TASK ": Something went wrong; force reset\r\n");
  Watchdog.enable(1000);
  while (true)
    ;
}

/*********************************************************************
 * 
 */
void watchdog_start() {
  Watchdog.enable(WATCHDOG_TIMEOUT);
  LOG_TS(TASK ": Watchdog started\r\n");
}

/*********************************************************************
 * Custom watchdog reset to prevent very slow synchronization of SAMD chip
 */
void watchdog_reset(bool force) {
  static uint32_t last = 0;
  if ( force || ((millis() - last) > WATCHDOG_TIMEOUT/4) ) {
    LOG_TS(TASK ": Watchdog reset\r\n");
    Watchdog.reset();
    last = millis();
  }
}

/*********************************************************************
 * Put the device into sleep mode for the specified number of seconds
 */
void sleeping_wait(uint16_t seconds) {
  if (seconds == 0) return;
  LOG_TS(TASK ": Going to sleep for %d second(s)\r\n", seconds);
#if defined(DEBUG)
  // Cannot sleep for real in debug mode
  uint32_t ts = millis();
  wake_now = false;
  while (millis() - ts < seconds*1000) {
    watchdog_reset();
    if (wake_now) break;
  }
  wake_now = false;
#else
  uint32_t until = get_total_seconds() + seconds;
  uint32_t remain;
  while ((remain = until - get_total_seconds()) > 0) {
    if (remain > 8)
      adjustTime(Watchdog.sleep(8000)/1000);
    else if (remain > 4)
      adjustTime(Watchdog.sleep(4000)/1000);
    else if (remain > 2)
      adjustTime(Watchdog.sleep(2000)/1000);
    else if (remain > 1)
      adjustTime(Watchdog.sleep(1000)/1000);
  }
#endif
  LOG_TS(TASK ": Waking up\r\n");
}

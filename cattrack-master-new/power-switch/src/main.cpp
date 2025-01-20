#include "Arduino.h"
#include <Wire.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>

#define CONTROL_PIN          0
#define DEFAULT_OFF_DURATION (30*60)  // seconds
#define FORCE_SLEEP_AFTER    (5*60)  // seconds
#define I2C_ADDRESS          0x70
#define I2C_CMD_POWEROFF     0x81

volatile uint32_t _rtc_ticks;
volatile uint8_t _poweroff_sec_low;
volatile uint8_t _poweroff_sec_high;
uint32_t power_on_ts;
uint32_t last_poweroff_duration = 0;

/************************************************************
 * Switch on the power
 */
void power_on() {
    digitalWrite(CONTROL_PIN, LOW);
}

/************************************************************
 * Switch off the power
 */
void power_off() {
    digitalWrite(CONTROL_PIN, HIGH);
}

/************************************************************
 * Initialize RTC with PIT interrupt
 * (adapted from https://github.com/SpenceKonde/megaTinyCore/blob/master/megaavr/extras/PowerSave.md)
 */
void rtc_init(void)
{
    while (RTC.STATUS > 0) {}             /* Wait for all register to be synchronized */
    RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;    /* 32.768kHz Internal Ultra-Low-Power Oscillator (OSCULP32K) */

    RTC.PITINTCTRL = RTC_PI_bm;           /* PIT Interrupt: enabled */

    RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc
                 | RTC_PITEN_bm;          /* Enable PIT counter: enabled */
}

ISR(RTC_PIT_vect)
{
    RTC.PITINTFLAGS = RTC_PI_bm;          /* Clear interrupt flag by writing '1' (required) */
    _rtc_ticks++;
}

/************************************************************
 * Return the current value of RTC's ticks
 */
uint32_t rtc_ticks() {
    uint32_t val;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        val = _rtc_ticks;
    }
    return val;
}

/************************************************************
 * Reset RTC's ticks
 */
void rtc_reset_ticks() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        _rtc_ticks = 0;
    }
}

/************************************************************
 * Enable RTC
 */
void rtc_start() {
    while (RTC.STATUS > 0) {} // Wait for all register to be synchronized
    RTC.CNT = 0;
    RTC.CTRLA |= RTC_RTCEN_bm;
    rtc_reset_ticks();
}

/************************************************************
 * Disable RTC
 */
void rtc_stop() {
    while (RTC.STATUS > 0) {} // Wait for all register to be synchronized
    RTC.CTRLA &= ~RTC_RTCEN_bm;
}

/************************************************************
 * Go into standby sleep for the specified seconds
 */
void sleep_for(uint32_t seconds) {
    uint32_t ts = rtc_ticks();
    while (rtc_ticks() - ts < seconds) {
        sleep_cpu();
    }
}

/************************************************************
 * Enable pull-up on all pins to lower power
 * taken from https://www.avrfreaks.net/comment/3209046#comment-3209046
 */
void pullup_all_pins() {
    for (uint8_t i = 0; i < 8; i++) {
        *((uint8_t *)&PORTA + 0x10 + i) = PORT_PULLUPEN_bm; // PINiCTRL = PORTx.DIR+0x10+i
    }
}


/************************************************************
 *
 */
void i2c_data_recv(int count) {
    if (count == 3) {
        uint8_t cmd = Wire.read();
        if (cmd == I2C_CMD_POWEROFF) {
            _poweroff_sec_low = Wire.read();
            _poweroff_sec_high = Wire.read();
        }
    }
}

/************************************************************
 *
 */
void i2c_data_send() {
}


/************************************************************
 *
 */
void setup()
{
    pullup_all_pins();
    pinMode(CONTROL_PIN, OUTPUT);
    power_on();
    Wire.begin(I2C_ADDRESS);
    Wire.onReceive(i2c_data_recv);
    //Wire.onRequest(i2c_data_send);
    rtc_init();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();     
    _poweroff_sec_low = _poweroff_sec_high = 0;

    power_on();
    power_on_ts = rtc_ticks();
}

/************************************************************
 *
 */
void loop()
{
    // the host has sent a request to power off over I2C with desired duration
    if (_poweroff_sec_low || _poweroff_sec_high) {
        uint16_t duration = _poweroff_sec_low + _poweroff_sec_high*256;
        last_poweroff_duration = duration;
        power_off();
        _poweroff_sec_low = _poweroff_sec_high = 0;
        sleep_for(duration);
        power_on();
        power_on_ts = rtc_ticks();
    }

    // in case the host controller does not switch off the power for the predefined
    // duration, power off automatically
    if (rtc_ticks() - power_on_ts > FORCE_SLEEP_AFTER) {
        uint16_t duration;
        if (last_poweroff_duration != 0) {
            duration = last_poweroff_duration;
        }
        else {
            duration = DEFAULT_OFF_DURATION;
        }
        power_off();
        sleep_for(duration);
        power_on();
        power_on_ts = rtc_ticks();
    }
}

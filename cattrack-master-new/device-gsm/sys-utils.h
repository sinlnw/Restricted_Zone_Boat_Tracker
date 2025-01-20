#ifndef __SYS_UTILS_H__
#define __SYS_UTILS_H__

#include <stdint.h>

void error_blink_loop(uint8_t code, bool logging);
uint16_t read_vbat_mv();
uint16_t read_temperature();
void force_reset();
void watchdog_start();
void watchdog_reset(bool force=false);
void sleeping_wait(uint16_t seconds);

#endif

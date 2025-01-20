#ifndef __TIMER_H__
#define __TIMER_H__

#include "sys-utils.h"

#define TIMER_START(tsvar)         tsvar = millis()
#define TIMED_OUT(tsvar,timeout)   ((millis() - tsvar) > (timeout))
#define WAIT_UNTIL(cond)           do { watchdog_reset(); } while (!(cond))

#endif

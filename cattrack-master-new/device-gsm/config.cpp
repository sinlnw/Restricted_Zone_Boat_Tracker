#include <Adafruit_SleepyDog.h>
#include "settings.h"
#include "debug.h"
#include "logger.h"
#include "io.h"
#include "sys-utils.h"
#include "config.h"

#define TASK  "CONFIG"

Config config;

/*********************************************************************
 * 
 */
Config::Config() {
  off_duration = CONF_POWER_OFF_DURATION_DEFAULT;
  fix_wait = CONF_MAX_WAIT_FOR_FIX_DEFAULT;
  boot_count = CONF_WAKEUP_COUNT_UPLOAD_DEFAULT;
}

/*********************************************************************
 * 
 */
void Config::save(AT24EEPROM* eeprom) {
  eeprom->write(CONFIG_START_EEPROM_ADDR, (uint8_t*)this, sizeof(*this));
  LOG_TS(TASK ": saved with off_duration=%d fix_wait=%d boot_count=%d\r\n",
      off_duration,
      fix_wait,
      boot_count);
}

/*********************************************************************
 * 
 */
void Config::load(AT24EEPROM* eeprom) {
  eeprom->read(CONFIG_START_EEPROM_ADDR, (uint8_t*)this, sizeof(*this));
  LOG_TS(TASK ": loaded with off_duration=%d fix_wait=%d boot_count=%d\r\n",
      off_duration, fix_wait, boot_count);
  if (this->sanitize()) {
    save(eeprom);
  }
}

/*********************************************************************
 * Sanitize config values; return true if a value has been sanitized.
 */
bool Config::sanitize() {

  bool sanitized = false;

  if ( (off_duration < CONF_POWER_OFF_DURATION_MIN) ||
       (off_duration > CONF_POWER_OFF_DURATION_MAX) )
  {
    off_duration = CONF_POWER_OFF_DURATION_DEFAULT;
    sanitized = true;
  }

  if ( (fix_wait < CONF_MAX_WAIT_FOR_FIX_MIN) ||
       (fix_wait > CONF_MAX_WAIT_FOR_FIX_MAX) )
  {
    fix_wait = CONF_MAX_WAIT_FOR_FIX_DEFAULT;
    sanitized = true;
  }

  if ( (boot_count < CONF_WAKEUP_COUNT_UPLOAD_MIN) ||
       (boot_count > CONF_WAKEUP_COUNT_UPLOAD_MAX) )
  {
    boot_count = CONF_WAKEUP_COUNT_UPLOAD_DEFAULT;
    sanitized = true;
  }

  return sanitized;
}

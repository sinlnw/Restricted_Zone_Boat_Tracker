#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "AT24EEPROM.h"

struct Config {
  uint16_t off_duration; // seconds
  uint16_t fix_wait; // seconds
  uint8_t boot_count;

  Config();
  void save(AT24EEPROM* eeprom);
  void load(AT24EEPROM* eeprom);
  bool sanitize();
} __attribute__((packed));

extern Config config;

#endif

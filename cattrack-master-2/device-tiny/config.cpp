#include "config.h"
#include "card-utils.h"

/*******************************
 * Constructor
 */
Config::Config() {
  radio_device_address   = DEFAULT_RADIO_DEVICE_ADDRESS;
  radio_gateway_address  = DEFAULT_RADIO_GATEWAY_ADDRESS;
  radio_freq             = DEFAULT_RADIO_FREQ;
  radio_tx_power         = DEFAULT_RADIO_TX_POWER;
  collect_interval_day   = DEFAULT_COLLECT_INTERVAL_DAY;
  collect_interval_night = DEFAULT_COLLECT_INTERVAL_NIGHT;
  day_start_hour         = DEFAULT_DAY_START_HOUR;
  day_end_hour           = DEFAULT_DAY_END_HOUR;
  time_zone              = DEFAULT_TIME_ZONE;
  advertise_interval     = DEFAULT_ADVERTISE_INTERVAL;
  use_ack                = DEFAULT_USE_ACK;
  ack_timeout            = DEFAULT_ACK_TIMEOUT;
  long_range             = DEFAULT_LONG_RANGE;
  tx_repeat              = DEFAULT_TX_REPEAT;
  gps_max_wait_for_fix   = DEFAULT_GPS_MAX_WAIT_FOR_FIX;
  next_collect_no_fix    = DEFAULT_NEXT_COLLECT_NO_FIX;
  total_slots            = DEFAULT_TOTAL_SLOTS;
  slot_interval          = DEFAULT_SLOT_INTERVAL;
  prog_file_name[0]      = '\0';
}

/*******************************
 *
 */
char* read_line(SdFile& file, char* buf, uint8_t maxlen) {
  uint8_t pos = 0;

  while (true) {
    int c = file.read();
    if (c == -1)
      if (pos == 0)
        return NULL;
      else
        break;
    if (c == '\n' || c=='\r') break;
    buf[pos++] = c;
    if (pos >= maxlen-1) break;
  }
  buf[pos] = '\0';
  return buf;
}

/*******************************
 *
 */
bool Config::read_from_volume(SdVolume& volume) {
  char linebuf[128];
  char name[30];
  char value[10];

  SdFile config_file;
  if (!open_file_in_root(config_file,volume,CONFIG_FILE_NAME,O_READ)) {
    return false;
  }

  while (true) {
    char* s = read_line(config_file, linebuf, sizeof(linebuf));
    if (s == NULL) break;
    sscanf(s,"%s %s",name,value);
    if (name[0] == '#') continue;
    if (strcmp_P(name,PSTR("radio_device_address")) == 0) {
      radio_device_address = atoi(value);
    }
    else if (strcmp_P(name,PSTR("radio_gateway_address")) == 0) {
      radio_gateway_address = atoi(value);
    }
    else if (strcmp_P(name,PSTR("radio_freq")) == 0) {
      radio_freq = atof(value);
    }
    else if (strcmp_P(name,PSTR("radio_tx_power")) == 0) {
      radio_tx_power = atoi(value);
    }
    else if (strcmp_P(name,PSTR("collect_interval_day")) == 0) {
      collect_interval_day = atoi(value);
    }
    else if (strcmp_P(name,PSTR("collect_interval_night")) == 0) {
      collect_interval_night = atoi(value);
    }
    else if (strcmp_P(name,PSTR("day_start_hour")) == 0) {
      day_start_hour = atoi(value);
    }
    else if (strcmp_P(name,PSTR("day_end_hour")) == 0) {
      day_end_hour = atoi(value);
    }
    else if (strcmp_P(name,PSTR("time_zone")) == 0) {
      time_zone = atoi(value);
    }
    else if (strcmp_P(name,PSTR("long_range")) == 0) {
      long_range = atoi(value);
    }
    else if (strcmp_P(name,PSTR("tx_repeat")) == 0) {
      tx_repeat = atoi(value);
    }
    else if (strcmp_P(name,PSTR("gps_max_wait_for_fix")) == 0) {
      gps_max_wait_for_fix = atoi(value);
    }
    else if (strcmp_P(name,PSTR("next_collect_no_fix")) == 0) {
      next_collect_no_fix = atoi(value);
    }
    else if (strcmp_P(name,PSTR("total_slots")) == 0) {
      total_slots = atoi(value);
    }
    else if (strcmp_P(name,PSTR("slot_interval")) == 0) {
      slot_interval = atoi(value);
    }
  }

  config_file.close();
  return true;
}

#define DEBUG

#include "storage.h"
#include "logger.h"
#include "report.h"
#include "io.h"
#include "time-utils.h"
#include "debug.h"

#define CIRBUF_FILE  "TEST1.BIN"
#define LOG_FILE     "LOG1.TXT"

Storage<ReportItem,10> storage;
Logger logger;

/***********************************************/
void error_blink_loop() {
  for (;;) {
    LED_ON();
    delay(200);
    LED_OFF();
    delay(200);
  }
}

/***********************************************/
bool init_storage() {
  DEBUG_PRINT("Initializing SD card...");
  if (!SD.begin(PIN_SD_SS)) {
    DEBUG_PRINTLN("failed!");
    return false;
  }
  DEBUG_PRINTLN("done.");

  DEBUG_PRINTF("Opening circular buffer file %s...",CIRBUF_FILE);
  if (!storage.setup(CIRBUF_FILE,false)) {
    DEBUG_PRINTLN("failed!");
    return false;
  }
  DEBUG_PRINTLN("done.");

  DEBUG_PRINTF("Opening log file %s...",LOG_FILE);
  if (!logger.setup(LOG_FILE,false)) {
    DEBUG_PRINTLN("failed!");
    return false;
  }

  DEBUG_PRINTLN("done.");
  return true;
}

/***********************************************/
void setup() {
  pinMode(PIN_RFM95_CS, OUTPUT);
  digitalWrite(PIN_RFM95_CS, HIGH);
  pinMode(LED_BUILTIN, OUTPUT);

  LED_OFF();
  Serial.begin(115200);
  while (!Serial)
    ;
  LED_OFF();

  if (!init_storage())
    error_blink_loop();
  init_time();

  logger.printf("=== LOGGER STARTED ===\r\n");
  ReportItem r;
  storage.push(r);
  storage.push(r);
  logger.printf("Num records: %d\r\n", storage.count());
  logger.printf("Time now is: %s\r\n", get_current_time_str());
  DEBUG_PRINTLN("logged");

  delay(2000);
  logger.printf("Time now is: %s\r\n", get_current_time_str());
  DEBUG_PRINTLN("logged");
}

/***********************************************/
void loop() {
}

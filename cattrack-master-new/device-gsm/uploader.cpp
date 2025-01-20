#include <ArduinoJson.h>
#include "settings.h"
#include "debug.h"
#include "logger.h"
#include "uploader.h"
#include "timer.h"
#include "io.h"
#include "config.h"

#define TASK "UPLOADER"

Access_t Uploader::_access;
Modem_t Uploader::_modem;
String Uploader::_imei;
Client_t Uploader::_net;
GPRS Uploader::_gprs;
String Uploader::_topic_upload;
String Uploader::_topic_config;
String Uploader::_topic_config_ack;
uint32_t Uploader::_seqno;
uint32_t Uploader::_ackno;
PubSubClient Uploader::_mqtt;
Storage<ReportItem>* Uploader::_storage;

Uploader uploader;
extern AT24EEPROM eeprom;

void Uploader::begin(Storage<ReportItem>* storage)
{
  int count = 0;

  _mqtt.setClient(_net);
  _mqtt.setServer(NET_MQTT_BROKER, NET_MQTT_PORT);
  _mqtt.setCallback(_sub_callback);

  //while (true) {
  //  watchdog_reset();
  //  if (count >= 5) {
  //    force_reset();
  //  }

  //  LOG_TS(TASK ": Initializing modem\r\n");
  //  if (!_modem.begin()) {  // start and reset modem
  //    LOG_TS(TASK ": Failed to start modem; try again in 2 seconds\r\n");
  //    delay(2000);
  //    count++;
  //    continue;
  //  }
  //  _imei = _modem.getIMEI();
  //  if (_imei.length() == 0) {
  //    LOG_TS(TASK ": Failed to obtain IMEI; try again in 2 seconds\r\n");
  //    delay(2000);
  //    count++;
  //    continue;
  //  }
  //  break;
  //}
  //LOG_TS(TASK ": Device IMEI = %s\r\n", _imei.c_str());

  _seqno = millis();
  _ackno = 0;
  _storage = storage;
}

void Uploader::_sub_callback(char* topic,
                             uint8_t* payload,
                             unsigned int length)
{
  char buf[2048];
  StaticJsonDocument<sizeof(buf)> doc;

  LOG_TS(TASK ": received message with topic %s\r\n", topic);
  if (length >= sizeof(buf))
    return;
  // copy the content to a buffer so that we can terminate the string
  memcpy(buf, payload, length);
  buf[length] = 0;

  if (_topic_upload == topic) {
    deserializeJson(doc, buf);
    _ackno = doc["seq"].as<uint32_t>();
    uint8_t type = doc["type"].as<uint8_t>();
    LOG_TS(TASK ": got published message with seq=%d type=%d\r\n", _ackno, type);
  }

  else if (_topic_config == topic) {
    LOG_TS(TASK ": got config message %s\r\n", buf);
    deserializeJson(doc, buf);

    Config tmpcfg;
    memcpy(&tmpcfg, &config, sizeof(Config));

    if (doc.containsKey("off_duration")) {
      tmpcfg.off_duration = doc["off_duration"];
    }

    if (doc.containsKey("fix_wait")) {
      tmpcfg.fix_wait = doc["fix_wait"];
    }

    if (doc.containsKey("boot_count")) {
      tmpcfg.boot_count = doc["boot_count"];
    }

    tmpcfg.sanitize();

    // save to EEPROM only when the new config is different
    if (memcmp(&tmpcfg, &config, sizeof(Config))) {
      memcpy(&config, &tmpcfg, sizeof(Config));
      config.save(&eeprom);
    }
    else {
      LOG_TS(TASK ": No change detected; configuration not save\r\n");
    }

    // always publish configuration acknowledgment
    char json[200];
    sprintf(json, "{\"off_duration\":%d,"
                  "\"fix_wait\":%d,"
                  "\"boot_count\":%d}",
      config.off_duration,
      config.fix_wait,
      config.boot_count);
    if (_mqtt.publish(_topic_config_ack.c_str(), json)) {
      LOG_TS(TASK ": Published configuration ack %s\r\n", json);
    }
    else {
      LOG_TS(TASK ": Failed to publish configuration ack\r\n");
    }
  }
}

/*********************************************************************
 * 
 */
bool Uploader::_connect_gsm() {
  uint32_t ts;
  LOG_TS(TASK ": Connecting to cellular network...\r\n");
  _access.begin(NET_PIN, true, false); // trigger connect asynchronously
  TIMER_START(ts);
  WAIT_UNTIL(_access.ready() || TIMED_OUT(ts, NET_CONNECT_TIMEOUT*1000));
  if (!_access.ready()) {
    LOG_TS(TASK ": GSM connection failed\r\n");
    _disconnect_gsm();
    return false;
  }
  LOG_TS(TASK ": GSM ready.\r\n");
  return true;
}

/*********************************************************************
 * 
 */
void Uploader::_disconnect_gsm() {
  LOG_TS(TASK ": Shutting down GSM\r\n");
  _access.shutdown();
}

/*********************************************************************
 * 
 */
bool Uploader::_connect_gprs() {
  uint32_t ts;
  LOG_TS(TASK ": Attaching GPRS\r\n");
#if defined(ARDUINO_SAMD_MKRGSM1400)
  _gprs.attachGPRS(NET_APN, NET_LOGIN, NET_PASSWORD, false);
  TIMER_START(ts);
  WAIT_UNTIL(_gprs.ready() || TIMED_OUT(ts, NET_CONNECT_TIMEOUT*1000));
  if (_gprs.status() != GPRS_READY) {
    LOG_TS(TASK ": GPRS attachment failed\r\n");
    _disconnect_gsm();
    return false;
  }
#elif defined(ARDUINO_SAMD_MKRNB1500)
  // TODO cannot use non-blocking mode; this can be problematic due to watchdog
  if (_gprs.attachGPRS() != GPRS_READY) {
    _disconnect_gsm();
    return false;
  }
#else
#error Unsupported board
#endif
  LOG_TS(TASK ": Connected.\r\n");
  return true;
}

/*********************************************************************
 * 
 */
void Uploader::_disconnect_gprs() {
  uint32_t ts;
  LOG_TS(TASK ": Detaching GPRS\r\n");
  _gprs.detachGPRS(false);
  TIMER_START(ts);
  WAIT_UNTIL(_gprs.ready());
  LOG_TS(TASK ": GPRS detached\r\n");
}

/*********************************************************************
 * 
 */
bool Uploader::_connect_mqtt() {
  uint32_t ts;

  if (!_mqtt.connect("", NET_MQTT_USERNAME, NET_MQTT_PASSWORD)) {
    LOG_TS(TASK ": Failed to connect to MQTT broker\r\n");
    _disconnect_gprs();
    _disconnect_gsm();
    return false;
  }

  LOG_TS(TASK ": MQTT broker connected.\r\n");

  // construct MQTT topics for subscription and publication
  _imei = _modem.getIMEI();
  LOG_TS(TASK ": Device IMEI = %s\r\n", _imei.c_str());
  _topic_upload = NET_MQTT_TOPIC_PREFIX;
  _topic_upload += "/";
  _topic_upload += _imei;
  _topic_config = _topic_upload + "/config";
  _topic_config_ack = _topic_config + "/ack";

  LOG_TS(TASK ": subscribing to %s\r\n", _topic_upload.c_str());
  _mqtt.subscribe(_topic_upload.c_str());
  LOG_TS(TASK ": subscribing to %s\r\n", _topic_config.c_str());
  _mqtt.subscribe(_topic_config.c_str());

  return true;
}

/*********************************************************************
 * 
 */
void Uploader::_disconnect_mqtt() {
  _mqtt.disconnect();
  LOG_TS(TASK ": Disconnecting MQTT\r\n");
}

/*********************************************************************
 * 
 */
bool Uploader::_publish_report(ReportItem& report) {
  uint32_t ts;
  char json[500];

  if (report.type == REPORT_TYPE_DATA) {
    sprintf(json, "{\"seq\":%d,"
                  "\"type\":%d,"
                  "\"ts\":\"%04d-%02d-%02d %02d:%02d:%02d\","
                  "\"vbat\":%d,"
                  "\"lat\":%d,\"lon\":%d,"
                  "\"siv\":%d,\"temp\":%d,\"ttf\":%d}",
      _seqno,
      report.type,
      report.year+2000, report.month, report.day,
      report.hour, report.minute, report.second,
      report.vbat,
      report.latitude, report.longitude,
      report.siv, report.temperature, report.time_to_fix);
  }
  else if (report.type == REPORT_TYPE_WAKEUP) {
    sprintf(json, "{"
        "\"seq\":%d,"
        "\"type\":%d,"
        "\"reset_flags\":%d,"
        "\"vbat\":%d,"
        "\"temp\":%d,"
        "\"config\": {"
          "\"off_duration\":%d,\"fix_wait\":%d,\"boot_count\":%d"
        "}"
      "}",
      _seqno,
      report.type, report.second, report.vbat, report.temperature,
      config.off_duration, config.fix_wait, config.boot_count);
  }

  LOG_TS(TASK ": Publishing %s\r\n", json);
  if (_mqtt.publish(_topic_upload.c_str(), json)) {
    _net.flush();
    LOG_TS(TASK ": Waiting for the published message to come back\r\n");
    TIMER_START(ts);
    while (!TIMED_OUT(ts, NET_MQTT_ACK_TIMEOUT*1000)) {
      _mqtt.loop();
      watchdog_reset();
      if (_ackno == _seqno) {
        LOG_TS(TASK ": Published successfully\r\n");
        _storage->pop();
        _seqno++;
        _ackno = 0;
        SHORT_BLINK(5, 0);
        return true;
      }
    }
    LOG_TS(TASK ": Timed out waiting for published message\r\n");
  }
  LOG_TS(TASK ": Publish failed\r\n");
  return false;
}

/*********************************************************************
 * 
 */
void Uploader::upload() {
  uint32_t ts;
  if (!_connect_gsm()) return;
  if (!_connect_gprs()) return;
  if (!_connect_mqtt()) return;

  ReportItem report;
  while (!_storage->empty()) {
    _storage->top(&report);
    if (!_publish_report(report))
      break;
    watchdog_reset();
  }

  // continue listening for config message
  LED_ON();
  LOG_TS(TASK ": Hold the connection in case there is a new config\r\n");
  TIMER_START(ts);
  while (!TIMED_OUT(ts, NET_HOLD_TIME*1000)) {
    _mqtt.loop();
    watchdog_reset();
  }
  LOG_TS(TASK ": Stopped listening\r\n");
  // don't turn off LED; use it to indicate power-off

  _disconnect_mqtt();
#if defined(ARDUINO_SAMD_MKRGSM1400)
  // TODO investigate why this is not working with NB 1500
  _disconnect_gprs();
  _disconnect_gsm();
#endif
}

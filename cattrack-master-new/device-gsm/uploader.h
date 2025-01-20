#ifndef __UPLOADER__H__
#define __UPLOADER__H__

#if defined(ARDUINO_SAMD_MKRGSM1400)
#include <MKRGSM.h>
typedef GSMModem Modem_t;
typedef GSM Access_t;
typedef GSMClient Client_t;
#elif defined(ARDUINO_SAMD_MKRNB1500)
#include <MKRNB.h>
typedef NBModem Modem_t;
typedef NB Access_t;
typedef NBClient Client_t;
#else
#error Unsupported board
#endif

#include <PubSubClient.h>

#include "debug.h"
#include "logger.h"
#include "storage.h"
#include "report.h"

// TODO due to the difficulty of passing a member function pointer as a
// classical function pointer, we need to force a single instance with all
// static members.  Obviously, doing so prevents usage of multiple uploader
// modules, i.e., connecting to multiple MQTT brokers.
class Uploader {
  private:
    static Access_t _access;
    static Modem_t _modem;
    static Client_t _net;
    static GPRS _gprs;
    static String _topic_upload;
    static String _topic_config;
    static String _topic_config_ack;
    static uint32_t _seqno;
    static uint32_t _ackno;
    static PubSubClient _mqtt;
    static Storage<ReportItem>* _storage;
    static String _imei;

    static void _sub_callback(char* topic, uint8_t* payload, unsigned int length);
    static bool _connect_gsm();
    static bool _connect_gprs();
    static bool _connect_mqtt();
    static void _disconnect_gsm();
    static void _disconnect_gprs();
    static void _disconnect_mqtt();
    static bool _publish_report(ReportItem& report);

  public:
    static void begin(Storage<ReportItem>* storage);
    static void upload();
};

extern Uploader uploader;

#endif

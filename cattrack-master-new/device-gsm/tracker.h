#ifndef __TRACKER__H__
#define __TRACKER__H__

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "storage.h"
#include "report.h"

// TODO due to the difficulty of passing a member function pointer as a
// classical function pointer, we need to force a single instance with all
// static members.  Obviously, doing so prevents usage of multiple tracking
// modules.
class Tracker {
  private:
    static SFE_UBLOX_GNSS _gnss;
    static bool _fix_acquired, _datetime_valid;
    static UBX_NAV_PVT_data_t _gnss_data;
    static void _pvt_callback(UBX_NAV_PVT_data_t ubxDataStruct);
    static Storage<ReportItem>* storage;

  public:
    static bool begin(HardwareSerial& serial, Storage<ReportItem>* storage);
    static void track();
};

extern Tracker tracker;

#endif


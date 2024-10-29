#ifndef __RADIO_H__
#define __RADIO_H__

#include <SPI.h>
#include <RH_RF95.h>
#include "netif.h"
#include "io.h"

class Radio : NetworkInterface {
public:
  Radio();
  void reset();
  bool init(Address my_addr,float freq,int8_t tx_power,uint8_t long_range);
  bool available();
  void send(Address dst, const uint8_t* pkt, uint8_t pkt_len);
  bool send_done();
  void recv(Address *src, uint8_t* buf, uint8_t* len);
  void sleep() { _rf95.sleep(); }
  void idle()  { _rf95.setModeIdle(); }

  int8_t last_rssi;

private:
  RH_RF95 _rf95;
  Address _my_addr;
};

#endif

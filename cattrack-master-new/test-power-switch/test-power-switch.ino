#include <Wire.h>

#define POWER_SWITCH_ADDR           0x70
#define POWER_SWITCH_OFF_CMD        0x81

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  Wire.begin();
}

void power_off(uint16_t seconds) {
  Wire.beginTransmission(POWER_SWITCH_ADDR);
  Wire.write(POWER_SWITCH_OFF_CMD);
  Wire.write(seconds & 0xff);
  Wire.write(seconds >> 8);
  Wire.endTransmission();
}

void loop() {
  Serial.println("Wait for 5 seconds");
  delay(5000);
  Serial.println("Ask poweroff for 260 seconds");
  power_off(260);
  delay(260000);
  Serial.println("Power should be back now");
}

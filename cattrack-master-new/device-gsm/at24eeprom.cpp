// Modified from https://github.com/dantudose/AT24C256
#include "Arduino.h"
#include "Wire.h"
#include "at24eeprom.h"

void AT24EEPROM::setup(int address) {
  Wire.begin();
  _address = address;
}

void AT24EEPROM::write_byte(uint16_t writeAddress, uint8_t data) {
  Wire.beginTransmission(_address);
  Wire.write((byte)((writeAddress & 0xFF00) >> 8));
  Wire.write((byte)(writeAddress & 0x00FF));
  Wire.write(data);
  Wire.endTransmission();
  //Serial.print("Write @"); Serial.print(writeAddress, HEX);
  //Serial.print(' '); Serial.println(data);
  delay(5);  // min time required to write a byte
}

uint8_t AT24EEPROM::read_byte(uint16_t readAddress) {
  Wire.beginTransmission(_address);
  Wire.write((byte)((readAddress & 0xFF00) >> 8));
  Wire.write((byte)(readAddress & 0x00FF));
  Wire.endTransmission();
  Wire.requestFrom(_address, 1);
  //uint8_t avail = Wire.available();
  uint8_t data = Wire.read();
  //Serial.print("Read @"); Serial.print(readAddress, HEX);
  //Serial.print(' '); Serial.print(data);
  //Serial.print(' '); Serial.println(avail);

  return data;
}

void AT24EEPROM::write(uint16_t writeAddress, uint8_t* data, uint16_t len) {
  //Wire.beginTransmission(_address);
  //Wire.write((byte)(writeAddress & 0xFF00) >> 8);
  //Wire.write((byte)(writeAddress & 0x00FF));
  //for(uint16_t i = 0; i < len; i++){
  //  Wire.write(data[i]);
  //  delay(5);  // min time required to write a byte
  //}
  //Wire.endTransmission();
  //delay(5);

  for (int i=0; i<len; i++) {
    write_byte(writeAddress + i, data[i]);
  }
}

void AT24EEPROM::read(uint16_t readAddress, uint8_t* data, uint16_t len) {
  //Wire.beginTransmission(_address);
  //Wire.write((byte)(readAddress & 0xFF00) >> 8);
  //Wire.write((byte)(readAddress & 0x00FF));
  //Wire.endTransmission();
  //Wire.requestFrom(_address, len);
  //for (uint16_t i = 0; i < len; i++){
  //  data[i] = Wire.read();
  //}

  for (int i=0; i<len; i++) {
    data[i] = read_byte(readAddress + i);
  }
}

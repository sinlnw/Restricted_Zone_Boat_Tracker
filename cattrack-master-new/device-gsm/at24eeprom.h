// Modified from https://github.com/dantudose/AT24C256
#ifndef __AT24EEPROM_H__
#define __AT24EEPROM_H__

#include "Arduino.h"
#include "Wire.h"

class AT24EEPROM
{
public: 
    void setup(int address);
    void write_byte(uint16_t writeAddress, uint8_t data);
    uint8_t read_byte(uint16_t readAddress);
    void write(uint16_t writeAddress, uint8_t* data, uint16_t len);
    void read(uint16_t readAddress, uint8_t* data, uint16_t len);

private:
    int _address;
};

#endif

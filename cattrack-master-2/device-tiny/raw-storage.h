/** 
 * @file
 * Utility class declaration for raw access to SD card
 *
 * @author Chaiporn Jaikaeo <chaiporn.j@ku.ac.th>
 *
 */
#ifndef __RAW_STORAGE_H__
#define __RAW_STORAGE_H__

#include <stdint.h>
#include <SPI.h>
#include <SD.h>

#define BLOCK_POSITION(bytepos)  (bytepos/512)
#define BLOCK_OFFSET(bytepos)    (bytepos%512)
#define RAW_INVALID_VALUE        0xFFFFFFFFFFFFFFFF

class RawStorage {
public:
  RawStorage(Sd2Card& card):_card(card) {  }
  bool read(uint64_t offset, uint8_t* data, uint16_t len) __attribute__((__optimize__("O2")));
  bool write(uint64_t offset, const uint8_t* data, uint16_t len) __attribute__((__optimize__("O2")));

private:
  Sd2Card& _card;
};
#endif

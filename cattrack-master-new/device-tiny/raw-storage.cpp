/** 
 * @file
 * Utility class definition for raw access to SD card
 *
 * @author Chaiporn Jaikaeo <chaiporn.j@ku.ac.th>
 *
 */
#include <assert.h>
#include "raw-storage.h"

/**********************************************
 *
 */
bool RawStorage::read(uint64_t offset, uint8_t* data, uint16_t len) {
  uint32_t block_pos;
  uint16_t block_offset;

  block_pos = BLOCK_POSITION(offset);
  block_offset = BLOCK_OFFSET(offset);

  assert(block_offset + len <= 512);

  _card.readData(block_pos,block_offset,len,data);
}

/**********************************************
 *
 */
bool RawStorage::write(uint64_t offset, const uint8_t* data, uint16_t len) {
  char block_data[512];
  uint32_t block_pos;
  uint16_t block_offset;

  block_pos = BLOCK_POSITION(offset);
  block_offset = BLOCK_OFFSET(offset);

  assert(block_offset + len <= 512);
  _card.readBlock(block_pos,(uint8_t*)block_data);
  memcpy(block_data+block_offset,data,len);
  _card.writeBlock(block_pos,(uint8_t*)block_data);
}


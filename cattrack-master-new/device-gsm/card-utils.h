#ifndef __CARD_UTILS_H__
#define __CARD_UTILS_H__

#include <SD.h>

#define CONFIG_PARTITION_ID   0x0E
#define STORAGE_PARTITION_ID  0x33
#define LOG_PARTITION_ID      0x34

/**************************************************************************
 * Partition ID on SD card for storing logs
 */
void get_data_and_log_partitions(
    Sd2Card& card,
    uint8_t*  config_part_idx,
    uint32_t* data_start, 
    uint32_t* data_size, 
    uint32_t* log_start, 
    uint32_t* log_size);

bool open_file_in_root(SdFile& file, SdVolume& volume, const char* filename, uint8_t oflag);

#endif

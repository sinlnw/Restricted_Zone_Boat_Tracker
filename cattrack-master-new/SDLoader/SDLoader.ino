/* File: SDLoader.ino
 *
 * Bootloader using an image stored in an SD card.
 * Modified from SDUBoot at https://github.com/arduino/ArduinoCore-samd/tree/master/libraries/SDU
 */

#include <SD.h>
#include <FlashStorage.h>

//#define DEBUG

#include "io.h"
#include "config.h"
#include "card-utils.h"
#include "debug.h"

#define SDU_START    0x2000
#define SDU_SIZE     0xC000

#define SKETCH_START (uint32_t*)(SDU_START + SDU_SIZE)

FlashStorage(checksum_store, uint32_t);
FlashClass flash;
Sd2Card card;
SdVolume config_volume;
Config config;


// Initialize C library
extern "C" void __libc_init_array(void);

/*********************************************
 * Initialize SD card and return the pointer to the configuration volume
 * handler
 */
bool init_config_volume() {
  uint8_t config_part_idx;

  // initialize SD card subsystem
  if (!card.init(SPI_HALF_SPEED,PIN_SD_SS)) {
    DEBUG_PRINTF("Cannot initialize SD card\r\n");
    return false;
  }

  // obtain configuration partition index
  get_data_and_log_partitions(
    card, &config_part_idx, nullptr, nullptr, nullptr, nullptr);
  if (config_part_idx == 0) {
    DEBUG_PRINTF("Config partition not found.\r\n");
    return false;
  }

  // accessing configuration file
  if (!config_volume.init(card,config_part_idx)) {
    DEBUG_PRINTF("Cannot init config volume.\r\n");
    return false;
  }

  DEBUG_PRINTF("Config volume initialized.\r\n");
  return true;
}

/*********************************************
 * Read program's name from configuration file.
 * Return null if failed.
 */
char* get_prog_file_name() {
  if (!config.read_from_volume(config_volume)) {
    DEBUG_PRINTF("Cannot read configuration file.\r\n");
    return nullptr;
  }

  if (config.prog_file_name[0] == '\0') {
    DEBUG_PRINTF("Program file not specified in the configuration.\r\n");
    return nullptr;
  }

  DEBUG_PRINTF("Configured program: %s\r\n", config.prog_file_name);
  return config.prog_file_name;
}

/*********************************************
 * Compute checksum for the specified program file
 */
uint32_t compute_checksum(SdFile& prog_file) {
  uint32_t prog_size = prog_file.fileSize();
  uint8_t buffer[512];

  // skip the SDU section
  prog_file.seekSet(SDU_SIZE);
  prog_size -= SDU_SIZE;

  uint32_t checksum = prog_size;
  for (uint32_t i = 0; i < prog_size; i += sizeof(buffer)) {
    uint32_t nbytes = prog_file.read(buffer, sizeof(buffer));

    // compute checksum
    for (uint32_t j = 0; j < nbytes; j++)
      checksum += buffer[j];
  }

  return checksum;
}

/*********************************************
 *
 */
void write_prog(SdFile& prog_file) {
  uint32_t prog_size = prog_file.fileSize();
  uint8_t buffer[512];
  // skip the SDU section
  prog_file.seekSet(SDU_SIZE);
  prog_size -= SDU_SIZE;

  uint32_t flashAddress = (uint32_t)SKETCH_START;

  // erase the pages
  flash.erase((void*)flashAddress, prog_size);

  // write the pages
  for (uint32_t i = 0; i < prog_size; i += sizeof(buffer)) {
    uint32_t nbytes = prog_file.read(buffer, sizeof(buffer));
    flash.write((void*)flashAddress, buffer, sizeof(buffer));
    flashAddress += sizeof(buffer);
  }
}

/*********************************************
 * Blink the built-in LED for the specified number of times
 */
void blink(uint8_t times) {
  for (uint8_t i = 0; i < times; i++) {
    LED_ON();
    delay(250);
    LED_OFF();
    delay(250);
  }
}

/*********************************************
 *
 */
int main() {
  uint32_t new_checksum, cur_checksum;
  char* prog_file_name;
  bool prog_file_open_successful;
  SdFile prog_file;

  init();

  __libc_init_array();

  delay(1);

  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
  pinMode(PIN_RFM95_CS,OUTPUT);
  digitalWrite(PIN_RFM95_CS,HIGH);

#ifdef DEBUG
#if defined(USBCON)
  USBDevice.init();
  USBDevice.attach();
#endif
  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.println("Inside SDLoader...");
#endif

  if (!init_config_volume()) {
    DEBUG_PRINTF("Failed to initialize configuration volume.\r\n");
    goto start_prog;
  }

  prog_file_name = get_prog_file_name();

  if (!prog_file_name) {
    DEBUG_PRINTF("Program file not specified.\r\n");
    goto start_prog;
  }

  prog_file_open_successful = open_file_in_root(
    prog_file,
    config_volume,
    prog_file_name,
    O_READ);
  if (!prog_file_open_successful) {
    DEBUG_PRINTF("Cannot open program file: %s.\r\n", prog_file_name);
    goto start_prog;
  }

  DEBUG_PRINTF("Reading program file: %s\r\n", prog_file_name);

  // compute checksum and rewrite flash only when checksum is different
  // (i.e., loading new program)
  new_checksum = compute_checksum(prog_file);
  cur_checksum = checksum_store.read();

  if (cur_checksum != new_checksum) { // new program to be loaded
    DEBUG_PRINTF("New program found; updating.\r\n");
    write_prog(prog_file);
    checksum_store.write(new_checksum);
    blink(3);
  }
  else {
    DEBUG_PRINTF("Same program found; skip updating.\r\n");
    blink(1);
  }

  prog_file.close();

start_prog:
  DEBUG_PRINTF("Jump to the sketch.\r\n");
  // jump to the sketch
  __set_MSP(*SKETCH_START);

  //Reset vector table address
  SCB->VTOR = ((uint32_t)(SKETCH_START) & SCB_VTOR_TBLOFF_Msk);

  // address of Reset_Handler is written by the linker at the beginning of the .text section (see linker script)
  uint32_t resetHandlerAddress = (uint32_t) * (SKETCH_START + 1);
  // jump to reset handler
  asm("bx %0"::"r"(resetHandlerAddress));
}


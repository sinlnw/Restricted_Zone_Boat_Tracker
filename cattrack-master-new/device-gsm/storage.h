/** 
 * @file
 * @brief Circular storage on AT24C256 EEPROM
 *
 * Definitions for creating and maintaining circular buffers on an AT24C256
 * EEPROM chip.
 *
 * @author Chaiporn Jaikaeo <chaiporn.j@ku.ac.th>
 *
 */
#ifndef __STORAGE_H__
#define __STORAGE_H__

#include <stdint.h>
#include "at24eeprom.h"

#ifdef DEBUG_STORAGE
#define STORAGE_DEBUG_PRINT   DEBUG_PRINT
#define STORAGE_DEBUG_PRINTLN DEBUG_PRINTLN
#define STORAGE_DEBUG_PRINTF  DEBUG_PRINTF
#else
#define STORAGE_DEBUG_PRINT(x)
#define STORAGE_DEBUG_PRINTLN(x)
#define STORAGE_DEBUG_PRINTF(...)
#endif

#define STORAGE_INVALID_ADDRESS   0xFFFFFFFF

template <class T> class Storage {
public: 
  /******************************************************************
   * Construct a circular storage
   *
   * The circular storage is NOT ready for use until setup() is called
   */
  Storage(AT24EEPROM& eeprom):_eeprom(eeprom) { _start_addr = STORAGE_INVALID_ADDRESS; }

  /******************************************************************
   * Return the total number of bytes required for this storage instance
   */
  uint32_t required_bytes() {
    return sizeof(_first) + sizeof(_used) + sizeof(T)*_capacity;
  }

  /******************************************************************
   * Set up the storage blocks on EEPROM
   * The capacity parameter specifies the number of records.  If overwrite is
   * true, the existing blocks will be overwritten
   *
   * @return true if the setup is sucessful; false otherwise
   */
  bool setup(uint32_t start_address, uint32_t capacity, bool overwrite);

  /******************************************************************
   * Return the maximum capacity (in #records) of the circular buffer
   */
  uint32_t capacity() const { return _capacity; }

  /******************************************************************
   * Check if the circular buffer is empty
   * @return true if the buffer is empty; false otherwise
   */
  bool empty() const { return _used == 0; }

  /******************************************************************
   * Check if the circular buffer is full
   * @return true if the buffer is full; false otherwise
   */
  bool full() const { return _used == _capacity; }

  /******************************************************************
   * Return the number of free records left in the buffer
   */
  uint32_t free() const { return _capacity - _used; }

  /******************************************************************
   * Return the number of records stored in the buffer
   */
  uint32_t count() const { return _used; }

  /******************************************************************
   * Push a new data item into the storage
   * @return true if successful (i.e., the storage is available); false otherwise
   */
  bool push(const T& data);

  /******************************************************************
   * Remove the earliest data item from the storage
   * @return true if successful (i.e., the storage is not empty); false otherwise
   */
  bool pop();

  /******************************************************************
   * Retrieve the specified data item without popping it out
   */
  bool peek(T* data, uint32_t index);

  /******************************************************************
   * Retrieve the earliest data item without popping it out
   */
  bool top(T* data) { return peek(data, 0); }

  /******************************************************************
   * Remove all items from the storage
   */
  void clear();

private:
  AT24EEPROM& _eeprom;
  uint32_t _start_addr;  ///< first address
  uint32_t _capacity;    ///< maximum number of records plus one
  uint32_t _first;       ///< index of the first used slot
  uint32_t _used;        ///< number of used items

  // compute byte position for the specified slot index
  uint32_t _position(uint32_t index)
  { return _start_addr + sizeof(_first) + sizeof(_used) + index*sizeof(T); }

  // compute index of the first free slot
  uint32_t _first_free() { return (_first+_used)%_capacity; }

  void _save_state();
  void _load_state();
};

template <class T>
void Storage<T>::_save_state()
{
  uint8_t state[sizeof(_first)+sizeof(_used)];

  if (_start_addr == STORAGE_INVALID_ADDRESS) return;

  STORAGE_DEBUG_PRINTF("STORAGE: Saving state: first=%u used=%u\r\n",
      _first, _used);
  memcpy(state, &_first, sizeof(_first));
  memcpy(state+sizeof(_first), &_used, sizeof(_used));
  _eeprom.write(_start_addr, state, sizeof(state));
}

template <class T>
void Storage<T>::_load_state()
{
  uint8_t state[sizeof(_first)+sizeof(_used)];

  if (_start_addr == STORAGE_INVALID_ADDRESS) return; // not intialized

  STORAGE_DEBUG_PRINTLN("STORAGE: Loading state");
  _eeprom.read(_start_addr, state, sizeof(state));
  memcpy(&_first, state, sizeof(_first));
  memcpy(&_used, state+sizeof(_first), sizeof(_used));

  if ( (_first >= _capacity) || (_used > _capacity) ) {
    STORAGE_DEBUG_PRINTF("STORAGE: Invalid states first=%u used=%u; reset states\r\n",
      _first, _used);
    _first = _used = 0;
    return;
  }

  STORAGE_DEBUG_PRINTF("STORAGE: State loaded, first=%u used=%u\r\n",
      _first, _used);
}

template <class T>
bool Storage<T>::setup(uint32_t start_addr, uint32_t capacity, bool overwrite) 
{
  _start_addr = start_addr;
  _capacity = capacity;
  if (overwrite) 
  {
    _first = 0;
    _used = 0;
    _save_state();
  }
  else
  {
    // retrieve previous state from the EEPROM
    _load_state();
  }

  return true;
}

template <class T>
bool Storage<T>::push(const T& data)
{
  if (_start_addr == STORAGE_INVALID_ADDRESS) return false;

  if (full()) return false;
  _eeprom.write(_position(_first_free()), (uint8_t*)&data, sizeof(T));
  _used++;
  _save_state();
  return true;
}

template <class T>
bool Storage<T>::pop()
{
  if (_start_addr == STORAGE_INVALID_ADDRESS) return false;

  if (empty()) return false;
  _first = (_first+1)%_capacity;
  _used--;
  _save_state();
  return true;
}

template <class T>
bool Storage<T>::peek(T* data, uint32_t index)
{
  if (_start_addr == STORAGE_INVALID_ADDRESS) return false;

  if (index >= _used) return false;
  _eeprom.read(_position((_first+index)%_capacity), (uint8_t*)data, sizeof(T));
  return true;
}

template <class T>
void Storage<T>::clear()
{
  if (_start_addr == STORAGE_INVALID_ADDRESS) return;

  _used = 0;
  _first = 0;
  _save_state();
}
#endif


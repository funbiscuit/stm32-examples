#ifndef H7_SPI_EXAMPLE_CRC_H
#define H7_SPI_EXAMPLE_CRC_H

#include <stdint.h>

// functions to calculate crc in streaming mode
// first call pre, then call upd for each byte/array and in the end call post
uint32_t crc32pre();
uint32_t crc32upd(const uint8_t* bytes, uint16_t length, uint32_t crc);
uint32_t crc32post(uint32_t crc);

uint32_t crc32(uint8_t* bytes, uint16_t length);

#endif //H7_SPI_EXAMPLE_CRC_H

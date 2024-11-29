#ifndef __CRC8_H__
#define __CRC8_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/**
 * Calculates CRC8 of a given seq. of bytes
 * 
 * @param buf a pointer to data we want to calculate CRC8 from
 * @param sz size of the chunk of data the pointer is pointing to
 * @return calculated CRC8 (one byte)
 */
uint8_t crc8(const uint8_t * buf, size_t sz);

#ifdef __cplusplus
}
#endif

#endif /* __CRC8_H__ */
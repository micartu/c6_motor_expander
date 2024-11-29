
#include "crc8.h"

uint8_t crc8(const uint8_t * buf, size_t sz)
{
    uint8_t crc = buf[0];
    for (size_t i = 1; i < sz; ++i)
    {
        crc ^= buf[i];
    }
    return crc;
}
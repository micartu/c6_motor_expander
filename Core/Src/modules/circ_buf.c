#include "circ_buf.h"
#include <string.h>

void circ_buf_init(struct circ_buf_t * buf, uint8_t * buffer, size_t chunksz, size_t buflen)
{
    memset(buf, 0, sizeof(*buf));
    memset(buffer, 0, buflen);
    buf->buffer = buffer;
    buf->chunksz = chunksz;
    buf->buflen = buflen;
    buf->bufmaxind = buflen / chunksz;
}

void circ_buf_push(struct circ_buf_t * c, uint8_t * value)
{
    size_t next = c->head;
    size_t nextsz = c->head * c->chunksz;
    if (nextsz >= c->buflen)
    {
        nextsz = 0;
        next = 0;
    }
    memcpy(c->buffer + nextsz, value, c->chunksz);
    c->head = next + 1;
}

void * circ_buf_value(struct circ_buf_t * c, size_t index)
{
    int vind = c->head - (index % c->bufmaxind) - 1;
    if (vind < 0)
        vind += c->bufmaxind;
    if (vind >= 0 && vind < c->bufmaxind)
        return c->buffer + vind * c->chunksz;
    return 0;
}

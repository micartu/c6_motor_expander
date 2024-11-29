#include "circ_dbuf.h"
#include "circ_buf.h"
#include <string.h>

void circ_double_buf_init(struct circ_buf_t * buf, double * buffer, int maxlen)
{
    size_t sz = sizeof(*buffer);
    circ_buf_init(buf, (uint8_t*)buffer, sz, sz * maxlen);
}

void circ_double_buf_push(struct circ_buf_t * c, double value)
{
   circ_buf_push(c, (uint8_t*)&value);
}

double circ_double_buf_value(struct circ_buf_t * c, int index)
{
    void * data = circ_buf_value(c, index);
    return *(double*)data;
}
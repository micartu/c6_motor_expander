#ifndef __CIRC_DBLBUF_H__
#define __CIRC_DBLBUF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

struct circ_buf_t;

void circ_double_buf_init(struct circ_buf_t * buf, double * buffer, int maxlen);

void circ_double_buf_push(struct circ_buf_t * buf, double value);

double circ_double_buf_value(struct circ_buf_t * buf, int index);

#ifdef __cplusplus
}
#endif

#endif /* __CIRC_DBLBUF_H__ */
#ifndef __CIRC_BUF_H__
#define __CIRC_BUF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

struct circ_buf_t {
    uint8_t * buffer;
    size_t head;
    size_t buflen;
    size_t chunksz;
    size_t bufmaxind;
} ;

void circ_buf_init(struct circ_buf_t * buf, uint8_t * buffer, size_t chunksz, size_t buflen);

void circ_buf_push(struct circ_buf_t * buf, uint8_t * value);

void * circ_buf_value(struct circ_buf_t * buf, size_t index);

#ifdef __cplusplus
}
#endif

#endif /* __CIRC_BUF_H__ */
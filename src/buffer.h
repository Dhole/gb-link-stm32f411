#ifndef BUFFER_H
#define BUFFER_H

#include <stdint.h>

#define BUF_LEN 2048

struct circular_buf {
	uint8_t buf[BUF_LEN];
	uint32_t head;
	uint32_t tail;
	uint32_t len;
};

static inline void
buf_push(struct circular_buf *buf, uint8_t b)
{
	buf->len++;
	buf->buf[buf->head] = b;
	buf->head = (buf->head + 1) % BUF_LEN;
}

static inline uint8_t
buf_pop(struct circular_buf *buf)
{
	uint8_t b = buf->buf[buf->tail];
	buf->tail = (buf->tail + 1) % BUF_LEN;
	buf->len--;
	return b;
}

static inline void
buf_clear(struct circular_buf *buf)
{
	buf->head = 0;
	buf->tail = 0;
	buf->len = 0;
}

static inline int
buf_empty(struct circular_buf *buf)
{
	return (buf->tail == buf->head);
}

#endif

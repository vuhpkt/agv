#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef struct {
    uint8_t *buffer;
    size_t size;
    volatile size_t head;
    volatile size_t tail;
} RingBuffer_t;

void RingBuffer_Init(RingBuffer_t *rb, uint8_t *storage, size_t size);
bool RingBuffer_Push(RingBuffer_t *rb, uint8_t data);
size_t RingBuffer_PushArray(RingBuffer_t *rb, const uint8_t *data, size_t len);
bool RingBuffer_Pop(RingBuffer_t *rb, uint8_t *data);
size_t RingBuffer_PopArray(RingBuffer_t *rb, uint8_t *data, size_t max_len);
size_t RingBuffer_Available(const RingBuffer_t *rb);
size_t RingBuffer_FreeSpace(const RingBuffer_t *rb);
void RingBuffer_Flush(RingBuffer_t *rb);


#endif
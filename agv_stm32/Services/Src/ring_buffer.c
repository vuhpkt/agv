#include "ring_buffer.h"

void RingBuffer_Init(RingBuffer_t *rb, uint8_t *storage, size_t size) {
    rb->buffer = storage;
    rb->size = size;
    rb->head = 0;
    rb->tail = 0;
}

bool RingBuffer_Push(RingBuffer_t *rb, uint8_t data) {
    size_t next_head = (rb->head + 1) % rb->size;
    if (next_head == rb->tail) return false;
    rb->buffer[rb->head] = data;
    rb->head = next_head;
    return true;
}

size_t RingBuffer_PushArray(RingBuffer_t *rb, const uint8_t *data, size_t len) {
    size_t byte_pushed = 0;
    for (size_t i = 0; i < len; i++) {
        if (!RingBuffer_Push(rb, data[i])) break;
        byte_pushed++;
    }
    return byte_pushed;
}

bool RingBuffer_Pop(RingBuffer_t *rb, uint8_t *data) {
    if (rb->head == rb->tail) return false;
    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % rb->size;
    return true;
}

size_t RingBuffer_PopArray(RingBuffer_t *rb, uint8_t *data, size_t max_len) {
    size_t bytes_popped = 0;
    while (bytes_popped < max_len && RingBuffer_Pop(rb, &data[bytes_popped])) {
        bytes_popped++;
    }
    return bytes_popped;
}

size_t RingBuffer_Available(const RingBuffer_t *rb) {
    return (rb->head >= rb->tail) ? (rb->head - rb->tail) : (rb->size + rb->head - rb->tail);
}

size_t RingBuffer_FreeSpace(const RingBuffer_t *rb) {
    return (rb->size - 1) - RingBuffer_Available(rb);
}

void RingBuffer_Flush(RingBuffer_t *rb) {
    rb->tail = rb->head;
}

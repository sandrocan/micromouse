#ifndef QUEUE_H
#define QUEUE_H

#include <stdbool.h>
#include <stdint.h>

#define QUEUE_CAPACITY 36

typedef struct
{
    uint8_t x;
    uint8_t y;
} Pos;

typedef struct
{
    Pos data[QUEUE_CAPACITY];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} PosQueue;

void queue_init(volatile PosQueue *q);
bool queue_is_empty(const volatile PosQueue *q);
bool queue_is_full(const volatile PosQueue *q);
bool queue_push(volatile PosQueue *q, Pos value);
bool queue_pop(volatile PosQueue *q, volatile Pos *value);

#endif /* QUEUE_H */

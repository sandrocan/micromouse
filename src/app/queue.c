#include "queue.h"

void queue_init(volatile PosQueue *q)
{
    q->head = 0;
    q->tail = 0;
    q->count = 0;
}

bool queue_is_empty(const volatile PosQueue *q)
{
    return q->count == 0;
}

bool queue_is_full(const volatile PosQueue *q)
{
    return q->count == QUEUE_CAPACITY;
}

bool queue_contains(const volatile PosQueue *q, Pos value)
{
    for (uint8_t i = 0; i < q->count; ++i)
    {
        uint8_t index = (q->head + i) % QUEUE_CAPACITY;
        if (q->data[index].x == value.x && q->data[index].y == value.y)
        {
            return true;
        }
    }

    return false;
}

bool queue_push(volatile PosQueue *q, Pos value)
{
    if (queue_is_full(q))
    {
        return false;
    }

    q->head = (q->head + QUEUE_CAPACITY - 1) % QUEUE_CAPACITY;
    q->data[q->head] = value;
    q->count++;
    q->tail = (q->head + q->count) % QUEUE_CAPACITY;
    return true;
}

bool queue_pop(volatile PosQueue *q, volatile Pos *value)
{
    if (queue_is_empty(q))
    {
        return false;
    }

    *value = q->data[q->head];
    q->head = (q->head + 1) % QUEUE_CAPACITY;
    q->count--;
    q->tail = (q->head + q->count) % QUEUE_CAPACITY;
    return true;
}

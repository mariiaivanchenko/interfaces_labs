/*
 * parserqueue.cpp
 *
 *  Created on: Sep 14, 2025
 *      Author: mariia
 */

#include "parserqueue.h"

bool isEmpty(volatile parser_queue* q) { return (q->count == 0); }
bool isFull(volatile parser_queue* q) { return (q->count == QUEUE_CAPACITY); }


void initialize(volatile parser_queue* q)
{
	q->head = 0;
	q->tail = 0;
	q->count = 0;
}


bool enqueue(volatile parser_queue* q, const uint8_t *data, uint16_t size)
{
    if (isFull(q)) {
        return false;
    }

    if (size > MESSAGE_SIZE) {
        return false;
    }

    memcpy(q->items[q->head], data, size);
    q->sizes[q->head] = size;

    q->head = (q->head + 1) % QUEUE_CAPACITY;
    q->count++;

    return true;
}

bool dequeue(volatile parser_queue* q, uint8_t *out, uint16_t *size)
{
	if (isEmpty(q)) {
		return false;
	}

	uint16_t msg_size = q->sizes[q->tail];
	memcpy(out, q->items[q->tail], msg_size);

	*size = msg_size;

	q->tail = (q->tail + 1) % QUEUE_CAPACITY;
	q->count--;

	return true;
}



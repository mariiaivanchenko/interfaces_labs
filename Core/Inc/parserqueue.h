/*
 * parserqueue.h
 *
 *  Created on: Sep 14, 2025
 *      Author: mariia
 */

#ifndef INC_PARSERQUEUE_H_
#define INC_PARSERQUEUE_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#define QUEUE_CAPACITY 10
#define MESSAGE_SIZE 512


typedef struct {
    uint8_t items[QUEUE_CAPACITY][MESSAGE_SIZE];
    uint16_t sizes[QUEUE_CAPACITY];
    uint8_t head;  // index of first portion
    uint8_t tail;  // index of last portion
    uint8_t count; // number of portions in queue
} parser_queue;


bool isEmpty(volatile parser_queue* q);
bool isFull(volatile parser_queue* q);

void initialize(volatile parser_queue* q);
bool enqueue(volatile parser_queue* q, const uint8_t *data, uint16_t size);
bool dequeue(volatile parser_queue* q, uint8_t *out, uint16_t *size);

void print_queue(UART_HandleTypeDef *huart, volatile parser_queue* q);

#endif /* INC_PARSERQUEUE_H_ */

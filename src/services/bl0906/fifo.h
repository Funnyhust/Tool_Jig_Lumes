/*******************************************************************************
 *				 _ _                                             _ _
				|   |                                           (_ _)
				|   |        _ _     _ _   _ _ _ _ _ _ _ _ _ _   _ _
				|   |       |   |   |   | |    _ _     _ _    | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |_ _ _  |   |_ _|   | |   |   |   |   |   | |   |
				|_ _ _ _ _| |_ _ _ _ _ _| |_ _|   |_ _|   |_ _| |_ _|
								(C)2022 Lumi
 * Copyright (c) 2022
 * Lumi, JSC.
 * All Rights Reserved
 *
 * File name: fifo.h
 *
 * Description:
 *
 *
 * Last Changed By:  $Author: duongnv $
 * Revision:         $Revision: 1.0.0 $
 * Last Changed:     $Date: January 21, 2026 $
 *
 * Code sample:
 ******************************************************************************/
#ifndef FIFO_H
#define FIFO_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct {
  uint8_t *buffer;
  uint16_t element_size;
  uint16_t capacity;
  uint16_t head;
  uint16_t tail;
  uint16_t count;
} Fifo_t;

static inline void FifoInit(Fifo_t *fifo, void *buffer, uint16_t element_size,
                            uint16_t capacity) {
  fifo->buffer = (uint8_t *)buffer;
  fifo->element_size = element_size;
  fifo->capacity = capacity;
  fifo->head = 0;
  fifo->tail = 0;
  fifo->count = 0;
}

static inline bool FifoIsEmpty(Fifo_t *fifo) { return fifo->count == 0; }

static inline bool FifoIsFull(Fifo_t *fifo) {
  return fifo->count >= fifo->capacity;
}

static inline bool FifoPush(Fifo_t *fifo, void *element) {
  if (FifoIsFull(fifo)) {
    return false;
  }
  uint16_t index = fifo->tail * fifo->element_size;
  memcpy(&fifo->buffer[index], element, fifo->element_size);
  fifo->tail = (fifo->tail + 1) % fifo->capacity;
  fifo->count++;
  return true;
}

static inline bool FifoPop(Fifo_t *fifo, void *element) {
  if (FifoIsEmpty(fifo)) {
    return false;
  }
  uint16_t index = fifo->head * fifo->element_size;
  memcpy(element, &fifo->buffer[index], fifo->element_size);
  fifo->head = (fifo->head + 1) % fifo->capacity;
  fifo->count--;
  return true;
}

#endif // FIFO_H

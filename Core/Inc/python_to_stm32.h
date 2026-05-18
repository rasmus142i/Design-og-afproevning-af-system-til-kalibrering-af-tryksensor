#ifndef PYTHON_TO_STM32_H
#define PYTHON_TO_STM32_H

#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "ads.h"

#define RING_BUF_SIZE 4000

typedef struct {
    uint8_t buffer[RING_BUF_SIZE];
    volatile uint16_t head;  // write index
    volatile uint16_t tail;  // read index
} RingBuffer;


int RingBuffer_Read(RingBuffer *rb, uint8_t *byte);
void RingBuffer_Write(RingBuffer *rb, uint8_t byte);

// Start the background interrupt
void init_ringbuffer_interrupt(UART_HandleTypeDef *uart);

//parse_commands
void parse_commands(RingBuffer *rb, ADS_Stats *ads);







#endif

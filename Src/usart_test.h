// File: usart_test.h
// Author: Robert Gałat
// Email: robgal519@gmail.com

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>

void init_Hal_UART(void **data, uint32_t baudrate);

bool transfer_IT_Hal_UART(void *internal, uint8_t *data, uint16_t size);
bool transfer_DMA_Hal_UART(void *internal, uint8_t *data, uint16_t size);
bool deinit_Hal_UART(void *internal);
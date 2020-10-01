// File: usart_test.h
// Author: Robert Gałat
// Email: robgal519@gmail.com

#include "stm32f4xx.h"
#include <stdint.h>

void init_Hal_UART(UART_HandleTypeDef *huart, uint32_t baudrate);

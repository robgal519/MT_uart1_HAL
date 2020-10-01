// File: calibration.h
// Author: Robert Gałat
// Email: robgal519@gmail.com

#include "stdint.h"
#include "stm32f4xx.h"

void setup_timer1(UART_HandleTypeDef *uart, uint32_t baudrate);
HAL_StatusTypeDef start_timer(UART_HandleTypeDef *uart, uint8_t *data,
                              uint16_t size) ;
HAL_StatusTypeDef deinit_timer(UART_HandleTypeDef *huart);
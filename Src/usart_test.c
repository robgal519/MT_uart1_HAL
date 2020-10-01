// File: usart_test.c
// Author: Robert Gałat
// Email: robgal519@gmail.com

#include "usart_test.h"

#include "stm32f4xx.h"
#include <stdbool.h>
#include <stdint.h>

extern volatile bool UART_TransferComplete;
extern UART_HandleTypeDef huart1;

void init_Hal_UART(UART_HandleTypeDef *huart, uint32_t baudrate) {
  huart->Instance = USART1;
  huart->Init.BaudRate = baudrate;
  huart->Init.WordLength = UART_WORDLENGTH_8B;
  huart->Init.StopBits = UART_STOPBITS_1;
  huart->Init.Parity = UART_PARITY_NONE;
  huart->Init.Mode = UART_MODE_TX_RX;
  huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart->Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(huart) != HAL_OK) {
    Error_Handler();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1)
    UART_TransferComplete = true;
}
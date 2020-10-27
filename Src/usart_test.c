// File: usart_test.c
// Author: Robert Gałat
// Email: robgal519@gmail.com

#include "usart_test.h"

#include "stm32f4xx.h"
#include <stdbool.h>
#include <stdint.h>

extern volatile bool UART_TransferComplete;
extern UART_HandleTypeDef huart1;

void init_Hal_UART(void **data, uint32_t baudrate) {

  huart1.Instance = USART1;
  huart1.Init.BaudRate = baudrate;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  HAL_UART_Init(&huart1);
  *data = &huart1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1)
    UART_TransferComplete = true;
}

bool transfer_IT_Hal_UART(void *internal, uint8_t *data, uint16_t size) {
  return HAL_UART_Transmit_IT(internal, data, size) == HAL_OK;
}

bool transfer_DMA_Hal_UART(void *internal, uint8_t *data, uint16_t size) {
  return HAL_UART_Transmit_DMA(internal, data, size) == HAL_OK;
}
bool deinit_Hal_UART(void *internal) {
  return HAL_UART_DeInit(internal) == HAL_OK;
}
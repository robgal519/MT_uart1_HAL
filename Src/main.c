/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Driver_USART.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern ARM_DRIVER_USART Driver_USART1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void initUSART(ARM_USART_SignalEvent_t f, unsigned int baudrate, ARM_DRIVER_USART *uart)
{
	uart->Initialize(f);
	uart->PowerControl(ARM_POWER_FULL);
	uart->Control(ARM_USART_MODE_ASYNCHRONOUS |
					  ARM_USART_DATA_BITS_8 |
					  ARM_USART_PARITY_NONE |
					  ARM_USART_STOP_BITS_1 |
					  ARM_USART_FLOW_CONTROL_NONE,
				  baudrate);
	uart->Control(ARM_USART_CONTROL_TX, 1);
	uart->Control(ARM_USART_CONTROL_RX, 1);
}

void initGPIOA(){
	static GPIO_InitTypeDef outputPins;
	outputPins.Pin = GPIO_PIN_4;
	outputPins.Mode = GPIO_MODE_OUTPUT_PP;
	outputPins.Pull = GPIO_PULLDOWN;
	outputPins.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	// outputPins.Alternate  not set

	HAL_GPIO_Init(GPIOA,&outputPins);
	
}

size_t testBaudrate(size_t baudrate, ARM_DRIVER_USART *uart){
  static uint8_t dummy[500];
  static size_t counter = 0;

  initUSART(NULL, baudrate, uart);
  counter = 0;
  uart->Send(dummy, sizeof(dummy));
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
  while(uart->GetStatus().tx_busy){
    counter++;
  }
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
  uart->Uninitialize();
  return counter;
}

void printResult(size_t*results, size_t size, ARM_DRIVER_USART* uart ){
    initUSART(NULL,9600,uart);
  for(size_t d = 0; d< size; d++){
    static char message[10];
    sprintf(message, "%d|", results[d]);
    uart->Send(message, strlen(message));
    while(uart->GetStatus().tx_busy){;}
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  initGPIOA();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  static ARM_DRIVER_USART *uart = &Driver_USART1;
  static size_t baudrates[] = {
    4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600
  };
  #define TEST_COUNT (sizeof(baudrates)/sizeof(*baudrates))
  #define RETRY 3
  static size_t results [TEST_COUNT*RETRY] = {0};

  for(size_t test = 0; test < TEST_COUNT; test++){
    for(size_t retry = 0; retry<RETRY; retry++){
      results[test*RETRY+retry] = testBaudrate(baudrates[test],uart);
    }
  }
  printResult(results, TEST_COUNT*RETRY, uart);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

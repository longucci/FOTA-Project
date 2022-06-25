/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <memory.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;
char command[27] = {0};
int size_of_fw = 0;
__attribute__((section (".RamVectorTable"))) volatile char vector_in_ram[0x198] = {0};
__attribute__((section (".RamFunc"))) void SysTick_Handle()
{

}
/* USER CODE BEGIN PV */
char flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
__attribute__((section (".RamFunc"))) void flash_erase(char sector){
	if((FLASH->CR>>31 & 1)==1)
	{
		FLASH->KEYR = 0x45670123;
		FLASH->KEYR = 0xCDEF89AB;
	}

	while(((FLASH->SR>>16)&1)==1);	//Check busy flag
	FLASH->CR |= 1<<1;	//Activate sector erase selection
	FLASH->CR |= sector << 3;	//Choose sector
	FLASH->CR |= 1<<16;	//Set START bit
	while(((FLASH->SR>>16)&1)==1);	//wait busy flag is cleared
	FLASH->CR &= ~(1<<1);
}
__attribute__((section (".RamFunc"))) void flash_program(char* addr, char data){
	if(((FLASH->CR>>31) & 1)==1)
	{
		FLASH->KEYR = 0x45670123;
		FLASH->KEYR = 0xCDEF89AB;
	}

	while(((FLASH->SR>>16)&1)==1);	//Check busy flag

	FLASH->CR |= 1<<0;	//Activate programming (PG bit)
	*addr = data;

	while(((FLASH->SR>>16)&1)==1);	//wait busy flag is cleared
	FLASH->CR = 0;
}

__attribute__((section(".RamFunc"))) void update(uint8_t* data, int size){
	flash_erase(0);	//Erase sector 0
	char* start = (char*)0x08000000;
	for(int i=0; i < size; i++ ){
		flash_program(start+i, data[i]);	//Write new firmware into sector 0
	}
	uint32_t* AIRCR =(uint32_t*)0xE000ED0C;	//Software reset to load new firmware
	*AIRCR = (0x5FA << 16) | (1<<2);
	while(1);
}
/* USER CODE END PFP */
void Led_Toggle(){
	__HAL_RCC_GPIOD_CLK_ENABLE();
	GPIOD->MODER |= 0b01<<26;
	GPIOD->ODR |= 1<<13;
	HAL_Delay(1000);
	GPIOD->ODR &= ~(1<<13);
	HAL_Delay(1000);
}
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();


  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  uint8_t* msg = (uint8_t*)"Update new firmware\r\n";
  uint8_t* require_msg = (uint8_t*)"Please enter your firmware size !!!!\n";
  HAL_UART_Transmit(&huart6, msg, strlen((char*)msg), HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart6, require_msg, strlen((char*)require_msg), HAL_MAX_DELAY);
  HAL_UART_Receive(&huart6,command, 27, HAL_MAX_DELAY);
  sscanf((char*)command,"firmware size: %d bytes", &size_of_fw);
  uint8_t* data_of_fw = malloc(size_of_fw);
  HAL_UART_Receive_DMA(&huart6, data_of_fw, size_of_fw);

  while (1)
  {
    Led_Toggle();
    if(flag == 1){
	uint8_t* confirm_msg = (uint8_t*)"Updated firmware\r\n";
	HAL_UART_Transmit(&huart6, confirm_msg, strlen((char*)confirm_msg), HAL_MAX_DELAY);
	memcpy((char*)vector_in_ram,(uint32_t*)0x08000000,0x198);
	SCB->VTOR |= 0x20000000;
	uint32_t* systick_handler_addr = (uint32_t*)0x2000003C;
	*systick_handler_addr = (uint32_t)SysTick_Handle |1;
	update(data_of_fw, size_of_fw);
    }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


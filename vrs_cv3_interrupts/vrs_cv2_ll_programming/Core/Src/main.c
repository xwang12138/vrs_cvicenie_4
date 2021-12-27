/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "assignment.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/*
	   * DO NOT WRITE TO THE WHOLE REGISTER!!!
	   *
	   * Write to bits, that are meant for change.
	   */

	  /*Enables clock for GPIO port B*/
/*	  *((volatile uint32_t *) (uint32_t)(0x40021000 + 0x00000014U)) |= (uint32_t)(1 << 18);
	  *((volatile uint32_t *)((uint32_t)0x48000400)) &= ~(uint32_t)(0x3 << 6);
	  *((volatile uint32_t *)((uint32_t)0x48000400)) |= (uint32_t)(1 << 6);
	  *((volatile uint32_t *)((uint32_t)0x48000400)) &= ~(uint32_t)(0x3 << 12);
	  *((volatile uint32_t *)((uint32_t)(0x48000400 + 0x04U))) &= ~(1 << 3);
	  *((volatile uint32_t *)((uint32_t)(0x48000400 + 0x08U))) &= ~(0x3 << 6);
	  *((volatile uint32_t *)((uint32_t)(0x48000400 + 0x0CU))) |= (1 << 12);
	  *((volatile uint32_t *)((uint32_t)(0x48000400 + 0x0CU))) &= ~(0x3 << 6);
*/
	  RCC_AHBENR_REG|= (uint32_t)(1 << 18);//RCC寄存器，给PB口使能时钟。
	  /*GPIOB pin 3 and 6 setup*/
	  /*GPIO MODER register*/
	  //Set mode for pin 3，，所有数据在写寄存器运算时要转化为32位
	  GPIOB_MODER_REG &= ~(uint32_t)(0x3 << 6);//设定gpio寄存器模式 PB3口为输入复位
	  GPIOB_MODER_REG|= (uint32_t)(1 << 6);//设定PB3口为01，通用输出模式。
	  //Set mode for pin 6 设置PB6口为输入复位状态
	  GPIOB_MODER_REG &= ~(uint32_t)(0x3 << 12);//设置PB6口为输入复位状态
	  /*GPIO OTYPER register*/
	  GPIOB_OTYPER_REG &= ~(1 << 3);
	  /*GPIO OSPEEDR register*/
	  //Set Low speed for GPIOB pin 3
	  GPIOB_OSPEEDER_REG &= ~(0x3 << 6);
	  /*GPIO PUPDR register, reset*/
	  //Set pull up for GPIOB pin 6 (input)
	  GPIOB_PUPDR_REG |= (1 << 12);//01上拉
	  //Set no pull for GPIOB pin 3
	  GPIOB_PUPDR_REG &= ~(0x3 << 6);

	  while (1)
	  {
		  //GPIO IDR, read input from pin 6 用输入数据寄存器读取PB6口数据
		  if(!(GPIOB_IDR_REG & (1 << 6)))
		  {
			  //GPIO BSRR register, set output pin 3
			  LED_ON;
			  //delay
			  for(uint16_t i = 0; i < 0xFF00; i++){}
			  //GPIO BRR, reset output pin 3
			  LED_OFF;
			  //delay
			  for(uint16_t i = 0; i < 0xFF00; i++){}
		  }
		  else
		  {
			  //GPIO BSRR register, set output pin 3
			  LED_ON;
			  //delay
			  for(uint32_t i = 0; i < 0xFFFF0; i++){}
			  //GPIO BRR, reset output pin 3
			  LED_OFF;
			  //delay
			  for(uint32_t i = 0; i < 0xFFF00; i++){}
		  }
	  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_Init1msTick(8000000);
  LL_SetSystemCoreClock(8000000);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

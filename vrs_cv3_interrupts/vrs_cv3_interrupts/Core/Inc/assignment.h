/*
 * assignment.h
 *
 *  Created on: Nov 7, 2021
 *      Author: 10786
 */

#ifndef INC_ASSIGNMENT_H_
#define INC_ASSIGNMENT_H_



#endif /* INC_ASSIGNMENT_H_ */
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_ll_exti.h"
#include "stm32f3xx_ll_cortex.h"
#include "stm32f3xx_ll_utils.h"
#include "stm32f3xx_ll_pwr.h"
#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

//GPIOA peripheral base address
#define	GPIOA_BASE_ADDR			(uint32_t)(0x48000000U)//U 表示无符号型，unit32_t 表示强制转化为32位数

//GPIOB peripheral base address gpiob寄存器的基地址
#define	GPIOB_BASE_ADDR			(uint32_t)(0x48000400U)
//MODER register 模式寄存器
#define	GPIOB_MODER_REG			*(uint32_t *)(GPIOB_BASE_ADDR + 0x00U)//强制转化为32位指针类型，
//OTYPER register类型寄存器
#define	GPIOB_OTYPER_REG		*(uint32_t *)(GPIOB_BASE_ADDR + 0x04U)
//OSPEEDER register 端口输出速度寄存器
#define GPIOB_OSPEEDER_REG		*(uint32_t *)(GPIOB_BASE_ADDR + 0x08U)
//PUPDR register  上下拉
#define GPIOB_PUPDR_REG			*(uint32_t *)(GPIOB_BASE_ADDR + 0x0CU)
//IDR register 输入数据寄存器
#define GPIOB_IDR_REG			*(uint32_t *)(GPIOB_BASE_ADDR + 0x10U)
//ODR register  输出数据寄存器
#define GPIOB_ODR_REG			*(uint32_t *)(GPIOB_BASE_ADDR + 0x14U)
//BSRR register 端口复位 置位寄存器
#define GPIOB_BSRR_REG			*(uint32_t *)(GPIOB_BASE_ADDR + 0x18U)
//BRR register  端口位置寄存器
#define GPIOB_BRR_REG			*(uint32_t *)(GPIOB_BASE_ADDR + 0x28U)

//RCC base address
#define	RCC_BASE_ADDR			(uint32_t)(0x40021000U)
//AHBEN register
#define	RCC_AHBENR_REG			*((volatile uint32_t *) (uint32_t)(RCC_BASE_ADDR + 0x00000014U))

#define LED_ON					*((volatile uint32_t *)((uint32_t)(0x48000400 + 0x18U))) |= (1 << 3)   	//GPIOB pin 3
#define LED_OFF					*((volatile uint32_t *)((uint32_t)0x48000400 + 0x28U)) |= (1 << 3);		//GPIOB pin 3

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif



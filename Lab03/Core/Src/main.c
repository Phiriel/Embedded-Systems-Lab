/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
	HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); //Configure the system clock
	/* This example uses HAL library calls to control
	the GPIOC peripheral. You’ll be redoing this code
	with hardware register access. */
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable the GPIOC clock in the RCC
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; 
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Enable the TIM2 & TIM3 clock in the RCC
	//uint32_t click = 0;
	
	GPIOC->MODER |= (0x1<<13) |  (0x1<<15) |  (0x1<<16) | (0x1<<18);
	GPIOC->MODER &= ~(0x1<<12);
	GPIOC->MODER &= ~(0x1<<14);
	GPIOC->OTYPER &= ~(0b1<<6) | (0b1<<7) | (0b1<<8) | (0b1<<9);
	GPIOC->OSPEEDR &= ~(0x1<<12) | (0x0<<13) | (0x1<<14) | (0x0<<15) | (0x1<<16) | (0x0<<17) | (0x1<<18) | (0x0<<19);
	GPIOC->PUPDR &= ~(0x1<<12) | (0x1<<13) | (0x1<<14) | (0x1<<15) | (0x1<<16) | (0x1<<17) | (0x1<<18) | (0x1<<19);
	
	TIM2->PSC = 0x1F3F;
	TIM2->ARR = 0xFA;
	TIM2->DIER |= (0x1<<0);
	TIM2->CR1 |= (0x1<<0);
	
	TIM3->PSC = 499;
	TIM3->ARR = 20;
	TIM3->CCMR1 &= ~(0x1<<0) | (0x1<<1);
	TIM3->CCMR2 &= ~(0x1<<0) | (0x1<<1);
	TIM3->CCMR1 &= ~(0x1<<8) | (0x1<<9);
	TIM3->CCMR2 &= ~(0x1<<8) | (0x1<<9);
	TIM3->CCMR1 |= (0x1<<4) | (0x1<<5) | (0x1<<6);
	TIM3->CCMR1 |= (0x1<<12) | (0x1<<13);
	TIM3->CCMR1 &= ~(0x1<<14);
	TIM3->CCMR2 |= (0x1<<4) | (0x1<<5) | (0x1<<6);
	TIM3->CCMR2 |= (0x1<<12) | (0x1<<13);
	TIM3->CCMR2 &= ~(0x1<<14);
	TIM3->CCMR1 |= (0x1<<11) | (0x1<<3);
	TIM3->CCMR2 |= (0x1<<11) | (0x1<<3);
	TIM3->CCER |= (0x1<<0) | (0x1<<4);
	TIM3->CCR1 |= 16;
	TIM3->CCR2 |= 16;
	TIM3->CR1 |= (0x1<<0);
	GPIOC->AFR[0] &= ~(GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7) ;
	
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn,1);
	
	GPIOC->ODR |= (0x1<<9);

	
	while (1) {

	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void TIM2_IRQHandler(void){
	GPIOC->ODR ^= (0x1<<8) | (0x1<<9);
	TIM2->SR &= ~(0x1<<0);
}

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

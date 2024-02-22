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
	the GPIOC peripheral. You�ll be redoing this code
	with hardware register access. */
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable the GPIOC clock in the RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;	// Enable the GPIOA clock in the RCC
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	// Set up a configuration struct to pass to the initialization function
	GPIOC->MODER |= (0x1<<12) | (0x0<<13) | (0x1<<14) | (0x0<<15) | (0x1<<16) | (0x0<<17) | (0x1<<18) | (0x0<<19);
	GPIOC->OTYPER &= ~(0b1<<6) | (0b1<<7) | (0b1<<8) | (0b1<<9);
	GPIOC->OSPEEDR &= ~(0x1<<12) | (0x0<<13) | (0x1<<14) | (0x0<<15) | (0x1<<16) | (0x0<<17) | (0x1<<18) | (0x0<<19);
	GPIOC->PUPDR &= ~(0x1<<12) | (0x1<<13) | (0x1<<14) | (0x1<<15) | (0x1<<16) | (0x1<<17) | (0x1<<18) | (0x1<<19);
	
	/*GPIOC->MODER |= (0x1<<12) | (0x0<<13);
	GPIOC->OSPEEDR &= ~(0x1<<12) | (0x0<<13);
	GPIOC->OTYPER &= ~(0b1<<6);
	GPIOC->PUPDR &= ~(0x1<<12) | (0x1<<13);*/
	
	GPIOA->MODER |= (0x0<<0) | (0x0<<1);
	GPIOA->OSPEEDR &= ~(0x1<<0) | (0x0<<1);
	GPIOA->PUPDR |= (0x1<<0) | (0x0<<1);
	
	EXTI->IMR |= (0x1<<0);
	EXTI->RTSR |= (0x1<<0);
	
	SYSCFG->EXTICR[1] &= ~(0x0<<0) | (0x0<<1) | (0x0<<2) | (0x0<<3); 
	
	NVIC_EnableIRQ(5);
	NVIC_SetPriority(5,1); // Enabling and setting priority to EXTI
  /*NVIC_EnableIRQ(-1);
	NVIC_SetPriority(-1,3); // Enabling and setting priority to SysTick */
	
	GPIOC->ODR |= (0x1<<7);
	GPIOC->ODR |= (0x1<<9);
	GPIOC->ODR |= (0x1<<6);
	
	uint32_t debouncer = 0;
	//uint32_t click = 0;
	while (1) {
		GPIOC->ODR ^= (0x1<<6);
		HAL_Delay(400);
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
void EXTI0_1_IRQHandler(void){
  /* Interrupt Handler code */
	GPIOC->ODR ^= (0x1<<8);
	GPIOC->ODR ^= (0x1<<9);
	EXTI->PR |= (0x1<<0);
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
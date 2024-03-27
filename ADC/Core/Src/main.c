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
char recieve;

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

		/* USER CODE END SysInit */

		/* Initialize all configured peripherals */
		/* USER CODE BEGIN 2 */

	GPIO_InitTypeDef initLED = {GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_6 | GPIO_PIN_7,
	GPIO_MODE_OUTPUT_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initLED);

  GPIOC->MODER |= ((0x01 << 8) | (0x01 << 9));
  GPIOC->PUPDR |= ((0x01 << 8) | (0x01 << 9));

  GPIOA->MODER |= ((0x01 << 8) | (0x01 << 9));
  GPIOA->PUPDR |= ((0x01 << 8) | (0x01 << 9));

  ADC->CFGR1 |= (0x1 << 13);
  ADC->CFGR1 &= ~((0x1 << 10) | (0x1 << 11));
  ADC->CFGR1 &= ~(0x1 << 4);
  ADC->CFGR1 |= (0x1 << 3);

  ADC->CHSELR |= (0x1 << 14);

  ADC->CR |= (0x1 << 0);
  ADC->CR |= (0x1 << 2);

  DAC1->CR |= ((0x1<<3) | (0x1<<4) | (0x1<<5) | (0x1<<0));

  const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
      232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};

  uint16_t adc = 0;
  unit16_t i = 0;

  while(1){
    adc = ADC->DR;
    if(adc < 204){
      GPIOC->ODR |= (0x1 << 6);
      GPIOC->ODR &= ~((0x1 << 7) | (0x1 << 8) | (0x1 << 9));
    }else if(adc < 153){
      GPIOC->ODR |= (0x1 << 7);
      GPIOC->ODR &= ~((0x1 << 6) | (0x1 << 8) | (0x1 << 9));
    }else if(adc < 102){
      GPIOC->ODR |= (0x1 << 8);
      GPIOC->ODR &= ~((0x1 << 7) | (0x1 << 6) | (0x1 << 9));
    }else if(adc < 51){
      GPIOC->ODR |= (0x1 << 9);
      GPIOC->ODR &= ~((0x1 << 7) | (0x1 << 8) | (0x1 << 6));
    }else{
      GPIOC->ODR &= ~((0x1 << 6) | (0x1 << 7) | (0x1 << 8) | (0x1 << 9));
    }
    
    for(i=0;i<32;i++){
      DAC1->DHR8R1 = sine_table[i];
      HAL_Delay(1);
    }

  }

	
		/* USER CODE END 2 */
		/* Infinite loop */
		/* USER CODE BEGIN WHILE */
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

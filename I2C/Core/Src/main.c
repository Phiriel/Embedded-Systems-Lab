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

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

		/* USER CODE END SysInit */

		/* Initialize all configured peripherals */
		/* USER CODE BEGIN 2 */

	GPIO_InitTypeDef initLED = {GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_6 | GPIO_PIN_7,
	GPIO_MODE_OUTPUT_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initLED);

	//Set PB11 to alternate function mode, open-drain output type, and select I2C2_SDA as its alternate function
	GPIOB->MODER |= (0x1<<23);
	GPIOB->MODER &= ~(0x1<<22);
	GPIOB->OTYPER |= (0x1<<11);
	GPIOB->AFR[1] |= (1 << 12);

	//Set PB13 to alternate function mode, open-drain output type, and select I2C2_SCL as its alternate function
	GPIOB->MODER |= (0x1<<27);
	GPIOB->MODER &= ~(0x1<<26);
	GPIOB->OTYPER |= (0x1<<13);
	GPIOB->AFR[1] |= (5 << 20);

	//Set PB14 to output mode, push-pull output type, and initialize/set the pin high
	GPIOB->MODER |= (0x1<<28);
	GPIOB->MODER &= ~(0x1<<29);
	GPIOB->OTYPER &= ~(0x0<<14);
	GPIOB->ODR |= (0x1<<14);

	//Set PC0 to output mode, push-pull output type, and initialize/set the pin high
	GPIOC->MODER |= (0x1<<0);
	GPIOC->MODER &= ~(0x1<<1);
	GPIOC->OTYPER &= ~(0x0<<0);
	GPIOC->ODR |= (0x1<<0);

	GPIOC->ODR |= (1<<6);
	GPIOC->ODR |= (1<<7);

	I2C2->TIMINGR |= ((0x01<<28)| (0x04<<20) | (0x02<<16) | (0xF<<8) | (0x13<<0)); //Set the parameters in the TIMINGR register to use 100 kHz standard-mode I2C
	I2C2->CR1 = I2C_CR1_PE; //Enabling I2C2 Peripheral

	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));

	I2C2->CR2 |= (1 << 16) | (0x69 << 1);

	I2C2->CR2 &= ~(0x1<<10);

	I2C2->CR2 |= (0x1<<13);

	while (!(I2C2->ISR & I2C_ISR_TXIS))
	;
	if (I2C2->ISR & I2C_ISR_NACKF)
		GPIOC->ODR ^= (1 << 6);
	I2C2->TXDR |= 0x0F;
	while (!(I2C2->ISR & I2C_ISR_TC))
	;
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	I2C2->CR2 |= (1 << 16) | (0x69 << 1);

	I2C2->CR2 |=(0x1<<10);
	I2C2->CR2 |= (0x1<<13);

	while (!(I2C2->ISR & I2C_ISR_RXNE))
	;
	if (I2C2->ISR & I2C_ISR_NACKF)
		GPIOC->ODR ^= (1 << 6);
	while (!(I2C2->ISR & I2C_ISR_TC))
	;
	if (I2C2->RXDR == 0xD3)
		HAL_Delay(200);
		GPIOC->ODR ^= (1 << 7);

	I2C2->CR2 |= I2C_CR2_STOP;
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

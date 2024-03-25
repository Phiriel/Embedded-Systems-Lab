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
  SystemClock_Config();
	
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
		
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
	
	GPIO_InitTypeDef X = {GPIO_PIN_8 | GPIO_PIN_9|GPIO_PIN_6|GPIO_PIN_7, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW,GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &X);
	
	//Set PB11 to alternate function mode, open-drain output type, and select I2C2_SDA as its alternate function
	GPIOB->MODER |= (0x1<<22);
	GPIOB->MODER &= ~(0x1<<23);
	GPIOB->OTYPER |= (0x1<<11);
	GPIOB->AFR[1] &= ~((1<<12) | (1<<13) | (1<<14));
	GPIOB->AFR[1] |= (1<<15);
	
	//Set PB13 to alternate function mode, open-drain output type, and select I2C2_SCL as its alternate function
	GPIOB->MODER |= (0x1<<26);
	GPIOB->MODER &= ~(0x1<<27);
	GPIOB->OTYPER |= (0x1<<13);
	GPIOB->AFR[1] &= ~((1<<20) | (1<<22));
	GPIOB->AFR[1] |= (1<<23) | (1<<21);
	
	//Set PB14 to output mode, push-pull output type, and initialize/set the pin high
	GPIOB->MODER |= (0x1<<29);
	GPIOB->MODER &= ~(0x1<<28);
	GPIOB->OTYPER &= ~(0x0<<14);
	GPIOB->ODR |= (0x1<<14);
	
	//Set the parameters in the TIMINGR register to use 100 kHz standard-mode I2C
  I2C2->TIMINGR |= (0x1  << I2C_TIMINGR_PRESC_Pos);  
  I2C2->TIMINGR |= (0x13 << I2C_TIMINGR_SCLL_Pos);  
  I2C2->TIMINGR |= (0x0F << I2C_TIMINGR_SCLH_Pos);  
  I2C2->TIMINGR |= (0x2  << I2C_TIMINGR_SDADEL_Pos);
  I2C2->TIMINGR |= (0x4  << I2C_TIMINGR_SCLDEL_Pos);

	I2C2->CR1 = I2C_CR1_PE; //Enabling I2C2 Peripheral
		
	testI2C();

  while(1){
    gyro();
  }
	
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
}

//part1
void testI2C(void){
  // Clear the NBYTES and SADD bit fields
  I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));

  I2C2->CR2 |= (slvAddr << I2C_CR2_SADD_Pos);   // Set the L3GD20 slave address = slvAddr
  I2C2->CR2 |= (noOfBits  << I2C_CR2_NBYTES_Pos); // Set the number of bytes to transmit = noOfBits
  I2C2->CR2 |= (I2C_CR2_RD_WRN_Msk);         // Set the RD_WRN to read operation
  I2C2->CR2 |= (I2C_CR2_START_Msk);

  if((I2C2->ISR & I2C_ISR_TXIS) == I2C_ISR_TXIS){
		GPIOC->ODR ^= (1<<7);
		I2C1->TXDR = 0x0F;
		
		if((I2C2->ISR & I2C_ISR_TC) == I2C_ISR_TC){
			I2C2->CR2 |= (0x1<<13);
			
			I2C2->CR2 |= (0x1<<10);
			if((I2C2->ISR & I2C_ISR_RXNE) == I2C_ISR_RXNE){
				if((I2C2->ISR & I2C_ISR_TC) == I2C_ISR_TC){
					if(I2C1->RXDR == 0xD3){
						I2C2->CR2 |= (0x1<<13);
						GPIOC->ODR ^= (1<<6);
					}
				}
			}
		}
	}
}

void initGyro(void){
  I2C2->CR2 |= (0xD2 << I2C_CR2_SADD_Pos);  
  I2C2->CR2 |= (0x2  << I2C_CR2_NBYTES_Pos);
  I2C2->CR2 &= ~(I2C_CR2_RD_WRN_Msk);
  I2C2->CR2 |= (I2C_CR2_START_Msk);
  
  // Wait until TXIS or NACKF flags are set (1)
  while(1) {
    if (I2C2->ISR & I2C_ISR_TXIS) {
      I2C2->TXDR = 0x20;
      break;
    }

    if (I2C2->ISR & I2C_ISR_NACKF) {
    }
  }

// Wait again until TXIS or NACKF flags are set (2)
  while(1) {
    if (I2C2->ISR & I2C_ISR_TXIS) {
      I2C2->TXDR = 0x0B;
      break;
    }
  }

  // Wait for TC flag is set
  while(1) {
    if (I2C2->ISR & I2C_ISR_TC) {
      break;
    }
  }
}

uint8_t x1;
uint8_t x2;
int16_t x;
int16_t x_dir = 0;

uint8_t y1;
uint8_t y2;
int16_t y;
int16_t y_dir = 0;

void gyro(void) {
      I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));

      I2C2->CR2 |= (0xD2 << I2C_CR2_SADD_Pos);   // Set the L3GD20 slave address = slvAddr
      I2C2->CR2 |= (0x1  << I2C_CR2_NBYTES_Pos); // Set the number of bytes to transmit = noOfBits
      I2C2->CR2 |= (I2C_CR2_RD_WRN_Msk);         // Set the RD_WRN to read operation
      I2C2->CR2 |= (I2C_CR2_START_Msk);

      while(1) {
            // Continue if TXIS flag is set
        if ((I2C2->ISR & I2C_ISR_TXIS)) {
          I2C2->TXDR = 0x28; // Set I2C2->TXDR = txdrData
          break;
        }
      }

      // Wait for TC flag is set
      while(1) {
        if (I2C2->ISR & I2C_ISR_TC) {
          break;
        }
      }
      I2C2->RXDR = 0xD2;

      while(1) {
            // Continue if TXIS flag is set
        if ((I2C2->ISR & I2C_ISR_TXIS)) {
          I2C2->TXDR = 0x29; // Set I2C2->TXDR = txdrData
          break;
        }
      }

      // Wait for TC flag is set
      while(1) {
        if (I2C2->ISR & I2C_ISR_TC) {
          break;
        }
      }
      I2C2->RXDR = 0xD2;
      
      while(1) {
            // Continue if TXIS flag is set
        if ((I2C2->ISR & I2C_ISR_TXIS)) {
          I2C2->TXDR = 0x2A; // Set I2C2->TXDR = txdrData
          break;
        }
      }

      // Wait for TC flag is set
      while(1) {
        if (I2C2->ISR & I2C_ISR_TC) {
          break;
        }
      }
      I2C2->RXDR = 0xD2;
      
      while(1) {
            // Continue if TXIS flag is set
        if ((I2C2->ISR & I2C_ISR_TXIS)) {
          I2C2->TXDR = 0x2B; // Set I2C2->TXDR = txdrData
          break;
        }
      }

      // Wait for TC flag is set
      while(1) {
        if (I2C2->ISR & I2C_ISR_TC) {
          break;
        }
      }
      I2C2->RXDR = 0xD2;
      
            
      x = (x2 << 8) | (x1 << 0);
      x_dir += x;
      
      y = (y2 << 8) | (y1 << 0);
      y_dir += y;
      
      /***********************************************************************/
      
      if (x_dir < -20) {
            GPIOC->ODR |= (1<<8);
            GPIOC->ODR &= ~(1<<9);
      } else if (x_dir > 20){
            GPIOC->ODR |= (1<<9);
            GPIOC->ODR &= ~(1<<8);
      } else{
            GPIOC->ODR |= (1<<8);
            GPIOC->ODR |= (1<<9);
      }
      
      if (y_dir < -20) {
            GPIOC->ODR |= (1<<6);
            GPIOC->ODR &= ~(1<<7);
      } else if((y_dir > 20)){
            GPIOC->ODR |= (1<<7);
            GPIOC->ODR &= ~(1<<6);
      }else{
            GPIOC->ODR |= (1<<6);
            GPIOC->ODR |= (1<<7);
      }
      
      HAL_Delay(100);
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


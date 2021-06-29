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
#include "gpio.h"

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
int flag1 = 0, flag2 = 0;
uint16_t LED[8] = {
		0x0001, 0x0002, 0x0004, 0x0008,
		0x0010, 0x0020, 0x0040, 0x0080
}; // Sw1 쉬프트용

uint16_t LED2[8] = {
		0x0080, 0x0040, 0x0020, 0x0010,
		0x0008, 0x0004, 0x0002, 0x0001
}; // Sw2 쉬프트용

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 if(flag1 == 1) //Sw3 push
	 {
	  	 for(int i = 0; i < 8; i++)
    	 {
  		 uint16_t LEDs = LED[i];
		 HAL_GPIO_WritePin(GPIOC, LEDs, 1);
		 HAL_Delay(1000);
		 HAL_GPIO_WritePin(GPIOC, LEDs, 0);
    	 }
	 }
	 else if(flag2 == 1) //Sw4 push
	 {
		for(int i = 0; i < 8; i++)
		{
			uint16_t LEDs = LED2[i];
		    HAL_GPIO_WritePin(GPIOC, LEDs, 1);
		    HAL_Delay(1000);
		    HAL_GPIO_WritePin(GPIOC, LEDs, 0);
		 }
	 }
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* EXTI4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_8)    //SW1
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All, 1);
	}
	else if(GPIO_Pin == GPIO_PIN_4) //SW2
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All, 0);
	}
	else if(GPIO_Pin == GPIO_PIN_5) //SW3
	{
		flag1 = 1;
	}
	else if(GPIO_Pin == GPIO_PIN_10) //SW4
	{
		flag2 = 1;
	}
}

void LED_Shift()
{
  	  uint16_t LED = 0x0001; // LED1 번 값

  	 do{
		 HAL_GPIO_WritePin(GPIOC, LED, 1);
		 HAL_Delay(1000);

		 LED = LED << 1;
		 //LED = LED & 0xFE;

	 }while(LED != 0x0100); // LED가 8 이상 될때까지 do문 반복

	 do{
 		 HAL_GPIO_WritePin(GPIOC, LED, 0);
 		 HAL_Delay(1000);

 		 LED = LED >> 1;
 		 //LED = LED & 0xFE;

	 	 }while(LED != 0x0000); // LED가 0 이하 될 때 까지 do문 반복
}

void LED_Shift_Macro()
{
	      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1);
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1);
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1);
	      HAL_Delay(1000);
	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0);
	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1);
	      HAL_Delay(1000);
	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);
	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
	      HAL_Delay(1000);
	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
	      HAL_Delay(1000);
	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
	      HAL_Delay(1000);
	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
	  	  HAL_Delay(1000);

}

void LED_SW1()
{
 	 for(int i = 0; i < 8; i++)
 	 {
 		 uint16_t LEDs = LED[i];
		 HAL_GPIO_WritePin(GPIOC, LEDs, 1);
		 HAL_Delay(1000);
	 }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
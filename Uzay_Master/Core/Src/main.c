/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_SLAVE1_ADD	0x50
#define I2C_SLAVE2_ADD	0x51
#define I2C_SLAVE3_ADD	0x52
#define I2C_SLAVE4_ADD	0x53
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t buffer[64];
uint8_t rx_data[10];
uint8_t counter = 0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
//void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef * hi2c)
//{
//  counter++;
//}
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
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	uint8_t slave1[10] = {0};
	uint8_t slave2[10] = {0};
	uint8_t slave3[10] = {0};
	uint8_t slave4[10] = {0};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		CDC_Transmit_FS(buffer, 64);
//		HAL_Delay(5);
		//	  if(rx_data[0] == 1){
		//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, SET);
		//	  }
		//
		//	  else{
		//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, RESET);
		//	  }
		//	  if(rx_data[1] == 2){
		//	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, SET);
		//	  	  }
		//
		//	  	  else{
		//	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, RESET);
		//	  	  }
		//	  if(rx_data[2] == 3){
		//	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET);
		//	  	  }
		//
		//	  	  else{
		//	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, RESET);
		//	  	  }
		//	  if(rx_data[3] == 4){
		//	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, SET);
		//	  	  }
		//
		//	  	  else{
		//	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, RESET);
		//	  	  }
		//	  if(rx_data[4] == 5){
		//	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, SET);
		//	  	  }
		//
		//	  	  else{
		//	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, RESET);
		//	  	  }
		//	  if(rx_data[5] == 6){
		//	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, SET);
		//	  	  }
		//
		//	  	  else{
		//	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, RESET);
		//	  	  }
		//	  if(rx_data[6] == 7){
		//	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, SET);
		//	  	  }
		//
		//	  	  else{
		//	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, RESET);
		//	  	  }
		//	  if(rx_data[7] == 8){
		//	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, SET);
		//	  	  }
		//
		//
		//	  	  else{
		//	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, RESET);
		//	  	  }
		//	  if(rx_data[8] == 9){
		//	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, SET);
		//	  	  }
		//
		//	  	  else{
		//	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, RESET);
		//	  	  }
		//	  if(rx_data[9] == 10){
		//	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, SET);
		//	  	  }
		//
		//	  	  else{
		//	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, RESET);
		//	  	  }

		//	  HAL_I2C_Master_Transmit_IT(&hi2c1, I2C_SLAVE1_ADD << 1, rx_data, sizeof(rx_data));
		//	  HAL_I2C_Master_Transmit_IT(&hi2c1, I2C_SLAVE2_ADD << 1, rx_data, sizeof(rx_data));
		//	  HAL_I2C_Master_Transmit_IT(&hi2c1, I2C_SLAVE3_ADD << 1, rx_data, sizeof(rx_data));
		//	  HAL_I2C_Master_Transmit_IT(&hi2c1, I2C_SLAVE4_ADD << 1, rx_data, sizeof(rx_data));

		for(uint8_t i = 0; i < 10; i++)
		{


			if((rx_data[i]>0) && rx_data[i]<=28)
			{
				slave1[i] = rx_data[i];
			}
			else	slave1[i] = 0;


			if((rx_data[i]>28) && rx_data[i]<=56)
			{
				slave2[i] = rx_data[i] - 28;
			}
			else	slave2[i] = 0;


			if((rx_data[i]>56) && rx_data[i]<=84)
			{
				slave3[i] = rx_data[i] - 56;
			}
			else	slave3[i] = 0;


			if((rx_data[i]>84) && rx_data[i]<=112)
			{
				slave4[i] = rx_data[i] - 84;
			}
			else	slave4[i] = 0;


		}
		HAL_I2C_Master_Transmit_IT(&hi2c1, I2C_SLAVE1_ADD << 1, slave1, sizeof(slave1));
		HAL_Delay(5);
		HAL_I2C_Master_Transmit_IT(&hi2c1, I2C_SLAVE2_ADD << 1, slave2, sizeof(slave2));
		HAL_Delay(5);
		HAL_I2C_Master_Transmit_IT(&hi2c1, I2C_SLAVE3_ADD << 1, slave3, sizeof(slave3));
		HAL_Delay(5);
		HAL_I2C_Master_Transmit_IT(&hi2c1, I2C_SLAVE4_ADD << 1, slave4, sizeof(slave4));
		HAL_Delay(5);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15
                           PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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

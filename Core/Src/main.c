/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t led_count = 0x0000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t GPIO_ReadPin_Filter(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	static uint8_t pin_value = 1;
	static uint8_t pin_buffer = 0xff;

	pin_buffer = pin_buffer << 1;
	if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET)
	{
		pin_buffer ++;
	}

	if((pin_value == 1) && (pin_buffer == 0x00))
	{
		pin_value = 0;
	}
	else if((pin_value == 0) && (pin_buffer == 0xff)){
		pin_value = 1;
	}

	return pin_value;
}


HAL_I2C_StateTypeDef write_reg(uint8_t sys_adr, uint8_t reg_adr, uint8_t size, uint8_t* pData)
{
	uint8_t data_buff[size+1];
	for(int i = 1; i<=size; i++)
	{
		data_buff[i] = *pData;
		pData++;
	}
	data_buff[0] = reg_adr;

	HAL_I2C_StateTypeDef tx_status = HAL_I2C_Master_Transmit(&hi2c3, (sys_adr<<1), data_buff, size+1, 1000);
	return tx_status;
}


HAL_I2C_StateTypeDef read_reg(uint8_t sys_adr, uint8_t reg_adr, uint8_t size, uint8_t* pData)
{
	if(HAL_I2C_Master_Transmit(&hi2c3, (sys_adr<<1), &reg_adr, 1, 1000) != HAL_OK)
	{
		return HAL_ERROR;
	}

	return HAL_I2C_Master_Receive(&hi2c3, (sys_adr<<1), pData, size, 1000);
}



uint8_t enable_system(uint8_t sys_adr, int timeout_calls)
{
	uint8_t tx_data[4];
	uint8_t rx_data[4];
	uint8_t count = 0;
	led_count = 0xFFFF;

	//Check that the PowManager is active
	while(read_reg(sys_adr, 1, 1, rx_data) != HAL_OK)
	{
		if(count == timeout_calls) return 0;
		count ++;
		HAL_Delay(500);
	}

	//If PowManager running enable power mode (Mode1)
	tx_data[0] = 0x04;
	write_reg(sys_adr, 1, 1, tx_data);

	//Pol for valid voltage after the system is turned on
	count = 0;
	do
	{
		if(count == 20) return 0;
		HAL_Delay(500);
		read_reg(sys_adr, 1, 1, rx_data);
		count ++;
	}while((rx_data[0] & 0x02) == 0);

	//Enable the voltage mode afterwards (Mode1 - ENB)
	tx_data[0] = 0x05;
	write_reg(sys_adr, 1, 1, tx_data);

	return 1;
}


uint8_t disable_system(uint8_t sys_adr)
{
	uint8_t tx_data = 0;
	if(write_reg(sys_adr,1,1,&tx_data) == HAL_OK)
	{
		led_count = 0x00FF;
		return 1;
	}
	else return 0;
}


uint8_t discharge_system(uint8_t sys_adr)
{
	uint8_t tx_data = 0x08;
	if(write_reg(sys_adr,1,1,&tx_data) == HAL_OK)
	{
		led_count = 0xCCCC;
		return 1;
	}
	else return 0;
}


float read_voltage(uint8_t sys_adr)
{
	uint8_t rx_data[2];

	read_reg(sys_adr,3,2,rx_data);

	return (float) ((rx_data[1] << 8) + rx_data[0]) * 3.3/4095 * 10560/560;
}

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
  uint8_t loop_valid = 0;
  int dis_voltage_val = 0;
  uint8_t switch_state = 1;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Read Main-Switch State
	  switch_state = GPIO_ReadPin_Filter(GPIOA, GPIO_PIN_4);

	  //Main Switch state control
	  if((loop_valid == 0) && (switch_state == 0))
	  	{
	  		enable_system(0x03, 10);
	  		loop_valid = 1;
	  	}
	  	else if((loop_valid == 1) && (switch_state == 1))
	  	{
	  		disable_system(0x03);
	  		loop_valid = 2;
	  	}
	  	else if((loop_valid == 2) && (switch_state == 0))
	  	{
	  		discharge_system(0x03);
	  		loop_valid = 3;
	  	}
	  	else if((loop_valid == 3) && ((switch_state == 1) || (dis_voltage_val == 1)))
	  	{
	  		disable_system(0x03);
	  		loop_valid = 4;
	  		dis_voltage_val = 0;
	  	}
	  	else if((loop_valid == 4 ) && (switch_state == 1))
	  	{
	  		loop_valid = 0;
	  		led_count = 0x0000;
	  	}


	  //Led blinking system
	  	if((led_count & 0x01) == 0)
	  	{
	  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	  		led_count = led_count >> 1;
	  	}
	  	else
	  	{
	  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
	  		led_count = 0x8000 | (led_count >> 1);
	  	}


	  	//Voltage read
	  	if(loop_valid == 3)
	  	{
	  		if(read_voltage(0x03) < 3.00)
	  		{
	  			dis_voltage_val = 1;
	  		}
	  	}

	  	HAL_Delay(200);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

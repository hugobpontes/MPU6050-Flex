
#include <Mpu6050Flex.h>
#include "main.h"

#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>


I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart4;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_UART4_Init(void);

/*Function to facilitate debugging through uart2, adapt to your needs */
int stm32_printf(const char *format, ...)
{
    char str[200];
    va_list args;
    va_start(args, format);
    int n = vsprintf(str, format, args);
    va_end(args);
    HAL_UART_Transmit(&huart4, (uint8_t *)str, n, HAL_MAX_DELAY);
    return n;
}

static IOStatus_t Stm32f767zi_I2C_Write(uint8_t Address,uint32_t Size,uint8_t* DataPtr)
{

	IOStatus_t Status = IO_SUCCESS;
	HAL_StatusTypeDef HAL_Status;

	uint8_t* WritePointer = (uint8_t*) malloc(Size+1);
	memcpy(WritePointer,&Address,1);
	memcpy(WritePointer+1,DataPtr,Size);

	HAL_Status = HAL_I2C_Master_Transmit(&hi2c2, MPU6050_I2C_ADDRESS<<1, WritePointer, Size+1, 100);

	free(WritePointer);

	if (HAL_Status != HAL_OK)
	{
		Status = IO_FAILURE;
	}
	return Status;
}

static void Stm32f767zi_Delay(uint32_t ms)
{
	HAL_Delay(ms);
}

static uint32_t Stm32f767zi_GetMs()
{
	return HAL_GetTick();
}

static IOStatus_t Stm32f767zi_I2C_Read(uint8_t Address,uint32_t Size,uint8_t* DataPtr)
{
	IOStatus_t Status = IO_SUCCESS;
	HAL_StatusTypeDef HAL_Status;

	HAL_Status = HAL_I2C_Master_Transmit(&hi2c2, MPU6050_I2C_ADDRESS<<1, &Address, 1, 100);
	HAL_Delay(1);
	if (HAL_Status == HAL_OK)
	{
		HAL_Status = HAL_I2C_Master_Receive(&hi2c2, MPU6050_I2C_ADDRESS<<1, DataPtr, Size, 100);
		if (HAL_Status != HAL_OK)
		{
			Status = IO_FAILURE;
		}
	}
	return Status;
}

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_I2C2_Init();
	MX_UART4_Init();

	Mpu6050Flex_t Mpu6050;

	Mpu6050 = Mpu6050Flex_Create();

	stm32_printf("----------Welcome to Mpu6050 Flex----------\n");

	Mpu6050Flex_SetIOWrite(Mpu6050,Stm32f767zi_I2C_Write);
	Mpu6050Flex_SetIORead(Mpu6050,Stm32f767zi_I2C_Read);
	Mpu6050Flex_SetGetMs(Mpu6050,Stm32f767zi_GetMs);
	Mpu6050Flex_SetDelay(Mpu6050,Stm32f767zi_Delay);

	uint8_t ID;
	ID = Mpu6050Flex_WhoAmI(Mpu6050);

	if (ID == MPU6050_I2C_ADDRESS)
	{
		stm32_printf("MPU6050 Found!\n");
	}
	else
	{
		stm32_printf("ERROR! MPU6050 Not Found!\n");
	}

	stm32_printf("Calibrating...\n");
	Mpu6050Flex_Calibrate(Mpu6050);
	stm32_printf("Waking up...\n");
	Mpu6050Flex_WakeUp(Mpu6050);
	stm32_printf("Ready!!!\n");

	while (1)
	{

		Mpu6050Flex_EulerAngles_t RetEuler;
		RetEuler = Mpu6050Flex_GetEuler(Mpu6050);

		char Msg[50];
		sprintf(Msg,"Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n",RetEuler.Roll,RetEuler.Pitch,RetEuler.Yaw);
		HAL_UART_Transmit(&huart4, (uint8_t *)Msg, 38, HAL_MAX_DELAY);

		HAL_Delay(100);
	}

}


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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00303D5B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
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

}
#endif /* USE_FULL_ASSERT */

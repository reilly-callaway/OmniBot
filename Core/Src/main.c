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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "crsf.h"
#include <math.h>
#include "motors.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEADZONE 0.05
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t uart_data[CRSF_MAX_PACKET_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// TODO: Split most of this out into a control loop (task?), instead of when packet is received
void CRSF_OnPacketChannels(uint16_t* channels)
{
	char buf[21];
	uint8_t len = sprintf(buf, "%d %d %d %d\n", channels[0], channels[1], channels[2], channels[3]);
	CDC_Transmit_FS((uint8_t *)buf, len);

	float throttle = (channels[0] - 1000.0f) / 1000.0f;

	if (throttle < DEADZONE)
	{
		throttle = 0.0f;
	}

	// throttle, roll, pitch, yaw
	float sticks[4] = {};
	sticks[0] = throttle;

	for (uint8_t i = 1; i < 4; ++i)
	{
		sticks[i] = (channels[i] - 1500.0f) / 500.0f;
		if (sticks[i] < DEADZONE && sticks[i] > -DEADZONE)
		{
			sticks[i] = 0.0f;
		}
	}

	// Rotation is also scaled by throttle
	float rotation = sticks[3] * throttle;
	// "air mode" aka you can spin at zero throttle
	if (throttle < 0.3f)
	{
		rotation = sticks[3] * 0.3f;
	}

	// Right stick controls direction
	float angle = atan2f(sticks[2], sticks[1]);
	float mag = sqrtf(sticks[2]*sticks[2] + sticks[1]*sticks[1]);
	if (mag > 1.0f)
	{
		mag = 1.0f;
	}

	// If right stick has barely moved, don't translate
#define MAG_THROTTLE_ZONE 0.4f
	if (mag < 0.1)
	{
		throttle = 0.0f;
	}
	// Pushing hard on the right stick, increase the throttle
	else if (mag > MAG_THROTTLE_ZONE)
	{
		// Mag starts to contribute to (half) the "remaining" throttle
		throttle += 0.5f * (1-throttle) * ((mag - MAG_THROTTLE_ZONE) / (1 - MAG_THROTTLE_ZONE));
	}

	Motors_SetMovement(throttle, angle, rotation);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// TODO: Capture these in the original interrupt handler instead of using the (slow) callback
void DMA_USART_TransferCompleteCallback(struct __DMA_HandleTypeDef * hdma)
{
	HAL_UARTEx_RxEventCallback(NULL, CRSF_MAX_PACKET_SIZE);
}

// UART Idle callback
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	// Setup to capture the next frame
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_data, 64);

	// We've hit the end of a frame
	// All the data is stored in the DMA buffer
	// Just to be safe, lets copy it over into a temp buffer so we can load data into the uart_data buffer simultaneously
	uint8_t rxBuf[64];
	memcpy(rxBuf, uart_data, Size);

	CRSF_BytesReceived(rxBuf, Size);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

	CRSF_Init();

	// Ensure motors off to start
	Motors_SetDuty(0.0f, 0.0f, 0.0f);

	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);

	// Set the DMA transfer complete callback
	hdma_usart1_rx.XferCpltCallback = &DMA_USART_TransferCompleteCallback;
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_data, 64);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		uint8_t state = HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin);
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, state);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

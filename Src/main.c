/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>

#include "cs5451.h"
#include "usbd_cdc_if.h"

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
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

volatile int16_t adc_data[12];
volatile uint32_t ready = 0;
volatile uint32_t interrupt_count = 0;
volatile uint8_t usart_buf[16];
uint8_t out_mode = 2; // 0 - bin, 1 - text, 2 - rms
uint8_t out_on = 1;
int64_t rms_sum[6];
volatile uint8_t adc_se = 1;
volatile uint8_t adc_reset = 1;
volatile uint8_t usb_buf[8];
volatile uint8_t *cur_ptr = usb_buf;

int __errno;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* TODO:
 * попробовать сделать дополнительную подтяжку вниз для линии SCK
 * попробовать переинициализировать SPI по сигналу FSO от CS5451
 */

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
	return;
	RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;

	HAL_GPIO_TogglePin(Led_GPIO_Port, LED1_Pin);
	spi1_init();

	HAL_GPIO_TogglePin(Led_GPIO_Port, LED1_Pin);
//HAL_SPI_DMAResume(&hspi1);
}

void HAL_SPI_RxCpltCallback( SPI_HandleTypeDef *hspi)
{
	ready = 1;
	interrupt_count++;
	//HAL_SPI_DMAPause(&hspi1);
}

void parse_cmd(const uint8_t *cmd)
{
	uint8_t cmd_arg = cmd[1] - '0';;
	GPIOB->ODR ^= 0x2000;
		if( cmd[0] == 'a')
		{
				// reset control
				if( cmd_arg == 0 )
				{
					adc_reset = 1;
				}
				else
				{
					cs5451_reset(cmd[1] - '0');
					adc_reset=0;
					interrupt_count = 0;
				}
		}
		else if( cmd[0] == 'b' )
		{
				// SE control
				if( cmd_arg == 1)
				{
					cs5451_se(1);
					adc_se=1;
				}
				else
					adc_se = 0;
		}
		else if( cmd[0] == 'c' )
		{
				// OWRS control
				cs5451_owrs(cmd[1] - '0');
		}
		else if( cmd[0] == 'd' )
		{
				// gain control
				cs5451_gain(cmd[1] - '0');
		}
		else if( cmd[0] == 'e' )
		{
				// bin/text control
				out_mode = cmd[1] - '0';
		}
		else if( cmd[0] == 'f' )
		{
				// data out on/off
				out_on = cmd[1] - '0';
		}
	HAL_GPIO_TogglePin(Led_GPIO_Port, LED2_Pin);
}

void HAL_UART_RxCpltCallback (UART_HandleTypeDef *husart)
{
	parse_cmd(usart_buf);
		HAL_UART_Receive_IT(&huart1, (uint8_t*)usart_buf, 2);
}

void print_s(const char *s)
{
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)(s), strlen(s));
}

void spi1_init(void)
{
	SPI1->CR1 = SPI_CR1_DFF | SPI_CR1_RXONLY | SPI_CR1_SSM | SPI_CR1_BR;
	SPI1->CR2 = SPI_CR2_ERRIE | SPI_CR2_RXDMAEN;
}

void parse_cmd_buffer( uint8_t *buffer)
{
	parse_cmd(buffer);
	if( buffer[5] == '\n' )
	{
		parse_cmd(buffer+3);
		buffer[5] = 0;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	char buffer[100];
	int16_t adc_buf[8];
	int32_t rms_vals[6];
	int rms_count = 0;
	const int RMS_COUNT1 = 39;
	uint32_t count=0;
	int len = 0;;
	int i;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  HAL_SPI_MspInit(&hspi1);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim1);


	print_s("wait 4s\r\n");
	HAL_Delay(4000);


	cs5451_gain(1); // unity gain
	cs5451_owrs(1);
	cs5451_reset(1);
	HAL_Delay(100);
	HAL_UART_Receive_IT(&huart1, (uint8_t*)usart_buf, 2);
	HAL_SPI_Receive_DMA(&hspi1, (uint8_t*)adc_data, 12);
	cs5451_se(adc_se);
	print_s("started\r\n");
	CDC_Transmit_FS((uint8_t *)"u\r\n", 3);
	HAL_GPIO_TogglePin(Led_GPIO_Port, LED2_Pin);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if( __HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) )
		{
			__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
			sprintf(buffer, "%d\r\n", (uint8_t)(cur_ptr-usb_buf));
			print_s(buffer);
		}

		if( *cur_ptr == '\n' )
		{
			parse_cmd_buffer(usb_buf);
			*cur_ptr = 0;
			print_s(usb_buf);
			sprintf(buffer, "%d\r\n", (uint8_t)(cur_ptr-usb_buf));
			print_s(buffer);

			cur_ptr = usb_buf;
//			HAL_GPIO_TogglePin(Led_GPIO_Port, LED1_Pin);
		}

		if( ready )
		{
			ready = 0;
			if( adc_se == 0 )
			{
				cs5451_se(0);
			}
			if( adc_reset )
			{
				cs5451_reset(0);
			}

			__disable_irq();
			memcpy(adc_buf, (void*)adc_data, sizeof(adc_data));
			__enable_irq();
			count+=1;

			if( out_on )
			{
					switch(out_mode )
					{
					case 1:
					{

							len = sprintf(buffer, "%04hX%04hX%04hX%04hX%04hX%04hX\r\n", adc_buf[0], adc_buf[1], adc_buf[2],
										adc_buf[3], adc_buf[4], adc_buf[5]);
					}
					break;
					case 0:
					{
							buffer[0] = 0;
							buffer[1] = 0;
							memcpy(buffer+2, (void*)adc_buf, sizeof(adc_data));
							buffer[14] = 0;
							buffer[15] = 0;
							len = 16;
					}
					break;
					case 2:
						for(i=0; i<6; ++i)
						{
								rms_sum[i] += adc_buf[i]*adc_buf[i];
						}
						++rms_count;
						if(rms_count == RMS_COUNT1)
						{
								rms_count=0;
								for( i=0; i<6; ++i)
								{
										rms_vals[i] = (uint16_t)(sqrtf((float)(rms_sum[i])/RMS_COUNT1)*0.60999f);
										rms_sum[i] = 0;
								}
							len = sprintf(buffer, "%ld %6ld %6ld %6ld %6ld %6ld %6ld\r\n", count, rms_vals[0], rms_vals[1], rms_vals[2],
										rms_vals[3], rms_vals[4], rms_vals[5]);
								
						}
						else
						{
								len = 0;
						}

					break;
					case 3:
						*(uint32_t*)(&buffer) = count;
						memcpy(buffer+4, (void*)adc_buf, sizeof(adc_data));
									
						len = 16;
					break;
					}

					if( len)//  && interrupt_count < 40)
					{
					//	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)(buffer), len);
						CDC_Transmit_FS((uint8_t *)buffer, len);
					}
			}

		}


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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 36000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 3000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|Led_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ADC_OWRS_Pin|ADC_Reset_Pin|ADC_Gain_Pin|ADC_SE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin Led_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|Led_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ADC_OWRS_Pin ADC_Reset_Pin ADC_Gain_Pin ADC_SE_Pin */
  GPIO_InitStruct.Pin = ADC_OWRS_Pin|ADC_Reset_Pin|ADC_Gain_Pin|ADC_SE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_FSO_Pin */
  GPIO_InitStruct.Pin = ADC_FSO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC_FSO_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
		while(1) 
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

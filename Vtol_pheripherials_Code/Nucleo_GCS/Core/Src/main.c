/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* WS2812 driver — left here rather than pulled into its own module, since
   the approved restructure was specifically tlv_protocol/sync_handshake/
   buttons_leds/joystick_adc. Say the word if you want this extracted too. */
#define WS2812_BIT0   0x80U
#define WS2812_BIT1   0xFCU
#define WS2812_RESET_BYTES   50U

#define WS2812_NUM_LEDS            6
#define WS2812_ENCODED_BYTES_PER_LED  24U   /* 8 green + 8 red + 8 blue encoded bytes, MSB first */

#define WS2812_COLOR_BYTES   (WS2812_NUM_LEDS * WS2812_ENCODED_BYTES_PER_LED)
#define WS2812_BUFFER_LEN    (WS2812_COLOR_BYTES + WS2812_RESET_BYTES)

static uint8_t s_buffer[WS2812_BUFFER_LEN];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ── WS2812 driver — see the note in USER CODE BEGIN PD above. ──────────── */

static void WS2812_EncodeByte(uint8_t *dst, uint8_t value)
{
    for (uint8_t bit = 0; bit < 8; bit++)
    {
        dst[bit] = (value & 0x80) ? WS2812_BIT1 : WS2812_BIT0;
        value <<= 1;
    }
}

void WS2812_SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= WS2812_NUM_LEDS) return;

    uint8_t *p = &s_buffer[index * WS2812_ENCODED_BYTES_PER_LED];
    WS2812_EncodeByte(p + 0,  g);
    WS2812_EncodeByte(p + 8,  r);
    WS2812_EncodeByte(p + 16, b);
}

static SPI_HandleTypeDef *s_hspi = NULL;

void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint16_t i = 0; i < WS2812_NUM_LEDS; i++)
    {
        WS2812_SetPixel(i, r, g, b);
    }
}

void WS2812_Clear(void)
{
    WS2812_SetAll(0, 0, 0);
}

void WS2812_Init(SPI_HandleTypeDef *hspi)
{
    s_hspi = hspi;
    memset(s_buffer, 0x00, sizeof(s_buffer));  /* reset-padding region wants raw 0x00 — correct as-is */
    WS2812_Clear();                            /* color region gets properly re-encoded next line */
}

static volatile uint8_t s_busy = 0;

uint8_t WS2812_Show(void)
{
    if (s_busy || s_hspi == NULL) return 0;

    s_busy = 1;
    if (HAL_SPI_Transmit_DMA(s_hspi, s_buffer, WS2812_BUFFER_LEN) != HAL_OK)
    {
        s_busy = 0;
        return 0;
    }
    return 1;
}

uint8_t WS2812_IsBusy(void)
{
    return s_busy;
}

/* HAL calls this automatically the instant the DMA transfer finishes —
 * must exist somewhere in the project exactly once. */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == s_hspi->Instance)
    {
        s_busy = 0;
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

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_ADC3_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  Send_Hello();
  /* Belt-and-suspenders: HAL_UART_Transmit (used by TLV_Send) is a
     blocking/polling call and never needed USART2_IRQn enabled, so its
     working the whole time proved nothing about NVIC RX interrupt
     config. Enabling it explicitly here means RX doesn't silently
     depend on HAL_UART_MspInit() having done it — safe to call even if
     it's already enabled elsewhere. */
  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  TLV_ArmReceive();   /* starts the interrupt-driven RX chain — see tlv_protocol.c */

  WS2812_Init(&hspi1);
  WS2812_SetPixel(3,10,10,10);
  WS2812_Show();

  uint16_t pot1 = 0, pot2 = 0, pot3 = 0, pot4 = 0;

  static uint32_t last_usb_send = 0;
  static uint16_t avg1 = 0;
  static uint16_t avg2 = 0;
  static uint16_t avg3 = 0;
  static uint16_t avg4 = 0;
  static uint8_t counter = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Process_Incoming_Status();   /* adopt any STATUS frame the PC sent, every iteration, in either state */

    if (awaiting_sync == 0) {
        Hello_mode_off();
        uint32_t now = HAL_GetTick();
        Buttons_Poll(now);   /* debounce, menu handling, PAYLOAD_MESSAGE bits, button/menu LEDs — all in one call now */

        //pot1 += ADC_Read_Channel(ADC_CHANNEL_9);
        //pot2 += ADC_Read_Channel(ADC_CHANNEL_15);
        pot3 += ADC_Read_Channel(ADC_CHANNEL_6);
        pot4 += ADC_Read_Channel(ADC_CHANNEL_7);

        counter++;

        if (counter >= 16)
        {
            avg1 = pot1 / 16;
            avg2 = pot2 / 16;
            avg3 = pot3 / 16;
            avg4 = pot4 / 16;

            counter = 0;
            pot1 = 0;
            pot2 = 0;
            pot3 = 0;
            pot4 = 0;
        }
        if ((HAL_GetTick() - last_usb_send) >= 10)
        {
            uint8_t btn_payload[4] = {
                (uint8_t)(USB_MESSAGE & 0xFF), (uint8_t)((USB_MESSAGE >> 8) & 0xFF),
                (uint8_t)((USB_MESSAGE >> 16) & 0xFF), (uint8_t)((USB_MESSAGE >> 24) & 0xFF),
            };
            TLV_Send(TLV_TYPE_BUTTON_STATE, btn_payload, sizeof(btn_payload));

            uint8_t joy_payload[4] = {
                (uint8_t)(avg1 & 0xFF), (uint8_t)((avg1 >> 8) & 0xFF),
                (uint8_t)(avg2 & 0xFF), (uint8_t)((avg2 >> 8) & 0xFF),
            };
            TLV_Send(TLV_TYPE_JOYSTICK, joy_payload, sizeof(joy_payload));

            uint8_t joy2_payload[4] = {
                (uint8_t)(avg3 & 0xFF), (uint8_t)((avg3 >> 8) & 0xFF),
                (uint8_t)(avg4 & 0xFF), (uint8_t)((avg4 >> 8) & 0xFF),
            };
            TLV_Send(TLV_TYPE_JOYSTICK2, joy2_payload, sizeof(joy2_payload));

            uint8_t payload_cmd_payload[4] = {
                (uint8_t)(PAYLOAD_MESSAGE & 0xFF), (uint8_t)((PAYLOAD_MESSAGE >> 8) & 0xFF),
                (uint8_t)((PAYLOAD_MESSAGE >> 16) & 0xFF), (uint8_t)((PAYLOAD_MESSAGE >> 24) & 0xFF),
            };
            TLV_Send(TLV_TYPE_PAYLOAD_COMMAND, payload_cmd_payload, sizeof(payload_cmd_payload));
            last_usb_send = HAL_GetTick();
        }

    } else {
        Hello_mode_On();

        if ((HAL_GetTick() - last_usb_send) >= 100)
        {
            Send_Hello();

            uint8_t sentinel_payload[4] = {
                (uint8_t)(SYNC_SENTINEL & 0xFF), (uint8_t)((SYNC_SENTINEL >> 8) & 0xFF),
                (uint8_t)((SYNC_SENTINEL >> 16) & 0xFF), (uint8_t)((SYNC_SENTINEL >> 24) & 0xFF),
            };
            TLV_Send(TLV_TYPE_BUTTON_STATE, sentinel_payload, sizeof(sentinel_payload));

            last_usb_send = HAL_GetTick();
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  hdma_spi1_tx.Instance                 = DMA2_Stream3;
  hdma_spi1_tx.Init.Channel             = DMA_CHANNEL_3;
  hdma_spi1_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_spi1_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_spi1_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_spi1_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_spi1_tx.Init.Mode                = DMA_NORMAL;
  hdma_spi1_tx.Init.Priority            = DMA_PRIORITY_LOW;
  hdma_spi1_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_LINKDMA(&hspi1, hdmatx, hdma_spi1_tx);

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PF12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PF13 PF14 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PE9 PE11 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE10 PE12 PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11 PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PG9 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
#ifdef USE_FULL_ASSERT
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

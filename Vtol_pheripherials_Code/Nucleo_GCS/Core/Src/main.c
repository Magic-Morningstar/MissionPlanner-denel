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
#define BIT_ARM                 0
#define BIT_ARM_STATUS          1
#define BIT_AUTO                2
#define BIT_AUTO_STATUS         3
#define BIT_MANUAL              4
#define BIT_MANUAL_STATUS       5
#define BIT_AUTO_LAND           6
#define BIT_AUTO_LAND_STATUS    7
#define BIT_SPEED_UP            8
#define BIT_SPEED_DOWN          9
#define BIT_ZOOM_IN             10
#define BIT_ZOOM_OUT            11
#define BIT_WIDE_IN             12
#define BIT_WIDE_OUT            13
#define DEBOUNCE_MS  15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       active_low;   /* 1 = pressed reads LOW, 0 = pressed reads HIGH */
} button_t;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} led_t;

/* Order must match: buttons[i] drives leds[i] */
static const button_t buttons[10] = {
    { GPIOF, GPIO_PIN_12, 1 },  /* PF12 */
    { GPIOD, GPIO_PIN_14, 1 },  /* PD14 */
    { GPIOD, GPIO_PIN_15, 1 },  /* PD15 */
    { GPIOC, GPIO_PIN_7,  1 },  /* PC7  */
    { GPIOE, GPIO_PIN_10, 0 },  /* PE10 */
    { GPIOE, GPIO_PIN_12, 0 },  /* PE12 */
    { GPIOE, GPIO_PIN_14, 0 },  /* PE14 */
    { GPIOD, GPIO_PIN_11, 0 },  /* PD11 */
    { GPIOD, GPIO_PIN_12, 0 },  /* PD12 */
    { GPIOD, GPIO_PIN_13, 0 },  /* PD13 */
};

static const led_t leds[10] = {
    { GPIOB, GPIO_PIN_0  },
    { GPIOB, GPIO_PIN_7  },
    { GPIOF, GPIO_PIN_13 },
    { GPIOF, GPIO_PIN_14 },
    { GPIOF, GPIO_PIN_15 },
    { GPIOE, GPIO_PIN_9  },
    { GPIOE, GPIO_PIN_11 },
    { GPIOE, GPIO_PIN_13 },
	{ GPIOE, GPIO_PIN_8 },
    { GPIOG, GPIO_PIN_9  },
    { GPIOG, GPIO_PIN_14 },
};

static uint8_t rx_byte;
#define TLV_SYNC 0xAA
#define TLV_END  0x55
#define MASTER_BTN_PORT GPIOA
#define MASTER_BTN_PIN   GPIO_PIN_6
#define TLV_TYPE_BUTTON_STATE 0x01
#define TLV_TYPE_JOYSTICK     0x02
#define TLV_TYPE_JOYSTICK2    0x03
static uint32_t lastArmTime      = 0;
static uint32_t lastManualTime   = 0;
static uint32_t lastAutoTime     = 0;
static uint32_t lastAutoLandTime = 0;
static uint32_t lastSpeedUpTime  = 0;
static uint32_t lastSpeedDownTime= 0;
static uint32_t lastZoomInTime   = 0;
static uint32_t lastZoomOutTime  = 0;
static uint32_t lastWideInTime   = 0;
static uint32_t lastWideOutTime  = 0;
uint32_t USB_MESSAGE = 0x00;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t tlv_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) crc ^= data[i];
    return crc;
}

static uint8_t TLV_Send(uint8_t type, const uint8_t *payload, uint8_t len)
{
    static uint8_t frame[2][64];   // ping-pong buffers — a call can't overwrite bytes still mid-transfer from the previous one
    static uint8_t which = 0;

    if (len > sizeof(frame[0]) - 5) return 0;

    uint8_t *buf = frame[which];
    which ^= 1;

    buf[0] = TLV_SYNC;
    buf[1] = type;
    buf[2] = len;
    memcpy(&buf[3], payload, len);
    buf[3 + len] = tlv_crc8(payload, len);
    buf[4 + len] = TLV_END;

    return (HAL_UART_Transmit(&huart2, buf, len + 5, HAL_MAX_DELAY) == HAL_OK);
}

static uint8_t Button_Is_Pressed(const button_t *b)
{
    GPIO_PinState state = HAL_GPIO_ReadPin(b->port, b->pin);
    if (b->active_low)
        return (state == GPIO_PIN_RESET);
    else
        return (state == GPIO_PIN_SET);
}

/* Call this every loop iteration in place of your normal logic */
void Test_Loop(void)
{
    uint8_t master_pressed =
        (HAL_GPIO_ReadPin(MASTER_BTN_PORT, MASTER_BTN_PIN) == GPIO_PIN_RESET);

    for (int i = 0; i < 10; i++)
    {
        uint8_t on = master_pressed || Button_Is_Pressed(&buttons[i]);
        HAL_GPIO_WritePin(leds[i].port, leds[i].pin,
                           on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    HAL_Delay(10); /* light debounce */
}
uint16_t ADC_Read_Channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;

    HAL_ADC_ConfigChannel(&hadc3, &sConfig);

    HAL_ADC_Start(&hadc3);
	HAL_ADC_PollForConversion(&hadc3, 10);
	(void)HAL_ADC_GetValue(&hadc3);

    HAL_ADC_Start(&hadc3);

    HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY);

    uint16_t value = HAL_ADC_GetValue(&hadc3);

    HAL_ADC_Stop(&hadc3);

    return value;
}


static inline void set_bit_from_pin(GPIO_TypeDef *port, uint16_t pin,
                                     uint8_t active_low, uint32_t bit)
{
    GPIO_PinState state = HAL_GPIO_ReadPin(port, pin);
    uint8_t pressed = active_low ? (state == GPIO_PIN_RESET) : (state == GPIO_PIN_SET);

    if (pressed) USB_MESSAGE |= (1UL << bit);
    else         USB_MESSAGE &= ~(1UL << bit);
}

static inline uint8_t debounce_check(uint32_t *last_time, uint32_t now)
{
    if (now - *last_time < DEBOUNCE_MS) return 0;
    *last_time = now;
    return 1;
}

void onARM_Button_Press(void)
{
    if (USB_MESSAGE & (1 << BIT_ARM)) USB_MESSAGE &= ~(1 << BIT_ARM);
    else                              USB_MESSAGE |= (1 << BIT_ARM);
}

void onManual_Button_Press(void)
{

    if (USB_MESSAGE & (1 << BIT_MANUAL)) USB_MESSAGE &= ~(1 << BIT_MANUAL);
    else                                 USB_MESSAGE |= (1 << BIT_MANUAL);
}

void onAuto_Button_Press(void)
{
    if (USB_MESSAGE & (1 << BIT_AUTO)) USB_MESSAGE &= ~(1 << BIT_AUTO);
    else                                USB_MESSAGE |= (1 << BIT_AUTO);
}

void onAutoLand_Button_Press(void)
{
    if (USB_MESSAGE & (1 << BIT_AUTO_LAND)) USB_MESSAGE &= ~(1 << BIT_AUTO_LAND);
    else                                     USB_MESSAGE |= (1 << BIT_AUTO_LAND);
}

void onSpeedUp_Button_Press(void)
{
    if (USB_MESSAGE & (1 << BIT_SPEED_UP)) USB_MESSAGE &= ~(1 << BIT_SPEED_UP);
    else                                    USB_MESSAGE |= (1 << BIT_SPEED_UP);
}

void onSpeedDown_Button_Press(void)
{
    if (USB_MESSAGE & (1 << BIT_SPEED_DOWN)) USB_MESSAGE &= ~(1 << BIT_SPEED_DOWN);
    else                                      USB_MESSAGE |= (1 << BIT_SPEED_DOWN);
}

void onZoomIn_Button_Press(void)
{
    if (USB_MESSAGE & (1 << BIT_ZOOM_IN)) USB_MESSAGE &= ~(1 << BIT_ZOOM_IN);
    else                                   USB_MESSAGE |= (1 << BIT_ZOOM_IN);
}

void onZoomOUT_Button_Press(void)
{
    if (USB_MESSAGE & (1 << BIT_ZOOM_OUT)) USB_MESSAGE &= ~(1 << BIT_ZOOM_OUT);
    else                                    USB_MESSAGE |= (1 << BIT_ZOOM_OUT);
}


void onWideIn_Button_Press(void)
{
    if (USB_MESSAGE & (1 << BIT_WIDE_IN)) USB_MESSAGE &= ~(1 << BIT_WIDE_IN);
    else                                   USB_MESSAGE |= (1 << BIT_WIDE_IN);
}

void onWideOUT_Button_Press(void)
{
    if (USB_MESSAGE & (1 << BIT_WIDE_OUT)) USB_MESSAGE &= ~(1 << BIT_WIDE_OUT);
    else                                    USB_MESSAGE |= (1 << BIT_WIDE_OUT);
}

/* polarity: 1 = active-low (pull-up, press reads RESET), 0 = active-high (pull-down, press reads SET) */
static void poll_button(GPIO_TypeDef *port, uint16_t pin, uint8_t active_low,
                         GPIO_PinState *lastState, uint32_t *lastTime,
                         uint32_t now, void (*onPress)(void))
{
    GPIO_PinState nowState     = HAL_GPIO_ReadPin(port, pin);
    GPIO_PinState pressedState = active_low ? GPIO_PIN_RESET : GPIO_PIN_SET;
    GPIO_PinState idleState    = active_low ? GPIO_PIN_SET   : GPIO_PIN_RESET;

    if (nowState == pressedState && *lastState == idleState)
    {
        if (debounce_check(lastTime, now)) onPress();
    }
    *lastState = nowState;
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
  MX_ADC3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  uint16_t pot1 = 0, pot2 = 0, pot3 = 0, pot4 = 0;


  static GPIO_PinState armLastState       = GPIO_PIN_SET;    /* PA6  pull-up   */
  static GPIO_PinState manualLastState    = GPIO_PIN_RESET;  /* PF12 pull-down */
  static GPIO_PinState autoLastState      = GPIO_PIN_SET;    /* PD14 pull-up   */
  static GPIO_PinState autoLandLastState  = GPIO_PIN_SET;    /* PD15 pull-up   */
  static GPIO_PinState speedUpLastState   = GPIO_PIN_RESET;  /* PE10 pull-down */
  static GPIO_PinState speedDownLastState = GPIO_PIN_RESET;  /* PE12 pull-down */
  static GPIO_PinState zoomInLastState    = GPIO_PIN_RESET;  /* PE14 pull-down */
  static GPIO_PinState zoomOutLastState   = GPIO_PIN_RESET;  /* PD11 pull-down */
  static GPIO_PinState wideInLastState    = GPIO_PIN_RESET;  /* PD12 pull-down */
  static GPIO_PinState wideOutLastState   = GPIO_PIN_RESET;  /* PD13 pull-down */
  static uint32_t last_usb_send = 0;
  static uint16_t avg1 = 0;
  static uint16_t avg2 = 0;
  static uint16_t avg3 = 0;
  static uint16_t avg4 = 0;
  static uint8_t counter = 0;
  static uint8_t message_counter = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
  	  Test_Loop();
        pot1 += ADC_Read_Channel(ADC_CHANNEL_9);
        pot2 += ADC_Read_Channel(ADC_CHANNEL_15);
        pot3 += ADC_Read_Channel(ADC_CHANNEL_6);
        pot4 += ADC_Read_Channel(ADC_CHANNEL_7);

        uint32_t now = HAL_GetTick();

        /* Persistent MODE buttons: toggle/latch on press, stay set until pressed again. */
        /*                port      pin           active_low  lastState            lastTime            now  handler */
        poll_button(GPIOA, GPIO_PIN_6,  1, &armLastState,       &lastArmTime,       now, onARM_Button_Press);
        poll_button(GPIOC, GPIO_PIN_7, 1, &manualLastState,    &lastManualTime,    now, onManual_Button_Press);
        poll_button(GPIOD, GPIO_PIN_14, 1, &autoLastState,      &lastAutoTime,      now, onAuto_Button_Press);
        poll_button(GPIOD, GPIO_PIN_15, 1, &autoLandLastState,  &lastAutoLandTime,  now, onAutoLand_Button_Press);

        /* MOMENTARY command buttons: bit mirrors the pin, set only while held. */
        /*                  port      pin           active_low  bit */
        set_bit_from_pin(GPIOE, GPIO_PIN_10, 0, BIT_SPEED_UP);
        set_bit_from_pin(GPIOE, GPIO_PIN_12, 0, BIT_SPEED_DOWN);
        set_bit_from_pin(GPIOE, GPIO_PIN_14, 0, BIT_ZOOM_IN);
        set_bit_from_pin(GPIOD, GPIO_PIN_11, 0, BIT_ZOOM_OUT);
        set_bit_from_pin(GPIOD, GPIO_PIN_12, 0, BIT_WIDE_IN);
        set_bit_from_pin(GPIOD, GPIO_PIN_13, 0, BIT_WIDE_OUT);

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

            last_usb_send = HAL_GetTick();
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
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13 |GPIO_PIN_8, GPIO_PIN_RESET);

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
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PF13 PF14 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PE9 PE11 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_8;
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

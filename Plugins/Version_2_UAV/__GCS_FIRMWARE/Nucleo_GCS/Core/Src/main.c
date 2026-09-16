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
#include "fastLed_SPI.h"
#include "transport.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Panel protocol. Two frames, both carrying PURE PHYSICAL STATE. This
   firmware attaches no meaning to any button or LED: it reports which
   slots are pressed and what the pots read, and lights whatever the PC
   tells it to light. Every name — zoom, laser, tracking, menu — exists
   only on the PC, in API/config.py and panel_bindings.py.

   Consequence: rebinding a button or adding a menu is a Python edit and
   a Python restart. This file does not change and does not need to be
   reflashed. There is also nothing to synchronise at connect, which is
   why there is no handshake below. */
#define TLV_SYNC 0xAA
#define TLV_END  0x55

#define TLV_TYPE_PANEL_STATE  0x01   /* STM32 -> PC */
#define TLV_TYPE_LED_STATE    0x10   /* PC -> STM32 */

#define SLOT_COUNT     10
#define POT_COUNT      4

#define PANEL_STATE_LEN  (2 + POT_COUNT * 2)   /* 10 */
#define LED_STATE_LEN    1                     /* menu index */

/* Pixel 0 is the GCS status indicator; pixels 1..6 are the six menus.
   Seven pixels, six menus, one status light — it fits exactly. Move
   GCS_LED_INDEX and MENU_LED_OFFSET if your strip is laid out
   differently. */
#define GCS_LED_INDEX     0
#define MENU_LED_OFFSET   1

/* The strip is re-rendered on this period so the status patterns can
   animate. 25 ms is 40 fps — smooth breathing, negligible DMA load. */
#define LED_RENDER_PERIOD_MS  25

/* How long without a valid LED_STATE frame before the link counts as
   lost. The PC resends every LED_HEARTBEAT_S (200 ms) even when nothing
   changed, so this tolerates three missed frames. */
#define LINK_TIMEOUT_MS   600

/* How long the green "connected" double-flash runs before settling to
   solid white. */
#define CONNECT_FLASH_MS  1500


#define DEBOUNCE_MS      15
#define FRAME_PERIOD_MS  10
#define POT_AVG_SAMPLES  16
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
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       active_low;   /* 1 = pressed reads LOW, 0 = pressed reads HIGH */
} button_t;

/* Slot order IS the wire order: buttons[i] is bit i of the outgoing slot
   bitmap. The PC's system_config.py has the matching SLOT_* constants.
   All ten are active_low. The btn_db[] comments in the previous version
   claimed 4-9 were active_high, but every entry in this table says
   otherwise and the debounce initial values agree with active_low. */
static const button_t buttons[SLOT_COUNT] = {
    { GPIOA, GPIO_PIN_6,  1 },  /* 0: PA6  — menu up   */
    { GPIOD, GPIO_PIN_14, 1 },  /* 1: PD14 — spare     */
    { GPIOD, GPIO_PIN_15, 1 },  /* 2: PD15 — menu down */
    { GPIOC, GPIO_PIN_7,  1 },  /* 3: PC7  — spare     */
    { GPIOE, GPIO_PIN_10, 1 },  /* 4: PE10 — RS3 up    */
    { GPIOE, GPIO_PIN_11, 1 },  /* 5: PE11 — RS4 down  */
    { GPIOE, GPIO_PIN_14, 1 },  /* 6: PE14 — RS3 down  */
    { GPIOE, GPIO_PIN_12, 1 },  /* 7: PE12 — RS4 up    */
    { GPIOD, GPIO_PIN_11, 1 },  /* 8: PD11 — RS5 down  */
    { GPIOD, GPIO_PIN_13, 1 },  /* 9: PD13 — RS5 up    */
};

/* Debounce state, one per physical pin, indexed to match buttons[].
   raw_last/change_time track raw bounce; stable is only updated once a
   reading has held steady for DEBOUNCE_MS.

   edge_last is gone: this firmware does no edge detection at all. It
   reports levels, and the PC finds the edges — that's the only place
   with the menu context needed to know what an edge means. */
typedef struct {
    GPIO_PinState raw_last;
    GPIO_PinState stable;
    uint32_t      change_time;
} debounce_state_t;

/* Initial value must match each pin's electrical idle level. All pins
   are active_low, so idle reads SET. */
static debounce_state_t btn_db[SLOT_COUNT] = {
    { GPIO_PIN_SET, GPIO_PIN_SET, 0 },
    { GPIO_PIN_SET, GPIO_PIN_SET, 0 },
    { GPIO_PIN_SET, GPIO_PIN_SET, 0 },
    { GPIO_PIN_SET, GPIO_PIN_SET, 0 },
    { GPIO_PIN_SET, GPIO_PIN_SET, 0 },
    { GPIO_PIN_SET, GPIO_PIN_SET, 0 },
    { GPIO_PIN_SET, GPIO_PIN_SET, 0 },
    { GPIO_PIN_SET, GPIO_PIN_SET, 0 },
    { GPIO_PIN_SET, GPIO_PIN_SET, 0 },
    { GPIO_PIN_SET, GPIO_PIN_SET, 0 },
};

/* Which palette colour each menu shows. The colours themselves, and
   every blink rate and breathe period, live in fastLed_SPI.h — this is
   only the assignment of one to the other, in the PC's menu order (the
   order of MENU_BITS in input_router.py).

   Display only. An index past the end of this table lights nothing and
   changes no behaviour, so a PC with more menus than this table has
   colours degrades to a dark gauge rather than misbehaving. */
static const ws2812_color_t *MENU_COLORS[] = {
    &WS2812_WHITE,    /* 0: zoom / fov / focus */
    &WS2812_ORANGE,   /* 1: picture select     */
    &WS2812_TEAL,     /* 2: tracking           */
    &WS2812_RED,      /* 3: laser              */
    &WS2812_CYAN,     /* 4: capture            */
    &WS2812_VIOLET,   /* 5: display            */
};
#define MENU_COLOR_COUNT (sizeof(MENU_COLORS) / sizeof(MENU_COLORS[0]))

/* GCS status indicator states. The firmware decides which one is active
   — see Gcs_Current_State(). */
typedef enum {
    GCS_WAITING,     /* amber, slow breathe 0.5 Hz — powered, PC not talking yet */
    GCS_CONNECTED,   /* green, double flash — transient, first CONNECT_FLASH_MS */
    GCS_SYNCED,      /* white, solid — PC and micro in sync */
    GCS_LOST,        /* red, slow blink 1 Hz — was connected, frames stopped */
    GCS_FAULT,       /* red, fast blink 4 Hz — reserved, set via gcs_override */
    GCS_TESTING,     /* blue, double flash — set via gcs_override */
} gcs_state_t;

/* Which menu the PC says is selected. That's panel state, not LED
   logic — the PC owns the menu because menu up/down are just slots it
   edge-detects. What the indicator DOES with it is decided entirely
   below. */
static volatile uint8_t led_menu_index  = 0xFF;   /* 0xFF = nothing yet */

/* Locally-driven indicator override, for the two states that nothing
   about the link can reveal. Set it from firmware code — a boot
   self-test, a diagnostic button combo, a detected hardware fault — and
   the indicator follows. GCS_NONE means "derive from link state", which
   is the normal case.

   Deliberately NOT settable from the PC: the whole point is that this
   file decides what the light does. */
typedef enum { GCS_NONE, GCS_LOCAL_TESTING, GCS_LOCAL_FAULT } gcs_override_t;
static volatile gcs_override_t gcs_override = GCS_NONE;

/* Link tracking. last_led_rx_tick is the arrival time of the most recent
   valid LED_STATE; first_led_rx_tick is the first one since boot. The
   difference between "never heard from the PC" and "heard, then lost it"
   is exactly first_led_seen, which is why the PC never needs to report
   its own disconnection — it couldn't anyway. */
static volatile uint32_t last_led_rx_tick  = 0;
static volatile uint32_t first_led_rx_tick = 0;
static volatile uint8_t  first_led_seen    = 0;

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
static uint8_t tlv_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) crc ^= data[i];
    return crc;
}

static uint8_t TLV_Send(uint8_t type, const uint8_t *payload, uint8_t len)
{
    static uint8_t frame[2][64];
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

    /* Which wire this goes out on is transport.h's TRANSPORT_ACTIVE.
       Nothing in this function, or anywhere else in the protocol, knows
       or cares — the frame bytes are identical over UART, USB CDC or
       UDP, and the PC side is unaffected either way.

       The ping-pong buffer above matters more now than it did: a
       non-blocking transport may still be reading the previous frame
       when the next TLV_Send() is called. */
    return transport_send(buf, len + 5);
}

/* ── Incoming (PC -> STM32) TLV receive parser ────────────────────────────
   Mirrors the Python StreamParser's state machine exactly, including
   being self-resyncing: any corruption just drops that one frame and
   returns to hunting for the next SYNC byte. Registered with
   transport_set_rx_handler() and called one byte at a time by whichever
   link is active, so it stays short and non-blocking — the ws2812 blit
   is deferred to the main loop. */
typedef enum {
    RX_WAIT_SYNC,
    RX_READ_TYPE,
    RX_READ_LEN,
    RX_READ_PAYLOAD,
    RX_READ_CRC,
    RX_READ_END,
} rx_state_t;

static rx_state_t rx_state = RX_WAIT_SYNC;
static uint8_t    rx_type;
static uint8_t    rx_len;
static uint8_t    rx_payload[LED_STATE_LEN];
static uint8_t    rx_payload_idx;
static uint8_t    rx_crc_ok;

static void Process_Received_Byte(uint8_t byte)
{
    switch (rx_state)
    {
    case RX_WAIT_SYNC:
        if (byte == TLV_SYNC) rx_state = RX_READ_TYPE;
        break;

    case RX_READ_TYPE:
        rx_type = byte;
        rx_state = RX_READ_LEN;
        break;

    case RX_READ_LEN:
        rx_len = byte;
        rx_payload_idx = 0;
        if (rx_len > sizeof(rx_payload)) {
            rx_state = RX_WAIT_SYNC;   /* can't hold it — drop and resync */
        } else {
            rx_state = (rx_len > 0) ? RX_READ_PAYLOAD : RX_READ_CRC;
        }
        break;

    case RX_READ_PAYLOAD:
        rx_payload[rx_payload_idx++] = byte;
        if (rx_payload_idx == rx_len) rx_state = RX_READ_CRC;
        break;

    case RX_READ_CRC:
        rx_crc_ok = (byte == tlv_crc8(rx_payload, rx_len));
        rx_state = RX_READ_END;
        break;

    case RX_READ_END:
        rx_state = RX_WAIT_SYNC;
        if (byte == TLV_END && rx_crc_ok)
        {
            if (rx_type == TLV_TYPE_LED_STATE && rx_len == LED_STATE_LEN)
            {
                led_menu_index = rx_payload[0];

                last_led_rx_tick = HAL_GetTick();
                if (!first_led_seen) {
                    first_led_seen = 1;
                    first_led_rx_tick = last_led_rx_tick;
                }
            }
        }
        /* Bad CRC or bad end byte — silently drop and resync. */
        break;
    }
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
    HAL_ADC_Stop(&hadc3);

    HAL_ADC_Start(&hadc3);
    if (HAL_ADC_PollForConversion(&hadc3, 10) != HAL_OK)
    {
        HAL_ADC_Stop(&hadc3);
        return 0;
    }
    uint16_t value = HAL_ADC_GetValue(&hadc3);
    HAL_ADC_Stop(&hadc3);

    return value;
}

/* Samples one pin's raw state and updates its debounced "stable" value
   once the raw reading has held steady for DEBOUNCE_MS. */
static void Debounce_Update(GPIO_TypeDef *port, uint16_t pin, debounce_state_t *db, uint32_t now)
{
    GPIO_PinState raw = HAL_GPIO_ReadPin(port, pin);
    if (raw != db->raw_last) {
        db->raw_last = raw;
        db->change_time = now;
    } else if ((now - db->change_time) >= DEBOUNCE_MS) {
        db->stable = raw;
    }
}

static void Debounce_Sample_All(uint32_t now)
{
    for (int i = 0; i < SLOT_COUNT; i++) {
        Debounce_Update(buttons[i].port, buttons[i].pin, &btn_db[i], now);
    }
}

static inline uint8_t Debounced_Is_Pressed(const debounce_state_t *db, uint8_t active_low)
{
    return active_low ? (db->stable == GPIO_PIN_RESET) : (db->stable == GPIO_PIN_SET);
}

/* Packs the debounced level of every slot into one bitmap. This is the
   entire outgoing button payload — no registers, no bit names, no menu
   gating. Slots 1 and 3 are included like all the rest; the previous
   version sampled them and then threw them away because no handler was
   ever wired to them. */
static uint16_t Build_Slot_Bitmap(void)
{
    uint16_t bitmap = 0;
    for (int i = 0; i < SLOT_COUNT; i++) {
        if (Debounced_Is_Pressed(&btn_db[i], buttons[i].active_low)) {
            bitmap |= (1U << i);
        }
    }
    return bitmap;
}

/* ── GCS status indicator ─────────────────────────────────────────────────
   Every decision about the indicator is made here. The PC sends no
   status of any kind — it only says which menu is selected.

   Link state is derived, which is the only way it could work anyway:
   "Python isn't running yet" and "the link just dropped" are both
   unannounceable from the PC's side, but trivially distinguishable from
   this one. The first is "no frame has ever arrived", the second is "one
   arrived, then they stopped". A crashed process cannot report its own
   crash; the other end has to notice. */
static gcs_state_t Gcs_Current_State(uint32_t now)
{
    if (gcs_override == GCS_LOCAL_TESTING) return GCS_TESTING;
    if (gcs_override == GCS_LOCAL_FAULT)   return GCS_FAULT;

    if (!first_led_seen) return GCS_WAITING;

    if ((now - last_led_rx_tick) >= LINK_TIMEOUT_MS) return GCS_LOST;

    if ((now - first_led_rx_tick) < CONNECT_FLASH_MS) return GCS_CONNECTED;

    return GCS_SYNCED;
}

/* Picks a pattern and colour for each indicator, then lets the driver
   animate them. Every rate, every colour, and the shape of every pattern
   is in fastLed_SPI.c — this function contributes only the conditions,
   which is the part actually specific to this panel.

   Called from the main loop on a timer, never from an ISR: the strip
   push is a blocking SPI transmit. */
static void Apply_Led_State(uint32_t now)
{
    switch (Gcs_Current_State(now))
    {
    case GCS_WAITING:
        ws2812_set(GCS_LED_INDEX, WS2812_PATTERN_BREATHE_SLOW, WS2812_AMBER);
        break;
    case GCS_CONNECTED:
        ws2812_set(GCS_LED_INDEX, WS2812_PATTERN_DOUBLE_FLASH, WS2812_GREEN);
        break;
    case GCS_SYNCED:
        ws2812_set(GCS_LED_INDEX, WS2812_PATTERN_SOLID, WS2812_WHITE);
        break;
    case GCS_LOST:
        ws2812_set(GCS_LED_INDEX, WS2812_PATTERN_BLINK_SLOW, WS2812_RED);
        break;
    case GCS_FAULT:
        ws2812_set(GCS_LED_INDEX, WS2812_PATTERN_BLINK_FAST, WS2812_RED);
        break;
    case GCS_TESTING:
        ws2812_set(GCS_LED_INDEX, WS2812_PATTERN_DOUBLE_FLASH, WS2812_BLUE);
        break;
    }

    /* Menu gauge: every menu present in its own colour, the active one at
       full brightness. Same pattern for all of them — only the scale
       differs — which is exactly the split the driver is built around. */
    uint8_t active = led_menu_index;

    for (int m = 0; m < (int)MENU_COLOR_COUNT; m++)
    {
        int px = MENU_LED_OFFSET + m;
        if (px >= WS2812_NUM_LEDS) break;

        ws2812_set_dim(px, WS2812_PATTERN_SOLID, *MENU_COLORS[m],
                       (m == active) ? 255U : WS2812_SCALE_DIM);
    }

    ws2812_animate(now);
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
  /* Route received bytes into the frame parser, then bring up whichever
     link transport.h selects. Changing TRANSPORT_ACTIVE and rebuilding
     is the entire switch — nothing below this line changes. */
  transport_set_rx_handler(Process_Received_Byte);
  transport_init();

  ws2812_init();

  /* ws2812_init() already cleared the strip and reset every LED's
     pattern state, so nothing more is needed here. The first
     Apply_Led_State() tick puts the GCS pixel into breathing amber,
     which is the honest reading at power-on: running, nothing heard from
     the PC yet. Menus stay dark until an index arrives — this firmware
     has no menu_register and no Get_Menu_Index(). */

  uint32_t pot_acc[POT_COUNT] = {0};
  uint16_t pot_avg[POT_COUNT] = {0};
  uint8_t  pot_count = 0;

  uint32_t last_frame_send = 0;
  uint32_t last_led_render = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        /* No-op for UART and USB CDC; drives the LwIP stack when the
           Ethernet transport is active. */
        transport_poll();

        uint32_t now = HAL_GetTick();
        Debounce_Sample_All(now);

        /* Periodic, not on-change: the GCS patterns animate, so the
           strip has to be re-rendered continuously rather than only when
           the PC sends something. */
        if ((now - last_led_render) >= LED_RENDER_PERIOD_MS)
        {
            Apply_Led_State(now);
            last_led_render = now;
        }

        /* Channels 9 and 15 were commented out in the previous version,
           so the flight joystick always transmitted zeros. Re-enabled —
           if the stick now reads garbage, these two lines are why they
           were disabled and this is where to look. */
        pot_acc[0] += ADC_Read_Channel(ADC_CHANNEL_9);
        pot_acc[1] += ADC_Read_Channel(ADC_CHANNEL_15);
        pot_acc[2] += ADC_Read_Channel(ADC_CHANNEL_6);
        pot_acc[3] += ADC_Read_Channel(ADC_CHANNEL_7);

        if (++pot_count >= POT_AVG_SAMPLES)
        {
            for (int i = 0; i < POT_COUNT; i++) {
                pot_avg[i] = (uint16_t)(pot_acc[i] / POT_AVG_SAMPLES);
                pot_acc[i] = 0;
            }
            pot_count = 0;
        }

        if ((HAL_GetTick() - last_frame_send) >= FRAME_PERIOD_MS)
        {
            uint16_t slots = Build_Slot_Bitmap();

            uint8_t payload[PANEL_STATE_LEN];
            payload[0] = (uint8_t)(slots & 0xFF);
            payload[1] = (uint8_t)((slots >> 8) & 0xFF);
            for (int i = 0; i < POT_COUNT; i++) {
                payload[2 + i * 2] = (uint8_t)(pot_avg[i] & 0xFF);
                payload[3 + i * 2] = (uint8_t)((pot_avg[i] >> 8) & 0xFF);
            }

            TLV_Send(TLV_TYPE_PANEL_STATE, payload, sizeof(payload));
            last_frame_send = HAL_GetTick();
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
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11 PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
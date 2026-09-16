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
#include "usb_device.h"

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
/* BUTTON_STATE (0x01) — flight/status only. Payload/camera/laser/
   tracking concepts live on PAYLOAD_COMMAND (0x04) below instead. */
#define BIT_ARM                 0
#define BIT_ARM_STATUS          1
#define BIT_AUTO                2
#define BIT_AUTO_STATUS         3
#define BIT_MANUAL              4
#define BIT_MANUAL_STATUS       5
#define BIT_MENU_SELECT_0       6
#define BIT_MENU_SELECT_1       7
#define BIT_SPEED_UP            8
#define BIT_SPEED_DOWN          9
#define BIT_MENU_SELECT_2       10
#define BIT_MENU_SELECT_3       11
#define BIT_MENU_SELECT_4       12
#define BIT_MENU_SELECT_5       13
#define Bit_Stropes             14
#define DEBOUNCE_MS  15

/* PAYLOAD_COMMAND (0x04) — payload/gimbal/camera/laser/tracking, own
   32-bit register. Numbering matches bit_definitions.py's
   PAYLOAD_*_BIT constants exactly. */
#define PAYLOAD_BIT_ZOOM_IN                    0
#define PAYLOAD_BIT_ZOOM_OUT                   1
#define PAYLOAD_BIT_FOV_IN                     2   /* a.k.a. "wide in" */
#define PAYLOAD_BIT_FOV_OUT                    3   /* a.k.a. "wide out" */
#define PAYLOAD_BIT_FOCUS_IN                   4
#define PAYLOAD_BIT_FOCUS_OUT                  5
#define PAYLOAD_BIT_LASER_ON_OFF               6
#define PAYLOAD_BIT_LASER_CONT_MODE            7
#define PAYLOAD_BIT_LASER_SINGLE_MODE          8
#define PAYLOAD_BIT_LASER_ZOOM_IN              9
#define PAYLOAD_BIT_LASER_ZOOM_OUT              10
#define PAYLOAD_BIT_TRACKING_SEARCH_ON_OFF     11
#define PAYLOAD_BIT_AI_TRACKING_ON_OFF         12
#define PAYLOAD_BIT_TRACKING_TEMPLATE_TOGGLE   13
#define PAYLOAD_BIT_TRACKING_SOURCE_TOGGLE     14
#define PAYLOAD_BIT_JOYSTICK_TRACK             15
#define PAYLOAD_BIT_TAKE_PICTURE               16
#define PAYLOAD_BIT_START_RECORD               17
#define PAYLOAD_BIT_STOP_RECORD                18
#define PAYLOAD_BIT_PIC_RECORD_MODE_TOGGLE     19
#define PAYLOAD_BIT_IMAGE_SENSOR_CHANGE        20
#define PAYLOAD_BIT_IR_POLARITY                21
#define PAYLOAD_BIT_IR_DZOOM_PLUS              22
#define PAYLOAD_BIT_IR_DZOOM_MINUS             23
#define PAYLOAD_BIT_NEAR_IR_TOGGLE             24
#define PAYLOAD_BIT_EO_IMAGE_ON_OFF            25
#define PAYLOAD_BIT_MOTOR_ON_OFF               26
#define PAYLOAD_BIT_VIDEO_IP                   27
#define PAYLOAD_BIT_EO_DZOOM_TOGGLE            28
#define PAYLOAD_BIT_IR_RAINBOW                 29

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

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} led_t;

/* Order must match: buttons[i] drives leds[i] */
static const button_t buttons[10] = {
    { GPIOA, GPIO_PIN_6,  1 },  /* PF12: RA2 = DOWN */
    { GPIOD, GPIO_PIN_14, 1 },  /* PD14 */
	{ GPIOD, GPIO_PIN_15, 1 },  /* PD15: RS2 = UP */
    { GPIOC, GPIO_PIN_7,  1 },  /* PC7  */
	{ GPIOE, GPIO_PIN_10, 1 },  /* PE10: RS3 = UP */
	{ GPIOE, GPIO_PIN_11, 1 },  /* PE11: RS4 = DOWN */
    { GPIOE, GPIO_PIN_14, 1 },  /* PE14: RS3 = DOWN */
	{ GPIOE, GPIO_PIN_12, 1 },  /* PE12: RS4 = UP */
	{ GPIOD, GPIO_PIN_11, 1 },  /* PD11: RS5 = DOWN */
    { GPIOD, GPIO_PIN_13, 1 },  /* PD13: RS5 = UP */

};


/* Unsized on purpose. This was declared [10] with ELEVEN initializers,
   which gcc warns about and then silently drops the last entry — so
   "User Led 3" was never actually addressable. Letting the compiler
   count the rows means the size can't disagree with the contents again.

   HEADS UP: leds[2] is PE11, the same pin buttons[5] reads as a
   debounced input. One pin cannot be both. Nothing calls the three
   Set_LED_* helpers below, so it does no harm today, but it will the
   moment something does.

   The whole GPIO LED bank is dead code — the helpers are defined and
   never called anywhere. The ws2812 strip is the only live indicator.
   Deleting it would settle the PE11 conflict for free. */
static const led_t leds[] = {
	{ GPIOF, GPIO_PIN_13 }, // Led0
	{ GPIOE, GPIO_PIN_9  }, // Led1
	{ GPIOE, GPIO_PIN_11 }, // Led2
	{ GPIOF, GPIO_PIN_14 }, // Led3
	{ GPIOF, GPIO_PIN_15 }, // Led4
	{ GPIOG, GPIO_PIN_14 }, // Led5
	{ GPIOG, GPIO_PIN_9  }, // Led6
	{ GPIOE, GPIO_PIN_8 },  // Led7
    { GPIOB, GPIO_PIN_0  }, // User Led 1
    { GPIOB, GPIO_PIN_7  }, // User Led 2
    { GPIOE, GPIO_PIN_13 }, // User Led 3
};
#define LED_COUNT (sizeof(leds) / sizeof(leds[0]))

/* Debounce state, one struct per physical pin — indexed to match buttons[].
   raw_last/change_time track raw bounce; stable is only updated once a
   reading has held steady for DEBOUNCE_MS. edge_last is separate so
   Poll_Debounced() can detect a fresh idle->pressed transition regardless
   of how many different menus bind that same physical pin to a handler. */
typedef struct {
    GPIO_PinState raw_last;
    GPIO_PinState stable;
    GPIO_PinState edge_last;
    uint32_t      change_time;
} debounce_state_t;

/* Initial value must match each pin's electrical idle level
   (active_low -> idle reads SET, active_high -> idle reads RESET),
   or the first loop iteration could see a phantom press. */
static debounce_state_t btn_db[10] = {
    { GPIO_PIN_SET,   GPIO_PIN_SET,   GPIO_PIN_SET,   0 }, /* 0: PF12 active_low  */
    { GPIO_PIN_SET,   GPIO_PIN_SET,   GPIO_PIN_SET,   0 }, /* 1: PD14 active_low  */
	{ GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, 0 },  /* 2: PD15 active_low  */
    { GPIO_PIN_SET,   GPIO_PIN_SET,   GPIO_PIN_SET,   0 }, /* 3: PC7  active_low  */
	{ GPIO_PIN_SET,   GPIO_PIN_SET,   GPIO_PIN_SET,   0 }, /* 4: PE10 active_high */
	{ GPIO_PIN_SET,   GPIO_PIN_SET,   GPIO_PIN_SET,   0 }, /* 5: PE12 active_high */
	{ GPIO_PIN_SET,   GPIO_PIN_SET,   GPIO_PIN_SET,   0 }, /* 6: PE14 active_high */
	{ GPIO_PIN_SET,   GPIO_PIN_SET,   GPIO_PIN_SET,   0 }, /* 7: PD11 active_high */
	{ GPIO_PIN_SET,   GPIO_PIN_SET,   GPIO_PIN_SET,   0 }, /* 8: PD12 active_high */
	{ GPIO_PIN_SET,   GPIO_PIN_SET,   GPIO_PIN_SET,   0 }, /* 9: PD13 active_high */
};


#define TLV_SYNC 0xAA
#define TLV_END  0x55

#define TLV_TYPE_BUTTON_STATE 0x01
#define TLV_TYPE_JOYSTICK     0x02
#define TLV_TYPE_JOYSTICK2    0x03
#define TLV_TYPE_PAYLOAD_COMMAND 0x04
/* PC -> STM32, sent once right after the Python side opens the serial
   port. On receipt, the STM32 performs a full system reset so it
   always starts from a known-clean state in sync with a freshly
   launched Python process, rather than potentially carrying over
   stale menu_register/USB_MESSAGE/PAYLOAD_MESSAGE state from before
   Python restarted. Carries no payload (LEN=0). */
#define TLV_TYPE_HELLO           0x20
#define TLV_TYPE_GOODBYE         0x21
#define TLV_TYPE_HEARTBEAT       0x22

/* ── FSM tunables ─────────────────────────────────────────────────────────
   CHECK THESE AGAINST THE PC. The timeout must be a comfortable multiple
   of whatever rate Python actually sends heartbeats at — three missed
   beats is the usual choice, so a 150 ms heartbeat wants ~500 ms here.
   Too tight and a healthy link flickers red. */
#define HEARTBEAT_TIMEOUT_MS   500
#define BUTTON_FRAME_PERIOD_MS  10
#define CONNECT_FLASH_MS      1500
#define TEST_HOLD_MS          2000
#define LED_RENDER_PERIOD_MS    25

/* Which ws2812 pixel is the status indicator. */
#define STATUS_LED_INDEX         0
/* ── Top-level firmware state ─────────────────────────────────────────────
   Five states. Only Fsm_Tick() and the event flags below move between
   them; nothing else in this file writes `fsm`.

     WAITING    powered, no hello yet          amber, slow breathe
     SYNCED     hello seen, no beat yet        green, double flash
     CONNECTED  beat flowing, frames going out white, solid
     LOST       beat stopped                   red, slow blink
     TESTING    protocol bypassed              blue, blink

   Button frames go out in CONNECTED and TESTING only. In WAITING, SYNCED
   and LOST the panel is silent — nothing is listening, or nothing has
   asked yet. */
typedef enum {
    FSM_WAITING,
    FSM_SYNCED,
    FSM_CONNECTED,
    FSM_LOST,
    FSM_TESTING,
} fsm_state_t;

static fsm_state_t fsm = FSM_WAITING;

/* The Nucleo's B1 user button, PC13. Deliberately NOT part of buttons[]:
   that array's index IS the wire position of each slot bit, so appending
   to it would change the protocol. This is a panel-local control that the
   PC never sees.

   Note it is active HIGH — B1 sits on a pull-down and pressed connects to
   VDD, the opposite of all ten panel buttons. MX_GPIO_Init() already
   configures PC13 as a no-pull input, so no CubeMX change is needed. */
static const button_t user_button = { GPIOC, GPIO_PIN_13, 0 };

/* Idle for an active-high pin reads RESET. Getting this wrong would look
   like the button being held from the moment of boot. */
static debounce_state_t user_btn_db = { GPIO_PIN_RESET, GPIO_PIN_RESET,
                                        GPIO_PIN_RESET, 0 };

static uint32_t fsm_entered_tick = 0;   /* for the green connect flash */
static uint32_t last_beat_tick   = 0;   /* last heartbeat or hello */
static uint32_t user_btn_down_tick = 0; /* 0 = not currently held */
static uint8_t  user_btn_consumed  = 0; /* one toggle per hold */

/* Set in interrupt context by the RX parser, cleared in Fsm_Tick(). */
volatile uint8_t ev_hello     = 0;
volatile uint8_t ev_goodbye   = 0;
volatile uint8_t ev_heartbeat = 0;

uint32_t USB_MESSAGE = 0x00;
uint32_t PAYLOAD_MESSAGE = 0x00;
uint8_t menu_register = 0b000001;
uint8_t LED_number = 0;

static uint8_t is_recording = 0;
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

    /* Which wire this goes out on is transport.h's TRANSPORT_ACTIVE.
       The frame bytes are identical over UART and USB CDC, so nothing
       else here — and nothing on the PC side — changes with the switch. */
    return transport_send(buf, len + 5);
}

/* ── Incoming (PC -> STM32) TLV receive parser ────────────────────────────
   Mirrors the Python StreamParser's state machine exactly, including
   being self-resyncing: any corruption (bad sync, bad CRC, bad end
   byte) just drops that one frame and returns to hunting for the next
   SYNC byte. Registered with transport_set_rx_handler() and called one
   byte at a time by whichever link is active, so it works unchanged over
   UART or USB CDC. Runs one byte per call from interrupt context
   (interrupt context) — kept deliberately simple and non-blocking. The
   actual RESPONSE to a received frame (the system reset, for
   TLV_TYPE_HELLO, GOODBYE and HEARTBEAT) is deferred to the main loop
   via the event flags, not performed here — see Fsm_Tick(). */
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
static uint8_t    rx_payload[16];   /* big enough for any incoming type so far — HELLO carries 0 bytes */
static uint8_t    rx_payload_idx;
static uint8_t    rx_crc_ok;

/* HELLO no longer resets the MCU. A reset would wipe the FSM the moment
   it was supposed to advance, and over USB CDC it drops enumeration and
   leaves the PC holding a dead handle. Fsm_Tick() clears the three
   registers instead, which is all the reset was ever for. */

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
            rx_state = RX_WAIT_SYNC;   /* can't hold it — drop and resync, same spirit as the Python parser's bad-frame handling */
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
            /* ── ADD NEW MESSAGE TYPES HERE ───────────────────────────
               Set a flag and nothing else — this runs in interrupt
               context. Fsm_Tick() in the main loop does the work. */
            if (rx_type == TLV_TYPE_HELLO)
            {
                ev_hello = 1;
            }
            else if (rx_type == TLV_TYPE_GOODBYE)
            {
                ev_goodbye = 1;
            }
            else if (rx_type == TLV_TYPE_HEARTBEAT)
            {
                ev_heartbeat = 1;
            }
        }
        /* Bad CRC or bad end byte — silently drop and resync, same as
           the Python StreamParser does; no error path needed here. */
        break;
    }
}


/* HAL_UART_RxCpltCallback now lives in transport.c, which owns every
   link's receive path and funnels bytes into Process_Received_Byte via
   the handler registered in main(). Defining it here too would be a
   duplicate symbol at link time. */




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

    /* Real conversion. */
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


/* Samples one physical pin's raw state and updates its debounced "stable"
   value once the raw reading has held steady for DEBOUNCE_MS. Call for
   every pin, every loop iteration, regardless of which menu is active —
   that's what keeps debounce state from going stale across menu switches. */
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

/* Runs Debounce_Update for every physical button pin plus the menu-select
   button. Call this once, at the very top of the main loop. */
static void Debounce_Sample_All(uint32_t now)
{

    for (int i = 0; i < 10; i++) {
        Debounce_Update(buttons[i].port, buttons[i].pin, &btn_db[i], now);
    }
}

static inline uint8_t Debounced_Is_Pressed(const debounce_state_t *db, uint8_t active_low)
{
    return active_low ? (db->stable == GPIO_PIN_RESET) : (db->stable == GPIO_PIN_SET);
}

/* Replaces set_bit_from_pin(): mirrors the debounced (not raw) level into
   *target (USB_MESSAGE or PAYLOAD_MESSAGE, whichever the caller passes),
   so momentary/held buttons no longer flicker on bounce. */
static inline void Set_Bit_From_Debounced(debounce_state_t *db, uint8_t active_low, uint32_t *target, uint32_t bit)
{
    if (Debounced_Is_Pressed(db, active_low)) *target |= (1UL << bit);
    else                                       *target &= ~(1UL << bit);
}


static uint8_t Get_Menu_Index(void)
{
    if (menu_register & 0b0000001) return 0;
    if (menu_register & 0b0000010) return 1;
    if (menu_register & 0b0000100) return 2;
    if (menu_register & 0b0001000) return 3;
    if (menu_register & 0b0010000) return 4;
    if (menu_register & 0b0100000) return 5;
    if (menu_register & 0b1000000) return 6;
    return 0;
}



void onARM_Button_Press(void)
{
    if (USB_MESSAGE & (1 << BIT_ARM)) USB_MESSAGE &= ~(1 << BIT_ARM);
    else                              USB_MESSAGE |= (1 << BIT_ARM);
}

void onUpMenuSelect_Button_Press(void)
{

    if (menu_register & (1 << 6)) menu_register = 0b000001;
    else                          menu_register = menu_register << 1;


}

void onDownMenuSelect_Button_Press(void)
{

    if (menu_register & (1 << 0)) menu_register = 0b100000;
    else                          menu_register = menu_register >> 1;

}
/*
void onManual_Button_Press(void)
{

    if (USB_MESSAGE & (1 << BIT_MANUAL)) USB_MESSAGE &= ~(1 << BIT_MANUAL);
    else                                 USB_MESSAGE |= (1 << BIT_MANUAL);
}*/

void onAuto_Button_Press(void)
{
    if (USB_MESSAGE & (1 << BIT_AUTO)) USB_MESSAGE &= ~(1 << BIT_AUTO);
    else                                USB_MESSAGE |= (1 << BIT_AUTO);
}


/*
void onSpeedUp_Button_Press(void)
{
    if (USB_MESSAGE & (1 << BIT_SPEED_UP)) USB_MESSAGE &= ~(1 << BIT_SPEED_UP);
    else                                    USB_MESSAGE |= (1 << BIT_SPEED_UP);
}

void onSpeedDown_Button_Press(void)
{
    if (USB_MESSAGE & (1 << BIT_SPEED_DOWN)) USB_MESSAGE &= ~(1 << BIT_SPEED_DOWN);
    else                                      USB_MESSAGE |= (1 << BIT_SPEED_DOWN);
}*/


void onAI_JOYSTICK_TRACK_Button_Press(void)
{
	if (PAYLOAD_MESSAGE & (1 << PAYLOAD_BIT_AI_TRACKING_ON_OFF)) PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_AI_TRACKING_ON_OFF);
    if (PAYLOAD_MESSAGE & (1 << PAYLOAD_BIT_JOYSTICK_TRACK)) PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_JOYSTICK_TRACK);
    else                                    PAYLOAD_MESSAGE |= (1 << PAYLOAD_BIT_JOYSTICK_TRACK);
}


void onAI_TRACKING_Button_Press(void)
{
	if (PAYLOAD_MESSAGE & (1 << PAYLOAD_BIT_JOYSTICK_TRACK)) PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_JOYSTICK_TRACK);
    if (PAYLOAD_MESSAGE & (1 << PAYLOAD_BIT_AI_TRACKING_ON_OFF)) PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_AI_TRACKING_ON_OFF);
    else                                    PAYLOAD_MESSAGE |= (1 << PAYLOAD_BIT_AI_TRACKING_ON_OFF);
}


void onTRACKING_START_STOP_Button_Press(void)
{
    if (PAYLOAD_MESSAGE & (1 << PAYLOAD_BIT_TRACKING_SEARCH_ON_OFF)) PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_TRACKING_SEARCH_ON_OFF);
    else                                    PAYLOAD_MESSAGE |= (1 << PAYLOAD_BIT_TRACKING_SEARCH_ON_OFF);
}

/* NEW: tracking menu (config.py bit=2) needs these two, previously
   unhandled anywhere. */
void onTrackingTemplateToggle_Button_Press(void)
{
    if (PAYLOAD_MESSAGE & (1 << PAYLOAD_BIT_TRACKING_TEMPLATE_TOGGLE)) PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_TRACKING_TEMPLATE_TOGGLE);
    else                                    PAYLOAD_MESSAGE |= (1 << PAYLOAD_BIT_TRACKING_TEMPLATE_TOGGLE);
}

void onTrackingSourceToggle_Button_Press(void)
{
    if (PAYLOAD_MESSAGE & (1 << PAYLOAD_BIT_TRACKING_SOURCE_TOGGLE)) PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_TRACKING_SOURCE_TOGGLE);
    else                                    PAYLOAD_MESSAGE |= (1 << PAYLOAD_BIT_TRACKING_SOURCE_TOGGLE);
}


void onLASERSINGLE_Button_Press(void)
{
	if (PAYLOAD_MESSAGE & (1 << PAYLOAD_BIT_LASER_CONT_MODE)) PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_LASER_CONT_MODE);
    if (PAYLOAD_MESSAGE & (1 << PAYLOAD_BIT_LASER_SINGLE_MODE)) PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_LASER_SINGLE_MODE);
    else                                    PAYLOAD_MESSAGE |= (1 << PAYLOAD_BIT_LASER_SINGLE_MODE);
}



void onLASERCONT_Button_Press(void)
{
	if (PAYLOAD_MESSAGE & (1 << PAYLOAD_BIT_LASER_SINGLE_MODE)) PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_LASER_SINGLE_MODE);
    if (PAYLOAD_MESSAGE & (1 << PAYLOAD_BIT_LASER_CONT_MODE)) PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_LASER_CONT_MODE);
    else                                    PAYLOAD_MESSAGE |= (1 << PAYLOAD_BIT_LASER_CONT_MODE);
}


void onLASER_Button_Press(void)
{
    if (PAYLOAD_MESSAGE & (1 << PAYLOAD_BIT_LASER_ON_OFF)) PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_LASER_ON_OFF);
    else                                    PAYLOAD_MESSAGE |= (1 << PAYLOAD_BIT_LASER_ON_OFF);
}



void onVIDEOIP_Button_Press(void)
{
    if (PAYLOAD_MESSAGE & (1 << PAYLOAD_BIT_VIDEO_IP)) PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_VIDEO_IP);
    else                                    PAYLOAD_MESSAGE |= (1 << PAYLOAD_BIT_VIDEO_IP);
}

/* NEW: picture_select menu (config.py bit=1) needed this — was
   completely unhandled anywhere before. */
void onNearInfraredToggle_Button_Press(void)
{
    if (PAYLOAD_MESSAGE & (1 << PAYLOAD_BIT_NEAR_IR_TOGGLE)) PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_NEAR_IR_TOGGLE);
    else                                    PAYLOAD_MESSAGE |= (1 << PAYLOAD_BIT_NEAR_IR_TOGGLE);
}

/* NEW: display menu (config.py bit=5) fields, all previously unhandled. */
void onEOImageToggle_Button_Press(void)
{
    if (PAYLOAD_MESSAGE & (1 << PAYLOAD_BIT_EO_IMAGE_ON_OFF)) PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_EO_IMAGE_ON_OFF);
    else                                    PAYLOAD_MESSAGE |= (1 << PAYLOAD_BIT_EO_IMAGE_ON_OFF);
}

void onEODzoomToggle_Button_Press(void)
{
    if (PAYLOAD_MESSAGE & (1 << PAYLOAD_BIT_EO_DZOOM_TOGGLE)) PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_EO_DZOOM_TOGGLE);
    else                                    PAYLOAD_MESSAGE |= (1 << PAYLOAD_BIT_EO_DZOOM_TOGGLE);
}

void onIRRainbow_Button_Press(void)
{
    if (PAYLOAD_MESSAGE & (1 << PAYLOAD_BIT_IR_RAINBOW)) PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_IR_RAINBOW);
    else                                    PAYLOAD_MESSAGE |= (1 << PAYLOAD_BIT_IR_RAINBOW);
}

void onRECORD_Button_Press(void)
{
    if (is_recording)
    {
        PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_START_RECORD);
        PAYLOAD_MESSAGE |= (1 << PAYLOAD_BIT_STOP_RECORD);
        is_recording = 0;
    }
    else
    {
        PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_STOP_RECORD);
        PAYLOAD_MESSAGE |= (1 << PAYLOAD_BIT_START_RECORD);
        is_recording = 1;
    }
}

void onFocusIn_Button_Press(void)
{
    if (PAYLOAD_MESSAGE & (1 << PAYLOAD_BIT_FOCUS_IN)) PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_FOCUS_IN);
    else                                    PAYLOAD_MESSAGE |= (1 << PAYLOAD_BIT_FOCUS_IN);
}

void onFocusOut_Button_Press(void)
{
    if (PAYLOAD_MESSAGE & (1 << PAYLOAD_BIT_FOCUS_OUT)) PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_FOCUS_OUT);
    else                                      PAYLOAD_MESSAGE |= (1 << PAYLOAD_BIT_FOCUS_OUT);
}



void onZoomIn_Button_Press(void)
{
    if (PAYLOAD_MESSAGE & (1 << PAYLOAD_BIT_ZOOM_IN)) PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_ZOOM_IN);
    else                                   PAYLOAD_MESSAGE |= (1 << PAYLOAD_BIT_ZOOM_IN);
}

void onZoomOUT_Button_Press(void)
{
    if (PAYLOAD_MESSAGE & (1 << PAYLOAD_BIT_ZOOM_OUT)) PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_ZOOM_OUT);
    else                                    PAYLOAD_MESSAGE |= (1 << PAYLOAD_BIT_ZOOM_OUT);
}


void onWideIn_Button_Press(void)
{
    if (PAYLOAD_MESSAGE & (1 << PAYLOAD_BIT_FOV_IN)) PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_FOV_IN);
    else                                   PAYLOAD_MESSAGE |= (1 << PAYLOAD_BIT_FOV_IN);
}

void onWideOUT_Button_Press(void)
{
    if (PAYLOAD_MESSAGE & (1 << PAYLOAD_BIT_FOV_OUT)) PAYLOAD_MESSAGE &= ~(1 << PAYLOAD_BIT_FOV_OUT);
    else                                    PAYLOAD_MESSAGE |= (1 << PAYLOAD_BIT_FOV_OUT);
}

void onStropesToggle_Button_Press(void)
{
	if (USB_MESSAGE & (1 << Bit_Stropes)) USB_MESSAGE &= ~(1 << Bit_Stropes);
	    else                                 USB_MESSAGE |= (1 << Bit_Stropes);
}






static void Poll_Debounced(debounce_state_t *db, uint8_t active_low, void (*onPress)(void))
{
    GPIO_PinState pressedState = active_low ? GPIO_PIN_RESET : GPIO_PIN_SET;
    GPIO_PinState idleState    = active_low ? GPIO_PIN_SET   : GPIO_PIN_RESET;

    if (db->stable == pressedState && db->edge_last == idleState)
    {
        onPress();
    }
    db->edge_last = db->stable;
}

static inline void Set_LED_From_Bit(uint8_t led_idx, uint32_t bit)
{
    HAL_GPIO_WritePin(leds[led_idx].port, leds[led_idx].pin,
                       (USB_MESSAGE & (1UL << bit)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void Set_LED_From_Payload_Bit(uint8_t led_idx, uint32_t bit)
{
    HAL_GPIO_WritePin(leds[led_idx].port, leds[led_idx].pin,
                       (PAYLOAD_MESSAGE & (1UL << bit)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void Set_LED_Off(uint8_t led_idx)
{
    HAL_GPIO_WritePin(leds[led_idx].port, leds[led_idx].pin, GPIO_PIN_RESET);
}



/* ── The state machine ────────────────────────────────────────────────────
   One function, called every main-loop iteration. This is the only place
   `fsm` is written.

   Test mode is checked first and is reachable from every state, because a
   physical hold on the panel should work whether or not the PC is there
   — that's the point of a test mode. */
static void Fsm_Enter(fsm_state_t next, uint32_t now)
{
    if (fsm == next) return;
    fsm = next;
    fsm_entered_tick = now;
}

static void Fsm_Tick(uint32_t now)
{
    /* --- user button hold, any state ---------------------------------- */
    uint8_t held = Debounced_Is_Pressed(&user_btn_db, user_button.active_low);
    if (held)
    {
        if (user_btn_down_tick == 0)
        {
            user_btn_down_tick = now;
            user_btn_consumed  = 0;
        }
        else if (!user_btn_consumed && (now - user_btn_down_tick) >= TEST_HOLD_MS)
        {
            /* One toggle per hold — without this latch it would flip
               every iteration for as long as the button stayed down. */
            user_btn_consumed = 1;
            Fsm_Enter((fsm == FSM_TESTING) ? FSM_WAITING : FSM_TESTING, now);
        }
    }
    else
    {
        user_btn_down_tick = 0;
    }

    /* Test mode ignores the protocol entirely, so drop any events that
       arrived while in it rather than letting them queue up and fire the
       moment it exits. */
    if (fsm == FSM_TESTING)
    {
        ev_hello = ev_goodbye = ev_heartbeat = 0;
        return;
    }

    /* --- protocol events ---------------------------------------------- */
    if (ev_goodbye)
    {
        ev_goodbye = 0;
        ev_hello = ev_heartbeat = 0;   /* a clean exit outranks the rest */
        Fsm_Enter(FSM_WAITING, now);
        return;
    }

    if (ev_hello)
    {
        ev_hello = 0;
        last_beat_tick = now;

        /* A fresh hello restarts the handshake from wherever we were,
           including LOST — that's what "a new connection request" means. */
        USB_MESSAGE     = 0;
        PAYLOAD_MESSAGE = 0;
        menu_register   = 0b000001;


        Fsm_Enter(FSM_SYNCED, now);
    }

    if (ev_heartbeat)
    {
        ev_heartbeat = 0;
        last_beat_tick = now;

        /* From SYNCED this is the first beat and completes the handshake.
           From LOST it's the beat coming back, which resumes without a
           new handshake. From WAITING it's ignored: a heartbeat with no
           hello behind it is a PC we never agreed to talk to. */
        if (fsm == FSM_SYNCED || fsm == FSM_LOST)
        {
            Fsm_Enter(FSM_CONNECTED, now);
        }
    }

    /* --- heartbeat timeout -------------------------------------------- */
    if ((fsm == FSM_CONNECTED || fsm == FSM_SYNCED) &&
        (now - last_beat_tick) >= HEARTBEAT_TIMEOUT_MS)
    {
        /* SYNCED times out too: hello arrived and then Python died before
           its first beat. Without this it would sit green forever. */
        Fsm_Enter(FSM_LOST, now);
    }
}

/* ── Status indicator ─────────────────────────────────────────────────────
   Reports the FSM and nothing else. Every rate and colour lives in
   fastLed_SPI.h — this only picks which pattern is showing. */
static void Fsm_Apply_Led(uint32_t now)
{
    switch (fsm)
    {
    case FSM_WAITING:
        ws2812_set(STATUS_LED_INDEX, WS2812_PATTERN_BREATHE_SLOW, WS2812_AMBER);
        break;

    case FSM_SYNCED:
        ws2812_set(STATUS_LED_INDEX, WS2812_PATTERN_DOUBLE_FLASH, WS2812_GREEN);
        break;

    case FSM_CONNECTED:

        if ((now - fsm_entered_tick) < CONNECT_FLASH_MS)
            ws2812_set(STATUS_LED_INDEX, WS2812_PATTERN_DOUBLE_FLASH, WS2812_GREEN);
        else
            ws2812_set(STATUS_LED_INDEX, WS2812_PATTERN_SOLID, WS2812_WHITE);
        break;

    case FSM_LOST:
        ws2812_set(STATUS_LED_INDEX, WS2812_PATTERN_BLINK_SLOW, WS2812_RED);
        break;

    case FSM_TESTING:
        ws2812_set(STATUS_LED_INDEX, WS2812_PATTERN_BLINK_SLOW, WS2812_BLUE);
        break;
    }

    ws2812_animate(now);
}

/* True when button frames should be going out. */
static inline uint8_t Fsm_Should_Transmit(void)
{
    return (fsm == FSM_CONNECTED) || (fsm == FSM_TESTING);
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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* Route received bytes into the frame parser, then bring up whichever
     link transport.h selects. Changing TRANSPORT_ACTIVE and rebuilding
     is the entire switch between UART and USB. */
  transport_set_rx_handler(Process_Received_Byte);
  transport_init();

  ws2812_init();
  uint16_t pot1 = 0, pot2 = 0, pot3 = 0, pot4 = 0;


  static uint32_t last_usb_send = 0;
  static uint32_t last_led_render = 0;
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
	    /* Nearly a no-op for both current links; here so a polled
	       transport added later needs no change above this layer. */
	    transport_poll();

	    uint32_t now = HAL_GetTick();
        Debounce_Sample_All(now);   /* samples every physical pin, every iteration, no matter the menu */
        Debounce_Update(user_button.port, user_button.pin, &user_btn_db, now);

        /* The FSM runs on freshly sampled buttons — the test-mode hold is
           read from the same debounced state as everything else. */
        Fsm_Tick(now);

        /* Periodic, not on-change: the patterns animate, so the strip has
           to be re-rendered continuously. Never from an ISR — the SPI
           push blocks. */
        if ((now - last_led_render) >= LED_RENDER_PERIOD_MS)
        {
            last_led_render = now;
            Fsm_Apply_Led(now);
        }



        //pot1 += ADC_Read_Channel(ADC_CHANNEL_9);
        //pot2 += ADC_Read_Channel(ADC_CHANNEL_15);
        pot3 += ADC_Read_Channel(ADC_CHANNEL_6);
        pot4 += ADC_Read_Channel(ADC_CHANNEL_7);

        /* Persistent MODE buttons: toggle/latch on press, stay set until pressed again. */
        Poll_Debounced(&btn_db[0], buttons[0].active_low, onUpMenuSelect_Button_Press);   /* PA6, pull-up, active-low */
        Poll_Debounced(&btn_db[2], buttons[2].active_low, onDownMenuSelect_Button_Press);   /* PD15, pull-up, active-low */

        {

            uint8_t menu_idx = Get_Menu_Index();
            if (menu_idx == 0) USB_MESSAGE |= (1 << BIT_MENU_SELECT_0); else USB_MESSAGE &= ~(1 << BIT_MENU_SELECT_0);
            if (menu_idx == 1) USB_MESSAGE |= (1 << BIT_MENU_SELECT_1); else USB_MESSAGE &= ~(1 << BIT_MENU_SELECT_1);
            if (menu_idx == 2) USB_MESSAGE |= (1 << BIT_MENU_SELECT_2); else USB_MESSAGE &= ~(1 << BIT_MENU_SELECT_2);
            if (menu_idx == 3) USB_MESSAGE |= (1 << BIT_MENU_SELECT_3); else USB_MESSAGE &= ~(1 << BIT_MENU_SELECT_3);
            if (menu_idx == 4) USB_MESSAGE |= (1 << BIT_MENU_SELECT_4); else USB_MESSAGE &= ~(1 << BIT_MENU_SELECT_4);
            if (menu_idx == 5) USB_MESSAGE |= (1 << BIT_MENU_SELECT_5); else USB_MESSAGE &= ~(1 << BIT_MENU_SELECT_5);
        }

        /* MOMENTARY command buttons: bit mirrors the debounced pin level, set only while held. */
        /*                       btn_db[i]     active_low      target_register        bit */
        if(menu_register & (1<<0)){

        	Set_Bit_From_Debounced(&btn_db[4], buttons[4].active_low, &PAYLOAD_MESSAGE, PAYLOAD_BIT_ZOOM_IN);   // RS5 - UP
        	Set_Bit_From_Debounced(&btn_db[6], buttons[6].active_low, &PAYLOAD_MESSAGE, PAYLOAD_BIT_ZOOM_OUT);  // RS5 - DOWN
			Set_Bit_From_Debounced(&btn_db[7], buttons[7].active_low, &PAYLOAD_MESSAGE, PAYLOAD_BIT_FOV_IN);    // RS4 - UP
			Set_Bit_From_Debounced(&btn_db[5], buttons[5].active_low, &PAYLOAD_MESSAGE, PAYLOAD_BIT_FOV_OUT);   // RS4 - DOWN
			Set_Bit_From_Debounced(&btn_db[9], buttons[9].active_low, &PAYLOAD_MESSAGE, PAYLOAD_BIT_FOCUS_IN);  // RS3 - UP
			Set_Bit_From_Debounced(&btn_db[8], buttons[8].active_low, &PAYLOAD_MESSAGE, PAYLOAD_BIT_FOCUS_OUT); // RS3 - DOWN
        }

        if(menu_register & (1<<1)){
			Set_Bit_From_Debounced(&btn_db[4], buttons[4].active_low, &PAYLOAD_MESSAGE, PAYLOAD_BIT_IMAGE_SENSOR_CHANGE); //RS5 - UP
			Set_Bit_From_Debounced(&btn_db[6], buttons[6].active_low, &PAYLOAD_MESSAGE, PAYLOAD_BIT_IR_POLARITY);         // RS5 - DOWN
			Poll_Debounced(&btn_db[7], buttons[7].active_low, onNearInfraredToggle_Button_Press);                        // RS4 - UP
			Set_Bit_From_Debounced(&btn_db[8], buttons[8].active_low, &PAYLOAD_MESSAGE, PAYLOAD_BIT_IR_DZOOM_PLUS);       // RS3 - UP
			Set_Bit_From_Debounced(&btn_db[9], buttons[9].active_low, &PAYLOAD_MESSAGE, PAYLOAD_BIT_IR_DZOOM_MINUS);      // RS3 - DOWN
		}

        if(menu_register & (1<<2)){
        	Poll_Debounced(&btn_db[4], buttons[4].active_low, onTrackingSourceToggle_Button_Press);     // RS5 - UP
        	Poll_Debounced(&btn_db[6], buttons[6].active_low, onTRACKING_START_STOP_Button_Press);      // RS5 - DOWN
        	Poll_Debounced(&btn_db[8], buttons[8].active_low, onTrackingTemplateToggle_Button_Press);   // RS3 - UP
        	Poll_Debounced(&btn_db[7], buttons[7].active_low, onAI_TRACKING_Button_Press);              // RS4 - UP

		}

        if(menu_register & (1<<3)){
        	Poll_Debounced(&btn_db[5], buttons[5].active_low, onLASERCONT_Button_Press);   // RS4 - DOWN
			Poll_Debounced(&btn_db[7], buttons[7].active_low, onLASERSINGLE_Button_Press); // RS4 - UP
			Set_Bit_From_Debounced(&btn_db[9], buttons[9].active_low, &PAYLOAD_MESSAGE, PAYLOAD_BIT_LASER_ZOOM_IN);  // RS3 - UP
			Set_Bit_From_Debounced(&btn_db[8], buttons[8].active_low, &PAYLOAD_MESSAGE, PAYLOAD_BIT_LASER_ZOOM_OUT); // RS3 - DOWN
			Poll_Debounced(&btn_db[4], buttons[4].active_low, onLASER_Button_Press); // RS3 - UP
		}

        if(menu_register & (1<<4)){
        	Set_Bit_From_Debounced(&btn_db[4], buttons[4].active_low, &PAYLOAD_MESSAGE, PAYLOAD_BIT_TAKE_PICTURE); // RS5 - UP
        	Poll_Debounced(&btn_db[6], buttons[6].active_low, onRECORD_Button_Press);                              // RS5 - DOWN
        	Set_Bit_From_Debounced(&btn_db[5], buttons[5].active_low, &PAYLOAD_MESSAGE, PAYLOAD_BIT_PIC_RECORD_MODE_TOGGLE); // RS4 - DOWN
        	Set_Bit_From_Debounced(&btn_db[7], buttons[7].active_low, &PAYLOAD_MESSAGE, PAYLOAD_BIT_MOTOR_ON_OFF);           // RS4 - UP
        	Poll_Debounced(&btn_db[8], buttons[8].active_low, onStropesToggle_Button_Press);   // RS3 - UP

		}

        if(menu_register & (1<<5)){
        	Poll_Debounced(&btn_db[4], buttons[4].active_low, onVIDEOIP_Button_Press);      // RS5 - UP
        	Poll_Debounced(&btn_db[6], buttons[6].active_low, onEOImageToggle_Button_Press); // RS5 - DOWN
        	Poll_Debounced(&btn_db[5], buttons[5].active_low, onEODzoomToggle_Button_Press); // RS4 - DOWN
        	Poll_Debounced(&btn_db[8], buttons[8].active_low, onTrackingTemplateToggle_Button_Press);   // RS3 - UP
		}
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
        /* Silent unless connected or under test. In WAITING, SYNCED and
           LOST nothing is listening — or nothing has completed the
           handshake — so there is no reason to fill the link. */
        if (Fsm_Should_Transmit() &&
            (HAL_GetTick() - last_usb_send) >= BUTTON_FRAME_PERIOD_MS)
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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 4;
  hadc3.Init.DMAContinuousRequests = ENABLE;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
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

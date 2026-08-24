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
#define BIT_IR_POLARITY         6
#define BIT_IMAGE_SENSOR_CHANGE 7
#define BIT_SPEED_UP            8
#define BIT_SPEED_DOWN          9
#define BIT_ZOOM_IN             10
#define BIT_ZOOM_OUT            11
#define BIT_WIDE_IN             12
#define BIT_WIDE_OUT            13
#define BIT_FOCUS_IN            14
#define BIT_FOCUS_OUT           15
#define BIT_VIDEO_IP            21
#define BIT_LASER_ON_OFF        22
#define BIT_LASER_CONT_MODE     24
#define BIT_LASER_SINGLE_MODE   25
#define BIT_TRCKING_START_STOP  26
#define BIT_AI_TRACKING_ON_OFF  27
#define BIT_JOYSTICK_TRACK      28
#define DEBOUNCE_MS  15

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
    { GPIO_PIN_SET,   GPIO_PIN_SET,   GPIO_PIN_SET,   0 }, /* 2: PD15 active_low  */
    { GPIO_PIN_SET,   GPIO_PIN_SET,   GPIO_PIN_SET,   0 }, /* 3: PC7  active_low  */
    { GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, 0 }, /* 4: PE10 active_high */
    { GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, 0 }, /* 5: PE12 active_high */
    { GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, 0 }, /* 6: PE14 active_high */
    { GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, 0 }, /* 7: PD11 active_high */
    { GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, 0 }, /* 8: PD12 active_high */
    { GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, 0 }, /* 9: PD13 active_high */
};
static debounce_state_t menuBtnDb = { GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, 0 }; /* PA6, pull-up */

static uint8_t rx_byte;
#define TLV_SYNC 0xAA
#define TLV_END  0x55
#define MASTER_BTN_PORT GPIOA
#define MASTER_BTN_PIN   GPIO_PIN_6
#define TLV_TYPE_BUTTON_STATE 0x01
#define TLV_TYPE_JOYSTICK     0x02
#define TLV_TYPE_JOYSTICK2    0x03
uint32_t USB_MESSAGE = 0x00;
uint8_t menu_register = 0b001;
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
    Debounce_Update(MASTER_BTN_PORT, MASTER_BTN_PIN, &menuBtnDb, now);
    for (int i = 0; i < 10; i++) {
        Debounce_Update(buttons[i].port, buttons[i].pin, &btn_db[i], now);
    }
}

static inline uint8_t Debounced_Is_Pressed(const debounce_state_t *db, uint8_t active_low)
{
    return active_low ? (db->stable == GPIO_PIN_RESET) : (db->stable == GPIO_PIN_SET);
}

/* Replaces set_bit_from_pin(): mirrors the debounced (not raw) level into
   USB_MESSAGE, so momentary/held buttons no longer flicker on bounce. */
static inline void Set_Bit_From_Debounced(debounce_state_t *db, uint8_t active_low, uint32_t bit)
{
    if (Debounced_Is_Pressed(db, active_low)) USB_MESSAGE |= (1UL << bit);
    else                                       USB_MESSAGE &= ~(1UL << bit);
}

void onARM_Button_Press(void)
{
    if (USB_MESSAGE & (1 << BIT_ARM)) USB_MESSAGE &= ~(1 << BIT_ARM);
    else                              USB_MESSAGE |= (1 << BIT_ARM);
}

void onMenuSelect_Button_Press(void)
{
    if (menu_register & (1 << 2)) menu_register = 0b001;
    else                              menu_register = menu_register << 1 ;
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
	if (USB_MESSAGE & (1 << BIT_AI_TRACKING_ON_OFF)) USB_MESSAGE &= ~(1 << BIT_AI_TRACKING_ON_OFF);
    if (USB_MESSAGE & (1 << BIT_JOYSTICK_TRACK)) USB_MESSAGE &= ~(1 << BIT_JOYSTICK_TRACK);
    else                                    USB_MESSAGE |= (1 << BIT_JOYSTICK_TRACK);
}


void onAI_TRACKING_Button_Press(void)
{
	if (USB_MESSAGE & (1 << BIT_JOYSTICK_TRACK)) USB_MESSAGE &= ~(1 << BIT_JOYSTICK_TRACK);
    if (USB_MESSAGE & (1 << BIT_AI_TRACKING_ON_OFF)) USB_MESSAGE &= ~(1 << BIT_AI_TRACKING_ON_OFF);
    else                                    USB_MESSAGE |= (1 << BIT_AI_TRACKING_ON_OFF);
}


void onTRACKING_START_STOP_Button_Press(void)
{
    if (USB_MESSAGE & (1 << BIT_TRCKING_START_STOP)) USB_MESSAGE &= ~(1 << BIT_TRCKING_START_STOP);
    else                                    USB_MESSAGE |= (1 << BIT_TRCKING_START_STOP);
}





void onLASERSINGLE_Button_Press(void)
{
	if (USB_MESSAGE & (1 << BIT_LASER_CONT_MODE)) USB_MESSAGE &= ~(1 << BIT_LASER_CONT_MODE);
    if (USB_MESSAGE & (1 << BIT_LASER_SINGLE_MODE)) USB_MESSAGE &= ~(1 << BIT_LASER_SINGLE_MODE);
    else                                    USB_MESSAGE |= (1 << BIT_LASER_SINGLE_MODE);
}



void onLASERCONT_Button_Press(void)
{
	if (USB_MESSAGE & (1 << BIT_LASER_SINGLE_MODE)) USB_MESSAGE &= ~(1 << BIT_LASER_SINGLE_MODE);
    if (USB_MESSAGE & (1 << BIT_LASER_CONT_MODE)) USB_MESSAGE &= ~(1 << BIT_LASER_CONT_MODE);
    else                                    USB_MESSAGE |= (1 << BIT_LASER_CONT_MODE);
}


void onLASER_Button_Press(void)
{
    if (USB_MESSAGE & (1 << BIT_LASER_ON_OFF)) USB_MESSAGE &= ~(1 << BIT_LASER_ON_OFF);
    else                                    USB_MESSAGE |= (1 << BIT_LASER_ON_OFF);
}



void onVIDEOIP_Button_Press(void)
{
    if (USB_MESSAGE & (1 << BIT_VIDEO_IP)) USB_MESSAGE &= ~(1 << BIT_VIDEO_IP);
    else                                    USB_MESSAGE |= (1 << BIT_VIDEO_IP);
}


void onFocusIn_Button_Press(void)
{
    if (USB_MESSAGE & (1 << BIT_FOCUS_IN)) USB_MESSAGE &= ~(1 << BIT_FOCUS_IN);
    else                                    USB_MESSAGE |= (1 << BIT_FOCUS_IN);
}

void onFocusOut_Button_Press(void)
{
    if (USB_MESSAGE & (1 << BIT_FOCUS_OUT)) USB_MESSAGE &= ~(1 << BIT_FOCUS_OUT);
    else                                      USB_MESSAGE |= (1 << BIT_FOCUS_OUT);
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

/* Menu-status indicator: exactly one LED lit for the active menu
   (menu_register is one-hot: 0b001 / 0b010 / 0b100).
   Reuses leds[0], leds[1], leds[3] — call AFTER Test_Loop() so it wins. */
static void Update_Menu_LEDs(void)
{
    HAL_GPIO_WritePin(leds[0].port, leds[0].pin, (menu_register & 0b001) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(leds[1].port, leds[1].pin, (menu_register & 0b010) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(leds[3].port, leds[3].pin, (menu_register & 0b100) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


/* Replaces poll_button(): fires onPress exactly once on the debounced
   idle->pressed transition. edge_last is tracked per physical pin (not
   per menu binding), so switching menus mid-press can't ghost-fire a
   different menu's handler, and multiple menus sharing one pin never
   fight over separate lastState variables the way the old code did.
   polarity: 1 = active-low (pull-up, press reads RESET), 0 = active-high (pull-down, press reads SET) */
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

static inline void Set_LED_Off(uint8_t led_idx)
{
    HAL_GPIO_WritePin(leds[led_idx].port, leds[led_idx].pin, GPIO_PIN_RESET);
}

/* Shows what each button currently DOES in USB_MESSAGE, per the active menu.
   leds[0]/[1]/[3] stay reserved for Update_Menu_LEDs(). leds[2] is currently
   unused (Auto-Land was removed) and free for whatever goes there next. */
static void Update_Button_LEDs(void)
{
    /* fixed pin->LED pairing across menus: E10=4  E12=5  E14=6  D11=7  D12=8  D13=9 */
    if (menu_register & (1 << 0))
    {
        Set_LED_From_Bit(7, BIT_WIDE_IN);      /* GPIOD11 - BLACK BTN */
        Set_LED_From_Bit(9, BIT_FOCUS_IN);     /* GPIOD13 - RED BTN   */
        Set_LED_From_Bit(6, BIT_ZOOM_OUT);     /* GPIOE14 - YELLOW BTN*/
        Set_LED_From_Bit(5, BIT_WIDE_OUT);     /* GPIOE12 - WHITE BTN */
        Set_LED_From_Bit(8, BIT_FOCUS_OUT);    /* GPIOD12 - BLUE BTN  */
        Set_LED_From_Bit(4, BIT_ZOOM_IN);      /* GPIOE10 - GREEN BTN */
    }
    else if (menu_register & (1 << 1))
    {
        Set_LED_From_Bit(6, BIT_VIDEO_IP);             /* GPIOE14 - Yellow BTN */
        Set_LED_From_Bit(4, BIT_IMAGE_SENSOR_CHANGE);  /* GPIOE10 - GREEN BTN  */
        Set_LED_From_Bit(5, BIT_IR_POLARITY);          /* GPIOE12 - WHITE BTN  */
        Set_LED_Off(7);
        Set_LED_Off(8);
        Set_LED_Off(9);
    }
    else if (menu_register & (1 << 2))
    {
        Set_LED_From_Bit(6, BIT_TRCKING_START_STOP);   /* GPIOE14 - YELLOW BTN */
        Set_LED_From_Bit(5, BIT_JOYSTICK_TRACK);       /* GPIOE12 - WHITE BTN  */
        Set_LED_From_Bit(8, BIT_AI_TRACKING_ON_OFF);   /* GPIOD12 - BLUE BTN   */
        Set_LED_From_Bit(4, BIT_LASER_ON_OFF);         /* GPIOE10 - GREEN BTN  */
        Set_LED_From_Bit(7, BIT_LASER_CONT_MODE);      /* GPIOD11 - BLACK BTN  */
        Set_LED_From_Bit(9, BIT_LASER_SINGLE_MODE);    /* GPIOD13 - BLUE BTN   */
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

  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  ws2812_init();
  uint16_t pot1 = 0, pot2 = 0, pot3 = 0, pot4 = 0;


  static uint32_t last_usb_send = 0;
  static uint16_t avg1 = 0;
  static uint16_t avg2 = 0;
  static uint16_t avg3 = 0;
  static uint16_t avg4 = 0;
  static uint8_t counter = 0;
  static uint8_t message_counter = 0;
  /* USER CODE END 2 */
  ws2812_pixel(2,0, 0, 255);
  ws2812_send_spi();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {

	    uint32_t now = HAL_GetTick();
        Debounce_Sample_All(now);   /* samples every physical pin, every iteration, no matter the menu */

        Update_Menu_LEDs();



        //pot1 += ADC_Read_Channel(ADC_CHANNEL_9);
        //pot2 += ADC_Read_Channel(ADC_CHANNEL_15);
        pot3 += ADC_Read_Channel(ADC_CHANNEL_6);
        pot4 += ADC_Read_Channel(ADC_CHANNEL_7);

        /* Persistent MODE buttons: toggle/latch on press, stay set until pressed again. */
        Poll_Debounced(&menuBtnDb, 1, onMenuSelect_Button_Press);   /* PA6, pull-up, active-low */

        /* MOMENTARY command buttons: bit mirrors the debounced pin level, set only while held. */
        /*                       btn_db[i]     active_low             bit */
        if(menu_register& (1<<0)){
        	/*Zoom*/
        	Set_Bit_From_Debounced(&btn_db[4], buttons[4].active_low, BIT_ZOOM_IN);   // GPIOE10 - GREEN BTN
        	Set_Bit_From_Debounced(&btn_db[6], buttons[6].active_low, BIT_ZOOM_OUT);  // GPIOE14 - YELLOW BTN


        	/*FOV*/
			Set_Bit_From_Debounced(&btn_db[7], buttons[7].active_low, BIT_WIDE_IN);   // GPIOD11 - BLACK BTN
			Set_Bit_From_Debounced(&btn_db[5], buttons[5].active_low, BIT_WIDE_OUT);  // GPIOE12 - WHITE BTN

			/*FOCUS*/
			Set_Bit_From_Debounced(&btn_db[9], buttons[9].active_low, BIT_FOCUS_IN);  // GPIOD13 - RED BTN
			Set_Bit_From_Debounced(&btn_db[8], buttons[8].active_low, BIT_FOCUS_OUT); // GPIOD12 - BLUE BTN

        }

        if(menu_register& (1<<1)){
        	/*Video*/
        	Poll_Debounced(&btn_db[6], buttons[6].active_low, onVIDEOIP_Button_Press);          // GPIOE14 - Yellow BTN
			Set_Bit_From_Debounced(&btn_db[4], buttons[4].active_low, BIT_IMAGE_SENSOR_CHANGE); // GPIOE10 - GREEN BTN
			Set_Bit_From_Debounced(&btn_db[5], buttons[5].active_low, BIT_IR_POLARITY);         // GPIOE12 - WHITE BTN
		}

        if(menu_register& (1<<2)){
        	/*Tracking*/
        	Poll_Debounced(&btn_db[6], buttons[6].active_low, onTRACKING_START_STOP_Button_Press); // GPIOE14 - YELLOW BTN
        	Poll_Debounced(&btn_db[5], buttons[5].active_low, onAI_JOYSTICK_TRACK_Button_Press);    // GPIOE12 - WHITE BTN
        	Poll_Debounced(&btn_db[8], buttons[8].active_low, onAI_TRACKING_Button_Press);          // GPIOD12 - BLUE BTN

        	/*Laser*/
        	Poll_Debounced(&btn_db[4], buttons[4].active_low, onLASER_Button_Press);                // GPIOE10 - GREEN BTN
			Poll_Debounced(&btn_db[7], buttons[7].active_low, onLASERCONT_Button_Press);            // GPIOD11 - BLACK BTN
			Poll_Debounced(&btn_db[9], buttons[9].active_low, onLASERSINGLE_Button_Press);          // GPIOD13 - BLUE BTN
		}
        Update_Button_LEDs();


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

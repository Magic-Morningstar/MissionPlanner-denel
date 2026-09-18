/*
 * fastLed_SPI.h
 *
 * WS2812/PL9823 driver via SPI's shift register as a waveform generator.
 * Blocking HAL_SPI_Transmit — simple, no DMA, ties up the CPU for the
 * duration of one send (~20 bytes/LED, well under 1ms total even for a
 * dozen LEDs). Requires SPI1 configured for 6.75MHz (Prescaler /16 off
 * this board's 108MHz APB2 clock) — see MX_SPI1_Init() in main.c.
 *
 * Above the raw driver sits a small animation layer. Every indicator on
 * this panel uses the same vocabulary of behaviours — solid, breathe,
 * blink, double flash — and differs only in colour and in what condition
 * selects it. So the behaviours and their rates live here, once, and
 * each LED is simply told which one to run.
 *
 * What is NOT here: the conditions. Nothing in this file knows what a
 * menu is or what "link lost" means. Callers decide that and call
 * ws2812_set(); this file only knows how to make light do things.
 */

#ifndef INC_FASTLED_SPI_H_
#define INC_FASTLED_SPI_H_

#define WS2812_NUM_LEDS     4   /* change to match your actual strip/count */
#define WS2812_SPI_HANDLE   hspi1

#define WS2812_SPI_0        0x80   /* short high pulse = logic 0 */
#define WS2812_SPI_1        0xFC   /* long high pulse  = logic 1 */

#define WS2812_RESET_BYTES  50
#define WS2812_BUFFER_SIZE  (WS2812_NUM_LEDS * 24 + WS2812_RESET_BYTES)

extern SPI_HandleTypeDef WS2812_SPI_HANDLE;
extern uint8_t ws2812_buffer[];

void ws2812_init(void);
void ws2812_send_spi(void);
void ws2812_pixel(uint16_t led_no, uint8_t r, uint8_t g, uint8_t b);
void ws2812_pixel_all(uint8_t r, uint8_t g, uint8_t b);


/* ── Pattern timing ───────────────────────────────────────────────────────
   All animation rates live here. Change a number and every indicator
   using that pattern follows — there is no second copy anywhere. */

#define WS2812_BREATHE_SLOW_PERIOD_MS   2000U   /* 0.5 Hz */
#define WS2812_BREATHE_FAST_PERIOD_MS    800U   /* 1.25 Hz */
#define WS2812_BLINK_SLOW_PERIOD_MS     1000U   /* 1 Hz */
#define WS2812_BLINK_FAST_PERIOD_MS      250U   /* 4 Hz */

#define WS2812_DOUBLE_FLASH_PERIOD_MS   1200U
#define WS2812_DOUBLE_FLASH_PULSE_MS     110U
#define WS2812_DOUBLE_FLASH_GAP_MS       110U

/* Suggested scale for a "present but not active" indicator, e.g. the
   inactive entries in a menu gauge. */
#define WS2812_SCALE_DIM                  32U


/* ── Patterns ─────────────────────────────────────────────────────────────
   The shared behaviour vocabulary. Every indicator uses these; they
   differ only in colour and in what condition selects them. */
typedef enum {
    WS2812_PATTERN_OFF,
    WS2812_PATTERN_SOLID,
    WS2812_PATTERN_BREATHE_SLOW,
    WS2812_PATTERN_BREATHE_FAST,
    WS2812_PATTERN_BLINK_SLOW,
    WS2812_PATTERN_BLINK_FAST,
    WS2812_PATTERN_DOUBLE_FLASH,
} ws2812_pattern_t;

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} ws2812_color_t;

typedef struct {
    ws2812_pattern_t pattern;
    ws2812_color_t   color;
    uint8_t          scale;   /* 0-255 fixed dimming, applied on top of the pattern */
} ws2812_led_state_t;


/* ── Palette ──────────────────────────────────────────────────────────────
   Full-brightness values. Dimming is a separate concern — pass a scale to
   ws2812_set_dim() rather than defining a second, darker colour. */
extern const ws2812_color_t WS2812_BLACK;
extern const ws2812_color_t WS2812_WHITE;
extern const ws2812_color_t WS2812_RED;
extern const ws2812_color_t WS2812_GREEN;
extern const ws2812_color_t WS2812_BLUE;
extern const ws2812_color_t WS2812_AMBER;
extern const ws2812_color_t WS2812_TEAL;
extern const ws2812_color_t WS2812_CYAN;
extern const ws2812_color_t WS2812_VIOLET;
extern const ws2812_color_t WS2812_ORANGE;


/* ── Animation API ────────────────────────────────────────────────────────
   Set what an LED should be doing, then call ws2812_animate() on a timer.
   ws2812_set* is cheap and idempotent — calling it every frame with the
   same values is fine, and is usually the simplest way to write a caller.

   Note this is a separate world from ws2812_pixel(): that writes the
   strip buffer directly and will be overwritten by the next
   ws2812_animate(). Use one or the other per LED, not both. */

void    ws2812_set(uint16_t led_no, ws2812_pattern_t pattern, ws2812_color_t color);
void    ws2812_set_dim(uint16_t led_no, ws2812_pattern_t pattern,
                       ws2812_color_t color, uint8_t scale);
void    ws2812_off(uint16_t led_no);

/* Recomputes every LED for the given tick and pushes the strip. Call from
   the main loop, not an ISR — the push is a blocking SPI transmit. */
void    ws2812_animate(uint32_t now);

/* Exposed mainly for testing a pattern's shape without a strip attached. */
uint8_t ws2812_pattern_level(ws2812_pattern_t pattern, uint32_t now);

#endif /* INC_FASTLED_SPI_H_ */
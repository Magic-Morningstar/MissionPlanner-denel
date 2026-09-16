/*
 * fastLed_SPI.c
 *
 * Low-level WS2812 bit-banging over SPI, plus a small animation layer.
 *
 * The animation layer exists because every indicator on this panel uses
 * the same vocabulary of behaviours — solid, breathe, blink, double
 * flash — and only differs in which colour it shows and what condition
 * puts it in that state. So the behaviours live here, once, and each
 * LED just gets told which one to run.
 *
 * What is NOT here: the conditions. Nothing in this file knows what a
 * menu is, or what "link lost" means. Callers decide that and call
 * ws2812_set(); this file only knows how to make light do things.
 */

#include <string.h>
#include "main.h"
#include "fastLed_SPI.h"

uint8_t ws2812_buffer[WS2812_BUFFER_SIZE];

/* What each LED is currently doing. Sized to the strip; ws2812_animate()
   walks this every frame and recomputes brightness from the timebase. */
static ws2812_led_state_t ws2812_state[WS2812_NUM_LEDS];

/* ── Named colours ────────────────────────────────────────────────────────
   Full-brightness values. Dimming is a separate concern — pass a scale to
   ws2812_set_dim() rather than defining a second, darker colour. */
const ws2812_color_t WS2812_BLACK = {   0,   0,   0 };
const ws2812_color_t WS2812_WHITE = { 255, 255, 255 };
const ws2812_color_t WS2812_RED   = { 255,   0,   0 };
const ws2812_color_t WS2812_GREEN = {   0, 255,   0 };
const ws2812_color_t WS2812_BLUE  = {   0,   0, 255 };
const ws2812_color_t WS2812_AMBER = { 255, 140,   0 };

/* Extra colours, here so menu indicators and anything else added later
   draw from the same palette instead of hardcoding triples at the call
   site. */
const ws2812_color_t WS2812_TEAL    = {   0, 255, 180 };
const ws2812_color_t WS2812_CYAN    = {   0, 120, 255 };
const ws2812_color_t WS2812_VIOLET  = { 180,   0, 255 };
const ws2812_color_t WS2812_ORANGE  = { 255, 120,   0 };


void ws2812_init(void) {
    memset(ws2812_buffer, 0, WS2812_BUFFER_SIZE);  /* reset-padding region wants raw 0x00 — correct as-is */
    ws2812_pixel_all(0, 0, 0);                      /* color region needs the real "off" encoding (0x80 per
                                                         bit, a short-high pulse), not raw 0x00 — a true 0x00
                                                         byte has no pulse at all, which isn't a valid logic-0
                                                         bit per the protocol. */

    for (uint16_t i = 0; i < WS2812_NUM_LEDS; ++i) {
        ws2812_state[i].pattern = WS2812_PATTERN_OFF;
        ws2812_state[i].color   = WS2812_BLACK;
        ws2812_state[i].scale   = 255;
    }

    ws2812_send_spi();
}

void ws2812_send_spi(void) {
    /* Blocking. At WS2812_NUM_LEDS = 7 that's 168 colour bytes plus reset
       padding; at a typical 2.4 MHz SPI clock it's well under a
       millisecond, so running this on a 25 ms animation tick is fine. If
       the strip grows a lot, switch to HAL_SPI_Transmit_DMA and skip the
       frame if the previous transfer is still in flight. */
    HAL_SPI_Transmit(&WS2812_SPI_HANDLE, ws2812_buffer, WS2812_BUFFER_SIZE, HAL_MAX_DELAY);
}

#define WS2812_FILL_BUFFER(COLOR)                        \
    do {                                                 \
        for( uint8_t mask = 0x80; mask; mask >>= 1 ) {   \
            if( (COLOR) & mask ) {                       \
                *ptr++ = WS2812_SPI_1;                   \
            } else {                                     \
                *ptr++ = WS2812_SPI_0;                   \
            }                                            \
        }                                                \
    } while(0)

void ws2812_pixel(uint16_t led_no, uint8_t r, uint8_t g, uint8_t b) {
    if (led_no >= WS2812_NUM_LEDS) return;
    uint8_t * ptr = &ws2812_buffer[24 * led_no];
    WS2812_FILL_BUFFER(g);   /* wire order is G, R, B, not R,G,B */
    WS2812_FILL_BUFFER(r);
    WS2812_FILL_BUFFER(b);
}

void ws2812_pixel_all(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t * ptr = ws2812_buffer;
    for( uint16_t i = 0; i < WS2812_NUM_LEDS; ++i) {
        WS2812_FILL_BUFFER(g);
        WS2812_FILL_BUFFER(r);
        WS2812_FILL_BUFFER(b);
    }
}

/* ── Pattern engine ───────────────────────────────────────────────────────
   Every pattern reduces to one number: a 0-255 brightness level for the
   current instant. The colour is then scaled by it. That's why adding a
   pattern means adding one case here and nothing else — no per-LED state
   machine, no phase tracking, no timers.

   All patterns key off absolute tick rather than a per-LED start time, so
   LEDs running the same pattern stay in phase with each other. If you
   ever want them deliberately out of phase, add an offset field to
   ws2812_led_state_t and subtract it from `now`. */

/* 0-255 triangle wave over `period_ms`. A triangle rather than a sine
   because it costs nothing and the eye can't tell at these rates. */
static uint8_t ws2812_breathe_level(uint32_t now, uint32_t period_ms) {
    uint32_t phase = now % period_ms;
    uint32_t half  = period_ms / 2;
    if (phase < half) return (uint8_t)((phase * 255U) / half);
    return (uint8_t)(((period_ms - phase) * 255U) / half);
}

/* Full on for `on_ms` at the start of each `period_ms`, off after. */
static uint8_t ws2812_blink_level(uint32_t now, uint32_t period_ms, uint32_t on_ms) {
    return ((now % period_ms) < on_ms) ? 255U : 0U;
}

/* Two short pulses, then a long gap. */
static uint8_t ws2812_double_flash_level(uint32_t now) {
    uint32_t phase = now % WS2812_DOUBLE_FLASH_PERIOD_MS;
    if (phase < WS2812_DOUBLE_FLASH_PULSE_MS) {
        return 255U;
    }
    uint32_t second = WS2812_DOUBLE_FLASH_PULSE_MS + WS2812_DOUBLE_FLASH_GAP_MS;
    if (phase >= second && phase < second + WS2812_DOUBLE_FLASH_PULSE_MS) {
        return 255U;
    }
    return 0U;
}

uint8_t ws2812_pattern_level(ws2812_pattern_t pattern, uint32_t now) {
    switch (pattern) {
    case WS2812_PATTERN_OFF:
        return 0U;

    case WS2812_PATTERN_SOLID:
        return 255U;

    case WS2812_PATTERN_BREATHE_SLOW:
        return ws2812_breathe_level(now, WS2812_BREATHE_SLOW_PERIOD_MS);

    case WS2812_PATTERN_BREATHE_FAST:
        return ws2812_breathe_level(now, WS2812_BREATHE_FAST_PERIOD_MS);

    case WS2812_PATTERN_BLINK_SLOW:
        return ws2812_blink_level(now, WS2812_BLINK_SLOW_PERIOD_MS,
                                       WS2812_BLINK_SLOW_PERIOD_MS / 2U);

    case WS2812_PATTERN_BLINK_FAST:
        return ws2812_blink_level(now, WS2812_BLINK_FAST_PERIOD_MS,
                                       WS2812_BLINK_FAST_PERIOD_MS / 2U);

    case WS2812_PATTERN_DOUBLE_FLASH:
        return ws2812_double_flash_level(now);
    }
    return 0U;
}

void ws2812_set_dim(uint16_t led_no, ws2812_pattern_t pattern,
                    ws2812_color_t color, uint8_t scale) {
    if (led_no >= WS2812_NUM_LEDS) return;
    ws2812_state[led_no].pattern = pattern;
    ws2812_state[led_no].color   = color;
    ws2812_state[led_no].scale   = scale;
}

void ws2812_set(uint16_t led_no, ws2812_pattern_t pattern, ws2812_color_t color) {
    ws2812_set_dim(led_no, pattern, color, 255U);
}

void ws2812_off(uint16_t led_no) {
    ws2812_set_dim(led_no, WS2812_PATTERN_OFF, WS2812_BLACK, 255U);
}

void ws2812_animate(uint32_t now) {
    for (uint16_t i = 0; i < WS2812_NUM_LEDS; ++i) {
        const ws2812_led_state_t *st = &ws2812_state[i];

        /* Two multiplies, not one: `level` is the pattern's instantaneous
           brightness and `scale` is the LED's fixed dimming. Keeping them
           separate is what lets a dimmed LED still breathe or blink
           properly instead of just sitting at a lower constant. */
        uint16_t level = ws2812_pattern_level(st->pattern, now);
        uint16_t gain  = (level * (uint16_t)st->scale) / 255U;

        ws2812_pixel(i,
                     (uint8_t)((st->color.r * gain) / 255U),
                     (uint8_t)((st->color.g * gain) / 255U),
                     (uint8_t)((st->color.b * gain) / 255U));
    }
    ws2812_send_spi();
}
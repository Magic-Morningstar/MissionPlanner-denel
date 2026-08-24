/*
 * fastLed_SPI.c
 */

#include <string.h>
#include "main.h"
#include "fastLed_SPI.h"

uint8_t ws2812_buffer[WS2812_BUFFER_SIZE];

void ws2812_init(void) {
    memset(ws2812_buffer, 0, WS2812_BUFFER_SIZE);  /* reset-padding region wants raw 0x00 — correct as-is */
    ws2812_pixel_all(0, 0, 0);                      /* color region needs the real "off" encoding (0x80 per
                                                         bit, a short-high pulse), not raw 0x00 — a true 0x00
                                                         byte has no pulse at all, which isn't a valid logic-0
                                                         bit per the protocol. */
    ws2812_send_spi();
}

void ws2812_send_spi(void) {
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

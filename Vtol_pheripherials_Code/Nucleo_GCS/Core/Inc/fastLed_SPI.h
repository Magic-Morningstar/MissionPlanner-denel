/*
 * fastLed_SPI.h
 *
 * WS2812/PL9823 driver via SPI's shift register as a waveform generator.
 * Blocking HAL_SPI_Transmit — simple, no DMA, ties up the CPU for the
 * duration of one send (~20 bytes/LED, well under 1ms total even for a
 * dozen LEDs). Requires SPI1 configured for 6.75MHz (Prescaler /16 off
 * this board's 108MHz APB2 clock) — see MX_SPI1_Init() in main.c.
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

#endif /* INC_FASTLED_SPI_H_ */

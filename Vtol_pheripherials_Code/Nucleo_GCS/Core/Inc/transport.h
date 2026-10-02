/*
 * transport.h
 *
 * One interface over the byte-stream links on the F767ZI: USART and
 * USB CDC. Any combination can be live at once — UART only, USB only,
 * or both. Every frame is sent on every enabled link, and bytes received
 * on any enabled link are delivered tagged with which link they came
 * from.
 *
 * Which links are enabled is chosen at startup by main() — see the
 * LINK_USE_* flags in main.c's USER CODE PD section. That's a runtime
 * choice passed to transport_init(), not a define in this file, because
 * main.c's defines don't reach this translation unit.
 *
 * ── Compile-time availability ────────────────────────────────────────
 * The TRANSPORT_ENABLE_* flags below are a different thing: they say
 * whether the middleware for a link exists in the project at all.
 *
 *   UART      nothing extra — USART2 is already configured. Receive
 *             also needs the USART2 global interrupt enabled in NVIC,
 *             or HAL_UART_Receive_IT() arms and never fires.
 *   USB_CDC   USB_OTG_FS in Device_Only mode + the CDC class; provides
 *             usbd_cdc_if.h and CDC_Transmit_FS()
 *
 * Asking main.c to use a link whose middleware is compiled out is a
 * compile error there, which is the intent.
 */

#ifndef INC_TRANSPORT_H_
#define INC_TRANSPORT_H_

#include <stdint.h>

/* ── Which links are built in ─────────────────────────────────────────── */
#define TRANSPORT_ENABLE_UART       1
#define TRANSPORT_ENABLE_USB_CDC    1

typedef enum {
    TRANSPORT_UART = 0,
    TRANSPORT_USB_CDC,
    TRANSPORT_COUNT
} transport_id_t;

/* Bit for one link in a mask passed to transport_init(). */
#define TRANSPORT_MASK(id)          (1U << (id))

/* ── Per-transport settings ───────────────────────────────────────────── */

/* UART. Must match BAUDRATE in the PC's system_config.py. */
#define TRANSPORT_UART_HANDLE       huart2

/* USB CDC. How long a send will wait for a previous IN transfer to
   finish before dropping the frame. Dropping is correct: panel state is
   resent every 10 ms, so a stale frame is worth less than a stalled main
   loop. Only applies once the host has enumerated the device — before
   that, USB sends return immediately (see transport_usb_ready()). */
#define TRANSPORT_USB_TX_TIMEOUT_MS 5

/* ── Interface ────────────────────────────────────────────────────────── */

/* Called once per received byte, from interrupt context, with the link
   it arrived on. The caller must keep separate parser state per link:
   two links feeding one byte-level state machine interleave their
   streams and neither ever completes a valid frame. */
typedef void (*transport_rx_fn)(transport_id_t link, uint8_t byte);

void transport_set_rx_handler(transport_rx_fn fn);

/* Brings up every link in `link_mask` (built from TRANSPORT_MASK()).
   Links not in the mask are neither sent on nor listened to. Returns the
   mask of links actually enabled — a requested link whose middleware is
   compiled out is silently dropped from it. */
uint8_t transport_init(uint8_t link_mask);

/* Sends one complete frame on every enabled link. Returns the mask of
   links it went out on. Treat a missing bit as "dropped, try next tick"
   rather than an error — every frame this panel sends is periodic. */
uint8_t transport_send(const uint8_t *data, uint16_t len);

/* Service call for any link that needs it. Safe to call every loop. */
void transport_poll(void);

uint8_t transport_enabled_mask(void);

/* True once the USB host has enumerated and configured the device. Until
   then there is nothing to send to, and CDC_Transmit_FS() would
   dereference a NULL class-data pointer. */
uint8_t transport_usb_ready(void);

/* Called from CDC_Receive_FS() in usbd_cdc_if.c. Defined unconditionally:
   once the CDC class exists, that generated file always references it,
   and it can't be wrapped in a guard. With USB compiled out or disabled
   it discards the bytes. */
void transport_usb_on_rx(uint8_t *buf, uint32_t len);

#endif /* INC_TRANSPORT_H_ */

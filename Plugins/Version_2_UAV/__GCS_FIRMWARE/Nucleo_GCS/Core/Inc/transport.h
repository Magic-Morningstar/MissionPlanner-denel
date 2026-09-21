/*
 * transport.h
 *
 * One interface over the three byte-stream links on the F767ZI: USART,
 * USB CDC, and UDP over Ethernet. Change TRANSPORT_ACTIVE, rebuild, and
 * the whole panel talks over a different wire. Nothing above this file
 * changes — TLV_Send() doesn't know or care which one is live, and the
 * PC side is entirely unaffected because the frames are byte-identical.
 *
 * Deliberately NOT here: SBUS. The other three carry arbitrary bytes, so
 * a TLV frame passes through untouched. SBUS is a fixed 25-byte frame of
 * 16x11-bit channels at 100k 8E2 inverted, with no payload field to put
 * a frame in. It can't sit behind send(data, len) without lying about
 * what it is. If the panel should drive a flight controller directly,
 * that's a second output with its own interface, mapping slots and pots
 * onto channels — not a swap-in for this one.
 *
 * ── Enabling a transport ──────────────────────────────────────────────
 * Each implementation is compiled out unless its ENABLE flag is 1,
 * because each needs CubeMX middleware that may not be in your project:
 *
 *   UART      nothing extra — USART2 is already configured
 *   USB_CDC   USB_OTG_FS in Device_Only mode + the CDC class; provides
 *             usbd_cdc_if.h and CDC_Transmit_FS()
 *   ETHERNET  ETH peripheral + LwIP with UDP enabled; provides lwip.h
 *             and MX_LWIP_Init()/MX_LWIP_Process()
 *
 * Turning a flag on without the middleware present is a compile error,
 * which is the intent — better than a link that silently does nothing.
 */

#ifndef INC_TRANSPORT_H_
#define INC_TRANSPORT_H_

#include <stdint.h>

/* ── Which links are built in ─────────────────────────────────────────── */
#define TRANSPORT_ENABLE_UART       1
#define TRANSPORT_ENABLE_USB_CDC    0
#define TRANSPORT_ENABLE_ETHERNET   0

typedef enum {
    TRANSPORT_UART = 0,
    TRANSPORT_USB_CDC,
    TRANSPORT_ETHERNET,
    TRANSPORT_COUNT
} transport_id_t;

/* ── THE VALUE YOU CHANGE ─────────────────────────────────────────────── */
#define TRANSPORT_ACTIVE   TRANSPORT_UART

/* ── Per-transport settings ───────────────────────────────────────────── */

/* UART. Must match BAUDRATE in the PC's system_config.py. */
#define TRANSPORT_UART_HANDLE       huart2

/* USB CDC. The host sets the line rate and the device ignores it, so
   there's no baud to match. This is how long TLV_Send() will wait for a
   previous IN transfer to finish before giving up on a frame. Dropping
   one is correct here: panel state is resent every 10 ms anyway, so a
   stale frame is worth less than a stalled main loop. */
#define TRANSPORT_USB_TX_TIMEOUT_MS 5

/* UDP over Ethernet. The board's address comes from MX_LWIP_Init()
   (static or DHCP, set in CubeMX). These are the peer and the ports. */
#define TRANSPORT_UDP_PEER_IP0      192
#define TRANSPORT_UDP_PEER_IP1      168
#define TRANSPORT_UDP_PEER_IP2        1
#define TRANSPORT_UDP_PEER_IP3      100
#define TRANSPORT_UDP_PEER_PORT     14555   /* PC listens here  */
#define TRANSPORT_UDP_LOCAL_PORT    14556   /* board listens here */

/* ── Interface ────────────────────────────────────────────────────────── */

/* Called once per received byte, from whichever transport is live. Set
   this before transport_init(). Keeping the frame parser out of this
   file is what lets a transport be added without touching the protocol,
   and vice versa. */
typedef void (*transport_rx_fn)(uint8_t byte);

void transport_set_rx_handler(transport_rx_fn fn);

/* Brings up the active transport. Returns 1 on success. */
uint8_t transport_init(void);

/* Sends one complete frame. Returns 1 if it went out. Callers should
   treat 0 as "dropped, try again next tick" rather than an error — every
   frame this panel sends is periodic and idempotent. */
uint8_t transport_send(const uint8_t *data, uint16_t len);

/* Service call for transports that need it (Ethernet does; UART and USB
   CDC are interrupt-driven and this is a no-op for them). Safe and cheap
   to call every main-loop iteration regardless of which is active. */
void transport_poll(void);

/* Swaps transports at runtime. Only works for links compiled in, and
   returns 0 otherwise. Mostly useful for a diagnostic build that tries
   one and falls back — the normal path is to set TRANSPORT_ACTIVE and
   rebuild. */
uint8_t transport_select(transport_id_t id);

transport_id_t transport_active(void);
const char    *transport_name(void);

#endif /* INC_TRANSPORT_H_ */

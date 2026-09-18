/*
 * transport.c
 *
 * Three implementations of the same three-function shape (init, send,
 * poll), dispatched through a table. See transport.h for how to pick one
 * and what middleware each needs.
 */

#include <string.h>
#include "main.h"
#include "transport.h"

#if TRANSPORT_ENABLE_USB_CDC
#include "usbd_cdc_if.h"
#endif

#if TRANSPORT_ENABLE_ETHERNET
#include "lwip.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#endif

/* Set by transport_set_rx_handler(). Every transport funnels received
   bytes through this one pointer, so the frame parser is written once
   and works over any link. */
static transport_rx_fn rx_handler = 0;

static inline void deliver(uint8_t byte)
{
    if (rx_handler) rx_handler(byte);
}

/* Each implementation fills one of these. */
typedef struct {
    const char *name;
    uint8_t (*init)(void);
    uint8_t (*send)(const uint8_t *data, uint16_t len);
    void    (*poll)(void);
} transport_vtable_t;


/* ── UART ─────────────────────────────────────────────────────────────────
   Interrupt-driven single-byte receive, blocking transmit.

   The blocking transmit is the one cost worth knowing: at 115200 a
   15-byte frame stalls the main loop ~1.3 ms, which also delays the next
   debounce sample. Moving to HAL_UART_Transmit_IT would fix it, but the
   caller's ping-pong frame buffer has to stay valid until the transfer
   completes — that's why it's a ping-pong in the first place. */
#if TRANSPORT_ENABLE_UART

extern UART_HandleTypeDef TRANSPORT_UART_HANDLE;

static uint8_t uart_rx_byte;
static volatile uint8_t uart_rearm_pending = 0;

static uint8_t uart_init(void)
{
    uart_rearm_pending = 0;
    return (HAL_UART_Receive_IT(&TRANSPORT_UART_HANDLE, &uart_rx_byte, 1) == HAL_OK);
}

static uint8_t uart_send(const uint8_t *data, uint16_t len)
{
    return (HAL_UART_Transmit(&TRANSPORT_UART_HANDLE, (uint8_t *)data, len,
                              HAL_MAX_DELAY) == HAL_OK);
}

static void uart_poll(void)
{
    /* Re-arm outside the ISR if the in-ISR attempt failed. Retrying in
       the callback risks recursing into the HAL while it's still
       unwinding the previous completion. */
    if (uart_rearm_pending)
    {
        if (HAL_UART_Receive_IT(&TRANSPORT_UART_HANDLE, &uart_rx_byte, 1) == HAL_OK)
        {
            uart_rearm_pending = 0;
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == TRANSPORT_UART_HANDLE.Instance)
    {
        deliver(uart_rx_byte);
        if (HAL_UART_Receive_IT(&TRANSPORT_UART_HANDLE, &uart_rx_byte, 1) != HAL_OK)
        {
            uart_rearm_pending = 1;
        }
    }
}

static const transport_vtable_t vt_uart = { "uart", uart_init, uart_send, uart_poll };

#endif /* TRANSPORT_ENABLE_UART */


/* ── USB CDC ──────────────────────────────────────────────────────────────
   CDC_Transmit_FS() returns USBD_BUSY while the previous IN transfer is
   still in flight, and keeps returning it forever if the host hasn't
   enumerated the device. So the retry is bounded: a dropped frame costs
   nothing (panel state resends in 10 ms), a stalled main loop costs
   debounce timing and the LED animation. */
#if TRANSPORT_ENABLE_USB_CDC

static uint8_t usb_init(void)
{
    /* MX_USB_DEVICE_Init() runs from main()'s generated init section.
       Receive is push-based: CDC_Receive_FS() in usbd_cdc_if.c calls
       transport_usb_on_rx() — see the note at the bottom of this file
       for the one line that needs adding there. */
    return 1;
}

static uint8_t usb_send(const uint8_t *data, uint16_t len)
{
    uint32_t deadline = HAL_GetTick() + TRANSPORT_USB_TX_TIMEOUT_MS;

    while (CDC_Transmit_FS((uint8_t *)data, len) == USBD_BUSY)
    {
        if (HAL_GetTick() >= deadline) return 0;   /* drop it, not worth stalling */
    }
    return 1;
}

static void usb_poll(void) { }

/* Called from CDC_Receive_FS(). */
void transport_usb_on_rx(uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) deliver(buf[i]);
}

static const transport_vtable_t vt_usb = { "usb_cdc", usb_init, usb_send, usb_poll };

#endif /* TRANSPORT_ENABLE_USB_CDC */


/* ── UDP over Ethernet ────────────────────────────────────────────────────
   Raw LwIP UDP, no sockets, so no RTOS needed.

   One property worth being deliberate about: UDP can reorder and drop.
   That's fine here and needs no retry layer, because every frame in this
   protocol is a complete current-state snapshot rather than a delta. A
   lost PANEL_STATE is replaced 10 ms later; a lost LED_STATE is replaced
   by the 200 ms heartbeat. An older frame arriving after a newer one
   would briefly show stale state, which on a link this short is not
   worth a sequence number. */
#if TRANSPORT_ENABLE_ETHERNET

static struct udp_pcb *udp_handle = 0;

static void udp_on_rx(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                      const ip_addr_t *addr, u16_t port)
{
    (void)arg; (void)pcb; (void)addr; (void)port;
    if (p == 0) return;

    /* Walk the chain — LwIP may split a datagram across pbufs. */
    for (struct pbuf *q = p; q != 0; q = q->next)
    {
        const uint8_t *bytes = (const uint8_t *)q->payload;
        for (u16_t i = 0; i < q->len; i++) deliver(bytes[i]);
    }

    pbuf_free(p);
}

static uint8_t udp_init(void)
{
    ip_addr_t peer;
    IP4_ADDR(&peer, TRANSPORT_UDP_PEER_IP0, TRANSPORT_UDP_PEER_IP1,
                    TRANSPORT_UDP_PEER_IP2, TRANSPORT_UDP_PEER_IP3);

    udp_handle = udp_new();
    if (udp_handle == 0) return 0;

    if (udp_bind(udp_handle, IP_ADDR_ANY, TRANSPORT_UDP_LOCAL_PORT) != ERR_OK)
    {
        udp_remove(udp_handle);
        udp_handle = 0;
        return 0;
    }

    if (udp_connect(udp_handle, &peer, TRANSPORT_UDP_PEER_PORT) != ERR_OK)
    {
        udp_remove(udp_handle);
        udp_handle = 0;
        return 0;
    }

    udp_recv(udp_handle, udp_on_rx, 0);
    return 1;
}

static uint8_t udp_send_frame(const uint8_t *data, uint16_t len)
{
    if (udp_handle == 0) return 0;

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    if (p == 0) return 0;

    memcpy(p->payload, data, len);
    err_t err = udp_send(udp_handle, p);
    pbuf_free(p);

    return (err == ERR_OK);
}

static void udp_poll(void)
{
    /* Drives the LwIP timers and hands received frames to the stack.
       Without this on every iteration, nothing arrives. */
    MX_LWIP_Process();
}

static const transport_vtable_t vt_udp = { "udp", udp_init, udp_send_frame, udp_poll };

#endif /* TRANSPORT_ENABLE_ETHERNET */


/* ── Dispatch ─────────────────────────────────────────────────────────── */

static const transport_vtable_t *table[TRANSPORT_COUNT] = {
#if TRANSPORT_ENABLE_UART
    [TRANSPORT_UART]     = &vt_uart,
#endif
#if TRANSPORT_ENABLE_USB_CDC
    [TRANSPORT_USB_CDC]  = &vt_usb,
#endif
#if TRANSPORT_ENABLE_ETHERNET
    [TRANSPORT_ETHERNET] = &vt_udp,
#endif
};

static transport_id_t active_id = TRANSPORT_ACTIVE;

void transport_set_rx_handler(transport_rx_fn fn)
{
    rx_handler = fn;
}

uint8_t transport_init(void)
{
    if (active_id >= TRANSPORT_COUNT || table[active_id] == 0) return 0;
    return table[active_id]->init();
}

uint8_t transport_send(const uint8_t *data, uint16_t len)
{
    if (table[active_id] == 0) return 0;
    return table[active_id]->send(data, len);
}

void transport_poll(void)
{
    if (table[active_id] != 0) table[active_id]->poll();
}

uint8_t transport_select(transport_id_t id)
{
    if (id >= TRANSPORT_COUNT || table[id] == 0) return 0;
    active_id = id;
    return table[id]->init();
}

transport_id_t transport_active(void)
{
    return active_id;
}

const char *transport_name(void)
{
    return (table[active_id] != 0) ? table[active_id]->name : "none";
}


/* ── USB CDC receive hook ─────────────────────────────────────────────────
 * With TRANSPORT_ENABLE_USB_CDC set, add these two lines to
 * CDC_Receive_FS() in Src/usbd_cdc_if.c, inside USER CODE 6:
 *
 *     extern void transport_usb_on_rx(uint8_t *buf, uint32_t len);
 *     transport_usb_on_rx(Buf, *Len);
 *
 * before the existing USBD_CDC_ReceivePacket() call. CubeMX regenerates
 * that file, so it has to go inside the USER CODE markers.
 */

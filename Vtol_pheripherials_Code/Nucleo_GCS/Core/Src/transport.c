/*
 * transport.c
 *
 * Sends every frame on every enabled link and delivers received bytes
 * tagged with the link they came from. See transport.h for how links are
 * chosen and what middleware each needs.
 */

#include <string.h>
#include "main.h"
#include "transport.h"

#if TRANSPORT_ENABLE_USB_CDC
#include "usbd_cdc_if.h"
extern USBD_HandleTypeDef hUsbDeviceFS;
#endif

/* Set by transport_set_rx_handler(). Every link funnels received bytes
   through this one pointer, tagged with its id, so main.c can keep a
   separate parser per link. */
static transport_rx_fn rx_handler = 0;

/* Links chosen by main() at startup. Checked on both send and receive,
   so a disabled link neither transmits nor feeds the parser. */
static volatile uint8_t enabled_mask = 0;

static inline uint8_t is_enabled(transport_id_t id)
{
    return (enabled_mask & TRANSPORT_MASK(id)) != 0;
}

static inline void deliver(transport_id_t link, uint8_t byte)
{
    if (rx_handler && is_enabled(link)) rx_handler(link, byte);
}

typedef struct {
    const char *name;
    uint8_t (*init)(void);
    uint8_t (*send)(const uint8_t *data, uint16_t len);
    void    (*poll)(void);
} transport_vtable_t;


/* ── UART ─────────────────────────────────────────────────────────────────
   Interrupt-driven single-byte receive, blocking transmit.

   With both links enabled the blocking transmit is worth watching: at
   115200 each byte is ~87 us, so a tick of four frames costs roughly
   3 ms of main-loop stall before USB even gets a turn. */
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
    /* Re-arm outside the ISR if the in-ISR attempt failed. */
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
        /* Re-arm FIRST, parse second. Receiving one byte at a time means
           the next byte can arrive while this callback runs, and with no
           receive armed that sets the overrun flag. Arming again before
           doing the parse work shortens that window to almost nothing.

           At 115200 a byte takes ~87 us; the parser is a few hundred
           cycles, so this ordering closes the gap in practice. */
        uint8_t b = uart_rx_byte;

        if (HAL_UART_Receive_IT(&TRANSPORT_UART_HANDLE, &uart_rx_byte, 1) != HAL_OK)
        {
            uart_rearm_pending = 1;
        }

        deliver(TRANSPORT_UART, b);
    }
}

/* Without this, a single overrun kills UART receive permanently — and can
   leave the peripheral asserting its interrupt continuously.
 *
 * HAL aborts the transfer on any RX error (overrun, framing, noise,
 * parity) and calls this callback, which is weak and empty by default.
 * Nothing re-arms reception, so RX simply stops; uart_rearm_pending is
 * never set either, because the earlier HAL_UART_Receive_IT had
 * succeeded. The result looks like "the link died" with no error
 * anywhere.
 *
 * Overruns are expected here, not exceptional: reception is one byte at
 * a time, so any burst from the PC can outrun the ISR. Clearing the
 * flags and re-arming is the normal path, not an error path.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != TRANSPORT_UART_HANDLE.Instance) return;

    /* Clear whatever latched. ORE in particular must be cleared or the
       peripheral keeps asserting its interrupt, and at NVIC priority 0
       that starves the main loop — which stops transmit as well as
       receive. */
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    huart->ErrorCode = HAL_UART_ERROR_NONE;

    /* Deliberately do NOT reset the parser context here. A dropped byte
       corrupts one frame; the TLV parser is self-resyncing and will find
       the next SYNC on its own, exactly as it does for a bad CRC. */
    if (HAL_UART_Receive_IT(&TRANSPORT_UART_HANDLE, &uart_rx_byte, 1) != HAL_OK)
    {
        uart_rearm_pending = 1;
    }
}

static const transport_vtable_t vt_uart = { "uart", uart_init, uart_send, uart_poll };

#endif /* TRANSPORT_ENABLE_UART */


/* ── USB CDC ──────────────────────────────────────────────────────────── */
#if TRANSPORT_ENABLE_USB_CDC

static uint8_t usb_init(void)
{
    /* MX_USB_DEVICE_Init() runs from main()'s generated init section.
       Receive is push-based through transport_usb_on_rx(). */
    return 1;
}

static uint8_t usb_send(const uint8_t *data, uint16_t len)
{
    /* This guard is not optional. Until the host enumerates and sets a
       configuration, hUsbDeviceFS.pClassData is NULL — and the generated
       CDC_Transmit_FS() dereferences it unconditionally:

           USBD_CDC_HandleTypeDef *hcdc = hUsbDeviceFS.pClassData;
           if (hcdc->TxState != 0) ...

       With UART as the only link that path was never reached. With both
       links enabled and no cable in CN13, every frame would take it, and
       the first one would hard-fault.

       It also means an unplugged USB costs nothing: this returns at once
       instead of spinning for TRANSPORT_USB_TX_TIMEOUT_MS on every frame. */
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return 0;

    uint32_t deadline = HAL_GetTick() + TRANSPORT_USB_TX_TIMEOUT_MS;
    while (CDC_Transmit_FS((uint8_t *)data, len) == USBD_BUSY)
    {
        if (HAL_GetTick() >= deadline) return 0;   /* drop it, not worth stalling */
    }
    return 1;
}

static void usb_poll(void) { }

static const transport_vtable_t vt_usb = { "usb_cdc", usb_init, usb_send, usb_poll };

#endif /* TRANSPORT_ENABLE_USB_CDC */


/* ── USB CDC receive hook ─────────────────────────────────────────────────
   Outside the #if on purpose — see transport.h. deliver() drops the
   bytes if USB isn't an enabled link, so with LINK_USE_USB off a PC
   talking over CN13 is simply ignored rather than half-heard. */
void transport_usb_on_rx(uint8_t *buf, uint32_t len)
{
#if TRANSPORT_ENABLE_USB_CDC
    for (uint32_t i = 0; i < len; i++) deliver(TRANSPORT_USB_CDC, buf[i]);
#else
    (void)buf;
    (void)len;
#endif
}


/* ── Dispatch ─────────────────────────────────────────────────────────── */

static const transport_vtable_t *table[TRANSPORT_COUNT] = {
#if TRANSPORT_ENABLE_UART
    [TRANSPORT_UART]     = &vt_uart,
#endif
#if TRANSPORT_ENABLE_USB_CDC
    [TRANSPORT_USB_CDC]  = &vt_usb,
#endif
};

void transport_set_rx_handler(transport_rx_fn fn)
{
    rx_handler = fn;
}

uint8_t transport_init(uint8_t link_mask)
{
    uint8_t mask = 0;

    for (int id = 0; id < TRANSPORT_COUNT; id++)
    {
        if (!(link_mask & TRANSPORT_MASK(id))) continue;
        if (table[id] == 0) continue;        /* middleware compiled out */

        table[id]->init();
        mask |= TRANSPORT_MASK(id);
    }

    enabled_mask = mask;
    return mask;
}

uint8_t transport_send(const uint8_t *data, uint16_t len)
{
    uint8_t sent = 0;

    for (int id = 0; id < TRANSPORT_COUNT; id++)
    {
        if (!is_enabled((transport_id_t)id) || table[id] == 0) continue;
        if (table[id]->send(data, len)) sent |= TRANSPORT_MASK(id);
    }
    return sent;
}

void transport_poll(void)
{
    for (int id = 0; id < TRANSPORT_COUNT; id++)
    {
        if (is_enabled((transport_id_t)id) && table[id] != 0) table[id]->poll();
    }
}

uint8_t transport_enabled_mask(void)
{
    return enabled_mask;
}

uint8_t transport_usb_ready(void)
{
#if TRANSPORT_ENABLE_USB_CDC
    return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;
#else
    return 0;
#endif
}

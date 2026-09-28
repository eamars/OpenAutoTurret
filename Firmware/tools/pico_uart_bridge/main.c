#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "tusb.h"
#include "uart.pio.h"

#define TX_PIN 0u
#define RX_PIN 1u
#define RING_SIZE 16384u
#define RING_MASK (RING_SIZE - 1u)
#define TX_SM 0u
#define RX_SM 1u

// Single producer/single consumer on each ring. Publish data before the index.
typedef struct { uint8_t data[RING_SIZE]; volatile uint32_t read, write; } ring_t;
static ring_t to_uart, to_usb;
static volatile uint32_t requested_baud = 115200, requested_generation = 1;
static volatile uint32_t applied_generation, requested_valid = 1, applied_valid;
volatile uint32_t bridge_rx_overruns, bridge_framing_events;
volatile uint32_t bridge_uart_tx_bytes, bridge_uart_rx_bytes;
volatile uint32_t bridge_divider_256;
static uint tx_offset, rx_offset;

static uint32_t ring_count(const ring_t *r) {
    uint32_t write = r->write; __dmb(); return write - r->read;
}
static bool ring_put(ring_t *r, uint8_t b) {
    uint32_t read = r->read; __dmb();
    if (r->write - read == RING_SIZE) return false;
    r->data[r->write & RING_MASK] = b; __dmb(); ++r->write; return true;
}
static bool ring_get(ring_t *r, uint8_t *b) {
    uint32_t write = r->write; __dmb();
    if (write == r->read) return false;
    *b = r->data[r->read & RING_MASK]; __dmb(); ++r->read; return true;
}

static void configure_uart(uint32_t baud, bool valid) {
    pio_sm_set_enabled(pio0, TX_SM, false);
    pio_sm_set_enabled(pio0, RX_SM, false);
    pio_sm_clear_fifos(pio0, TX_SM); pio_sm_clear_fifos(pio0, RX_SM);
    pio_sm_set_pins_with_mask(pio0, TX_SM, 1u << TX_PIN, 1u << TX_PIN);
    if (!valid) return;
    // Rounded 16.8 divider; 8 PIO instructions per UART bit.
    uint32_t div256 = ((uint64_t)clock_get_hz(clk_sys) * 32u + baud / 2u) / baud;
    bridge_divider_256 = div256;
    pio_sm_config tx = bridge_tx_program_get_default_config(tx_offset);
    sm_config_set_out_pins(&tx, TX_PIN, 1); sm_config_set_sideset_pins(&tx, TX_PIN);
    sm_config_set_out_shift(&tx, true, false, 32);
    sm_config_set_fifo_join(&tx, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv_int_frac(&tx, div256 >> 8, div256 & 255);
    pio_sm_init(pio0, TX_SM, tx_offset, &tx);
    pio_sm_config rx = bridge_rx_program_get_default_config(rx_offset);
    sm_config_set_in_pins(&rx, RX_PIN); sm_config_set_jmp_pin(&rx, RX_PIN);
    sm_config_set_in_shift(&rx, true, false, 32);
    sm_config_set_fifo_join(&rx, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv_int_frac(&rx, div256 >> 8, div256 & 255);
    pio_sm_init(pio0, RX_SM, rx_offset, &rx);
    pio_interrupt_clear(pio0, 0);
    pio_enable_sm_mask_in_sync(pio0, (1u << TX_SM) | (1u << RX_SM));
}

static void uart_core(void) {
    pio_gpio_init(pio0, TX_PIN); pio_gpio_init(pio0, RX_PIN); gpio_pull_up(RX_PIN);
    pio_sm_set_consecutive_pindirs(pio0, RX_SM, RX_PIN, 1, false);
    pio_sm_set_consecutive_pindirs(pio0, TX_SM, TX_PIN, 1, true);
    bool configured = false;
    while (true) {
        uint32_t generation = requested_generation; __dmb();
        // Let the last old-baud byte finish (stall at idle PULL), then switch.
        if (generation != applied_generation && ring_count(&to_uart) == 0 &&
            (!configured || !applied_valid ||
             (pio_sm_is_tx_fifo_empty(pio0, TX_SM) &&
              (pio0->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + TX_SM)))))) {
            uint32_t baud = requested_baud, valid = requested_valid;
            configure_uart(baud, valid);
            applied_valid = valid; configured = true; __dmb(); applied_generation = generation;
        }
        if (!applied_valid) continue;
        while (!pio_sm_is_rx_fifo_empty(pio0, RX_SM)) {
            uint8_t b = pio_sm_get(pio0, RX_SM) >> 24;
            ++bridge_uart_rx_bytes;
            if (!ring_put(&to_usb, b)) ++bridge_rx_overruns;
        }
        if (pio_interrupt_get(pio0, 0)) { ++bridge_framing_events; pio_interrupt_clear(pio0, 0); }
        uint8_t b;
        if (!pio_sm_is_tx_fifo_full(pio0, TX_SM) && ring_get(&to_uart, &b)) {
            // Clearing after writing prevents a stale idle flag during a byte.
            pio_sm_put(pio0, TX_SM, b);
            pio0->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + TX_SM);
            ++bridge_uart_tx_bytes;
        }
    }
}

void tud_cdc_line_coding_cb(uint8_t itf, const cdc_line_coding_t *coding) {
    (void)itf;
    requested_baud = coding->bit_rate;
    requested_valid = coding->data_bits == 8 && coding->parity == 0 &&
        coding->stop_bits == 0 && coding->bit_rate >= 9600 && coding->bit_rate <= 1000000;
    __dmb(); ++requested_generation;
}

int main(void) {
    tx_offset = pio_add_program(pio0, &bridge_tx_program);
    rx_offset = pio_add_program(pio0, &bridge_rx_program);
    multicore_launch_core1(uart_core);
    tusb_init();
    uint8_t buffer[64];
    while (true) {
        tud_task();
        // tud_cdc_connected() requires DTR; RoboMaster deliberately clears DTR.
        if (!tud_ready()) {
            while (ring_get(&to_usb, buffer)) { }
            continue;
        }
        if (applied_generation == requested_generation && applied_valid) {
            uint32_t space = RING_SIZE - ring_count(&to_uart);
            uint32_t n = tud_cdc_read(buffer, space < sizeof(buffer) ? space : sizeof(buffer));
            for (uint32_t i = 0; i < n; ++i) ring_put(&to_uart, buffer[i]);
        }
        uint32_t space = tud_cdc_write_available(), n = 0;
        while (n < sizeof(buffer) && n < space && ring_get(&to_usb, &buffer[n])) ++n;
        if (n) tud_cdc_write(buffer, n);
        tud_cdc_write_flush();
    }
}

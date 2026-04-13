#include "pico/stdlib.h"
#include "quadrature_encoder.hpp"
#include "quadrature_encoder.pio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include <cstdint>
#include <cstring>

// ---------------------------------------------------------------------------
// Gray-code transition table
// Index = (prev_AB << 2) | new_AB  →  delta ∈ { -1, 0, +1 }
// ---------------------------------------------------------------------------
static const int8_t QEM[16] = {
//  new: 00  01  10  11
         0, +1, -1,  0,   // prev 00
        -1,  0,  0, +1,   // prev 01
        +1,  0,  0, -1,   // prev 10
         0, -1, +1,  0    // prev 11
};

// ---------------------------------------------------------------------------
// DMA ring buffer — DMA writes here continuously (ping-pong via RING)
// Aligned to its own size so the DMA address-wrap works correctly
// ---------------------------------------------------------------------------
static uint32_t  s_dma_buf[QE_DMA_BUFFER_SIZE] __attribute__((aligned(QE_DMA_BUFFER_SIZE * sizeof(uint32_t))));

// ---------------------------------------------------------------------------
// Module state
// ---------------------------------------------------------------------------
static PIO      s_pio;
static uint     s_sm;
static int      s_dma_chan;
static uint32_t s_read_idx  = 0;      // Next index the CPU will consume
static uint8_t  s_prev_ab   = 0;
static int32_t  s_count     = 0;

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
void quadrature_encoder_init(PIO pio, uint sm, uint gpio_a, uint gpio_b, float clk_div) {
    s_pio = pio;
    s_sm  = sm;

    // ── PIO ────────────────────────────────────────────────────────────────
    uint offset = pio_add_program(pio, &quadrature_encoder_program);

    pio_gpio_init(pio, gpio_a);
    pio_gpio_init(pio, gpio_b);
    gpio_pull_up(gpio_a);
    gpio_pull_up(gpio_b);

    pio_sm_config c = quadrature_encoder_program_get_default_config(offset);
    sm_config_set_in_pins(&c, gpio_b);
    sm_config_set_jmp_pin(&c, gpio_a);
    sm_config_set_in_shift(&c, false, false, 32);  // shift left, no autopush
    sm_config_set_clkdiv(&c, clk_div);

    pio_sm_set_pindirs_with_mask(pio, sm, 0u,
        (1u << gpio_a) | (1u << gpio_b));
    pio_sm_init(pio, sm, offset, &c);

    uint8_t init_ab = (gpio_get(gpio_a) ? 2u : 0u) | (gpio_get(gpio_b) ? 1u : 0u);
    pio_sm_put(pio, sm, init_ab);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false)); // pull OSR from FIFO  
    pio_sm_exec(pio, sm, pio_encode_mov(pio_y, pio_osr)); // Y = OSR

    // ── DMA ────────────────────────────────────────────────────────────────
    s_dma_chan = dma_claim_unused_channel(true);

    dma_channel_config dc = dma_channel_get_default_config(s_dma_chan);

    // Transfer 32-bit words from PIO RX FIFO
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, false);   // always read from FIFO
    channel_config_set_write_increment(&dc, true);   // advance through buffer

    // Wrap the write address after QE_DMA_BUFFER_SIZE words.
    // ring_size = log2(QE_DMA_BUFFER_SIZE * 4 bytes)
    channel_config_set_ring(&dc, true, __builtin_ctz(QE_DMA_BUFFER_SIZE * sizeof(uint32_t)));

    // Pace transfers to PIO RX FIFO data-request signal — no polling waste
    channel_config_set_dreq(&dc, pio_get_dreq(pio, sm, false));

    dma_channel_configure(
        s_dma_chan,
        &dc,
        s_dma_buf,                          // write destination (ring)
        &pio->rxf[sm],                      // read source: PIO RX FIFO
        0xFFFFFFFF,                         // transfer count: run indefinitely
        false                               // don't start yet
    );

    // ── Start both together ─────────────────────────────────────────────────
    pio_sm_set_enabled(pio, sm, true);
    dma_channel_start(s_dma_chan);

    sleep_ms(10);

    // Start consuming from current write position — not from 0
    uint32_t write_addr = dma_channel_hw_addr(s_dma_chan)->write_addr;
    uint32_t write_idx  = ((write_addr - (uint32_t)s_dma_buf) / sizeof(uint32_t))
                          & (QE_DMA_BUFFER_SIZE - 1);
    s_read_idx = write_idx;

    // Seed prev_ab from the last entry DMA wrote
    uint32_t last = s_dma_buf[(write_idx - 1u) & (QE_DMA_BUFFER_SIZE - 1)];
    s_prev_ab = last & 0x3u;
}

// ---------------------------------------------------------------------------
// quadrature_encoder_update — call this from your main loop
// Consumes all new samples the DMA has written since the last call.
// ---------------------------------------------------------------------------
void quadrature_encoder_update(void) {
    uint32_t write_addr = dma_channel_hw_addr(s_dma_chan)->write_addr;
    uint32_t buf_base   = (uint32_t)s_dma_buf;
    uint32_t write_idx  = ((write_addr - buf_base) / sizeof(uint32_t))
                          & (QE_DMA_BUFFER_SIZE - 1);

    while (s_read_idx != write_idx) {
        uint32_t raw = s_dma_buf[s_read_idx];

        uint8_t new_ab = raw & 0x3u;   // matches QEM table layout

        s_count += QEM[(s_prev_ab << 2) | new_ab];
        s_prev_ab = new_ab;

        s_read_idx = (s_read_idx + 1) & (QE_DMA_BUFFER_SIZE - 1);
    }
}

// ---------------------------------------------------------------------------
// quadrature_encoder_get_count — safe to call anytime after update()
// ---------------------------------------------------------------------------
int32_t quadrature_encoder_get_count(void) {
    return s_count;
}

void quadrature_encoder_clean_count(void) {
    s_count = 0;
}

void quadrature_encoder_debug(void) {
    uint32_t write_addr = dma_channel_hw_addr(s_dma_chan)->write_addr;
    uint32_t buf_base   = (uint32_t)s_dma_buf;
    uint32_t write_idx  = ((write_addr - buf_base) / sizeof(uint32_t))
                          & (QE_DMA_BUFFER_SIZE - 1);

    // 1. Is DMA moving at all?
    printf("[DBG] write_idx=%lu  read_idx=%lu\n", write_idx, s_read_idx);

    // 2. Is the PIO FIFO filling?
    printf("[DBG] PIO RX FIFO level=%u\n", pio_sm_get_rx_fifo_level(s_pio, s_sm));

    // 3. What are the raw GPIO levels right now?
    bool a = gpio_get(27);   // your gpio_a
    bool b = gpio_get(7);    // your gpio_b
    printf("[DBG] GPIO27(A)=%d  GPIO7(B)=%d\n", a, b);

    // 4. Dump first 4 DMA buffer entries
    for (int i = 0; i < 4; i++) {
        printf("[DBG] buf[%d] = 0x%08lX\n", i, s_dma_buf[i]);
    }
}
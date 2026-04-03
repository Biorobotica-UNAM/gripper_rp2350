#pragma once
#include "hardware/pio.h"
#include <cstdio>
#include <cstdint>

// 100,000 samples/sec × 0.5 sec loop + 2× safety margin = 100,000
// Round up to next power of 2:
#define QE_DMA_BUFFER_SIZE  4096u    // 256KB, covers 0.65 sec at 100kHz

void    quadrature_encoder_init(PIO pio, uint sm, uint gpio_a, uint gpio_b, float clk_div);
void    quadrature_encoder_update(void);
int32_t quadrature_encoder_get_count(void);
void quadrature_encoder_debug(void);
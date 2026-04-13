#pragma once
#include <cstdio>
#include <cstdint>
#include "hardware/pio.h"

#define QE_DMA_BUFFER_SIZE  512u    // 2KB, at ~8.32kHz

void    quadrature_encoder_init(PIO pio, uint sm, uint gpio_a, uint gpio_b, float clk_div);
void    quadrature_encoder_update(void);
int32_t quadrature_encoder_get_count(void);
void quadrature_encoder_clean_count(void);
void quadrature_encoder_debug(void);
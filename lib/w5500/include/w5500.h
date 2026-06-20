#pragma once

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "w5500_spi.pio.h"
#include "w5500_config.h"
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif
// ─────────────────────────────────────────────────────────────────────────────
// Internal state — all static, lives in .bss
// ─────────────────────────────────────────────────────────────────────────────

// Flat TX buffer: [addr_hi][addr_lo][ctrl][payload...]
extern uint32_t  _tx_buf[W5500_BUF_SIZE];

// Real RX buffer used during read transactions (after the 3-byte header)
extern uint32_t  _rx_buf[W5500_BUF_SIZE];  // used by w5500_dma_read  — incrementing write
extern uint32_t _rx_scratch;              // used by w5500_dma_write — fixed write, discard

// Use runtime claim:
extern int _dma_tx;
extern int _dma_rx;

// ─────────────────────────────────────────────────────────────────────────────
// Internal: block until the current DMA transfer completes.
// After returning the caller may safely read _rx_buf or reuse _tx_buf.
// ─────────────────────────────────────────────────────────────────────────────
void _w5500_wait(void);

// ─────────────────────────────────────────────────────────────────────────────
// CS helpers
// ─────────────────────────────────────────────────────────────────────────────
void _cs_select(void);

void _cs_deselect(void);

// ─────────────────────────────────────────────────────────────────────────────
// w5500_dma_write()
//
// Sends a complete W5500 write frame via DMA:
//   _tx_buf = [addr_hi][addr_lo][ctrl][payload...]
//   Total bytes on the wire = 3 + len
//
// RX DMA uses a 1-byte ring so MISO bytes are discarded without touching RAM
// beyond that single scratch byte. The ring wrap is configured on the
// WRITE_ADDR with size = 0, which means 2^0 = 1 byte ring.
//
// The function returns immediately after kicking off both DMA channels.
// Call _w5500_wait() before touching _tx_buf again.
// ─────────────────────────────────────────────────────────────────────────────
void w5500_dma_write(uint16_t addr, uint8_t bsb,
                             const uint8_t *payload, uint16_t len);

// ─────────────────────────────────────────────────────────────────────────────
// w5500_dma_read()
//
// Sends a 3-byte W5500 read header, then clocks in `len` bytes from MISO.
// Result lands in _rx_buf[0..len-1].
//
// TX DMA sends the header (3 bytes) + `len` dummy 0x00 bytes to keep the
// clock running while MISO data arrives. We build the full dummy TX frame
// in _tx_buf (header + zeroes).
//
// RX DMA captures all (3 + len) bytes. The first 3 are the echo of the header
// (W5500 outputs 0x00 during address phase) — we skip them and expose only
// _rx_buf[3..3+len-1] to the caller via the returned pointer.
// ─────────────────────────────────────────────────────────────────────────────
void w5500_dma_read(uint16_t addr, uint8_t bsb,
                            uint8_t *out, uint16_t len);

// ─────────────────────────────────────────────────────────────────────────────
// Blocking write helper — fires DMA, waits, releases CS.
// Use this for all register writes where the caller doesn't need async.
// ─────────────────────────────────────────────────────────────────────────────
void _w5500_write_blocking(uint16_t addr, uint8_t bsb,
                                   const uint8_t *buf, uint16_t len);

// ─────────────────────────────────────────────────────────────────────────────
// Common register helpers
// ─────────────────────────────────────────────────────────────────────────────
void w5500_write_common(uint16_t addr,
                                       const uint8_t *buf, uint16_t len);

void w5500_read_common(uint16_t addr,
                                      uint8_t *buf, uint16_t len);

uint8_t w5500_read_common_byte(uint16_t addr);

void w5500_write_common_byte(uint16_t addr, uint8_t val);

// ─────────────────────────────────────────────────────────────────────────────
// Socket register helpers
// ─────────────────────────────────────────────────────────────────────────────
uint8_t w5500_read_sock_byte(uint8_t sn, uint16_t addr);

void w5500_write_sock_byte(uint8_t sn, uint16_t addr, uint8_t val);

void w5500_write_sock_u16(uint8_t sn, uint16_t addr, uint16_t val);

// ─────────────────────────────────────────────────────────────────────────────
// Initialisation
// ─────────────────────────────────────────────────────────────────────────────
void w5500_init_spi(void);

void w5500_soft_reset(void);

uint8_t w5500_get_version(void);
uint8_t w5500_get_phycfgr(void);

void w5500_set_mac(const uint8_t mac[6]);
void w5500_set_ip (const uint8_t ip[4]);
void w5500_set_gateway(const uint8_t gw[4]);
void w5500_set_subnet (const uint8_t sn[4]);
void w5500_set_pc (const uint8_t pc_ip[4], const uint16_t port, 
                            const uint8_t sn, const uint8_t pc_mac[6]);

uint8_t pio_spi_transfer_byte(uint8_t tx);
uint8_t read_version_no_dma(void);

void w5500_set_sock_keepalive(uint8_t sn, uint8_t kpalvtr_5s_units,
                               uint16_t rtr_100us_units, uint8_t rcr_retries);

// DEBUG
uint8_t read_version_dma_debug(void);
void debug_write_then_read(void);

#ifdef __cplusplus
}
#endif
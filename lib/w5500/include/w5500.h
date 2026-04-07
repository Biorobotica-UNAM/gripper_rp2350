#pragma once

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "w5500_spi.pio.h"
#include <stdint.h>
#include <string.h>

// ─────────────────────────────────────────────────────────────────────────────
// Pin & PIO configuration
//
// XIAO RP2350 label → actual GPIO:
//   D8  = GPIO2  → SCK
//   D9  = GPIO4  → MISO
//   D10 = GPIO3  → MOSI
//   D7  = GPIO1  → CS   (plain GPIO, bit-banged)
// ─────────────────────────────────────────────────────────────────────────────
#define W5500_PIO         pio0
#define W5500_SM          1            // SM0 is taken by quadrature encoder
#define W5500_SPI_FREQ    (10 * 1000 * 1000)

#define W5500_PIN_CS      1
#define W5500_PIN_SCK     2
#define W5500_PIN_MOSI    3
#define W5500_PIN_MISO    4

// ─────────────────────────────────────────────────────────────────────────────
// DMA_IRQ_1  is used here, intended to run on Core 1.
// ─────────────────────────────────────────────────────────────────────────────
#define W5500_DMA_IRQ     DMA_IRQ_1

// ─────────────────────────────────────────────────────────────────────────────
// W5500 SPI frame encoding
// ─────────────────────────────────────────────────────────────────────────────
#define BSB_COMMON_REG    (0x00 << 3)
#define BSB_SOCK_REG(n)   (((n)*4 + 1) << 3)
#define BSB_SOCK_TX(n)    (((n)*4 + 2) << 3)
#define BSB_SOCK_RX(n)    (((n)*4 + 3) << 3)

#define RWB_READ          0x00
#define RWB_WRITE         0x04
#define OM_VDM            0x00

// ─────────────────────────────────────────────────────────────────────────────
// W5500 common register addresses
// ─────────────────────────────────────────────────────────────────────────────
#define REG_MR            0x0000
#define REG_GAR           0x0001
#define REG_SUBR          0x0005
#define REG_SHAR          0x0009
#define REG_SIPR          0x000F
#define REG_PHYCFGR       0x002E
#define REG_VERSIONR      0x0039

// ─────────────────────────────────────────────────────────────────────────────
// W5500 socket register offsets
// ─────────────────────────────────────────────────────────────────────────────
#define Sn_MR             0x0000
#define Sn_CR             0x0001
#define Sn_IR             0x0002
#define Sn_SR             0x0003
#define Sn_PORT           0x0004
#define Sn_DHAR           0x0006
#define Sn_DIPR           0x000C
#define Sn_DPORT          0x0010
#define Sn_TX_FSR         0x0020
#define Sn_TX_WR          0x0024
#define Sn_RX_RSR         0x0026
#define Sn_RX_RD          0x0028

#define Sn_CR_OPEN        0x01
#define Sn_CR_LISTEN      0x02
#define Sn_CR_DISCON      0x08
#define Sn_CR_CLOSE       0x10
#define Sn_CR_SEND        0x20
#define Sn_CR_RECV        0x40

#define SOCK_CLOSED       0x00
#define SOCK_INIT         0x13
#define SOCK_LISTEN       0x14
#define SOCK_ESTABLISHED  0x17
#define SOCK_CLOSE_WAIT   0x1C
#define SOCK_UDP          0x22

#define Sn_MR_TCP         0x01
#define Sn_MR_UDP         0x02
#define Sn_MR_MACRAW      0x04

// ─────────────────────────────────────────────────────────────────────────────
// Maximum SPI frame size:
//   3 bytes header  +  up to N bytes payload
//   For W5500 registers the largest payload is 6 bytes (MAC).
//   For TX/RX buffers you may need more — adjust as needed.
// ─────────────────────────────────────────────────────────────────────────────
#define W5500_BUF_SIZE    256

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

// DEBUG
uint8_t read_version_dma_debug(void);
void debug_write_then_read(void);

#ifdef __cplusplus
}
#endif
#pragma once

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

// ── Keep-alive ──────────────────────────────────────────────────────────────
#define Sn_RTR            0x0019   // Retry Time-value Register (2 bytes, unidad 100us)
#define Sn_RCR            0x001B   // Retry Count Register (1 byte)
#define Sn_KPALVTR        0x002F   // Keep Alive Timer Register (1 byte, unidad 5s)

// ─────────────────────────────────────────────────────────────────────────────
// Maximum SPI frame size:
//   3 bytes header  +  up to N bytes payload
//   For W5500 registers the largest payload is 6 bytes (MAC).
//   For TX/RX buffers you may need more — adjust as needed.
// ─────────────────────────────────────────────────────────────────────────────
#define W5500_BUF_SIZE    256
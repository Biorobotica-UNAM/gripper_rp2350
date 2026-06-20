#include "w5500.h"

uint32_t  _tx_buf[W5500_BUF_SIZE];
uint32_t  _rx_buf[W5500_BUF_SIZE];  // used by w5500_dma_read  — incrementing write
uint32_t   _rx_scratch;               // used by w5500_dma_write — fixed write, discard

int _dma_tx;
int _dma_rx;

void _w5500_wait(void) {
    // Just poll TX channel directly — no IRQ needed
    dma_channel_wait_for_finish_blocking(_dma_tx);
    dma_channel_wait_for_finish_blocking(_dma_rx);
}

void _cs_select(void) {
    asm volatile("nop \n nop \n nop");
    gpio_put(W5500_PIN_CS, 0);
    asm volatile("nop \n nop \n nop");
}

void _cs_deselect(void) {
    asm volatile("nop \n nop \n nop");
    gpio_put(W5500_PIN_CS, 1);
    asm volatile("nop \n nop \n nop");
}

void w5500_dma_write(uint16_t addr, uint8_t bsb,
                             const uint8_t *payload, uint16_t len)
{
    uint16_t total = 3 + len;

    // Build frame in _tx_buf
    _tx_buf[0] = (uint32_t)((addr >> 8) & 0xFF)       << 24;
    _tx_buf[1] = (uint32_t)(addr & 0xFF)              << 24;
    _tx_buf[2] = (uint32_t)(bsb | RWB_WRITE | OM_VDM) << 24;
    for (uint16_t i = 0; i < len; i++)
        _tx_buf[3 + i] = (uint32_t)(payload ? payload[i] : 0x00) << 24;

    _cs_select();
    pio_sm_clear_fifos(W5500_PIO, W5500_SM);

    // ── TX channel ────────────────────────────────────────────────────────────
    // Reads from _tx_buf (incrementing), writes to PIO TX FIFO (fixed address).
    dma_channel_config tx_cfg = dma_channel_get_default_config(_dma_tx);
    channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&tx_cfg, true);
    channel_config_set_write_increment(&tx_cfg, false);
    channel_config_set_dreq(&tx_cfg, pio_get_dreq(W5500_PIO, W5500_SM, true));

    dma_channel_configure(
        _dma_tx,
        &tx_cfg,
        &W5500_PIO->txf[W5500_SM],   // write to PIO TX FIFO
        _tx_buf,                       // read from frame buffer
        total,
        false                          // don't start yet
    );

    // ── RX channel — 1-byte ring ──────────────────────────────────────────────
    // Reads from PIO RX FIFO (fixed), writes to _rx_scratch (1-byte ring).
    // ring_size = 0  =>  wrap boundary = 2^(0+1) ... actually ring_size in
    // the SDK is the LOG2 of the byte count, so ring_size = 0 means 1 byte.
    dma_channel_config rx_cfg = dma_channel_get_default_config(_dma_rx);
    channel_config_set_transfer_data_size(&rx_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&rx_cfg, false);
    channel_config_set_write_increment(&rx_cfg, false);    // ring handles this
    channel_config_set_dreq(&rx_cfg, pio_get_dreq(W5500_PIO, W5500_SM, false));

    dma_channel_configure(
        _dma_rx,
        &rx_cfg,
        &_rx_scratch,                  // always write to this one byte
        &W5500_PIO->rxf[W5500_SM],      // read from PIO RX FIFO
        total,
        false
    );

    // ── Fire both channels simultaneously ─────────────────────────────────────
    dma_start_channel_mask((1u << _dma_tx) | (1u << _dma_rx));
    // IRQ fires when TX finishes → _w5500_dma_irq_handler sets _spi_done

    // This makes dma_write self-contained and safe to call sequentially
    _w5500_wait();

    // Wait for PIO to finish clocking out the last byte
    while (!pio_sm_is_tx_fifo_empty(W5500_PIO, W5500_SM))
        tight_loop_contents();
    // Extra settling time before releasing CS
    sleep_us(1);

    _cs_deselect();
}

void w5500_dma_read(uint16_t addr, uint8_t bsb,
                            uint8_t *out, uint16_t len)
{
    uint16_t total = 3 + len;

    _tx_buf[0] = (uint32_t)((addr >> 8) & 0xFF) << 24;
    _tx_buf[1] = (uint32_t)(addr & 0xFF)        << 24;
    _tx_buf[2] = (uint32_t)(bsb | RWB_READ)     << 24;
    for (uint16_t i = 0; i < len; i++)
        _tx_buf[3 + i] = 0x00000000;

    _cs_select();
    pio_sm_clear_fifos(W5500_PIO, W5500_SM);

    dma_channel_config tx_cfg = dma_channel_get_default_config(_dma_tx);
    channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&tx_cfg, true);
    channel_config_set_write_increment(&tx_cfg, false);
    channel_config_set_dreq(&tx_cfg, pio_get_dreq(W5500_PIO, W5500_SM, true));

    dma_channel_configure(
        _dma_tx,
        &tx_cfg,
        &W5500_PIO->txf[W5500_SM],
        _tx_buf,
        total,
        false
    );

    dma_channel_config rx_cfg = dma_channel_get_default_config(_dma_rx);
    channel_config_set_transfer_data_size(&rx_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&rx_cfg, false);
    channel_config_set_write_increment(&rx_cfg, true);
    channel_config_set_dreq(&rx_cfg, pio_get_dreq(W5500_PIO, W5500_SM, false));

    dma_channel_configure(
        _dma_rx,
        &rx_cfg,
        _rx_buf,                        // capture everything into _rx_buf
        &W5500_PIO->rxf[W5500_SM],
        total,
        false
    );

    // ── Fire ──────────────────────────────────────────────────────────────────
    dma_start_channel_mask((1u << _dma_tx) | (1u << _dma_rx));

    // Wait here — the caller needs the data before returning
    _w5500_wait();
    _cs_deselect();

    for (uint16_t i = 0; i < len; i++) {
        out[i] = (uint8_t)(_rx_buf[3 + i] & 0xFF);
    }
}

void _w5500_write_blocking(uint16_t addr, uint8_t bsb,
                                   const uint8_t *buf, uint16_t len)
{
    w5500_dma_write(addr, bsb, buf, len);
}

void w5500_write_common(uint16_t addr,
                                       const uint8_t *buf, uint16_t len) {
    _w5500_write_blocking(addr, BSB_COMMON_REG, buf, len);
}

void w5500_read_common(uint16_t addr, uint8_t *buf, uint16_t len) {
    w5500_dma_read(addr, BSB_COMMON_REG, buf, len);
}

uint8_t w5500_read_common_byte(uint16_t addr) {
    uint8_t val;
    w5500_read_common(addr, &val, 1);
    printf("Val: %u\n", val);
    return val;
}

void w5500_write_common_byte(uint16_t addr, uint8_t val) {
    w5500_write_common(addr, &val, 1);
}

uint8_t w5500_read_sock_byte(uint8_t sn, uint16_t addr) {
    uint8_t val;
    w5500_dma_read(addr, BSB_SOCK_REG(sn), &val, 1);
    return val;
}

void w5500_write_sock_byte(uint8_t sn, uint16_t addr, uint8_t val) {
    _w5500_write_blocking(addr, BSB_SOCK_REG(sn), &val, 1);
}

void w5500_write_sock_u16(uint8_t sn, uint16_t addr, uint16_t val) {
    uint8_t buf[2] = { val >> 8, val & 0xFF };
    _w5500_write_blocking(addr, BSB_SOCK_REG(sn), buf, 2);
}

void w5500_init_spi(void) {
    // ── PIO ───────────────────────────────────────────────────────────────────
    uint offset = pio_add_program(W5500_PIO, &w5500_spi_program);
    w5500_spi_init(W5500_PIO, W5500_SM, offset,
                 W5500_PIN_SCK,
                 W5500_PIN_MOSI,
                 W5500_PIN_MISO,
                 W5500_SPI_FREQ);

    // ── CS & RST GPIOs ────────────────────────────────────────────────────────
    gpio_init(W5500_PIN_CS);
    gpio_set_dir(W5500_PIN_CS, GPIO_OUT);
    gpio_put(W5500_PIN_CS, 1);

    // DMA
    // ── TX channel ────────────────────────────────────────────────────────────
    _dma_tx = dma_claim_unused_channel(true);

    // ── RX channel — real buffer, incrementing ────────────────────────────────
    _dma_rx = dma_claim_unused_channel(true);
}

void w5500_soft_reset(void) {
    uint8_t rst = 0x80;
    w5500_dma_write(REG_MR, BSB_COMMON_REG, &rst, 1);
    sleep_ms(100);  // was 10, try 100
}

uint8_t w5500_get_version(void)  { return w5500_read_common_byte(REG_VERSIONR); }
uint8_t w5500_get_phycfgr(void)  { return w5500_read_common_byte(REG_PHYCFGR);  }

void w5500_set_mac(const uint8_t mac[6]) { w5500_write_common(REG_SHAR, mac, 6); }
void w5500_set_ip (const uint8_t ip[4])  { w5500_write_common(REG_SIPR, ip,  4); }
void w5500_set_gateway(const uint8_t gw[4]) { w5500_write_common(REG_GAR,  gw, 4); }
void w5500_set_subnet (const uint8_t sn[4]) { w5500_write_common(REG_SUBR, sn, 4); }
void w5500_set_pc (const uint8_t pc_ip[4], const uint16_t port, 
                            const uint8_t sn, const uint8_t pc_mac[6]) 
{
    uint8_t dp[2] = {port >> 8, port & 0xFF};

    _w5500_write_blocking(Sn_DHAR, BSB_SOCK_REG(sn), pc_mac, sizeof(pc_mac));
    _w5500_write_blocking(Sn_DIPR, BSB_SOCK_REG(sn), pc_ip, sizeof(pc_ip));
    _w5500_write_blocking(Sn_DPORT, BSB_SOCK_REG(sn), dp, sizeof(dp));
}

uint8_t pio_spi_transfer_byte(uint8_t tx) {
    // Left-justify the byte for the PIO left-shift OSR
    pio_sm_put_blocking(W5500_PIO, W5500_SM, (uint32_t)tx << 24);
    return (uint8_t)pio_sm_get_blocking(W5500_PIO, W5500_SM);
}

uint8_t read_version_no_dma(void) {
    // Manually assert CS
    gpio_put(W5500_PIN_CS, 0);
    sleep_us(1);

    // Send 3-byte header: addr=0x0039, ctrl=READ|COMMON_REG
    pio_spi_transfer_byte(0x00);              // addr high
    pio_spi_transfer_byte(0x39);              // addr low
    pio_spi_transfer_byte(0x00 | 0x00 | 0x00); // BSB_COMMON_REG | RWB_READ | OM_VDM

    // Clock in 1 byte from W5500
    uint8_t result = pio_spi_transfer_byte(0x00);

    sleep_us(1);
    gpio_put(W5500_PIN_CS, 1);
    return result;
}

void w5500_set_sock_keepalive(uint8_t sn, uint8_t kpalvtr_5s_units,
                               uint16_t rtr_100us_units, uint8_t rcr_retries)
{
    // Tiempo de retransmisión para SYN/datos/keepalive (también afecta connect)
    w5500_write_sock_u16(sn, Sn_RTR, rtr_100us_units);

    // Cantidad de reintentos antes de declarar TIMEOUT
    w5500_write_sock_byte(sn, Sn_RCR, rcr_retries);

    // Intervalo de keep-alive (0 = deshabilitado)
    w5500_write_sock_byte(sn, Sn_KPALVTR, kpalvtr_5s_units);
}

// DEBUG

uint8_t read_version_dma_debug(void) {
    uint8_t ver = 0;
    printf("  firing dma_read...\n");
    w5500_dma_read(REG_VERSIONR, BSB_COMMON_REG, &ver, 1);
    printf("  dma_read returned, ver=0x%02X\n", ver);
    printf("  _rx_buf raw: %081X %081X %081X %081X\n",
       _rx_buf[0],
       _rx_buf[1],
       _rx_buf[2],
       _rx_buf[3]);
    printf("PIO RXF addr: %p\n", (void*)&W5500_PIO->rxf[W5500_SM]);
    printf("RX DREQ: %d\n", pio_get_dreq(W5500_PIO, W5500_SM, false));
    return ver;
}

void debug_write_then_read(void) {
    printf("--- Step 1: read version BEFORE write ---\n");
    uint8_t ver1 = 0;
    w5500_dma_read(REG_VERSIONR, BSB_COMMON_REG, &ver1, 1);
    printf("ver1 = 0x%02X\n", ver1);

    printf("--- Step 2: dma_write (soft reset) ---\n");
    uint8_t rst = 0x80;
    w5500_dma_write(REG_MR, BSB_COMMON_REG, &rst, 1);
    printf("write done\n");
    sleep_ms(10);

    printf("--- Step 3: read version AFTER write ---\n");
    uint8_t ver2 = 0;
    w5500_dma_read(REG_VERSIONR, BSB_COMMON_REG, &ver2, 1);
    printf("ver2 = 0x%02X\n", ver2);

    printf("--- Step 4: PIO state after write ---\n");
    printf("TX FIFO level: %d\n", pio_sm_get_tx_fifo_level(W5500_PIO, W5500_SM));
    printf("RX FIFO level: %d\n", pio_sm_get_rx_fifo_level(W5500_PIO, W5500_SM));
    printf("TX DMA busy: %d\n", dma_channel_is_busy(_dma_tx));
    printf("RX DMA busy: %d\n", dma_channel_is_busy(_dma_rx));
}
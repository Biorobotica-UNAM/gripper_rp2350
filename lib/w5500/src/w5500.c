#include "w5500.h"

uint8_t  _tx_buf[W5500_BUF_SIZE];
uint8_t  _rx_buf[W5500_BUF_SIZE];
uint8_t  _rx_scratch;

volatile bool _spi_done;

int _dma_tx;
int _dma_rx;

void _w5500_dma_irq_handler(void) {
    if (dma_channel_get_irq1_status(_dma_tx)) {
        dma_channel_acknowledge_irq1(_dma_tx);
        _spi_done = true;
    }
}

void _w5500_wait(void) {
    while (!_spi_done)
        tight_loop_contents();
    _spi_done = false;
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
    _tx_buf[0] = (addr >> 8) & 0xFF;
    _tx_buf[1] =  addr       & 0xFF;
    _tx_buf[2] =  bsb | RWB_WRITE | OM_VDM;
    if (len && payload)
        memcpy(&_tx_buf[3], payload, len);

    _cs_select();
    _spi_done = false;

    // ── TX channel ────────────────────────────────────────────────────────────
    // Reads from _tx_buf (incrementing), writes to PIO TX FIFO (fixed address).
    dma_channel_config tx_cfg = dma_channel_get_default_config(_dma_tx);
    channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_8);
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
    channel_config_set_transfer_data_size(&rx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_cfg, false);
    channel_config_set_write_increment(&rx_cfg, false);    // ring handles this
    channel_config_set_ring(&rx_cfg, true, 0);             // WRITE ring, 2^0=1 byte
    channel_config_set_dreq(&rx_cfg, pio_get_dreq(W5500_PIO, W5500_SM, false));

    dma_channel_configure(
        _dma_rx,
        &rx_cfg,
        &_rx_scratch,                  // always write to this one byte
        &W5500_PIO->rxf[W5500_SM],    // read from PIO RX FIFO
        total,
        false
    );

    // ── Fire both channels simultaneously ─────────────────────────────────────
    dma_start_channel_mask((1u << _dma_tx) | (1u << _dma_rx));
    // IRQ fires when TX finishes → _w5500_dma_irq_handler sets _spi_done
}

void w5500_dma_read(uint16_t addr, uint8_t bsb,
                            uint8_t *out, uint16_t len)
{
    uint16_t total = 3 + len;

    // Header + dummy TX bytes (keep SCK running during RX phase)
    _tx_buf[0] = (addr >> 8) & 0xFF;
    _tx_buf[1] =  addr       & 0xFF;
    _tx_buf[2] =  bsb | RWB_READ | OM_VDM;
    memset(&_tx_buf[3], 0x00, len);

    _cs_select();
    _spi_done = false;

    dma_channel_config tx_cfg = dma_channel_get_default_config(_dma_tx);
    channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_8);
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
    channel_config_set_transfer_data_size(&rx_cfg, DMA_SIZE_8);
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

    // Skip the 3 echo bytes from the header phase, copy real data to caller
    memcpy(out, &_rx_buf[3], len);
}

void _w5500_write_blocking(uint16_t addr, uint8_t bsb,
                                   const uint8_t *buf, uint16_t len)
{
    w5500_dma_write(addr, bsb, buf, len);
    _w5500_wait();
    _cs_deselect();
}

void w5500_write_common(uint16_t addr,
                                       const uint8_t *buf, uint16_t len) {
    _w5500_write_blocking(addr, BSB_COMMON_REG, buf, len);
}

void w5500_read_common(uint16_t addr,
                                      uint8_t *buf, uint16_t len) {
    w5500_dma_read(addr, BSB_COMMON_REG, buf, len);
}

uint8_t w5500_read_common_byte(uint16_t addr) {
    uint8_t val;
    w5500_read_common(addr, &val, 1);
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

    // ── DMA_IRQ_1 — to be called from Core 1 ─────────────────────────────────
    // Only the TX channel triggers the interrupt. The RX channel runs silently.
    dma_channel_set_irq1_enabled(_dma_tx, true);
    irq_set_exclusive_handler(W5500_DMA_IRQ, _w5500_dma_irq_handler);
    irq_set_enabled(W5500_DMA_IRQ, true);

    _spi_done = false;
}

void w5500_soft_reset(void) {
    w5500_write_common_byte(REG_MR, 0x80);
    sleep_ms(10);
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
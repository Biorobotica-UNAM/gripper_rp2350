#include "tcp_ip.h"

bool tcp_init(void) {
    // Software reset
    w5500_soft_reset();

    // Verify chip
    if (w5500_get_version() != 0x04) {
        printf("W5500 version: 0x%02X\n", w5500_get_version());
        return false;
    }

    // Configure network identity
    w5500_set_mac(MY_MAC);
    w5500_set_ip(MY_IP);
    w5500_set_gateway(GATEWAY);
    w5500_set_subnet(SUBNET);

    // DEBUG
    uint8_t mac[6], ip[4], gw[4], sn[4];
    w5500_dma_read(REG_SHAR, BSB_COMMON_REG, mac, 6);
    w5500_dma_read(REG_SIPR, BSB_COMMON_REG, ip,  4);
    w5500_dma_read(REG_GAR,  BSB_COMMON_REG, gw,  4);
    w5500_dma_read(REG_SUBR, BSB_COMMON_REG, sn,  4);

    printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    printf("IP:  %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);
    printf("GW:  %d.%d.%d.%d\n", gw[0], gw[1], gw[2], gw[3]);
    printf("SN:  %d.%d.%d.%d\n", sn[0], sn[1], sn[2], sn[3]);

    return true;
}

bool tcp_open(void) {
    w5500_write_sock_byte(TCP_SOCKET, Sn_CR,   Sn_CR_CLOSE);
    sleep_ms(5);
    w5500_write_sock_byte(TCP_SOCKET, Sn_MR,   Sn_MR_TCP);
    w5500_write_sock_u16 (TCP_SOCKET, Sn_PORT, LOCAL_PORT);
    w5500_write_sock_byte(TCP_SOCKET, Sn_CR,   Sn_CR_OPEN);
    sleep_ms(5);

    if (w5500_read_sock_byte(TCP_SOCKET, Sn_SR) != SOCK_INIT) {
        printf("Not Socket INIT: 0x%02X\n", w5500_read_sock_byte(TCP_SOCKET, Sn_SR));
        return false;
    }

    w5500_write_sock_byte(TCP_SOCKET, Sn_CR, Sn_CR_LISTEN);
    sleep_ms(5);
    return w5500_read_sock_byte(TCP_SOCKET, Sn_SR) == SOCK_LISTEN;
}

void tcp_reconnect(void) {
    w5500_write_sock_byte(TCP_SOCKET, Sn_CR, Sn_CR_DISCON);
    sleep_ms(100);
    tcp_open();   // re-arm and go back to LISTEN
}

void tcp_send_response(uint8_t *pkt, uint16_t actual_rpm, 
                        uint16_t actual_current, uint8_t status) {
    pkt[4] = status;
    pkt[5] = (actual_rpm >> 4) & 0xFF;
    pkt[6] = (actual_rpm & 0x0F) | ((actual_current >> 8) & 0x0F);
    pkt[7] = actual_current & 0xFF;

    // Read TX write pointer
    uint8_t ptr[2];
    w5500_dma_read(Sn_TX_WR, BSB_SOCK_REG(TCP_SOCKET), ptr, 2);
    uint16_t tx_wr = ((uint16_t)ptr[0] << 8) | ptr[1];

    // Write payload — no destination fields needed, TCP is connected
    w5500_dma_write(tx_wr, BSB_SOCK_TX(TCP_SOCKET), pkt, 8);
    _w5500_wait();

    tx_wr += 8;
    uint8_t new_ptr[2] = { tx_wr >> 8, tx_wr & 0xFF };
    _w5500_write_blocking(Sn_TX_WR, BSB_SOCK_REG(TCP_SOCKET), new_ptr, 2);

    w5500_write_sock_byte(TCP_SOCKET, Sn_IR, 0xFF);
    w5500_write_sock_byte(TCP_SOCKET, Sn_CR, Sn_CR_SEND);

    uint32_t t = 500;
    while (t--) {
        uint8_t ir = w5500_read_sock_byte(TCP_SOCKET, Sn_IR);
        if (ir & 0x10) { w5500_write_sock_byte(TCP_SOCKET, Sn_IR, 0xFF); break; }
        if (ir & 0x08) { w5500_write_sock_byte(TCP_SOCKET, Sn_IR, 0xFF); break; }
        sleep_ms(1);
    }
}
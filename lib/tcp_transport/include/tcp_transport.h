#pragma once

#include "pico/stdlib.h"
#include "lwip/inet.h"
#include "w5500.h"
#include "network_config.h"

#ifdef __cplusplus
extern "C" {
#endif

bool tcp_init(void);
bool tcp_open(void);
uint8_t tcp_check_connection(void) { return w5500_read_sock_byte(TCP_SOCKET, Sn_SR); }
void tcp_reconnect(void);
bool tcp_send_packet(uint8_t *data, uint16_t len);
void tcp_send_response(uint8_t *pkt, motor_report_t res);
int16_t tcp_recv_packet(uint8_t *buf, uint16_t buf_size);

#ifdef __cplusplus
}
#endif
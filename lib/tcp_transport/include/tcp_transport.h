#pragma once

#include "pico/stdlib.h"
#include "w5500.h"
#include "network_config.h"

#ifdef __cplusplus
extern "C" {
#endif

bool tcp_init(void);
bool tcp_open(void);
uint8_t tcp_check_connection(void);
void tcp_reconnect(void);
uint16_t tcp_available();
bool tcp_send(const uint8_t *data, uint16_t len);
int16_t tcp_recv_exact(uint8_t *buf, uint16_t length);

#ifdef __cplusplus
}
#endif
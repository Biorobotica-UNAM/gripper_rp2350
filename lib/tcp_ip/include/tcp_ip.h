#pragma once

#include "pico/stdlib.h"
#include "w5500.h"

#define LOCAL_PORT 8888
#define TCP_SOCKET 0

typedef enum {
    INIT     = 0x00,
    OPENING  = 0x01,
    CLOSING  = 0x02,
    REACHED  = 0x03,
    IDLE     = 0x04,
    STALLED  = 0x05
} status_t;

enum {
    OPEN         = 0x01,
    CLOSE        = 0x02,
    STOP         = 0x03,
    QUERY_STATUS = 0x04
};

typedef struct {
    uint16_t rpm;
    uint16_t current;
    status_t status;
} motor_report_t;

#ifdef __cplusplus
extern "C" {
#endif

const uint8_t MY_MAC[6]  = { 0x00, 0x08, 0xDC, 0x12, 0x34, 0x56 };
const uint8_t MY_IP[4]   = { 172, 27, 1, 120 };
const uint8_t GATEWAY[4] = { 172, 27, 1, 1 };
const uint8_t SUBNET[4]  = { 255, 255, 0, 0 };

bool tcp_init(void);
bool tcp_open(void);
void tcp_reconnect(void);
void tcp_send_response(uint8_t *pkt, uint16_t actual_rpm, 
                        uint16_t actual_current, uint8_t status);

#ifdef __cplusplus
}
#endif
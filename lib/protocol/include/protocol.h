#pragma once

#include "tcp_transport.h"
#include "messages.h"

typedef enum { WAIT_HEADER, WAIT_PAYLOAD } read_state_t;
static read_state_t state = WAIT_HEADER;
static uint16_t pending_length = 0;

#ifdef __cplusplus
extern "C" {
#endif

bool is_connected(void);
bool protocol_read(protocol_msg_t* msg);
uint16_t serialize_motor_report(
    uint8_t* buffer,
    const motor_report_t* res);
bool protocol_send_packet(
    const uint8_t* payload,
    uint16_t payload_size);
bool protocol_send_response(const motor_report_t* res);

#ifdef __cplusplus
}
#endif
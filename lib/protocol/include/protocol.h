#pragma once

#include "tcp_transport.h"
#include "messages.h"

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
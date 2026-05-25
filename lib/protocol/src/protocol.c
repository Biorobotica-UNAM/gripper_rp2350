#include "protocol.h"

bool is_connected(void) {
	uint8_t status = tcp_check_connection();

	if (status == SOCK_CLOSE_WAIT || status == SOCK_CLOSED) {
        // Client disconnected — re-arm listener
        tcp_reconnect();
        sleep_ms(5);
        return false;
    }

    if (status != SOCK_ESTABLISHED) {
        // Still waiting for a client to connect — nothing to do
        return false;
    }

    return true;
}

bool protocol_read(protocol_msg_t* msg) {
    uint8_t header[4];

    if (!tcp_recv_exact(header, 4)) {
        return false;
    }

    uint16_t length =
        ((uint16_t)header[0] << 8) |
         (uint16_t)header[1];

    msg->payload_size = length - 2;

    if (msg->payload_size > sizeof(msg->payload)) {
        return false;
    }

    if (!tcp_recv_exact(
            msg->payload,
            msg->payload_size)) {
        return false;
    }

    return true;
}

uint16_t serialize_motor_report(
    uint8_t* buffer,
    const motor_report_t* res)
{
    uint8_t* p = buffer;

    *p++ = (res->rpm >> 8) & 0xFF;
    *p++ =  res->rpm & 0xFF;

    *p++ = (res->current >> 8) & 0xFF;
    *p++ =  res->current & 0xFF;

    *p++ = (res->pos_cnt >> 24) & 0xFF;
    *p++ = (res->pos_cnt >> 16) & 0xFF;
    *p++ = (res->pos_cnt >> 8)  & 0xFF;
    *p++ =  res->pos_cnt & 0xFF;

    *p++ = (uint8_t)res->status;

    return (uint16_t)(p - buffer);
}

bool protocol_send_packet(
    const uint8_t* payload,
    uint16_t payload_size)
{
    uint8_t packet[512];

    uint16_t total_length =
        payload_size + 2;

    packet[0] = (total_length >> 8) & 0xFF;
    packet[1] = total_length & 0xFF;

    memcpy(
        &packet[4],
        payload,
        payload_size);

    return tcp_send(
        packet,
        total_length + 2);
}

bool protocol_send_response(
    const motor_report_t* res)
{
    uint8_t payload[64];

    uint16_t payload_size =
        serialize_motor_report(
            payload,
            res);

    return protocol_send_packet(
        payload,
        payload_size);
}
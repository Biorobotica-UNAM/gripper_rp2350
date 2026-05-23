#pragma once

#include "tcp_transport.h"
#include "messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void send_response(motor_report_t res);

#ifdef __cplusplus
}
#endif
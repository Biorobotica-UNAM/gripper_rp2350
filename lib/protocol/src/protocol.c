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
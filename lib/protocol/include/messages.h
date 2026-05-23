#pragma once

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
    int32_t  pos_cnt;
    status_t status;
} motor_report_t;
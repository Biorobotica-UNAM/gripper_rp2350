#pragma once

#include <stdint.h>

#define LOCAL_PORT 8888 // 8080 -> Left, 8888 -> Right
#define TCP_SOCKET 0

#define MY_MAC      ((uint8_t[6]){ 0x00, 0x08, 0xDC, 0x12, 0x34, 0x57 })  // 56 -> Left, 57 -> Right
#define MY_IP       ((uint8_t[4]){ 172, 27, 1, 121 })  // 120 -> Left, 121 -> Right
#define GATEWAY  ((uint8_t[4]){ 172, 27, 1, 1 })
#define SUBNET   ((uint8_t[4]){ 255, 255, 0, 0 })
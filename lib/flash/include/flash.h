#pragma once

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

// Reserve the LAST sector of flash for our persistent storage.
// PICO_FLASH_SIZE_BYTES is defined by the SDK based on your board.
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

// This is the address where we read the data back (memory-mapped flash)
extern const uint8_t *flash_target_contents;

// A magic number to know if flash has been initialized by us before
#define MAGIC 0xDEADBEEF

typedef struct {
    uint32_t magic;
    bool my_flag;
    uint8_t padding[3]; // align to 4 bytes (good practice)
} PersistentData;

#ifdef __cplusplus
extern "C" {
#endif

// Read the stored value (or return a default if flash is blank)
bool flash_read();

// Write the value to flash
void flash_write(bool value);

#ifdef __cplusplus
}
#endif
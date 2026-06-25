#include "flash.h"

const uint8_t *flash_target_contents = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);

bool flash_read() {
    const PersistentData *data = (const PersistentData *)flash_target_contents;
    if (data->magic == MAGIC) {
        return data->my_flag;
    }
    return false; // default value if never written
}

void flash_write(bool value) {
    PersistentData data = {
        .magic = MAGIC,
        .my_flag = value,
    };

    // Buffer must be a multiple of FLASH_PAGE_SIZE (256 bytes)
    uint8_t buf[FLASH_PAGE_SIZE];
    memset(buf, 0xFF, sizeof(buf));       // 0xFF = erased state of flash
    memcpy(buf, &data, sizeof(data));

    // Disable interrupts — code must run from RAM during flash write
    uint32_t interrupts = save_and_disable_interrupts();

    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);  // erase 4 KB sector
    flash_range_program(FLASH_TARGET_OFFSET, buf, FLASH_PAGE_SIZE); // write 256 bytes

    restore_interrupts(interrupts);
}
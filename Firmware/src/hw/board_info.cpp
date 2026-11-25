#include "board_info.h"

#include <Arduino.h> // UID_BASE

#include "board_config.h"
#include "supply.h"

static_assert(sizeof(BOARD_MODEL) - 1 <= 30, "BOARD_MODEL is reported in 5 CAN pages of 6 characters: max 30");

uint32_t boardCapabilities() {
    // One line per part type that has host-visible features.
    return supplyCapabilities();
}

const char *boardModel() { return BOARD_MODEL; }

void boardSerialBytes(uint8_t out[12]) {
    const volatile uint32_t *uid = reinterpret_cast<const volatile uint32_t *>(UID_BASE);
    for (int w = 0; w < 3; w++) {
        const uint32_t v = uid[w];
        out[w * 4 + 0] = (uint8_t)(v >> 24);
        out[w * 4 + 1] = (uint8_t)(v >> 16);
        out[w * 4 + 2] = (uint8_t)(v >> 8);
        out[w * 4 + 3] = (uint8_t)v;
    }
}

void boardSerialText(char out[25]) {
    static const char hex[] = "0123456789ABCDEF";
    uint8_t b[12];
    boardSerialBytes(b);
    for (int i = 0; i < 12; i++) {
        out[i * 2]     = hex[b[i] >> 4];
        out[i * 2 + 1] = hex[b[i] & 15];
    }
    out[24] = 0;
}

#include "board_info.h"

#include "board_config.h"
#include "supply.h"

static_assert(sizeof(BOARD_MODEL) - 1 <= 30, "BOARD_MODEL is reported in 5 CAN pages of 6 characters: max 30");

uint32_t boardCapabilities() {
    // One line per part type that has host-visible features.
    return supplyCapabilities();
}

const char *boardModel() { return BOARD_MODEL; }

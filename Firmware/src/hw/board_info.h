#pragma once

#include <cstdint>

// What the board reports about itself over CAN (CMD_GET_BOARD_INFO, see
// can_protocol.h) and on the serial console: ID, revision and model name from
// the board header, capabilities from its parts.

// CAP_* bits from can_protocol.h. Each part type that adds something the host
// can use contributes its bits here (see board_info.cpp).
uint32_t boardCapabilities();

// BOARD_MODEL from the board header, e.g. "MiniFOC Mk2".
const char *boardModel();

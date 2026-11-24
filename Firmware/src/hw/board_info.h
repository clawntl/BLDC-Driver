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

// The MCU's 96-bit unique ID, the board's serial number: the three 32-bit
// words, each big-endian (CMD_GET_BOARD_INFO pages 6-7).
void boardSerialBytes(uint8_t out[12]);
// The same as 24 hex digits plus a terminating NUL.
void boardSerialText(char out[25]);

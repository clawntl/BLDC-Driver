#pragma once

// Board type IDs, reported in CMD_GET_INFO byte 6 (see can_protocol.h) so a
// host can tell which hardware it is talking to. Byte 7 carries the board's
// hardware revision (BOARD_HW_REV in its board header).
//
// Add a new number for each new board design; never reuse or renumber one.
#define BOARD_ID_UNKNOWN      0 // firmware before 1.3.0 (sends 0 here)
#define BOARD_ID_BLDC_DRIVER  1 // BLDC-Driver (Hardware/bldc_driver_v02)
#define BOARD_ID_MINIFOC      2 // MiniFOC (Hardware/minifoc_mk2)

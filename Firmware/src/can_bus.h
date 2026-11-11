#pragma once

#include <cstdint>

// Configures FDCAN1 (bit rate, filters for this board's IDs) and starts it.
// Call once from setup(), and only when the board is configured (the IDs
// come from the saved board settings).
void configureAndStartCAN();

bool canStarted();

// Sends `dat` as a 5-byte classic CAN frame: [cmd][4 big-endian float bytes].
// No-op if CAN isn't running.
void sendFloat(uint8_t cmd, float dat, uint32_t canID);

// [CMD_ERROR][float code] / [CMD_START_FOC][float 1 or 0] to the host.
void sendError(uint8_t errorCode);
void sendStartResult(bool ok);

// Handles all received frames (current commands, CAN_CMDs), sends the status
// heartbeat, recovers from bus-off and reports transmit errors (rate-limited).
// Call once per loop() iteration.
void pollCAN();

// Drops any received frames still queued (used after the blocking FOC start).
void discardPendingCAN();

bool canBusOff();

#pragma once

#include <cstdint>

// Which pair of identifiers a board uses is chosen at RUNTIME from the side
// saved in its board settings (`set side left|right` over USB serial, see
// board_settings.h), so the same firmware image runs on both drivers.

// Host -> both drivers: 8 bytes = two big-endian floats, target current in A.
// Bytes 0-3 are for the left motor, bytes 4-7 for the right motor.
#define BOTH_MOTOR_CAN_ADDR     0x020

// Host -> one driver: command frames ([CAN_CMD][payload...]).
#define LEFT_MOTOR_CAN_ADDR     0x021
#define RIGHT_MOTOR_CAN_ADDR    0x022

// Driver -> host: telemetry/status frames ([CAN_CMD][payload...]).
#define LEFT_MOTOR_SEND_TO      0x101
#define RIGHT_MOTOR_SEND_TO     0x100

#define CAN_BIT_RATE     1'000'000 // 1 Mbit/s classic CAN 2.0B (no FD / bit rate switch used)
#define CAN_SAMPLE_POINT 88        // % ; resolves to an exact 87.5% sample point at this bit rate/clock

enum CAN_CMD : uint8_t {
    // Driver -> host, every SHAFT_VELOCITY_TELEMETRY_INTERVAL_US while FOC runs.
    // [0x02][float rad/s, big-endian]. Sign follows the invert setting.
    CMD_SHAFT_VELOCITY = 0x02,

    // Driver -> host, every CAN_STATUS_INTERVAL_MS (also while FOC is not running).
    // [0x03][DriverState][status flags (STATUS_FLAG_*)][0][0]
    // The host can wait for DRIVER_STATE_IDLE before sending CMD_START_FOC.
    CMD_DRIVER_STATUS = 0x03,

    // Host -> driver: start FOC (motor init + alignment, blocks ~1-3 s).
    // Driver -> host reply when finished: [0x10][float 1.0 = ok, 0.0 = failed]
    CMD_START_FOC = 0x10,

    // Driver -> host: [0xFF][float error code (CAN_ERROR)].
    CMD_ERROR = 0xFF,
};

enum DriverState : uint8_t {
    DRIVER_STATE_UNCONFIGURED = 0, // never sent (CAN is off until configured)
    DRIVER_STATE_IDLE         = 1, // waiting for CMD_START_FOC
    DRIVER_STATE_RUNNING      = 2, // FOC running, accepting current commands
    DRIVER_STATE_FAULT        = 3, // FOC start failed; CMD_START_FOC retries
};

enum StatusFlag : uint8_t {
    STATUS_FLAG_COMMAND_TIMEOUT  = 1 << 0, // no current command for COMMAND_TIMEOUT_MS -> 0 A applied
    STATUS_FLAG_CALIBRATION_USED = 1 << 1, // stored alignment used, initFOC alignment skipped
    STATUS_FLAG_INVERTED         = 1 << 2, // direction invert setting is on
    STATUS_FLAG_DRIVER_FAULT     = 1 << 3, // DRV8316 reported a fault at the last poll
};

enum CAN_ERROR : uint8_t {
    ERR_MOTOR_INIT        = 1, // SimpleFOC motor.init() failed (driver not initialised)
    ERR_CURRENT_SENSE     = 2, // current sense init failed
    ERR_FOC_INIT          = 3, // initFOC() failed (sensor/current-sense alignment)
    ERR_DRIVER_FAULT      = 4, // DRV8316 fault flag set
};

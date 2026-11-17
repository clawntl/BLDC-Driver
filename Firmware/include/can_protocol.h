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

// Version of this protocol (reported by CMD_GET_INFO). 1 = start/telemetry/
// status only (no CMD_GET_INFO reply); 2 = adds the configuration commands.
#define CAN_PROTOCOL_VERSION 2

// Firmware version, reported by CMD_GET_INFO and the serial `config` command.
#define FIRMWARE_VERSION_MAJOR 1
#define FIRMWARE_VERSION_MINOR 1
#define FIRMWARE_VERSION_PATCH 0

#define CAN_BIT_RATE     1'000'000 // 1 Mbit/s classic CAN 2.0B (no FD / bit rate switch used)
#define CAN_SAMPLE_POINT 88        // % ; resolves to an exact 87.5% sample point at this bit rate/clock

enum CAN_CMD : uint8_t {
    // Driver -> host, every SHAFT_VELOCITY_TELEMETRY_INTERVAL_US while FOC runs.
    // [0x02][float rad/s, big-endian]. Sign follows the invert setting.
    CMD_SHAFT_VELOCITY = 0x02,

    // Driver -> host, every CAN_STATUS_INTERVAL_MS (also while FOC is not running).
    // [0x03][DriverState][status flags (STATUS_FLAG_*)][supply mV, uint16 big-endian]
    // The host can wait for DRIVER_STATE_IDLE before sending CMD_START_FOC.
    // Supply mV = 0 means no reading yet (firmware before this field also sent 0).
    CMD_DRIVER_STATUS = 0x03,

    // Host -> driver: start FOC (motor init + alignment, blocks ~1-3 s).
    // Driver -> host reply when finished: [0x10][float 1.0 = ok, 0.0 = failed]
    CMD_START_FOC = 0x10,

    // ---- Configuration (protocol 2) --------------------------------------
    // Host -> driver on the board's command ID; the reply comes back on its
    // telemetry ID with the same command byte. Status bytes are CONFIG_*.

    // [0x20] -> [0x20][protocol version][fw major][fw minor][fw patch]
    //           [image size, KB][0][0]
    CMD_GET_INFO = 0x20,

    // [0x21][0 = active, 1 = pending] ->
    // [0x21][which][side (MotorSide)][CONFIG_FLAG_* bits][zero electric angle, float]
    CMD_GET_CONFIG = 0x21,

    // [0x22][PARAM_*][float value] -> [0x22][param][status][float value now]
    // Saved parameters (side, invert) change the PENDING settings: they take
    // effect after CMD_SAVE (which reboots). Runtime parameters apply at once
    // and are back to their defaults after a reset.
    CMD_SET_PARAM = 0x22,

    // [0x23][CAL_STORE | CAL_CLEAR] -> [0x23][op][status]
    // STORE keeps this boot's FOC alignment (FOC must be running; if the start
    // used the stored alignment this just stores the same values again).
    // CLEAR forgets it, so the next start aligns. Both pending until saved.
    CMD_CALIBRATION = 0x23,

    // [0x24] -> [0x24][status], then (if OK) the board stops the motor,
    // writes the pending settings to flash and reboots.
    CMD_SAVE = 0x24,

    // [0x25] -> [0x25][status]: discard unsaved changes.
    CMD_REVERT = 0x25,

    // [0x26] -> [0x26][status], then the board resets (motor stops).
    CMD_REBOOT = 0x26,

    // Driver -> host: [0xFE][command byte][CONFIG_ERR_UNKNOWN]: command not
    // understood (e.g. a newer host talking to this firmware).
    CMD_NACK = 0xFE,

    // Driver -> host: [0xFF][float error code (CAN_ERROR)].
    CMD_ERROR = 0xFF,
};

// CMD_SET_PARAM parameter IDs.
enum ConfigParam : uint8_t {
    PARAM_SIDE   = 1,    // saved: 1 = left, 2 = right (changes the CAN IDs after the reboot!)
    PARAM_INVERT = 2,    // saved: 0 / 1
    PARAM_VELOCITY_INTERVAL_MS = 0x10, // runtime: CMD_SHAFT_VELOCITY period, 1-100 ms (default 10)
};

enum CalibrationOp : uint8_t {
    CAL_STORE = 1,
    CAL_CLEAR = 2,
};

// Status byte in configuration replies.
enum ConfigStatus : uint8_t {
    CONFIG_OK               = 0,
    CONFIG_ERR_UNKNOWN      = 1, // unknown command / parameter / operation
    CONFIG_ERR_BAD_VALUE    = 2, // value out of range
    CONFIG_ERR_NOT_ALIGNED  = 3, // CAL_STORE without a fresh alignment this boot
    CONFIG_ERR_FLASH        = 4, // the firmware image overlaps the settings page
};

// CMD_GET_CONFIG flags.
enum ConfigFlag : uint8_t {
    CONFIG_FLAG_INVERT          = 1 << 0,
    CONFIG_FLAG_CALIBRATION     = 1 << 1, // an alignment is stored
    CONFIG_FLAG_SENSOR_DIR_CCW  = 1 << 2, // stored sensor direction is CCW (-1)
    CONFIG_FLAG_UNSAVED         = 1 << 3, // pending differs from active (needs CMD_SAVE)
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

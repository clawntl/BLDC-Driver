#pragma once

#include <cstdint>

// Each board has a NODE ID, 1-15, saved in its board settings (`set node N`
// over USB serial, or PARAM_NODE_ID over CAN; see board_settings.h), so the
// same firmware image runs on every board and a bus can carry up to 15.
// Its identifiers follow from the node ID n:
//
//   Current command   CURRENT_FRAME_BASE + (n-1)/2   host -> boards, 8 bytes =
//                     two big-endian floats [A]: odd n reads bytes 0-3, even n
//                     bytes 4-7. NaN = no command for that node (ignored).
//                     The lowest IDs, so they win arbitration over everything.
//   Commands          COMMAND_ID_BASE + n            host -> board: [CAN_CMD][payload...]
//   Telemetry/status  TELEMETRY_ID_BASE + n          board -> host: [CAN_CMD][payload...]
//
// Nodes 1 and 2 (e.g. a robot's left and right wheels) share current frame
// 0x010, use commands 0x021 / 0x022 and telemetry 0x101 / 0x102.
#define CURRENT_FRAME_BASE  0x010
#define COMMAND_ID_BASE     0x020
#define TELEMETRY_ID_BASE   0x100
#define NODE_ID_MIN         1
#define NODE_ID_MAX         15

// Version of this protocol (reported by CMD_GET_INFO). 1 = start/telemetry/
// status only (no CMD_GET_INFO reply); 2 = adds the configuration commands;
// 3 = node IDs instead of left/right (new current frame and telemetry IDs);
// 4 = CMD_GET_BOARD_INFO (model name + capabilities) and input current in
//     the heartbeat (CAP_MEASURES_INPUT_CURRENT);
// 5 = CMD_GET_PARAM, current loop parameters (PARAM_CURRENT_*) and the FOC
//     loop rate (PARAM_FOC_LOOP_HZ);
// 6 = the measured motor current (q and d axis) in CMD_SHAFT_VELOCITY;
// 7 = anti-cogging (CMD_COGGING, PARAM_COG_COMP) and the MCU serial number
//     (CMD_GET_BOARD_INFO pages 6-7).
#define CAN_PROTOCOL_VERSION 7

// Firmware version, reported by CMD_GET_INFO and the serial `config` command.
#define FIRMWARE_VERSION_MAJOR 1
#define FIRMWARE_VERSION_MINOR 7
#define FIRMWARE_VERSION_PATCH 1

// Scale of the d current in CMD_SHAFT_VELOCITY: 16 mA per count, +-2.03 A.
#define D_CURRENT_LSB_MA 16

#define CAN_BIT_RATE     1'000'000 // 1 Mbit/s classic CAN 2.0B (no FD / bit rate switch used)
#define CAN_SAMPLE_POINT 88        // % ; resolves to an exact 87.5% sample point at this bit rate/clock

enum CAN_CMD : uint8_t {
    // Driver -> host, every SHAFT_VELOCITY_TELEMETRY_INTERVAL_US while FOC runs.
    // [0x02][float rad/s, big-endian][q current, int16 mA big-endian]
    //      [d current, int8, units of D_CURRENT_LSB_MA]
    // The currents are the MEASURED ones the current loop works with
    // (filtered, as SimpleFOC uses them); protocol 6, firmware 1.6+. The
    // velocity and q current follow the invert setting (the same sign as
    // the current commands); d is as measured. Older firmware sends only
    // the first 5 bytes, so check the length.
    CMD_SHAFT_VELOCITY = 0x02,

    // Driver -> host, every CAN_STATUS_INTERVAL_MS (also while FOC is not running).
    // [0x03][DriverState][status flags (STATUS_FLAG_*)][supply mV, uint16 big-endian]
    //      [input current mA, int16 big-endian]   <- only with CAP_MEASURES_INPUT_CURRENT
    // The host can wait for DRIVER_STATE_IDLE before sending CMD_START_FOC.
    // Supply mV = 0 means no reading yet (firmware before this field also sent 0).
    // Input current: positive = drawn from the supply; INT16_MIN (0x8000) = no
    // reading. Boards without CAP_MEASURES_INPUT_CURRENT send 5 bytes.
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
    // [0x21][which][node ID (0 = not set)][CONFIG_FLAG_* bits][zero electric angle, float]
    CMD_GET_CONFIG = 0x21,

    // [0x22][PARAM_*][float value] -> [0x22][param][status][float value now]
    // Saved parameters (node ID, invert) change the PENDING settings: they take
    // effect after CMD_SAVE (which reboots). Runtime parameters apply at once
    // and are back to their defaults after a reset. Tuning parameters (the
    // current loop) do both: they apply at once AND change the pending
    // settings, so CMD_SAVE keeps them and CMD_REVERT goes back to the saved
    // values.
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

    // Board description (protocol 4), so a host needs no table of boards:
    // [0x27][page] ->
    //   page 0:    [0x27][0][capabilities, uint32 big-endian (CAP_*)][hw rev][0]
    //   page 1-5:  [0x27][page][6 characters of the model name, e.g. "MiniFO"]
    //              (pages 1-5 = characters 0-29; NUL-padded after the end, so
    //              stop at the first page containing a NUL)
    //   page 6-7:  [0x27][page][6 bytes of the MCU's 96-bit unique ID]: the
    //              board's serial number (protocol 7). Page 6 = bytes 0-5,
    //              page 7 = bytes 6-11: the three 32-bit ID words, each
    //              big-endian. Shown as 24 hex digits. Older firmware answers
    //              as for "other".
    //   other:     [0xFE][0x27][CONFIG_ERR_BAD_VALUE]
    // Firmware before protocol 4 NACKs it ([0xFE][0x27][CONFIG_ERR_UNKNOWN]).
    CMD_GET_BOARD_INFO = 0x27,

    // Read one parameter (protocol 5): [0x28][PARAM_*] ->
    // [0x28][param][status][float value]. Saved parameters read the PENDING
    // value; runtime and tuning parameters the value in use.
    CMD_GET_PARAM = 0x28,

    // Anti-cogging (protocol 7): [0x29][COG_*] ->
    //   COG_STATUS, COG_CALIBRATE, COG_REFINE, COG_ABORT, COG_CLEAR:
    //     [0x29][op][status][CogState][progress % (during / after a
    //      calibration) or CogFailure (state COG_STATE_FAILED)]
    //     [COG_FLAG_* bits][table peak-to-peak, 4 mA units][friction, 2 mA units]
    //   COG_RESULT:
    //     [0x29][op][status][coverage %][forward/backward agreement %]
    //     [largest position error in the last calibration, 0.1 mrad units, 255 = more]
    //     [noise left out of the table, 0.5 mA units (0 = none since boot)][0]
    //   COG_READ: [0x29][6][bin hi][bin lo] ->
    //     [0x29][6][bin hi][bin lo][table counts of bins bin..bin+3, int8]
    //     (bin hi 0xFF in the reply = no table / bad bin). COG_INFO gives the scale.
    //   COG_INFO: [0x29][7] ->
    //     [0x29][7][status][A per table count, float big-endian][bins / 16][0]
    //     (status CONFIG_ERR_BAD_VALUE without a table)
    // CALIBRATE turns the rotor slowly one turn each way (~55 s) with the
    // board's own position loop and records the current it needs at each
    // angle: FOC must be running (else CONFIG_ERR_NOT_RUNNING) and the wheel
    // free to turn. REFINE does the same with the existing table applied and
    // adds what is left (CONFIG_ERR_BAD_VALUE without a table for this
    // motor). A current command above 50 mA aborts either. Poll COG_STATUS
    // for progress; the reply to CALIBRATE / REFINE comes straight away.
    // A new table, or CLEAR, is pending until CMD_SAVE (CMD_REVERT brings
    // back the stored one); CONFIG_FLAG_UNSAVED includes it. Compensation
    // is switched by PARAM_COG_COMP.
    CMD_COGGING = 0x29,

    // Driver -> host: [0xFE][command byte][CONFIG_ERR_UNKNOWN]: command not
    // understood (e.g. a newer host talking to this firmware).
    CMD_NACK = 0xFE,

    // Driver -> host: [0xFF][float error code (CAN_ERROR)].
    CMD_ERROR = 0xFF,
};

// CMD_SET_PARAM parameter IDs.
enum ConfigParam : uint8_t {
    PARAM_NODE_ID = 1,   // saved: 1-15 (the board's CAN IDs change after the reboot!)
    PARAM_INVERT = 2,    // saved: 0 / 1
    PARAM_VELOCITY_INTERVAL_MS = 0x10, // runtime: CMD_SHAFT_VELOCITY period, 1-100 ms (default 10)
    // Tuning (protocol 5): apply at once and are kept by CMD_SAVE.
    PARAM_CURRENT_P  = 0x20, // current loop P, V/A, 0-50 (default 3)
    PARAM_CURRENT_I  = 0x21, // current loop I, V/(A s), 0-100000 (default 300)
    PARAM_CURRENT_TF = 0x22, // current measurement filter time constant, s, 0-0.01 (default 0.00053)
    PARAM_COG_COMP   = 0x23, // anti-cogging compensation: 0 off (default), 1 on, 2 slots only (protocol 7)
    // Read-only (CMD_GET_PARAM; CMD_SET_PARAM answers CONFIG_ERR_UNKNOWN).
    PARAM_FOC_LOOP_HZ = 0x30, // FOC loop rate while running, Hz (0 = not running)
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
    CONFIG_ERR_NOT_RUNNING  = 5, // needs FOC running (CMD_COGGING calibrate / refine)
};

// CMD_COGGING operations.
enum CogOp : uint8_t {
    COG_STATUS    = 0,
    COG_CALIBRATE = 1,
    COG_REFINE    = 2,
    COG_ABORT     = 3,
    COG_CLEAR     = 4,
    COG_RESULT    = 5,
    COG_READ      = 6,
    COG_INFO      = 7,
};

// CMD_COGGING calibration state (Cogging::State).
enum CogState : uint8_t {
    COG_STATE_IDLE     = 0, // no calibration since boot
    COG_STATE_SETTLING = 1,
    COG_STATE_FORWARD  = 2,
    COG_STATE_BACKWARD = 3,
    COG_STATE_DONE     = 4, // last calibration succeeded
    COG_STATE_FAILED   = 5, // last calibration failed (CogFailure in byte 4)
};

enum CogFailure : uint8_t {
    COG_FAIL_NONE         = 0,
    COG_FAIL_NOT_RUNNING  = 1, // FOC stopped
    COG_FAIL_HOST_COMMAND = 2, // a current command arrived
    COG_FAIL_STALLED      = 3, // the rotor didn't follow (wheel blocked?)
    COG_FAIL_ABORTED      = 4, // COG_ABORT, `cog abort`
    COG_FAIL_COVERAGE     = 5, // angles missed in a pass
    COG_FAIL_NO_TABLE     = 6, // refine without a table
};

enum CogFlag : uint8_t {
    COG_FLAG_TABLE     = 1 << 0, // a table is loaded (or newly calibrated)
    COG_FLAG_ENABLED   = 1 << 1, // compensation switched on (PARAM_COG_COMP)
    COG_FLAG_UNSAVED   = 1 << 2, // table calibrated / cleared since the last save
    COG_FLAG_MISMATCH  = 1 << 3, // table measured with the other sensor direction: not applied
    COG_FLAG_APPLIED   = 1 << 4, // compensation is being applied now (enabled, table matches, FOC running)
    COG_FLAG_REFINED   = 1 << 5, // the table's last calibration was a refine
    COG_FLAG_SLOTS_ONLY = 1 << 6, // compensation mode 2: harmonics 1..COG_LOW_HARMONICS per turn left out
};

// CMD_GET_CONFIG flags.
enum ConfigFlag : uint8_t {
    CONFIG_FLAG_INVERT          = 1 << 0,
    CONFIG_FLAG_CALIBRATION     = 1 << 1, // an alignment is stored
    CONFIG_FLAG_SENSOR_DIR_CCW  = 1 << 2, // stored sensor direction is CCW (-1)
    CONFIG_FLAG_UNSAVED         = 1 << 3, // pending differs from active (needs CMD_SAVE)
};

// CMD_GET_BOARD_INFO capability bits: features a host can use, independent
// of which board it is. Add a bit when a board gains something the host can
// see; never reuse one.
enum BoardCapability : uint32_t {
    CAP_MEASURES_SUPPLY_VOLTAGE = 1u << 0, // heartbeat supply mV is a measurement
    CAP_MEASURES_INPUT_CURRENT  = 1u << 1, // heartbeat bytes 5-6: input current in mA
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

#pragma once

// Motor construction parameter: number of pole pairs (magnet pole pairs on
// the rotor). See the motor's datasheet or count magnets and divide by 2.
#define MOTOR_POLE_PAIRS 11

// Motor and control settings. Board hardware (pins, sensors, current sense,
// supply sensing) is in boards/<name>/board_config.h, see docs/firmware.md#boards.

// Supply voltage feeding the DRV8316, in Volts. Used only as a FALLBACK: the
// actual value is measured at startup by the board's supply sensor (see
// hw/supply.h) and used if it falls within the sanity range below.
#define MOTOR_VOLTAGE_POWER_SUPPLY 12.0f

// A measured VIN outside this range is treated as a bad reading (e.g. board
// powered from USB only) and MOTOR_VOLTAGE_POWER_SUPPLY is used instead.
// DRV8316 operating range is 4.5-35 V.
#define SUPPLY_VOLTAGE_MIN  6.0f
#define SUPPLY_VOLTAGE_MAX  30.0f

// Driver voltage limit, in Volts.
#define MOTOR_VOLTAGE_LIMIT 10.0f

// Motor phase current limit, in Amps.
#define MOTOR_CURRENT_LIMIT 2.0f

// Current (torque) loop: the q and d axis PI controllers and the filter on
// the measured currents. These are the defaults; each board keeps its own
// values in its settings (`set curp/curi/curtf` over USB, PARAM_CURRENT_* over
// CAN) and they can be changed while the motor runs. The defaults are
// SimpleFOC's own, which suit a low-resistance motor: on a 5.6 ohm gimbal
// motor (GM3506) the loop then follows a step in about 30 ms and lags behind
// back-EMF when the wheel speeds up. Raising I speeds it up (closed-loop time
// constant roughly (R + P) / I); see docs/firmware.md#current-loop.
#define CURRENT_LOOP_P_DEFAULT   3.0f      // V per A
#define CURRENT_LOOP_I_DEFAULT   300.0f    // V per A per s
#define CURRENT_LOOP_TF_DEFAULT  0.00053f  // s, = 1 / (2 pi 300 Hz)
// Accepted ranges.
#define CURRENT_LOOP_P_MAX       50.0f
#define CURRENT_LOOP_I_MAX       100000.0f
#define CURRENT_LOOP_TF_MAX      0.01f

// Voltage applied during sensor alignment at startup, in Volts.
#define MOTOR_VOLTAGE_SENSOR_ALIGN 3.0f

// How often the shaft velocity is broadcast over CAN, in microseconds.
#define SHAFT_VELOCITY_TELEMETRY_INTERVAL_US 10'000

// If no current command (in this node's current frame) arrives for this long while FOC
// is running, the target current is forced to 0 A (motor freewheels) until
// commands resume. Protects against a crashed host or unplugged CAN cable.
#define COMMAND_TIMEOUT_MS 100

// How often the CMD_DRIVER_STATUS heartbeat is sent over CAN, in milliseconds.
// It also carries the live supply voltage (see hw/supply.h).
#define CAN_STATUS_INTERVAL_MS 100

// How often the DRV8316 fault status is polled over SPI while FOC runs.
#define DRV_STATUS_POLL_INTERVAL_MS 500

// Independent watchdog timeout, in microseconds. Started once FOC is running
// (initFOC() blocks for seconds, so it can't be running before that) and
// reloaded every loop(). Once started it cannot be stopped until reset.
#define WATCHDOG_TIMEOUT_US 250'000

// At boot, wait up to this long for a serial terminal to open so the startup
// messages aren't missed. Continues immediately once a terminal connects.
#define BOOT_SERIAL_WAIT_MS 2'000

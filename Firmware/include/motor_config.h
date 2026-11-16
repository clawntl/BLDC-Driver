#pragma once

// Motor construction parameter: number of pole pairs (magnet pole pairs on
// the rotor). See the motor's datasheet or count magnets and divide by 2.
#define MOTOR_POLE_PAIRS 11

// Supply voltage feeding the DRV8316, in Volts. Used only as a FALLBACK: the
// actual value is measured at startup by the INA226 power monitor (see
// power_monitor.h) and used if it falls within the sanity range below.
#define MOTOR_VOLTAGE_POWER_SUPPLY 12.0f

// A measured VIN outside this range is treated as a bad reading (e.g. board
// powered from USB only, or INA226 not fitted) and MOTOR_VOLTAGE_POWER_SUPPLY
// is used instead. DRV8316 operating range is 4.5-35 V.
#define SUPPLY_VOLTAGE_MIN  6.0f
#define SUPPLY_VOLTAGE_MAX  30.0f

// INA226 power monitor on I2C1: shunt R1 (3 mOhm) between V_IN and V_M.
#define INA226_I2C_ADDR     0x40
#define INA226_SHUNT_OHMS   0.003f

// Inline phase current sense: 3 mOhm shunts (R4 / R8 / R17 on phases A / B / C)
// into INA241A3 amplifiers with a gain of 50 V/V -> 0.15 V/A, +-11 A range.
#define CURRENT_SENSE_SHUNT_OHMS 0.003f
#define CURRENT_SENSE_GAIN       50.0f

// Driver voltage limit, in Volts.
#define MOTOR_VOLTAGE_LIMIT 10.0f

// Motor phase current limit, in Amps.
#define MOTOR_CURRENT_LIMIT 2.0f

// Voltage applied during sensor alignment at startup, in Volts.
#define MOTOR_VOLTAGE_SENSOR_ALIGN 3.0f

// How often the shaft velocity is broadcast over CAN, in microseconds.
#define SHAFT_VELOCITY_TELEMETRY_INTERVAL_US 10'000

// If no BOTH_MOTOR_CAN_ADDR current command arrives for this long while FOC
// is running, the target current is forced to 0 A (motor freewheels) until
// commands resume. Protects against a crashed host or unplugged CAN cable.
#define COMMAND_TIMEOUT_MS 100

// How often the CMD_DRIVER_STATUS heartbeat is sent over CAN, in milliseconds.
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

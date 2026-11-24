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

// Phase voltage range, in Volts: the PWM outputs swing between 0 and this
// (driver.voltage_limit). It is set from the supply as the supply changes
// (see MOTOR_MAX_DUTY), and never above MOTOR_VOLTAGE_LIMIT.
//
// How much of it reaches the motor depends on the modulation. With Space
// Vector PWM (used since firmware 1.5) the largest undistorted q-axis
// voltage is this / sqrt(3), which is also what the current loop may ask
// for (motor.voltage_limit). With Sine PWM (before 1.5) it was only this / 2:
// 5 V with the old fixed 10 V limit, which capped a GM3506 at about 137
// rad/s with no current left at all, and full current only to ~20 rad/s.
//
// A cap for higher-voltage supplies; with a 3S pack (9.6-12.6 V) the duty
// limit below is what applies.
#define MOTOR_VOLTAGE_LIMIT 12.0f

// Largest PWM duty used: driver.voltage_limit = MOTOR_MAX_DUTY x the supply
// voltage (measured live, filtered). The DRV8316's current-sense amplifiers
// measure through the low-side FETs, so every phase needs some low-side on
// time in each PWM period for the current to be sampled. At 20 kHz, 0.92
// leaves 4 us. The q-axis voltage available is then
// 0.92 x supply / sqrt(3): 6.7 V at 12.6 V, 5.9 V at 11.1 V, 5.1 V at 9.6 V.
#define MOTOR_MAX_DUTY 0.92f

// How often the supply voltage is followed while FOC runs, in milliseconds,
// and the time constant of the filter on it, in seconds.
#define SUPPLY_TRACK_INTERVAL_MS 20
#define SUPPLY_TRACK_FILTER_S    0.5f

// Motor phase current limit, in Amps.
#define MOTOR_CURRENT_LIMIT 2.0f

// Current (torque) loop: the q and d axis PI controllers and the filter on
// the measured currents. These are the defaults; each board keeps its own
// values in its settings (`set curp/curi/curtf` over USB, PARAM_CURRENT_* over
// CAN) and they can be changed while the motor runs. The defaults are
// SimpleFOC's own, which suit a low-resistance motor: on a GM3506 gimbal
// motor (2.8 ohm per phase, 5.6 line-to-line) the loop then follows a step in
// about 20 ms and lags behind back-EMF when the wheel speeds up. Raising I
// speeds it up (closed-loop time constant roughly (R + P) / I); see
// docs/firmware.md#current-loop.
#define CURRENT_LOOP_P_DEFAULT   3.0f      // V per A
#define CURRENT_LOOP_I_DEFAULT   300.0f    // V per A per s
#define CURRENT_LOOP_TF_DEFAULT  0.00053f  // s, = 1 / (2 pi 300 Hz)
// Accepted ranges.
#define CURRENT_LOOP_P_MAX       50.0f
#define CURRENT_LOOP_I_MAX       100000.0f
#define CURRENT_LOOP_TF_MAX      0.01f

// Anti-cogging (see src/cogging.h and docs/firmware.md#anti-cogging).
// Calibration: the position loop that turns the rotor (defaults; `cog cal`
// takes others), how fast, the most current it may use, and when it gives
// up. Worked out in a simulation of a GM3506 wheel (inertia 1.4e-5 kg m^2,
// Kt 0.054 N m/A) with a 2 ms current loop: much stiffer and the loop
// oscillates, much softer and the rotor snaps from one cogging step to the
// next instead of following.
#define COG_CAL_KP             20.0f   // A/rad
#define COG_CAL_KD             0.08f   // A/(rad/s)
#define COG_CAL_SPEED          0.25f   // rad/s: one turn each way takes ~26 s
#define COG_CAL_CURRENT_LIMIT  1.0f    // A
#define COG_CAL_STALL_RAD      0.2f    // position error that counts as "not following"
// During a calibration the current loop's I is raised to at least this (the
// position loop needs the current within ~2 ms) and the velocity filter is
// shortened to this; both go back afterwards.
#define COG_CAL_CURRENT_I      3000.0f // V/(A s)
#define COG_CAL_VELOCITY_TF    0.001f  // s
// A host current command bigger than this aborts a calibration.
#define COG_HOST_ABORT_A       0.05f   // A
// The table keeps harmonics 1..COG_HARMONICS per turn (plus 264 and 528,
// the slot/pole cogging of a 24-slot 22-pole motor); the rest of each bin is
// measurement noise. The GM3506s measured so far cog mostly at 2, 22-26 and
// 42-48 per turn.
#define COG_HARMONICS          64
// "Slots only" compensation (cogComp 2) leaves out harmonics 1..this.
#define COG_LOW_HARMONICS      3
// Compensation: the table is read ahead of the rotor by the current loop's
// response time, (MOTOR_PHASE_RESISTANCE + P) / I (1.9 ms with P 3, I 3000),
// and fades out between these shaft speeds (at 15 rad/s the 48-per-turn
// component comes round 115 times a second, more than the current loop can
// follow).
#define MOTOR_PHASE_RESISTANCE 2.8f    // ohm per phase (GM3506; its datasheet's 5.6 is line-to-line)
#define COG_FADE_START         6.0f    // rad/s
#define COG_FADE_END           15.0f   // rad/s

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

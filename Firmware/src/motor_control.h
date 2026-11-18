#pragma once

#include <SimpleFOC.h>

#include "board_settings.h"

extern BLDCMotor motor;

// One-time driver/sensor/FOC configuration, including the direction and
// stored alignment from the board settings. Call once from setup().
void setupFOC();

// Asks for FOC to be started (motor init + alignment, blocks ~1-3 s). The
// start itself runs from motorControlLoop(), not from the caller, so it never
// happens in the middle of CAN/serial processing. The result is reported over
// CAN (CMD_START_FOC reply, CMD_ERROR on failure). A failed start can be
// retried by requesting again.
void requestStartFOC();

// New target current from the host, in Amps (before direction inversion).
// NaN/Inf are ignored; the value is clamped to +-MOTOR_CURRENT_LIMIT. Also
// feeds the command timeout (COMMAND_TIMEOUT_MS).
void setTargetCurrent(float amps);

// Runs a pending start request, then (once FOC runs) one FOC iteration, the
// command timeout, velocity telemetry and DRV8316 fault polling.
// Call once per loop() iteration.
void motorControlLoop();

// Disables the driver outputs and stops FOC (used before writing flash).
// FOC is only restarted by a reboot.
void stopMotor();

bool focRunning();

// Period of the CMD_SHAFT_VELOCITY telemetry. Runtime only (set by the host
// with PARAM_VELOCITY_INTERVAL_MS, back to the default after a reset).
void setVelocityTelemetryIntervalMs(uint8_t ms);
uint8_t velocityTelemetryIntervalMs();

// Current loop (PI gains on both axes and the current filter) from these
// settings, applied at once. The CAN and serial commands change the pending
// settings and apply them; a revert applies the active ones again.
void applyCurrentLoopSettings(const BoardSettings &s);
// Values in use.
float currentLoopP();
float currentLoopI();
float currentLoopTf();
// FOC loop rate [Hz] while running (SimpleFOC's filtered loop time), else 0.
float focLoopHz();

// DriverState / StatusFlag values from can_protocol.h, for the CAN heartbeat.
uint8_t driverState();
uint8_t statusFlags();

// Supply voltage measured at boot (or the fallback default), in Volts.
float measuredSupplyVoltage();

// Target current currently applied (after inversion/timeout), in Amps.
float commandedCurrent();

// True if motor.sensor_direction / zero_electric_angle are valid, i.e. FOC
// has started successfully this boot (needed for `cal store`).
bool motorCalibrated();

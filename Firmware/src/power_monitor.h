#pragma once

// Supply monitoring with the on-board INA226 (U13) on I2C1:
//   V_IN -> R1 3 mOhm shunt -> V_M (DRV8316 + 5 V buck)
// Bus voltage is measured on the V_IN side. Current is positive when the
// board draws from the supply and negative when the motor regenerates.
//
// Unlike the analogRead()-based V_SENS divider on BLDC-Driver, this goes over
// I2C, so it is safe to call at any time, including while FOC is running
// (each read blocks for roughly 100 us).

// Starts I2C1 and configures the INA226 (16-sample averaging, continuous).
// Returns false if the chip doesn't answer with the expected IDs.
bool powerMonitorBegin();

bool powerMonitorPresent();

// Each returns false (and leaves `out` unchanged) if the read failed.
bool readSupplyVoltage(float &volts);
bool readSupplyCurrent(float &amps);

// Measures VIN and returns it if it is within [SUPPLY_VOLTAGE_MIN,
// SUPPLY_VOLTAGE_MAX]; otherwise prints a warning and returns the fallback
// MOTOR_VOLTAGE_POWER_SUPPLY from motor_config.h.
float measuredSupplyVoltageOrDefault();

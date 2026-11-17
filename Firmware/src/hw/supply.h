#pragma once

#include <cstdint>

// Supply (VIN / battery) sensing interface. Implemented by exactly ONE file
// in src/parts/ (supply_*.cpp), chosen by the board header:
//   SUPPLY_SENSE_ADC_DIVIDER  supply_vsens.cpp   (voltage divider on an ADC1 pin)
//   SUPPLY_SENSE_INA226       supply_ina226.cpp  (INA226 on I2C1: voltage + current)
// None or two selected = link error. supply_common.cpp holds the parts every
// sensor shares.
//
// The live values go to the host in the CAN status heartbeat
// (CMD_DRIVER_STATUS, see can_protocol.h).

// CAP_* bits from can_protocol.h this sensor provides (CAP_SUPPLY_VOLTAGE,
// CAP_INPUT_CURRENT).
uint32_t supplyCapabilities();

// Starts the sensor (may do nothing). Call once from setup(), before setupFOC().
void supplyBegin();

// Direct, immediate readings. Return false (and leave `out` unchanged) if
// there is no reading: sensor missing / not answering, the sensor can't
// measure it, or - for the ADC divider - once the current sense has started
// (see supplyCurrentSenseStarted()).
bool readSupplyVoltage(float &volts);
// Input current, positive when the board draws from the supply.
bool readSupplyCurrent(float &amps);

// Measures VIN and returns it if it is within [SUPPLY_VOLTAGE_MIN,
// SUPPLY_VOLTAGE_MAX]; otherwise prints a warning and returns the fallback
// MOTOR_VOLTAGE_POWER_SUPPLY from motor_config.h. Used at boot for
// SimpleFOC's volts -> PWM duty scaling.
float measuredSupplyVoltageOrDefault();

// Call immediately after current_sense.init(). Sensors that share the ADC
// with the current sense have to switch how they read; others ignore it.
// Safe to call twice.
void supplyCurrentSenseStarted();

// Samples the live values now and then. Call once per loop().
void updateSupplyMonitor();

// Latest live supply voltage in volts, 0 if there is no reading.
float supplyVoltageNow();

// Latest live input current. False if there is no reading (or no current
// measurement on this board).
bool supplyCurrentNow(float &amps);

// For the heartbeat: millivolts clamped to 1..65535, 0 = no reading.
uint16_t supplyMillivolts();

// For the heartbeat: milliamps clamped to +-32767, INT16_MIN = no reading.
int16_t supplyMilliamps();

// Short description for messages, e.g. "V_SENS" or "INA226".
const char *supplySensorName();

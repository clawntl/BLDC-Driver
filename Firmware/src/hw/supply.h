#pragma once

#include <cstdint>

#include "board.h"

// Supply (VIN / battery) sensing. One implementation per sensor type, chosen
// by the board header:
//   SUPPLY_SENSE_ADC_DIVIDER  supply_adc_divider.cpp  (BLDC-Driver: V_SENS on ADC1)
//   SUPPLY_SENSE_INA226       supply_ina226.cpp       (MiniFOC: INA226 on I2C1, + current)
// The live value is reported to the host in the CAN status heartbeat
// (CMD_DRIVER_STATUS, see can_protocol.h).

#if defined(SUPPLY_SENSE_INA226)
#define SUPPLY_HAS_CURRENT 1
#endif

// Starts the sensor (a no-op for the ADC divider). Call once from setup(),
// before setupFOC().
void supplyBegin();

// Direct, immediate readings. Return false (and leave `out` unchanged) if
// there is no reading: sensor missing / not answering, or - for the ADC
// divider - once the current sense has started (see supplyCurrentSenseStarted()).
bool readSupplyVoltage(float &volts);
// Input current, positive when the board draws from the supply. Always false
// without SUPPLY_HAS_CURRENT.
bool readSupplyCurrent(float &amps);

// Measures VIN and returns it if it is within [SUPPLY_VOLTAGE_MIN,
// SUPPLY_VOLTAGE_MAX]; otherwise prints a warning and returns the fallback
// MOTOR_VOLTAGE_POWER_SUPPLY from motor_config.h. Used at boot for
// SimpleFOC's volts -> PWM duty scaling.
float measuredSupplyVoltageOrDefault();

// Call immediately after current_sense.init(). The ADC divider shares ADC1
// with the current-sense pins on BLDC-Driver and has to switch how it reads
// (see supply_adc_divider.cpp); other sensors ignore it. Safe to call twice.
void supplyCurrentSenseStarted();

// Samples the live supply voltage now and then. Call once per loop().
void updateSupplyMonitor();

// Latest live supply voltage in volts, 0 if there is no reading.
float supplyVoltageNow();

// The same for the heartbeat: millivolts clamped to 1..65535, 0 = no reading.
uint16_t supplyMillivolts();

// Short description for boot messages, e.g. "V_SENS" or "INA226".
const char *supplySensorName();

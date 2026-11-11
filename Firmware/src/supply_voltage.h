#pragma once

#include <cstdint>

// Supply (VIN) measurement via the on-board V_SENS divider:
//   VIN -> R17 100k -> PA3 (V_SENS) -> R16 10k -> GND   =>  V_PA3 = VIN / 11
//
// !! Only call these BEFORE current_sense.init() !!
// STM32duino's analogRead() initialises the ADC for each read and then
// HAL_ADC_DeInit()s it. PA3 is on ADC1 together with the SOA/SOB/SOC
// current-sense pins, so an analogRead() after SimpleFOC has configured the
// ADC for low-side current sensing would silently break current sensing.

// Averaged supply voltage in volts.
float readSupplyVoltage(uint16_t samples = 64);

// Measures VIN and returns it if it is within [SUPPLY_VOLTAGE_MIN,
// SUPPLY_VOLTAGE_MAX]; otherwise prints a warning and returns the fallback
// MOTOR_VOLTAGE_POWER_SUPPLY from motor_config.h.
float measuredSupplyVoltageOrDefault();

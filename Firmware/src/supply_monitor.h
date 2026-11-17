#pragma once

#include <cstdint>

// Live supply (VIN / battery) voltage, reported to the host in the CAN status
// heartbeat (see can_protocol.h, CMD_DRIVER_STATUS).
//
// V_SENS (PA3) is on ADC1, which SimpleFOC's low-side current sensing also
// uses, so it is read in two different ways:
//  - before the current sense has been initialised: analogRead() (it
//    initialises and de-initialises ADC1 for every read, which is fine while
//    nothing else uses ADC1);
//  - afterwards: a software-triggered REGULAR conversion on ADC1. The current
//    sense uses INJECTED conversions triggered by the PWM timer, which have
//    priority and simply interrupt a regular conversion, so the two coexist.
//    analogRead() must never be used again from that point (see
//    supply_voltage.h).
//
// The regular conversion is started in one call and collected in a later one
// (no busy-waiting in the FOC loop).

// Call once from setupFOC(), after the boot measurement (supply_voltage.h).
void initSupplyMonitor();

// Call immediately after a successful current_sense.init(). Switches to
// regular conversions and sets a long sample time on the V_SENS channel (the
// ~9 kohm divider has no filter capacitor, so the default 2.5-cycle sample
// time would read low). Safe to call more than once.
void supplyMonitorCurrentSenseStarted();

// Takes a sample every SUPPLY_SAMPLE_INTERVAL_MS. Call once per loop().
void updateSupplyMonitor();

// Filtered supply voltage in volts (time constant ~SUPPLY_FILTER_TAU_MS).
float supplyVoltageNow();

// For the heartbeat: millivolts, clamped to 1..65535 (0 means "unknown").
uint16_t supplyMillivolts();

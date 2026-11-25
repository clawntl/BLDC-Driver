#pragma once

// Phase current sense interface. Implemented by exactly ONE file in
// src/parts/ (current_*.cpp), chosen by the board header (e.g.
// CURRENT_SENSE_INLINE). None or two selected = link error.
//
// To add one: a new src/parts/current_<type>.cpp wrapped in
// #if defined(CURRENT_SENSE_<TYPE>), implementing everything below.

#include <SimpleFOC.h>

#include "drivers/drv8316/drv8316.h"

// The SimpleFOC current sense object.
CurrentSense &boardCurrentSense();

// e.g. "inline shunts"
const char *currentSenseName();

// True if the offset calibration in init() needs the PWM running (low-side
// sensing samples synchronised to the PWM); false = calibrate with the power
// stage off.
bool currentSenseCalibratesWithPwm();

// Expected zero-current offset, for the motor_test printout.
const char *currentSenseOffsetHint();

// True if init() checks the readings well enough (zero-current offsets in
// the expected range) that SimpleFOC's driverAlign() check - which pulses
// phases A and B for ~2.2 s, twitching the wheel - can be skipped when a
// stored alignment is used. The phase-to-ADC wiring is fixed on the PCB;
// a full alignment (no stored one) still runs the check.
bool currentSenseCanSkipAlign();

// Called from configureDRV8316(): DRV8316 settings this current sense needs
// (e.g. the CSA gain for the DRV8316's own amplifiers). May do nothing.
void currentSensePrepareDriver(DRV8316Driver &driver);

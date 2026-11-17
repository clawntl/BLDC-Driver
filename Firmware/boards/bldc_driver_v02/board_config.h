#pragma once

// BLDC-Driver v0.2 (this repository's Hardware/): STM32G431CBU6 + DRV8316 +
// AS5048A, DRV8316 low-side current sense, VIN through a resistor divider.
//
// Found as "board_config.h" through -I boards/bldc_driver_v02 (see board.ini here).
// See docs/firmware.md#boards for what a board header has to define.

#include "board_ids.h"

#define BOARD_ID      BOARD_ID_BLDC_DRIVER
#define BOARD_HW_REV  2 // v0.2
#define BOARD_MODEL   "BLDC-Driver v0.2"   // reported over CAN, max 30 characters

// ---- Pins -------------------------------------------------------------------

// The generic G431CB variant defines LED_BUILTIN as "not defined"; replace it.
#undef LED_BUILTIN
#define LED_BUILTIN PC6

#define SPI1_SCK   PA5
#define SPI1_MISO  PA6
#define SPI1_MOSI  PA7
#define DRV_CS     PA4
#define ENC_CS     PC4

#define CAN_EN  PC10 // CAN transceiver enable: HIGH = on

// FDCAN1 RX is on PB8, which is also BOOT0 (10k pull-down + BOOT button to
// 3V3). See softwareReset() in dfu.cpp for why that matters.
#define CAN_RX_BOOT0 PB8

#define PHA_H   PA10
#define PHA_L   PB15

#define PHB_H   PA9
#define PHB_L   PB14

#define PHC_H   PA8
#define PHC_L   PB13

#define SO_A    PA0
#define SO_B    PA1
#define SO_C    PA2

#define V_SENS  PA3 // VIN / 11 via R17 100k / R16 10k divider

// ---- Parts (each selects one file in src/parts/) ----------------------------

// USER button to GND: pressing it while the firmware runs enters USB DFU.
#define HAS_DFU_BUTTON 1
#define DFU_BUTTON     PB12

// Encoder on SPI1: src/parts/encoder_as5048a.cpp
#define ENCODER_AS5048A 1
#define ENCODER_SPI_HZ  5'000'000

// Phase current: the DRV8316's own low-side current-sense amplifiers,
// src/parts/current_drv8316_lowside.cpp.
// 600 mV/A matches the DRV8316 CSA gain register code below (Gain_0V25). The
// DRV8316 datasheet's gain table had an erratum: the real gains for codes
// 0..3 are 0.15 / 0.3 / 0.6 / 1.2 V/A, so the SimpleFOCDrivers enum name
// (0V25) is misleading - code 0b10 is 0.6 V/A. Verify with `cs` in motor_test.
#define CURRENT_SENSE_DRV8316_LOWSIDE 1
#define DRV8316_CSA_GAIN              Gain_0V25
#define CURRENT_SENSE_MV_PER_A        600.0f
#define CURRENT_SENSE_OFFSET_HINT     "~half the CSA reference (DRV8316 VREF/2)"

// The DRV8316's buck converter makes the board's 5 V. (Leave this undefined
// on boards with their own 5 V supply: the buck is then switched off.)
#define DRV8316_BUCK_VOLTAGE VB_5V

// Supply voltage: V_SENS divider on ADC1, src/parts/supply_vsens.cpp.
// Also needs -D ADC_SAMPLINGTIME=ADC_SAMPLETIME_247CYCLES_5 (board.ini).
#define SUPPLY_SENSE_ADC_DIVIDER 1
// V_SENS divider: VIN -> 100k -> PA3 -> 10k -> GND, ratio (100k+10k)/10k.
#define VSENS_DIVIDER_RATIO 11.0f
// ADC reference = VDDA = 3.3 V LDO (XC6206, +-2%).
#define VSENS_ADC_REF_V     3.3f
// Trim factor: set to (multimeter VIN) / (reported VIN) after a bench check.
#define VSENS_CALIBRATION   1.0f

#pragma once

// MiniFOC Mk2 Rev 01 (github.com/clawntl/MiniFOC): STM32G431CBT6 (LQFP-48) +
// DRV8316 + AS5047P, inline phase current sense (INA241A3), INA226 supply
// monitor, separate 5 V buck. Pins from the KiCad netlist.
//
// Found as "board_config.h" through -I boards/minifoc_mk2 (see board.ini here).
// See docs/firmware.md#boards for what a board header has to define.

#include "board_ids.h"

#define BOARD_ID      BOARD_ID_MINIFOC
#define BOARD_HW_REV  2 // Mk2
#define BOARD_MODEL   "MiniFOC Mk2"   // reported over CAN, max 30 characters

// ---- Pins -------------------------------------------------------------------

#undef LED_BUILTIN
#define LED_BUILTIN PB2   // USER LED (D7, active high)

#define RGB_LED     PA3   // WS2812B-2020 data in (D5 -> D3 chain). Not used yet.

// SPI1: DRV8316 + AS5047P encoder
#define SPI1_SCK    PA5
#define SPI1_MISO   PA6
#define SPI1_MOSI   PA7
#define DRV_CS      PB0
#define ENC_CS      PB1

// SPI3 header (J5) / optional LSM6DSV16X IMU (normally unpopulated)
#define SPI3_SCK    PB3
#define SPI3_MISO   PB4
#define SPI3_MOSI   PB5
#define SPI3_NSS    PA4
#define IMU_CS      PB6

// I2C1: INA226 power monitor (0x40), optional BME280 (0x76), I2C header (J8)
#define I2C1_SDA    PB7
#define I2C1_SCL    PA15

// CAN transceiver (ATA6561) logic supply. VIO is powered from this pin
// (10k pull-down R5), so HIGH = transceiver on, LOW = off. STBY is tied to GND.
#define CAN_EN      PC13

// FDCAN1 RX is on PB8, which is also BOOT0 (10k pull-down R23 + BOOT button
// SW2 to 3V3). See softwareReset() in dfu.cpp for why that matters.
// Because of this the BOOT button can't be read as a GPIO: enter DFU with
// BOOT held while pressing RST, or the `dfu` serial command.
#define CAN_RX_BOOT0 PB8

// DRV8316 6-PWM inputs
#define PHA_H       PA10
#define PHA_L       PB15

#define PHB_H       PA9
#define PHB_L       PB14

#define PHC_H       PA8
#define PHC_L       PB13

// Inline phase current sense outputs (note the A/C order vs BLDC-Driver).
#define SO_A        PA2
#define SO_B        PA1
#define SO_C        PA0

// ---- Parts (each selects one file in src/parts/) ----------------------------

// No DFU button that firmware can read (it is BOOT0, see above).

// Encoder on SPI1: src/parts/encoder_as5047p.cpp
#define ENCODER_AS5047P 1
#define ENCODER_SPI_HZ  8'000'000

// Phase current: 3 mOhm shunts (R4 / R8 / R17 on phases A / B / C) into
// INA241A3 amplifiers (50 V/V), output centred on 3V3/2 -> 0.15 V/A, +-11 A.
// The DRV8316's own CSA outputs are not connected. src/parts/current_inline.cpp
#define CURRENT_SENSE_INLINE      1
#define CURRENT_SENSE_SHUNT_OHMS  0.003f
#define CURRENT_SENSE_GAIN        50.0f
#define CURRENT_SENSE_OFFSET_HINT "~1.65 V (INA241 REF = 3V3/2)"

// No DRV8316_BUCK_VOLTAGE: 5 V comes from a separate AP63205 buck and the
// DRV8316's buck is unpopulated (R_BK/C_BK only), so it is switched off.

// Supply voltage and input current: INA226 on I2C1, src/parts/supply_ina226.cpp.
// Shunt R1 between V_IN and V_M.
#define SUPPLY_SENSE_INA226 1
#define INA226_I2C_ADDR     0x40
#define INA226_SHUNT_OHMS   0.003f

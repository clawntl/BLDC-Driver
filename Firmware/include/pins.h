#pragma once

// Pin mapping for MiniFOC Mk2 (Rev 01), STM32G431CBT6 (LQFP-48).
// Taken from the KiCad netlist (Hardware/Driver Mk2.kicad_sch).

// The generic G431CB variant defines LED_BUILTIN as "not defined"; replace it.
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

// Inline phase current sense: INA241A3 (50 V/V) across 3 mOhm shunts,
// output centred on 3V3/2. (The DRV8316's own CSA outputs are not connected.)
#define SO_A        PA2
#define SO_B        PA1
#define SO_C        PA0

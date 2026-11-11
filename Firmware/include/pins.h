#pragma once

// Board pin mapping for the custom STM32G431CBU6 motor driver board.

// The generic G431CB variant defines LED_BUILTIN as "not defined"; replace it.
#undef LED_BUILTIN
#define LED_BUILTIN PC6

#define DFU_BUTTON PB12 // pull low (button to GND) to force the USB DFU bootloader

#define SPI1_SCK   PA5
#define SPI1_MISO  PA6
#define SPI1_MOSI  PA7
#define DRV_CS     PA4
#define ENC_CS     PC4

#define CAN_EN  PC10

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

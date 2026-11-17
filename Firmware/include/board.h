#pragma once

// Selects the board header for this build. Each PlatformIO environment
// defines exactly one BOARD_* macro (see platformio.ini), and the board header
// gives the pin map, the hardware constants and which parts are fitted
// (encoder, current sense, supply sensing, DFU button, ...).
//
// Everything outside include/boards/ and src/hw/ should depend only on the
// capability macros, never on which board it is. To add a board, see
// docs/firmware.md#adding-a-board.

#include "board_ids.h"

#if defined(BOARD_BLDC_DRIVER_V02)
#include "boards/bldc_driver_v02.h"
#elif defined(BOARD_MINIFOC_MK2)
#include "boards/minifoc_mk2.h"
#else
#error "No board selected: build one of the board environments in platformio.ini (e.g. pio run -e bldc_driver_v02)"
#endif

// ---- Checks every board header must satisfy --------------------------------

#if !defined(BOARD_ID) || !defined(BOARD_HW_REV) || !defined(BOARD_MODEL)
#error "Board header must define BOARD_ID, BOARD_HW_REV and BOARD_MODEL"
#endif

#if (defined(ENCODER_AS5048A) + defined(ENCODER_AS5047P)) != 1
#error "Board header must define exactly one ENCODER_* type"
#endif

#if (defined(CURRENT_SENSE_DRV8316_LOWSIDE) + defined(CURRENT_SENSE_INLINE)) != 1
#error "Board header must define exactly one CURRENT_SENSE_* type"
#endif

#if (defined(SUPPLY_SENSE_ADC_DIVIDER) + defined(SUPPLY_SENSE_INA226)) != 1
#error "Board header must define exactly one SUPPLY_SENSE_* type"
#endif

#if defined(CURRENT_SENSE_DRV8316_LOWSIDE) && !(defined(CURRENT_SENSE_MV_PER_A) && defined(DRV8316_CSA_GAIN))
#error "CURRENT_SENSE_DRV8316_LOWSIDE needs CURRENT_SENSE_MV_PER_A and DRV8316_CSA_GAIN"
#endif
#if defined(CURRENT_SENSE_INLINE) && !(defined(CURRENT_SENSE_SHUNT_OHMS) && defined(CURRENT_SENSE_GAIN))
#error "CURRENT_SENSE_INLINE needs CURRENT_SENSE_SHUNT_OHMS and CURRENT_SENSE_GAIN"
#endif
#if !defined(CURRENT_SENSE_OFFSET_HINT)
#error "Board header must define CURRENT_SENSE_OFFSET_HINT (expected zero-current offset, for motor_test)"
#endif
#if !defined(DRV8316_BUCK_ENABLED) || (DRV8316_BUCK_ENABLED && !defined(DRV8316_BUCK_VOLTAGE))
#error "Board header must define DRV8316_BUCK_ENABLED (and DRV8316_BUCK_VOLTAGE if 1)"
#endif

#if defined(HAS_DFU_BUTTON) && !defined(DFU_BUTTON)
#error "HAS_DFU_BUTTON needs a DFU_BUTTON pin"
#endif

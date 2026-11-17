#pragma once

// SimpleFOC objects and DRV8316 setup for the selected board, so that
// motor_control.cpp and the motor_test bench firmware construct the same
// hardware without knowing which board they run on.

#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>

#include "drivers/drv8316/drv8316.h"

#include "board.h"

// ---- Encoder (SPI1) ----------------------------------------------------------

#if defined(ENCODER_AS5048A)
#include "encoders/as5048a/MagneticSensorAS5048A.h"
using BoardEncoder = MagneticSensorAS5048A;
#define ENCODER_NAME "AS5048A"
#define ENCODER_CPR  AS5048A_CPR
#elif defined(ENCODER_AS5047P)
#include "encoders/as5047/MagneticSensorAS5047.h"
using BoardEncoder = MagneticSensorAS5047;
#define ENCODER_NAME "AS5047P"
#define ENCODER_CPR  AS5047_CPR
#endif

// Constructor arguments: BoardEncoder sensor(BOARD_ENCODER_ARGS);
#define BOARD_ENCODER_ARGS ENC_CS, false, SPISettings(ENCODER_SPI_HZ, MSBFIRST, SPI_MODE1)

// The AS5048A and AS5047P report the same diagnostics under different names.
struct EncoderDiagnostics {
    uint16_t reg;         // raw DIAAGC register
    uint8_t  agc;         // automatic gain: ~255 = field weak, ~0 = field strong
    bool     offsetReady; // AS5048A OCF / AS5047P LF: offset compensation finished
    bool     cordicOverflow;
    bool     fieldTooWeak;   // AS5048A COMP_H / AS5047P MAGL
    bool     fieldTooStrong; // AS5048A COMP_L / AS5047P MAGH
};

struct EncoderErrors {
    bool parityError;
    bool commandInvalid;
    bool framingError;
};

inline EncoderDiagnostics readEncoderDiagnostics(BoardEncoder &sensor) {
    const auto d = sensor.readDiagnostics();
#if defined(ENCODER_AS5048A)
    return { d.reg, (uint8_t)d.agc, (bool)d.ocf, (bool)d.cof, (bool)d.compHigh, (bool)d.compLow };
#else
    return { d.reg, (uint8_t)d.agc, (bool)d.lf, (bool)d.cof, (bool)d.magl, (bool)d.magh };
#endif
}

inline EncoderErrors clearEncoderErrors(BoardEncoder &sensor) {
    const auto e = sensor.clearErrorFlag();
    return { e.parityError, e.commandInvalid, e.framingError };
}

// ---- Phase current sense -----------------------------------------------------

#if defined(CURRENT_SENSE_DRV8316_LOWSIDE)
using BoardCurrentSense = LowsideCurrentSense;
#define CURRENT_SENSE_NAME "DRV8316 low-side CSA"
#define BOARD_CURRENT_SENSE_ARGS CURRENT_SENSE_MV_PER_A, SO_A, SO_B, SO_C
// Low-side sampling is synchronised to the PWM, so the offset calibration in
// current_sense.init() needs the PWM running (motor enabled at 0 V).
#define CURRENT_SENSE_CALIBRATE_WITH_PWM 1
#elif defined(CURRENT_SENSE_INLINE)
using BoardCurrentSense = InlineCurrentSense;
#define CURRENT_SENSE_NAME "inline shunts"
#define BOARD_CURRENT_SENSE_ARGS CURRENT_SENSE_SHUNT_OHMS, CURRENT_SENSE_GAIN, SO_A, SO_B, SO_C
// Inline sensing reads at any time; offsets are calibrated at zero current,
// so the power stage stays off.
#define CURRENT_SENSE_CALIBRATE_WITH_PWM 0
#endif

// ---- DRV8316 -------------------------------------------------------------------

// Board-independent register setup after driver.init(): buck (per board),
// over-current protection, slew rate, PWM mode and, with the DRV8316's own
// current sense, the CSA gain. Shared by the real firmware and motor_test.
void configureDRV8316(DRV8316Driver &driver);

// Reads back two registers written by configureDRV8316(), chosen so that a
// bus reading all zeros or all ones fails either way. False means the DRV8316
// isn't answering on DRV_CS - usually firmware built for a different board.
bool drv8316Responding(DRV8316Driver &driver);

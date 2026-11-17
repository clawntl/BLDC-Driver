#pragma once

// Position encoder interface. Implemented by exactly ONE file in src/parts/
// (encoder_*.cpp), chosen by the board header (e.g. ENCODER_AS5047P). No
// encoder selected = undefined reference at link time; two = duplicate symbol.
//
// To add an encoder: a new src/parts/encoder_<part>.cpp wrapped in
// #if defined(ENCODER_<PART>), implementing everything below.

#include <SimpleFOC.h>
#include <SPI.h>

// The SimpleFOC sensor object (linked to the motor, updated by loopFOC()).
Sensor &boardEncoder();

// Starts the encoder on the given SPI bus. Call once from setup().
void encoderInit(SPIClass *spi);

// e.g. "AS5047P"
const char *encoderName();

// Counts per revolution of encoderReadRaw().
uint32_t encoderCountsPerRev();

// ---- Diagnostics (motor_test) -------------------------------------------------

struct EncoderDiagnostics {
    uint16_t reg;            // raw diagnostics register (0 / 0xFFFF hint at SPI wiring)
    uint8_t  agc;            // automatic gain: ~255 = field weak, ~0 = field strong
    bool     offsetReady;    // offset compensation finished
    bool     cordicOverflow;
    bool     fieldTooWeak;
    bool     fieldTooStrong;
};

struct EncoderErrors {
    bool parityError;
    bool commandInvalid;
    bool framingError;
};

uint16_t encoderReadRaw();          // raw angle, 0 .. encoderCountsPerRev()-1
bool encoderErrorFlag();            // error flag of the last SPI frame
uint16_t encoderReadMagnitude();    // field magnitude (CORDIC output)
EncoderDiagnostics encoderDiagnostics();
EncoderErrors encoderClearErrors(); // reads and clears the error register

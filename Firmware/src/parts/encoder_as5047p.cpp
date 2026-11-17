// AS5047P 14-bit magnetic encoder on SPI1 (MiniFOC).

#include "board_config.h"

#if defined(ENCODER_AS5047P)

#include <SimpleFOCDrivers.h>
#include "encoders/as5047/MagneticSensorAS5047.h"

#include "hw/encoder.h"

static MagneticSensorAS5047 sensor(ENC_CS, false, SPISettings(ENCODER_SPI_HZ, MSBFIRST, SPI_MODE1));

Sensor &boardEncoder() { return sensor; }
void encoderInit(SPIClass *spi) { sensor.init(spi); }
const char *encoderName() { return "AS5047P"; }
uint32_t encoderCountsPerRev() { return AS5047_CPR; }

uint16_t encoderReadRaw() { return sensor.readRawAngle(); }
bool encoderErrorFlag() { return sensor.isErrorFlag(); }
uint16_t encoderReadMagnitude() { return sensor.readMagnitude() & 0x3FFF; }

EncoderDiagnostics encoderDiagnostics() {
    const AS5047Diagnostics d = sensor.readDiagnostics();
    // LF = offset loop finished; MAGL = field too weak, MAGH = too strong.
    return { d.reg, (uint8_t)d.agc, (bool)d.lf, (bool)d.cof, (bool)d.magl, (bool)d.magh };
}

EncoderErrors encoderClearErrors() {
    const AS5047Error e = sensor.clearErrorFlag();
    return { e.parityError, e.commandInvalid, e.framingError };
}

#endif // ENCODER_AS5047P

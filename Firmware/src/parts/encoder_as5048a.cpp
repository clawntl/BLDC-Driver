// AS5048A 14-bit magnetic encoder on SPI1 (BLDC-Driver).

#include "board_config.h"

#if defined(ENCODER_AS5048A)

#include <SimpleFOCDrivers.h>
#include "encoders/as5048a/MagneticSensorAS5048A.h"

#include "hw/encoder.h"

static MagneticSensorAS5048A sensor(ENC_CS, false, SPISettings(ENCODER_SPI_HZ, MSBFIRST, SPI_MODE1));

Sensor &boardEncoder() { return sensor; }
void encoderInit(SPIClass *spi) { sensor.init(spi); }
const char *encoderName() { return "AS5048A"; }
uint32_t encoderCountsPerRev() { return AS5048A_CPR; }

uint16_t encoderReadRaw() { return sensor.readRawAngle(); }
bool encoderErrorFlag() { return sensor.isErrorFlag(); }
uint16_t encoderReadMagnitude() { return sensor.readMagnitude() & 0x3FFF; }

EncoderDiagnostics encoderDiagnostics() {
    const AS5048Diagnostics d = sensor.readDiagnostics();
    // OCF = offset compensation finished; COMP_H = field too weak, COMP_L = too strong.
    return { d.reg, (uint8_t)d.agc, (bool)d.ocf, (bool)d.cof, (bool)d.compHigh, (bool)d.compLow };
}

EncoderErrors encoderClearErrors() {
    const AS5048Error e = sensor.clearErrorFlag();
    return { e.parityError, e.commandInvalid, e.framingError };
}

#endif // ENCODER_AS5048A

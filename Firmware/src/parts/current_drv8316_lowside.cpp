// Phase current through the DRV8316's own low-side current-sense amplifiers
// (BLDC-Driver). Board header: CURRENT_SENSE_MV_PER_A, DRV8316_CSA_GAIN,
// CURRENT_SENSE_OFFSET_HINT, SO_A/B/C.

#include "board_config.h"

#if defined(CURRENT_SENSE_DRV8316_LOWSIDE)

#include "hw/current_sense.h"
#include "motor_config.h"

#include <current_sense/hardware_api.h>

namespace {

// LowsideCurrentSense::init() (Simple FOC 2.4.0) with a shorter offset
// calibration (CURRENT_SENSE_OFFSET_ROUNDS instead of 2000 x 1 ms) and a
// check that the offsets are where they should be (about VREF/2). A dead
// or disconnected amplifier reads near 0 V or VREF, so init() fails then
// (the host gets ERR_CURRENT_SENSE). Re-check against the library's
// init() when updating Simple FOC.
class CheckedLowsideCurrentSense : public LowsideCurrentSense {
public:
    using LowsideCurrentSense::LowsideCurrentSense;

    int init() override {
        initialized = false;
        if (driver == nullptr) return 0;
        params = _configureADCLowSide(driver->params, pinA, pinB, pinC);
        if (params == SIMPLEFOC_CURRENT_SENSE_INIT_FAILED) return 0;
        if (_driverSyncLowSide(driver->params, params) == SIMPLEFOC_CURRENT_SENSE_INIT_FAILED) return 0;

        BLDCDriver *bldc = static_cast<BLDCDriver *>(driver);
        // Centred PWM (zero voltage vector): the low-side samples are taken
        // while all three low-side switches are on.
        const float mid = driver->voltage_limit / 2;
        bldc->setPwm(mid, mid, mid);
        float a = 0, b = 0, c = 0;
        for (int i = 0; i < CURRENT_SENSE_OFFSET_ROUNDS; i++) {
            _startADC3PinConversionLowSide();
            if (_isset(pinA)) a += _readADCVoltageLowSide(pinA, params);
            if (_isset(pinB)) b += _readADCVoltageLowSide(pinB, params);
            if (_isset(pinC)) c += _readADCVoltageLowSide(pinC, params);
            delay(1);
        }
        offset_ia = a / CURRENT_SENSE_OFFSET_ROUNDS;
        offset_ib = b / CURRENT_SENSE_OFFSET_ROUNDS;
        offset_ic = c / CURRENT_SENSE_OFFSET_ROUNDS;
        bldc->setPwm(0, 0, 0);

        auto ok = [](int pin, float v) {
            return !_isset(pin) || (v >= CURRENT_SENSE_OFFSET_MIN_V && v <= CURRENT_SENSE_OFFSET_MAX_V);
        };
        if (!ok(pinA, offset_ia) || !ok(pinB, offset_ib) || !ok(pinC, offset_ic)) {
            Serial.print("Current sense offsets out of range: ");
            Serial.print(offset_ia, 3); Serial.print(" / ");
            Serial.print(offset_ib, 3); Serial.print(" / ");
            Serial.print(offset_ic, 3);
            Serial.print(" V (expected ");
            Serial.print(CURRENT_SENSE_OFFSET_MIN_V, 2); Serial.print("-");
            Serial.print(CURRENT_SENSE_OFFSET_MAX_V, 2); Serial.println(" V)");
            return 0;
        }
        initialized = true;
        return 1;
    }
};

CheckedLowsideCurrentSense current_sense(CURRENT_SENSE_MV_PER_A, SO_A, SO_B, SO_C);

} // namespace

CurrentSense &boardCurrentSense() { return current_sense; }
const char *currentSenseName() { return "DRV8316 low-side CSA"; }
// Low-side sampling is synchronised to the PWM, so offsets are calibrated
// with the PWM running (motor enabled at 0 V).
bool currentSenseCalibratesWithPwm() { return true; }
const char *currentSenseOffsetHint() { return CURRENT_SENSE_OFFSET_HINT; }
bool currentSenseCanSkipAlign() { return true; }

void currentSensePrepareDriver(DRV8316Driver &driver) {
    driver.setCurrentSenseGain(DRV8316_CSA_GAIN);
    delayMicroseconds(1);
}

#endif // CURRENT_SENSE_DRV8316_LOWSIDE

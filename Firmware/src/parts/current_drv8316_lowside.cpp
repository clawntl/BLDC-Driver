// Phase current through the DRV8316's own low-side current-sense amplifiers
// (BLDC-Driver). Board header: CURRENT_SENSE_MV_PER_A, DRV8316_CSA_GAIN,
// CURRENT_SENSE_OFFSET_HINT, SO_A/B/C.

#include "board_config.h"

#if defined(CURRENT_SENSE_DRV8316_LOWSIDE)

#include "hw/current_sense.h"

static LowsideCurrentSense current_sense(CURRENT_SENSE_MV_PER_A, SO_A, SO_B, SO_C);

CurrentSense &boardCurrentSense() { return current_sense; }
const char *currentSenseName() { return "DRV8316 low-side CSA"; }
// Low-side sampling is synchronised to the PWM, so offsets are calibrated
// with the PWM running (motor enabled at 0 V).
bool currentSenseCalibratesWithPwm() { return true; }
const char *currentSenseOffsetHint() { return CURRENT_SENSE_OFFSET_HINT; }

void currentSensePrepareDriver(DRV8316Driver &driver) {
    driver.setCurrentSenseGain(DRV8316_CSA_GAIN);
    delayMicroseconds(1);
}

#endif // CURRENT_SENSE_DRV8316_LOWSIDE

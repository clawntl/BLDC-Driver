// Phase current through inline shunts and external amplifiers (MiniFOC:
// INA241A3). Board header: CURRENT_SENSE_SHUNT_OHMS, CURRENT_SENSE_GAIN,
// CURRENT_SENSE_OFFSET_HINT, SO_A/B/C.

#include "board_config.h"

#if defined(CURRENT_SENSE_INLINE)

#include "hw/current_sense.h"

static InlineCurrentSense current_sense(CURRENT_SENSE_SHUNT_OHMS, CURRENT_SENSE_GAIN, SO_A, SO_B, SO_C);

CurrentSense &boardCurrentSense() { return current_sense; }
const char *currentSenseName() { return "inline shunts"; }
// Inline sensing reads at any time; offsets are calibrated at zero current
// with the power stage off.
bool currentSenseCalibratesWithPwm() { return false; }
const char *currentSenseOffsetHint() { return CURRENT_SENSE_OFFSET_HINT; }
// No offset check in init() here, so keep SimpleFOC's driverAlign().
bool currentSenseCanSkipAlign() { return false; }

void currentSensePrepareDriver(DRV8316Driver &) { }

#endif // CURRENT_SENSE_INLINE

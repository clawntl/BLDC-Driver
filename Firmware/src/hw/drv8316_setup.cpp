#include "drv8316_setup.h"

#include <Arduino.h>

#include "board_config.h"
#include "motor_config.h"
#include "current_sense.h"

void configureDRV8316(DRV8316Driver &driver) {
    // It's advised to have at least 400 ns between SPI read and write calls.
#if defined(DRV8316_BUCK_VOLTAGE)
    // The buck makes the board's 5 V (CAN transceiver, and the 3.3 V LDO when
    // not on USB). It powers up at 3.3 V; only wait when this changes it, so
    // a repeat call (every FOC start) costs nothing. Unpowered (USB only),
    // the read-back is garbage, so it's written again then, which is harmless.
    if (!driver.isBuckEnabled() || driver.getBuckVoltage() != DRV8316_BUCK_VOLTAGE) {
        driver.setBuckVoltage(DRV8316_BUCK_VOLTAGE);
        delay(DRV8316_BUCK_SETTLE_MS);
    }
#else
    driver.setBuckEnabled(false);
    delayMicroseconds(1);
#endif
    driver.setOCPLevel(Curr_16A);
    delayMicroseconds(1);
    driver.setOCPMode(AutoRetry_Fault);
    delayMicroseconds(1);
    driver.setOCPRetryTime(Retry500ms);
    delayMicroseconds(1);
    driver.setSlew(Slew_200Vus);
    delayMicroseconds(1);
    driver.setPWM100Frequency(FREQ_20KHz);
    delayMicroseconds(1);
    currentSensePrepareDriver(driver);
    driver.setPWMMode(PWM6_Mode);
    delayMicroseconds(1);
}

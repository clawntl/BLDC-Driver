#include "board_hw.h"

#include <Arduino.h>

void configureDRV8316(DRV8316Driver &driver) {
    // It's advised to have at least 400 ns between SPI read and write calls.
#if DRV8316_BUCK_ENABLED
    driver.setBuckVoltage(DRV8316_BUCK_VOLTAGE);
    // Wait for the buck output (and what it powers) to settle.
    delay(500);
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
#if defined(CURRENT_SENSE_DRV8316_LOWSIDE)
    driver.setCurrentSenseGain(DRV8316_CSA_GAIN);
    delayMicroseconds(1);
#endif
    driver.setPWMMode(PWM6_Mode);
    delayMicroseconds(1);
}

bool drv8316Responding(DRV8316Driver &driver) {
    // Slew_200Vus = 0b11 and Curr_16A = 0b0 (both differ from the reset
    // defaults), so neither an all-zeros nor an all-ones read can pass.
    const bool slewOk = driver.getSlew() == Slew_200Vus;
    delayMicroseconds(1);
    const bool ocpOk = driver.getOCPLevel() == Curr_16A;
    delayMicroseconds(1);
    return slewOk && ocpOk;
}

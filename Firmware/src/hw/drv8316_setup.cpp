#include "drv8316_setup.h"

#include <Arduino.h>

#include "board_config.h"
#include "current_sense.h"

void configureDRV8316(DRV8316Driver &driver) {
    // It's advised to have at least 400 ns between SPI read and write calls.
#if defined(DRV8316_BUCK_VOLTAGE)
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
    currentSensePrepareDriver(driver);
    driver.setPWMMode(PWM6_Mode);
    delayMicroseconds(1);
}

#pragma once

#include "drivers/drv8316/drv8316.h"

// DRV8316 register setup after driver.init(): buck (on if the board header
// defines DRV8316_BUCK_VOLTAGE, otherwise off), over-current protection,
// slew rate, PWM mode, plus whatever the current sense part needs. Shared by
// the real firmware and motor_test.
void configureDRV8316(DRV8316Driver &driver);

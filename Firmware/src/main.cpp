#include <Arduino.h>

#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <IWatchdog.h>

#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"

#include "pins.h"
#include "motor_config.h"
#include "board_settings.h"
#include "can_bus.h"
#include "can_protocol.h"
#include "motor_control.h"
#include "serial_console.h"
#include "dfu.h"

// LED patterns: unconfigured = fast blink (5 Hz), idle = short blip every
// second, running = solid on, FOC start failed = double blink.
static void updateStatusLed() {
    const uint32_t t = millis();
    bool on;
    switch (driverState()) {
        case DRIVER_STATE_RUNNING: on = true;                                 break;
        case DRIVER_STATE_IDLE:    on = (t % 1000) < 50;                      break;
        case DRIVER_STATE_FAULT:   { const uint32_t p = t % 1000;
                                     on = p < 100 || (p >= 200 && p < 300); } break;
        default:                   on = (t % 200) < 100;                      break;
    }
    digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
}

static void printUnconfiguredHelp() {
    Serial.println("*** Board not configured: CAN is disabled. ***");
    Serial.println("    Type `set side left` or `set side right`, then `save`. `help` lists all commands.");
}

void setup() {
    // Must come first: finishes a DFU request made just before a reset.
    checkDfuRequestAtBoot();

    Serial.begin(115200);

    // Give a serial terminal a moment to connect so the boot messages can be
    // seen; carries on as soon as one does (or after the timeout, e.g. when
    // running on the robot with no USB host).
    const uint32_t waitStart = millis();
    while (!Serial && millis() - waitStart < BOOT_SERIAL_WAIT_MS) { }
    Serial.println("begin");

    // Both devices share SPI1: drive both CS lines high (deselected) before
    // any SPI traffic, otherwise pinMode(OUTPUT) leaves them low and the
    // DRV8316 and AS5048A both drive MISO during sensor.init().
    pinMode(DRV_CS, OUTPUT);
    digitalWrite(DRV_CS, HIGH);
    pinMode(ENC_CS, OUTPUT);
    digitalWrite(ENC_CS, HIGH);

    pinMode(SO_A, INPUT);
    pinMode(SO_B, INPUT);
    pinMode(SO_C, INPUT);

    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(CAN_EN, OUTPUT);
    digitalWrite(CAN_EN, HIGH); // enable CAN transceiver for now

    // Idle high, pressed = low. Internal pull-up so the pin can't float (and
    // randomly reboot into DFU) if there's no external pull-up fitted.
    pinMode(DFU_BUTTON, INPUT_PULLUP);

    SimpleFOCDebug::enable(&Serial);

    SimpleFOC_CORDIC_Config();

    digitalWrite(LED_BUILTIN, HIGH);

    if (IWatchdog.isReset(true)) {
        Serial.println("WARNING: the last reset was caused by the watchdog (firmware stalled)");
    }

    if (firmwareOverlapsSettings()) {
        Serial.println("ERROR: firmware image overlaps the settings flash page (>126 KB) - settings can't be saved!");
    }
    if (!loadBoardSettings()) {
        Serial.println("No valid board settings in flash.");
    }
    printConfig();

    if (boardConfigured()) {
        configureAndStartCAN();
    } else {
        printUnconfiguredHelp();
    }

    setupFOC();

    digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
    // Only running once FOC has started (see motor_control.cpp).
    if (IWatchdog.isEnabled()) IWatchdog.reload();

    checkDfuButton();
    pollSerialConsole();

    motorControlLoop();

    pollCAN();

    updateStatusLed();

    // Periodic reminder, since the boot message is easy to miss.
    static uint32_t lastReminderMs = 0;
    if (!boardConfigured() && millis() - lastReminderMs > 10'000) {
        lastReminderMs = millis();
        printUnconfiguredHelp();
    }
}

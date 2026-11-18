#include "motor_control.h"

#include <SPI.h>
#include <SimpleFOCDrivers.h>
#include <IWatchdog.h>
#include <cmath>

#include "drivers/drv8316/drv8316.h"

#include "board_config.h"
#include "hw/encoder.h"
#include "hw/current_sense.h"
#include "hw/drv8316_setup.h"
#include "hw/supply.h"
#include "can_protocol.h"
#include "can_bus.h"
#include "motor_config.h"
#include "board_settings.h"

static SPIClass SPI_1(SPI1_MOSI, SPI1_MISO, SPI1_SCK);

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);

//  BLDCDriver6PWM( int phA_h, int phA_l, int phB_h, int phB_l, int phC_h, int phC_l, int en)
//  - phA_h, phA_l - A phase pwm pin high/low pair
//  - phB_h, phB_l - B phase pwm pin high/low pair
//  - phB_h, phC_l - C phase pwm pin high/low pair
static DRV8316Driver6PWM driver = DRV8316Driver6PWM(PHA_H, PHA_L, PHB_H, PHB_L, PHC_H, PHC_L, DRV_CS, false);

// Current sense and encoder: the parts the board header selects (src/parts/).
static CurrentSense &current_sense = boardCurrentSense();

static Sensor &sensor = boardEncoder();

// Commanded current (A, already direction-corrected). Only written through
// setTargetCurrent() and the command timeout.
static float target_current = 0;

// +1, or -1 when the invert setting is on. Applied to commands and telemetry.
static float direction_sign = 1.0f;

static bool foc_running = false;
static bool foc_failed = false;
static bool start_requested = false;
static bool calibration_used = false;
static bool command_timed_out = false;
static bool drv_fault = false;

static float supply_voltage = 0;

static uint32_t lastVelocitySendTime = 0;
static uint32_t velocity_interval_us = SHAFT_VELOCITY_TELEMETRY_INTERVAL_US;
static uint32_t lastCommandMs = 0;
static uint32_t lastDrvPollMs = 0;

static void printDRV8316Status() {
  DRV8316Status status = driver.getStatus();
	Serial.println("DRV8316 Status:");
	Serial.print("Fault: ");
	Serial.println(status.isFault());
	Serial.print("Buck Error: ");
	Serial.print(status.isBuckError());
	Serial.print("  Undervoltage: ");
	Serial.print(status.isBuckUnderVoltage());
	Serial.print("  OverCurrent: ");
	Serial.println(status.isBuckOverCurrent());
	Serial.print("Charge Pump UnderVoltage: ");
	Serial.println(status.isChargePumpUnderVoltage());
	Serial.print("OTP Error: ");
	Serial.println(status.isOneTimeProgrammingError());
	Serial.print("OverCurrent: ");
	Serial.print(status.isOverCurrent());
	Serial.print("  Ah: ");
	Serial.print(status.isOverCurrent_Ah());
	Serial.print("  Al: ");
	Serial.print(status.isOverCurrent_Al());
	Serial.print("  Bh: ");
	Serial.print(status.isOverCurrent_Bh());
	Serial.print("  Bl: ");
	Serial.print(status.isOverCurrent_Bl());
	Serial.print("  Ch: ");
	Serial.print(status.isOverCurrent_Ch());
	Serial.print("  Cl: ");
	Serial.println(status.isOverCurrent_Cl());
	Serial.print("OverTemperature: ");
	Serial.print(status.isOverTemperature());
	Serial.print("  Shutdown: ");
	Serial.print(status.isOverTemperatureShutdown());
	Serial.print("  Warning: ");
	Serial.println(status.isOverTemperatureWarning());
	Serial.print("OverVoltage: ");
	Serial.println(status.isOverVoltage());
	Serial.print("PowerOnReset: ");
	Serial.println(status.isPowerOnReset());
	Serial.print("SPI Error: ");
	Serial.print(status.isSPIError());
	Serial.print("  Address: ");
	Serial.print(status.isSPIAddressError());
	Serial.print("  Clock: ");
	Serial.print(status.isSPIClockFramingError());
	Serial.print("  Parity: ");
	Serial.println(status.isSPIParityError());
	if (status.isFault())
		driver.clearFault();
	delayMicroseconds(1); // ensure 400ns delay
	DRV8316_PWMMode val = driver.getPWMMode();
	Serial.print("PWM Mode: ");
	Serial.println(val);
	delayMicroseconds(1); // ensure 400ns delay
	bool lock = driver.isRegistersLocked();
	Serial.print("Lock: ");
	Serial.println(lock);
}

void setupFOC() {
    // initialise magnetic sensor hardware
    encoderInit(&SPI_1);
    // link the motor to the sensor
    motor.linkSensor(&sensor);

    // Measure the real supply so SimpleFOC's volts -> PWM duty conversion is
    // correct. Must happen before current_sense.init() (see hw/supply.h).
    supply_voltage = measuredSupplyVoltageOrDefault();
    driver.voltage_power_supply = supply_voltage;
    driver.voltage_limit = min(MOTOR_VOLTAGE_LIMIT, driver.voltage_power_supply);

    driver.init(&SPI_1);

    // small delay to allow SPI response
    delay(10);

    // Buck, protection, slew, PWM mode, current-sense settings (see hw/drv8316_setup.cpp).
    configureDRV8316(driver);

    Serial.println("Init complete...");

    delay(100);
    printDRV8316Status();

    motor.current_limit = MOTOR_CURRENT_LIMIT;

    current_sense.linkDriver(&driver);

    // Link driver to the motor
    motor.linkDriver(&driver);

    // aligning voltage
    motor.voltage_sensor_align = MOTOR_VOLTAGE_SENSOR_ALIGN;
    // choose FOC modulation
    motor.foc_modulation = FOCModulationType::SinePWM;

    motor.torque_controller = TorqueControlType::foc_current;

    // set motion control loop to be used
    motor.controller = MotionControlType::torque;

    // Board settings: direction and (optionally) a stored alignment, so that
    // initFOC() can skip the sensor alignment. See board_settings.h.
    const BoardSettings &cfg = activeSettings();
    direction_sign = cfg.invertDirection ? -1.0f : 1.0f;
    applyCurrentLoopSettings(cfg);
    if (cfg.calibrationValid) {
        motor.sensor_direction    = static_cast<Direction>(cfg.sensorDirection);
        motor.zero_electric_angle = cfg.zeroElectricAngle;
        calibration_used = true;
        Serial.print("Using stored alignment: dir ");
        Serial.print(cfg.sensorDirection);
        Serial.print(", zero electric angle ");
        Serial.println(cfg.zeroElectricAngle, 4);
    }
}

static bool startFOC() {
    // Not running yet: make sure nothing stale gets applied on the first move().
    target_current = 0;

    // Write the DRV8316 registers again. If the board booted on USB power
    // alone, the DRV8316 had no supply when setupFOC() configured it, so it
    // would otherwise run with its defaults (buck, over-current, slew).
    configureDRV8316(driver);

    if (!motor.init()) {
        Serial.println("Motor init failed!");
        sendError(ERR_MOTOR_INIT);
        return false;
    }

    // Some supply sensors share the ADC with the current sense and have to
    // know when it has taken over (see hw/supply.h).
    const bool csOk = current_sense.init();
    supplyCurrentSenseStarted();
    if (csOk)  Serial.println("Current sense init success!");
    else {
        Serial.println("Current sense init failed!");
        sendError(ERR_CURRENT_SENSE);
        return false;
    }
    // link the current sense to the motor
    motor.linkCurrentSense(&current_sense);

    // align encoder (skipped if a stored alignment was loaded) and start FOC.
    // On failure SimpleFOC disables the motor itself.
    if (!motor.initFOC()) {
        Serial.println("FOC init failed!");
        sendError(ERR_FOC_INIT);
        return false;
    }

    // Again, in case init changed them; the pending values, so a change made
    // before this start (and not saved) still applies.
    applyCurrentLoopSettings(pendingSettings());

    Serial.println(F("Motor ready."));
    return true;
}

static void runStartRequest() {
    start_requested = false;
    if (foc_running) {
        Serial.println("FOC already started");
        sendStartResult(true);
        return;
    }

    const bool ok = startFOC();

    // Anything the host sent while initFOC() was blocking is stale.
    discardPendingCAN();
    target_current = 0;
    lastCommandMs = millis();
    command_timed_out = false;

    foc_running = ok;
    foc_failed = !ok;
    sendStartResult(ok);

    if (ok) {
        // initFOC() blocks for seconds, so the watchdog is only started now.
        // From here on loop() must keep reloading it.
        IWatchdog.begin(WATCHDOG_TIMEOUT_US);
    }
}

void requestStartFOC() {
    start_requested = true;
}

void setTargetCurrent(float amps) {
    if (!std::isfinite(amps)) return; // ignore NaN / Inf
    target_current = direction_sign * constrain(amps, -MOTOR_CURRENT_LIMIT, MOTOR_CURRENT_LIMIT);
    lastCommandMs = millis();
    if (command_timed_out) {
        command_timed_out = false;
        if (foc_running) Serial.println("Current commands resumed");
    }
}

static void checkCommandTimeout() {
    if (command_timed_out) return;
    if (millis() - lastCommandMs > COMMAND_TIMEOUT_MS) {
        command_timed_out = true;
        target_current = 0;
        Serial.print("No current command for ");
        Serial.print(COMMAND_TIMEOUT_MS);
        Serial.println(" ms - applying 0 A");
    }
}

static void pollDriverStatus() {
    const uint32_t now = millis();
    if (now - lastDrvPollMs < DRV_STATUS_POLL_INTERVAL_MS) return;
    lastDrvPollMs = now;

    const bool fault = driver.getStatus().isFault();
    if (fault && !drv_fault) {
        Serial.println("DRV8316 fault detected:");
        printDRV8316Status(); // also clears the latched fault
        sendError(ERR_DRIVER_FAULT);
    }
    drv_fault = fault;
}

void motorControlLoop() {
    if (start_requested) runStartRequest();

    if (!foc_running) return;

    checkCommandTimeout();

    motor.loopFOC();
    motor.move(target_current);

    // move() has already updated motor.shaft_velocity; calling
    // motor.shaftVelocity() again would feed the velocity low-pass filter a
    // second sample per loop and distort it.
    const uint32_t currentTime = _micros();
    if (currentTime - lastVelocitySendTime >= velocity_interval_us) {
        sendFloat(CMD_SHAFT_VELOCITY, direction_sign * motor.shaft_velocity, telemetryCanId());
        lastVelocitySendTime = currentTime;
    }

    pollDriverStatus();
}

void stopMotor() {
    motor.disable();
    foc_running = false;
    target_current = 0;
}

bool focRunning() { return foc_running; }

void applyCurrentLoopSettings(const BoardSettings &s) {
    // Both axes alike. Changing P/I while running is safe: SimpleFOC's PID
    // keeps its integral in volts, so the output doesn't jump.
    motor.PID_current_q.P = s.currentP;
    motor.PID_current_q.I = s.currentI;
    motor.PID_current_d.P = s.currentP;
    motor.PID_current_d.I = s.currentI;
    motor.LPF_current_q.Tf = s.currentTf;
    motor.LPF_current_d.Tf = s.currentTf;
}

float currentLoopP()  { return motor.PID_current_q.P; }
float currentLoopI()  { return motor.PID_current_q.I; }
float currentLoopTf() { return motor.LPF_current_q.Tf; }

float focLoopHz() {
    if (!foc_running || motor.loopfoc_time_us == 0) return 0.0f;
    return 1e6f / (float)motor.loopfoc_time_us;
}

void setVelocityTelemetryIntervalMs(uint8_t ms) { velocity_interval_us = (uint32_t)ms * 1000u; }
uint8_t velocityTelemetryIntervalMs() { return (uint8_t)(velocity_interval_us / 1000u); }

uint8_t driverState() {
    if (!boardConfigured()) return DRIVER_STATE_UNCONFIGURED;
    if (foc_running)        return DRIVER_STATE_RUNNING;
    if (foc_failed)         return DRIVER_STATE_FAULT;
    return DRIVER_STATE_IDLE;
}

uint8_t statusFlags() {
    uint8_t flags = 0;
    if (foc_running && command_timed_out) flags |= STATUS_FLAG_COMMAND_TIMEOUT;
    if (calibration_used)                 flags |= STATUS_FLAG_CALIBRATION_USED;
    if (direction_sign < 0)               flags |= STATUS_FLAG_INVERTED;
    if (drv_fault)                        flags |= STATUS_FLAG_DRIVER_FAULT;
    return flags;
}

float measuredSupplyVoltage() { return supply_voltage; }

float commandedCurrent() { return target_current; }

bool motorCalibrated() {
    return motor.motor_status == FOCMotorStatus::motor_ready;
}

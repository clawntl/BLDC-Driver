#include "motor_control.h"

#include <SPI.h>
#include <SimpleFOCDrivers.h>
#include <IWatchdog.h>
#include <cmath>

#include "encoders/as5048a/MagneticSensorAS5048A.h"
#include "drivers/drv8316/drv8316.h"

#include "pins.h"
#include "can_protocol.h"
#include "can_bus.h"
#include "motor_config.h"
#include "supply_voltage.h"
#include "board_settings.h"

static SPIClass SPI_1(SPI1_MOSI, SPI1_MISO, SPI1_SCK);

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);

//  BLDCDriver6PWM( int phA_h, int phA_l, int phB_h, int phB_l, int phC_h, int phC_l, int en)
//  - phA_h, phA_l - A phase pwm pin high/low pair
//  - phB_h, phB_l - B phase pwm pin high/low pair
//  - phB_h, phC_l - C phase pwm pin high/low pair
static DRV8316Driver6PWM driver = DRV8316Driver6PWM(PHA_H, PHA_L, PHB_H, PHB_L, PHC_H, PHC_L, DRV_CS, false);

// 600 mV/A matches the DRV8316 CSA gain register code used in setupFOC()
// (Gain_0V25). The DRV8316 datasheet's gain table had an erratum: the real
// gains for codes 0..3 are 0.15 / 0.3 / 0.6 / 1.2 V/A, so the SimpleFOCDrivers
// enum name (0V25) is misleading - code 0b10 is 0.6 V/A. See motor_test.cpp.
static LowsideCurrentSense current_sense = LowsideCurrentSense(600.0, SO_A, SO_B, SO_C);

static MagneticSensorAS5048A sensor(ENC_CS, false, SPISettings(5'000'000, MSBFIRST, SPI_MODE1));

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
    sensor.init(&SPI_1);
    // link the motor to the sensor
    motor.linkSensor(&sensor);

    // Measure the real supply so SimpleFOC's volts -> PWM duty conversion is
    // correct. Must happen before current_sense.init() (see supply_voltage.h).
    supply_voltage = measuredSupplyVoltageOrDefault();
    driver.voltage_power_supply = supply_voltage;
    driver.voltage_limit = min(MOTOR_VOLTAGE_LIMIT, driver.voltage_power_supply);

    driver.init(&SPI_1);

    // small delay to allow SPI response
    delay(10);

    // Set the buck output tot 5V
    driver.setBuckVoltage(VB_5V);
    // Wait some time for everything to stabilise
    delay(500);

    // It's advised to have at least 400ns between read and write calls
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
    driver.setCurrentSenseGain(Gain_0V25); // register code 0b10 = 0.6 V/A (datasheet erratum, see current_sense above)
    delayMicroseconds(1);
    driver.setPWMMode(PWM6_Mode);
    delayMicroseconds(1);

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

    if (!motor.init()) {
        Serial.println("Motor init failed!");
        sendError(ERR_MOTOR_INIT);
        return false;
    }

    if (current_sense.init())  Serial.println("Current sense init success!");
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
    if (currentTime - lastVelocitySendTime > SHAFT_VELOCITY_TELEMETRY_INTERVAL_US) {
        sendFloat(CMD_SHAFT_VELOCITY, direction_sign * motor.shaft_velocity, telemetryCanAddr());
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

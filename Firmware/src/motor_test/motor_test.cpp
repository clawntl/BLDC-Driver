// =============================================================================
//  Motor / sensor bring-up test firmware for the STM32G431 + DRV8316 + AS5048A
//  BLDC driver board.
//
//  Built ONLY by the `motor_test` PlatformIO environment (see platformio.ini).
//  The normal `left` / `right` builds exclude this folder, and this build
//  excludes main.cpp / motor_control.cpp / can_bus.cpp. It shares pins.h,
//  motor_config.h, clock_config.cpp and dfu.cpp with the real firmware so the
//  hardware setup can't drift from what the robot actually runs.
//
//  No CAN: everything is driven from the USB serial console (115200, send a
//  newline after each command). Type `help` for the command list.
//
//  Suggested order:
//    1. s / n / w   - sensor only, power stage never enabled
//    2. drv         - DRV8316 register/fault status (enables the power stage)
//    3. ol 2        - open-loop spin at low voltage, compares sensor speed
//    4. rev         - open-loop single revolution -> pole-pair + direction check
//    5. cs          - current sense offsets / readings
//    6. foc, t 0.5  - SimpleFOC sensor alignment, then closed-loop voltage torque
//
//  SAFETY: open loop pushes current continuously even when the rotor is
//  stalled. Keep `v` low, wheels off the ground, and use a current-limited
//  supply if you have one. Every motor-on mode times out after RUN_TIMEOUT_MS.
// =============================================================================

#include <Arduino.h>
#include <SPI.h>

#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>

#include "encoders/as5048a/MagneticSensorAS5048A.h"
#include "drivers/drv8316/drv8316.h"
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"

#include "pins.h"
#include "motor_config.h"
#include "dfu.h"
#include "supply_voltage.h"

// ---------------------------------------------------------------------------
//  Test parameters
// ---------------------------------------------------------------------------

// DRV8316 CSA gain register setting and the matching SimpleFOC gain in mV/A.
// CAUTION: SimpleFOCDrivers' enum NAMES come from an early DRV8316 register map
// that TI later confirmed was wrong. The real gains for codes 0..3 are
// 0.15 / 0.3 / 0.6 / 1.2 V/A, so `Gain_0V25` (code 2) is actually 0.6 V/A =
// 600 mV/A - which matches motor_control.cpp. Verify on the bench with `cs`.
#define TEST_CSA_GAIN_SETTING   Gain_0V25   // register code 0b10 = 0.6 V/A
#define TEST_CSA_MV_PER_A       600.0f

static const float    DEFAULT_TEST_VOLTAGE = 1.0f;   // V, q-axis voltage for open loop
static const float    MAX_TEST_VOLTAGE     = 4.0f;   // V, hard cap on `v`
static const uint32_t RUN_TIMEOUT_MS       = 30000;  // motor-on modes auto-stop
static const uint32_t STATUS_INTERVAL_MS   = 250;    // stream rate while running

// ---------------------------------------------------------------------------
//  Hardware objects (same construction as motor_control.cpp)
// ---------------------------------------------------------------------------

static SPIClass SPI_1(SPI1_MOSI, SPI1_MISO, SPI1_SCK);

static BLDCMotor motor(MOTOR_POLE_PAIRS);
static DRV8316Driver6PWM driver(PHA_H, PHA_L, PHB_H, PHB_L, PHC_H, PHC_L, DRV_CS, false);
static LowsideCurrentSense current_sense(TEST_CSA_MV_PER_A, SO_A, SO_B, SO_C);
static MagneticSensorAS5048A sensor(ENC_CS, false, SPISettings(5'000'000, MSBFIRST, SPI_MODE1));

// ---------------------------------------------------------------------------
//  State
// ---------------------------------------------------------------------------

enum class Mode { IDLE, WATCH, OPENLOOP, CLOSEDLOOP, HOLD };

static Mode     mode = Mode::IDLE;
static bool     powerStageReady = false;   // driver configured + motor.init() done
static bool     currentSenseReady = false;
static bool     focReady = false;
static float    testVoltage = DEFAULT_TEST_VOLTAGE;
static float    lastVin = NAN;              // last V_SENS reading (only valid pre-`cs`)

static uint32_t modeStartMs = 0;
static uint32_t commandStartMs = 0;          // when the current command was received
static uint32_t lastStatusMs = 0;
static float    lastStatusAngle = 0;
static uint32_t loopCount = 0;

// watch-mode glitch tracking
static float    watchPrevAngle = 0;
static uint32_t watchGlitches = 0;
static float    watchMaxJump = 0;

// ---------------------------------------------------------------------------
//  Small print helpers
// ---------------------------------------------------------------------------

static void kv(const char* k, float v, int digits = 3) {
    Serial.print(k); Serial.print('='); Serial.print(v, digits); Serial.print("  ");
}
static void kv(const char* k, long v) {
    Serial.print(k); Serial.print('='); Serial.print(v); Serial.print("  ");
}

static float wrapPi(float a) {
    while (a >  _PI) a -= _2PI;
    while (a < -_PI) a += _2PI;
    return a;
}

// ---------------------------------------------------------------------------
//  Sensor tests (power stage stays off)
// ---------------------------------------------------------------------------

static void sensorSnapshot() {
    // The AS5048A answers each SPI frame with the result of the PREVIOUS
    // command, so reads are always issued in pairs by the library helpers.
    sensor.readRawAngle();
    const uint16_t raw = sensor.readRawAngle();
    const bool errFlag = sensor.isErrorFlag();
    const AS5048Diagnostics diag = sensor.readDiagnostics();
    const uint16_t mag = sensor.readMagnitude() & 0x3FFF;
    const AS5048Error err = sensor.clearErrorFlag();

    Serial.println(F("--- AS5048A snapshot ---"));
    kv("raw", (long)raw); kv("deg", raw * 360.0f / AS5048A_CPR, 2); kv("errFlag", (long)errFlag);
    Serial.println();
    kv("AGC", (long)diag.agc); kv("OCF", (long)diag.ocf); kv("COF", (long)diag.cof);
    kv("compLow", (long)diag.compLow); kv("compHigh", (long)diag.compHigh); kv("magnitude", (long)mag);
    Serial.println();
    kv("parityErr", (long)err.parityError); kv("cmdInvalid", (long)err.commandInvalid);
    kv("framingErr", (long)err.framingError);
    Serial.println();

    // Interpretation hints
    if (diag.reg == 0x0000 && raw == 0)
        Serial.println(F("!! All zeros: MISO stuck low, sensor unpowered, or CS not reaching the chip."));
    else if ((diag.reg & 0x3FFF) == 0x3FFF && raw == 0x3FFF)
        Serial.println(F("!! All ones: MISO floating/high - check MISO wiring, CS pin, SPI mode."));
    else {
        if (!diag.ocf) Serial.println(F("!! OCF=0: offset compensation not finished (sensor just powered / faulty)."));
        if (diag.cof)  Serial.println(F("!! COF=1: CORDIC overflow - angle data invalid."));
        if (diag.compHigh) Serial.println(F("!! compHigh=1: magnetic field too WEAK (magnet too far / wrong magnet)."));
        if (diag.compLow)  Serial.println(F("!! compLow=1: magnetic field too STRONG (magnet too close)."));
        if (diag.agc >= 250)
            Serial.println(F("!! AGC at/near 255 (max gain): field is marginally WEAK - move magnet closer / stronger magnet."));
        else if (diag.agc <= 5)
            Serial.println(F("!! AGC at/near 0 (min gain): field is marginally STRONG - move magnet further away."));
        else if (diag.ocf && !diag.cof && !diag.compHigh && !diag.compLow)
            Serial.println(F("OK: field in range, AGC has headroom both ways. AGC should stay roughly constant through a full turn."));
    }
}

// Reads N raw samples as fast as possible with the motor off. A healthy,
// stationary sensor shows a spread of a few counts and no glitches.
static void sensorNoiseTest(long n) {
    if (n < 10) n = 2000;
    sensor.readRawAngle();
    const uint16_t first = sensor.readRawAngle();
    int32_t minD = 0, maxD = 0;
    uint32_t errFlags = 0, stuck = 0, glitches = 0;
    uint16_t prev = first;
    const uint32_t t0 = micros();
    for (long i = 0; i < n; i++) {
        const uint16_t r = sensor.readRawAngle();
        if (sensor.isErrorFlag()) errFlags++;
        if (r == 0 || r == 0x3FFF) stuck++;
        int32_t d = (int32_t)r - (int32_t)first;
        if (d >  AS5048A_CPR / 2) d -= AS5048A_CPR;
        if (d < -AS5048A_CPR / 2) d += AS5048A_CPR;
        if (d < minD) minD = d;
        if (d > maxD) maxD = d;
        int32_t step = (int32_t)r - (int32_t)prev;
        if (step >  AS5048A_CPR / 2) step -= AS5048A_CPR;
        if (step < -AS5048A_CPR / 2) step += AS5048A_CPR;
        if (abs(step) > 50) glitches++;   // >1 deg between back-to-back reads at rest
        prev = r;
    }
    const uint32_t dt = micros() - t0;
    const AS5048Error err = sensor.clearErrorFlag();

    Serial.println(F("--- AS5048A noise test (hold the wheel still) ---"));
    kv("samples", n); kv("us/read", (float)dt / n, 2); Serial.println();
    kv("spread_counts", (long)(maxD - minD)); kv("spread_deg", (maxD - minD) * 360.0f / AS5048A_CPR, 3);
    Serial.println();
    kv("errFlags", (long)errFlags); kv("rawStuck(0/3FFF)", (long)stuck); kv("glitches", (long)glitches);
    kv("parityErr", (long)err.parityError); kv("framingErr", (long)err.framingError);
    Serial.println();
    if (glitches || errFlags) Serial.println(F("!! Glitches/error flags at rest: SPI signal integrity, CS contention or magnet issue."));
    else if (maxD - minD > 20) Serial.println(F("!! Spread >20 counts: noisy - check magnet distance/centring and AGC."));
    else Serial.println(F("OK"));
}

static void startWatch() {
    sensor.update();
    watchPrevAngle = sensor.getMechanicalAngle();
    watchGlitches = 0;
    watchMaxJump = 0;
    lastStatusAngle = sensor.getAngle();
    lastStatusMs = millis();
    mode = Mode::WATCH;
    Serial.println(F("Watching sensor - turn the wheel by hand. One turn should be +-6.283 rad. Enter/x to stop."));
}

static void serviceWatch() {
    sensor.update();
    const float a = sensor.getMechanicalAngle();
    const float jump = fabsf(wrapPi(a - watchPrevAngle));
    watchPrevAngle = a;
    if (jump > watchMaxJump) watchMaxJump = jump;
    if (jump > 0.3f) watchGlitches++;   // 0.3 rad in one loop (~tens of us) is not a real hand movement

    const uint32_t now = millis();
    if (now - lastStatusMs >= 100) {
        const float total = sensor.getAngle();
        const float vel = (total - lastStatusAngle) / ((now - lastStatusMs) * 1e-3f);
        lastStatusAngle = total;
        lastStatusMs = now;
        kv("mech_deg", a * RAD_TO_DEG, 1);
        kv("total_rad", total, 3);
        kv("rotations", (long)sensor.getFullRotations());
        kv("vel_rad_s", vel, 2);
        kv("glitches", (long)watchGlitches);
        kv("maxJump_rad", watchMaxJump, 3);
        Serial.println();
    }
}

// ---------------------------------------------------------------------------
//  DRV8316 / power stage
// ---------------------------------------------------------------------------

static void printDRV8316Status() {
    DRV8316Status s = driver.getStatus();
    Serial.println(F("--- DRV8316 status ---"));
    kv("fault", (long)s.isFault()); kv("overCurrent", (long)s.isOverCurrent());
    kv("overTemp", (long)s.isOverTemperature()); kv("overVoltage", (long)s.isOverVoltage());
    kv("powerOnReset", (long)s.isPowerOnReset());
    Serial.println();
    kv("OC_Ah", (long)s.isOverCurrent_Ah()); kv("OC_Al", (long)s.isOverCurrent_Al());
    kv("OC_Bh", (long)s.isOverCurrent_Bh()); kv("OC_Bl", (long)s.isOverCurrent_Bl());
    kv("OC_Ch", (long)s.isOverCurrent_Ch()); kv("OC_Cl", (long)s.isOverCurrent_Cl());
    Serial.println();
    kv("buckErr", (long)s.isBuckError()); kv("buckUV", (long)s.isBuckUnderVoltage());
    kv("buckOC", (long)s.isBuckOverCurrent()); kv("chargePumpUV", (long)s.isChargePumpUnderVoltage());
    kv("otpErr", (long)s.isOneTimeProgrammingError());
    Serial.println();
    kv("spiErr", (long)s.isSPIError()); kv("spiAddr", (long)s.isSPIAddressError());
    kv("spiClock", (long)s.isSPIClockFramingError()); kv("spiParity", (long)s.isSPIParityError());
    Serial.println();
    delayMicroseconds(1);
    kv("pwmMode", (long)driver.getPWMMode());
    delayMicroseconds(1);
    kv("regsLocked", (long)driver.isRegistersLocked());
    Serial.println();
    if (s.isFault()) {
        Serial.println(F("(fault latched - clearing; run `drv` again to see if it comes back)"));
        delayMicroseconds(1);
        driver.clearFault();
    }
}

// analogRead() de-initialises ADC1, which also serves the current-sense pins,
// so VIN can only be re-measured until `cs` has configured current sensing.
static void printSupplyVoltage() {
    if (!currentSenseReady) {
        lastVin = readSupplyVoltage();
        Serial.print(F("VIN = ")); Serial.print(lastVin, 2); Serial.println(F(" V"));
    } else {
        Serial.print(F("VIN = ")); Serial.print(lastVin, 2);
        Serial.println(F(" V (cached: can't re-measure after `cs` - analogRead would break current sensing; reset to re-measure)"));
    }
    if (lastVin < SUPPLY_VOLTAGE_MIN)
        Serial.println(F("!! VIN low/absent - is the motor supply connected (not just USB)?"));
    Serial.print(F("Check with a multimeter; if off, set VSENS_CALIBRATION = meter / reported (now "));
    Serial.print(VSENS_CALIBRATION, 4); Serial.println(F(")."));
}

// Mirrors setupFOC() in motor_control.cpp, then motor.init() (which ENABLES
// the power stage with zero voltage).
static bool ensurePowerStage() {
    if (powerStageReady) return true;

    Serial.println(F("Configuring DRV8316..."));
    driver.voltage_power_supply = measuredSupplyVoltageOrDefault();   // same as real firmware
    lastVin = driver.voltage_power_supply;
    driver.voltage_limit = min(MOTOR_VOLTAGE_LIMIT, driver.voltage_power_supply);
    driver.init(&SPI_1);
    delay(10);
    driver.setBuckVoltage(VB_5V);
    delay(500);
    driver.setOCPLevel(Curr_16A);             delayMicroseconds(1);
    driver.setOCPMode(AutoRetry_Fault);        delayMicroseconds(1);
    driver.setOCPRetryTime(Retry500ms);        delayMicroseconds(1);
    driver.setSlew(Slew_200Vus);               delayMicroseconds(1);
    driver.setPWM100Frequency(FREQ_20KHz);     delayMicroseconds(1);
    driver.setCurrentSenseGain(TEST_CSA_GAIN_SETTING); delayMicroseconds(1);
    driver.setPWMMode(PWM6_Mode);              delayMicroseconds(1);

    if (!driver.initialized) {
        Serial.println(F("!! BLDCDriver6PWM init failed (PWM pin/timer config)."));
        return false;
    }
    delay(100);
    printDRV8316Status();

    motor.linkDriver(&driver);
    motor.linkSensor(&sensor);
    current_sense.linkDriver(&driver);

    motor.voltage_limit = MAX_TEST_VOLTAGE;
    motor.current_limit = MOTOR_CURRENT_LIMIT;
    motor.velocity_limit = 20.0f;
    motor.foc_modulation = FOCModulationType::SinePWM;
    motor.torque_controller = TorqueControlType::voltage;
    motor.controller = MotionControlType::velocity_openloop;
    motor.target = 0;

    if (!motor.init()) {
        Serial.println(F("!! motor.init() failed."));
        return false;
    }
    motor.voltage_limit = testVoltage;
    motor.disable();
    powerStageReady = true;
    Serial.println(F("Power stage ready (currently disabled)."));
    return true;
}

static void stopMotor(const char* why = nullptr) {
    if (powerStageReady) {
        motor.target = 0;
        motor.disable();
    }
    if (why) { Serial.print(F("Stopped: ")); Serial.println(why); }
    mode = Mode::IDLE;
}

// Keeps the FOC loop running for `ms` while blocking; aborts on any serial input.
// Line-ending bytes arriving just after the command are the tail of that
// command's own "\r\n" (terminals that send CRLF), not an abort request.
static bool runFor(uint32_t ms) {
    const uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        motor.loopFOC();
        motor.move();
        while (Serial.available() && millis() - commandStartMs < 200 &&
               (Serial.peek() == '\n' || Serial.peek() == '\r'))
            Serial.read();
        if (Serial.available()) {
            while (Serial.available()) Serial.read();
            stopMotor("aborted by serial input");
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
//  Open loop
// ---------------------------------------------------------------------------

static void startOpenLoop(float shaftVel) {
    if (!ensurePowerStage()) return;
    motor.torque_controller = TorqueControlType::voltage;
    motor.controller = MotionControlType::velocity_openloop;
    motor.voltage_limit = testVoltage;
    motor.target = shaftVel;
    motor.enable();
    sensor.update();
    lastStatusAngle = sensor.getAngle();
    lastStatusMs = millis();
    modeStartMs = lastStatusMs;
    loopCount = 0;
    mode = Mode::OPENLOOP;
    Serial.print(F("Open loop: target ")); Serial.print(shaftVel, 2);
    Serial.print(F(" rad/s shaft at Uq=")); Serial.print(testVoltage, 2);
    Serial.println(F(" V. Enter/x to stop."));
    Serial.println(F("ratio = sensor/commanded speed. ~+1.00 ok; ~-1.00 sensor counts the other way (fine, foc handles it);"));
    Serial.println(F("constant other value = pole pairs wrong; ~0 / erratic = stalling (raise v) or sensor not seeing rotation."));
}

// Commands exactly `revs` mechanical revolutions in open-loop angle mode and
// compares with what the sensor saw. measured/commanded = pp_config/pp_real.
static void revolutionTest(float revs) {
    if (revs == 0) revs = 1;
    if (!ensurePowerStage()) return;
    mode = Mode::IDLE;
    motor.torque_controller = TorqueControlType::voltage;
    motor.controller = MotionControlType::angle_openloop;
    motor.voltage_limit = testVoltage;
    motor.velocity_limit = 1.0f;                // slow: ~0.16 rev/s
    motor.target = motor.shaft_angle;           // hold at current open-loop angle
    motor.enable();

    Serial.println(F("Rev test: aligning rotor to field..."));
    if (!runFor(800)) return;
    const float start = sensor.getAngle();
    const float commanded = revs * _2PI;
    motor.target = motor.shaft_angle + commanded;
    Serial.print(F("Rotating ")); Serial.print(revs, 2); Serial.println(F(" commanded rev(s)..."));
    if (!runFor((uint32_t)(fabsf(commanded) / motor.velocity_limit * 1000.0f) + 800)) return;
    const float measured = sensor.getAngle() - start;
    stopMotor();
    motor.velocity_limit = 20.0f;

    Serial.println(F("--- Revolution test ---"));
    kv("commanded_rad", commanded, 3); kv("measured_rad", measured, 3);
    const float ratio = measured / commanded;
    kv("ratio", ratio, 3);
    Serial.println();
    if (fabsf(ratio) < 0.05f) {
        Serial.println(F("!! Sensor saw ~no movement: motor didn't turn (check phases/DRV/v) or sensor isn't reading the rotor."));
        return;
    }
    kv("configured_pp", (long)MOTOR_POLE_PAIRS);
    kv("estimated_pp", MOTOR_POLE_PAIRS / fabsf(ratio), 2);
    Serial.print(F("direction=")); Serial.println(ratio > 0 ? F("CW (sensor agrees with phase order)") : F("CCW (sensor reversed vs phase order - normal, foc detects it)"));
    if (fabsf(fabsf(ratio) - 1.0f) > 0.05f)
        Serial.println(F("!! |ratio| != 1: wrong pole pairs, or rotor slipping/stalling (try more v, check by eye it did 1 rev)."));
    else
        Serial.println(F("OK: pole pairs consistent."));
}

// ---------------------------------------------------------------------------
//  Current sense
// ---------------------------------------------------------------------------

static void currentSenseTest() {
    if (!ensurePowerStage()) return;
    if (!currentSenseReady) {
        motor.enable();          // PWM must be running for low-side sampling/offset calibration
        delay(50);
        if (!current_sense.init()) {
            Serial.println(F("!! current_sense.init() failed (ADC/timer sync)."));
            motor.disable();
            return;
        }
        currentSenseReady = true;
        motor.linkCurrentSense(&current_sense);
        motor.disable();
    }
    Serial.println(F("--- Current sense (motor at 0 V) ---"));
    Serial.println(F("Offsets are ADC volts at zero current; expect ~half the CSA reference (DRV8316 VREF/2)."));
    kv("offA_V", current_sense.offset_ia, 3); kv("offB_V", current_sense.offset_ib, 3); kv("offC_V", current_sense.offset_ic, 3);
    Serial.println();
    kv("gainA", current_sense.gain_a, 3); kv("gainB", current_sense.gain_b, 3); kv("gainC", current_sense.gain_c, 3);
    Serial.println();
    motor.enable();
    delay(20);
    PhaseCurrent_s c = current_sense.readAverageCurrents(200);
    motor.disable();
    kv("Ia_A", c.a, 3); kv("Ib_A", c.b, 3); kv("Ic_A", c.c, 3);
    Serial.println();
    Serial.println(F("At rest these should all be within ~+-0.05 A. Run `ol` afterwards to see currents under load."));
}

// ---------------------------------------------------------------------------
//  Hold: fixed field for checking current-sense scaling with a multimeter
// ---------------------------------------------------------------------------
//
// Locks the stator field at the electrical angle where SimpleFOC puts the full
// voltage on phase A (Ua = +Uq, Ub = Uc = -Uq/2). With the rotor stationary
// the phase currents are DC (plus PWM ripple), so a multimeter in series with
// the phase A wire reads the true average Ia. Expect Ia ~ +I, Ib ~ Ic ~ -I/2,
// and true gain = configured mV/A * (Ia_firmware / Ia_meter).

static void startHold(float uq) {
    uq = constrain(uq, 0.0f, testVoltage);
    if (!ensurePowerStage()) return;
    if (!currentSenseReady) {
        currentSenseTest();          // initialises current sense (and prints offsets)
        if (!currentSenseReady) return;
    }
    motor.torque_controller = TorqueControlType::voltage;
    motor.controller = MotionControlType::angle_openloop;
    motor.voltage_limit = uq;
    motor.velocity_limit = 1.0f;
    // setPhaseVoltage(Uq, 0, e): Ualpha = -Uq*sin(e)  -> max on phase A at e = 3pi/2.
    // Pick the nearest shaft angle whose electrical angle is 3pi/2.
    const float ePeriod = _2PI / MOTOR_POLE_PAIRS;
    const float base = (3.0f * _PI_2) / MOTOR_POLE_PAIRS;
    const float k = roundf((motor.shaft_angle - base) / ePeriod);
    motor.target = base + k * ePeriod;
    motor.enable();
    modeStartMs = millis();
    lastStatusMs = modeStartMs;
    mode = Mode::HOLD;
    Serial.print(F("Holding field on phase A axis at Uq=")); Serial.print(uq, 2);
    Serial.println(F(" V (rotor may twitch to lock). Expect Ia ~ Uq / R_phase. Enter/x to stop."));
    Serial.println(F("Put a multimeter (DC A) in series with the phase A wire and compare with Ia."));
}

static void serviceHold() {
    motor.loopFOC();
    motor.move();
    const uint32_t now = millis();
    if (now - modeStartMs > RUN_TIMEOUT_MS) { stopMotor("timeout"); return; }
    if (now - lastStatusMs < 500) return;
    lastStatusMs = now;
    // Field is static once the target is reached, so blocking for the average is fine.
    PhaseCurrent_s c = current_sense.readAverageCurrents(100);
    kv("Ia", c.a, 3); kv("Ib", c.b, 3); kv("Ic", c.c, 3); kv("sum", c.a + c.b + c.c, 3);
    // Back-calculate the SOx pin voltages from SimpleFOC's own gain/offset.
    kv("SOA_V", c.a / current_sense.gain_a + current_sense.offset_ia, 3);
    kv("SOB_V", c.b / current_sense.gain_b + current_sense.offset_ib, 3);
    kv("SOC_V", c.c / current_sense.gain_c + current_sense.offset_ic, 3);
    Serial.println();
}

// ---------------------------------------------------------------------------
//  Closed loop
// ---------------------------------------------------------------------------

static void runInitFOC() {
    if (!ensurePowerStage()) return;
    mode = Mode::IDLE;
    // Force a fresh alignment every time so repeatability can be checked.
    motor.sensor_direction = Direction::UNKNOWN;
    motor.zero_electric_angle = NOT_SET;
    motor.voltage_sensor_align = min(testVoltage * 2.0f, MOTOR_VOLTAGE_SENSOR_ALIGN);
    motor.voltage_limit = testVoltage;
    motor.torque_controller = TorqueControlType::voltage;
    motor.controller = MotionControlType::torque;
    motor.target = 0;
    motor.enable();
    Serial.print(F("initFOC with align voltage ")); Serial.print(motor.voltage_sensor_align, 2);
    Serial.println(currentSenseReady ? F(" V (current sense linked - will also be aligned)") : F(" V (no current sense - run `cs` first to include it)"));
    focReady = motor.initFOC();
    Serial.println(F("--- initFOC result ---"));
    kv("success", (long)focReady);
    kv("sensor_direction", (long)motor.sensor_direction);
    kv("zero_electric_angle", motor.zero_electric_angle, 4);
    Serial.println();
    Serial.println(F("Run `foc` 2-3 times: direction must stay the same and zero angle within ~0.1 rad."));
    motor.disable();
}

static void startClosedLoop(float uq) {
    if (!focReady) { Serial.println(F("Run `foc` first.")); return; }
    if (fabsf(uq) > testVoltage) {
        uq = copysignf(testVoltage, uq);
        Serial.print(F("Clamped to v=")); Serial.println(testVoltage, 2);
    }
    motor.torque_controller = TorqueControlType::voltage;
    motor.controller = MotionControlType::torque;
    motor.voltage_limit = testVoltage;
    motor.target = uq;
    motor.enable();
    lastStatusAngle = sensor.getAngle();
    lastStatusMs = millis();
    modeStartMs = lastStatusMs;
    loopCount = 0;
    mode = Mode::CLOSEDLOOP;
    Serial.print(F("Closed-loop voltage torque, Uq=")); Serial.print(uq, 2);
    Serial.println(F(" V. Should spin smoothly in the sign's direction and get faster with |Uq|. Enter/x to stop."));
    Serial.println(F("Rough / buzzing / one-way-only = bad alignment or sensor; spins backwards fast = direction wrong."));
}

static void serviceRunning() {
    motor.loopFOC();
    motor.move();
    loopCount++;

    const uint32_t now = millis();
    if (now - modeStartMs > RUN_TIMEOUT_MS) { stopMotor("timeout"); return; }
    if (now - lastStatusMs < STATUS_INTERVAL_MS) return;

    const float dt = (now - lastStatusMs) * 1e-3f;
    const float total = sensor.getAngle();
    const float vel = (total - lastStatusAngle) / dt;
    lastStatusAngle = total;
    lastStatusMs = now;

    if (mode == Mode::OPENLOOP) {
        kv("cmd_rad_s", motor.target, 2);
        kv("sensor_rad_s", vel, 2);
        kv("ratio", motor.target != 0 ? vel / motor.target : 0.0f, 2);
    } else {
        kv("Uq", motor.target, 2);
        kv("sensor_rad_s", vel, 2);
    }
    kv("loop_Hz", (long)(loopCount / dt));
    loopCount = 0;
    if (currentSenseReady) {
        PhaseCurrent_s c = current_sense.getPhaseCurrents();
        kv("Ia", c.a, 2); kv("Ib", c.b, 2); kv("Ic", c.c, 2);
    }
    Serial.println();
}

// ---------------------------------------------------------------------------
//  Serial command handling
// ---------------------------------------------------------------------------

static void printHelp() {
    Serial.println(F("=== BLDC driver test firmware ==="));
    Serial.println(F(" Sensor (motor off):"));
    Serial.println(F("  s            AS5048A snapshot: angle, AGC, magnitude, field/error flags"));
    Serial.println(F("  n [N]        noise/glitch test, N reads at rest (default 2000)"));
    Serial.println(F("  w            watch angle while turning by hand"));
    Serial.println(F(" Power stage:"));
    Serial.println(F("  drv          configure DRV8316 (if needed) and print status/faults"));
    Serial.print  (F("  v <volts>    set test voltage (now ")); Serial.print(testVoltage, 2);
    Serial.print  (F(" V, max ")); Serial.print(MAX_TEST_VOLTAGE, 1); Serial.println(F(")"));
    Serial.println(F("  ol <rad/s>   open-loop velocity (shaft rad/s, try 2), streams sensor speed"));
    Serial.println(F("  rev [n]      open-loop n revolutions -> pole pairs + direction check"));
    Serial.println(F("  vin          measure supply voltage on V_SENS (only before `cs`)"));
    Serial.println(F("  cs           init current sense, print offsets and currents at rest"));
    Serial.println(F("  hold [volts] lock field on phase A axis, stream DC phase currents + SOx volts"));
    Serial.println(F("  foc          run SimpleFOC sensor alignment (initFOC)"));
    Serial.println(F("  t <volts>    closed-loop voltage torque (after foc)"));
    Serial.println(F("  x / Enter    stop, disable motor"));
    Serial.println(F("  info         reprint this help, VIN and sensor snapshot"));
    Serial.println(F("  reset        full MCU reset (clears cs/foc state, re-measures VIN)"));
    Serial.println(F("  dfu          reboot into DFU bootloader"));
}

static void printBanner() {
    printHelp();
    Serial.println(F("Power stage is NOT enabled until a motor command (drv/ol/rev/cs/foc/hold) is used."));
    printSupplyVoltage();
    sensorSnapshot();
}

static void handleCommand(char* line) {
    commandStartMs = millis();
    // split "cmd arg"
    char* arg = strchr(line, ' ');
    if (arg) { *arg++ = '\0'; while (*arg == ' ') arg++; }
    const bool hasArg = arg && *arg;
    const float fArg = hasArg ? strtof(arg, nullptr) : 0.0f;

    if (line[0] == '\0' || !strcmp(line, "x")) {
        if (mode != Mode::IDLE) stopMotor("user");
        return;
    }
    if (mode != Mode::IDLE) stopMotor("new command");

    if (!strcmp(line, "help") || !strcmp(line, "h") || !strcmp(line, "?")) printHelp();
    else if (!strcmp(line, "info")) printBanner();
    else if (!strcmp(line, "reset")) {
        Serial.println(F("Resetting... (USB will drop - reconnect the monitor, then press Enter)"));
        Serial.flush();
        delay(100);
        NVIC_SystemReset();
    }
    else if (!strcmp(line, "dfu")) { Serial.println(F("Entering DFU mode...")); Serial.flush(); rebootToDFU(); }
    else if (!strcmp(line, "s"))   sensorSnapshot();
    else if (!strcmp(line, "n"))   sensorNoiseTest(hasArg ? atol(arg) : 2000);
    else if (!strcmp(line, "w"))   startWatch();
    else if (!strcmp(line, "drv")) {
        if (ensurePowerStage()) {
            printDRV8316Status();
            kv("driver.voltage_power_supply", driver.voltage_power_supply, 2);
            kv("driver.voltage_limit", driver.voltage_limit, 2);
            Serial.println();
        }
    }
    else if (!strcmp(line, "vin")) printSupplyVoltage();
    else if (!strcmp(line, "hold")) startHold(hasArg ? fArg : testVoltage);
    else if (!strcmp(line, "v")) {
        if (hasArg) testVoltage = constrain(fArg, 0.0f, MAX_TEST_VOLTAGE);
        Serial.print(F("test voltage = ")); Serial.print(testVoltage, 2); Serial.println(F(" V"));
    }
    else if (!strcmp(line, "ol"))  startOpenLoop(hasArg ? fArg : 2.0f);
    else if (!strcmp(line, "rev")) revolutionTest(hasArg ? fArg : 1.0f);
    else if (!strcmp(line, "cs"))  currentSenseTest();
    else if (!strcmp(line, "foc")) runInitFOC();
    else if (!strcmp(line, "t"))   startClosedLoop(hasArg ? fArg : 0.5f);
    else { Serial.print(F("Unknown command: ")); Serial.println(line); Serial.println(F("Type help")); }
}

// Non-blocking line reader (same approach as checkSerialForDFU()).
static void pollSerial() {
    static char buf[32];
    static uint8_t len = 0;
    static bool prevCR = false;
    while (Serial.available()) {
        const char c = (char)Serial.read();
        if (c == '\n' && prevCR) { prevCR = false; continue; }   // \n of a \r\n pair
        prevCR = (c == '\r');
        if (c == '\n' || c == '\r') {
            buf[len] = '\0';      // an empty line is a valid "stop" command
            handleCommand(buf);
            len = 0;
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = c;
        }
    }
}

// ---------------------------------------------------------------------------

void setup() {
    // Deselect BOTH SPI devices before any SPI traffic: they share SPI1, and
    // pinMode(OUTPUT) alone leaves a CS low (selected).
    pinMode(DRV_CS, OUTPUT); digitalWrite(DRV_CS, HIGH);
    pinMode(ENC_CS, OUTPUT); digitalWrite(ENC_CS, HIGH);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(DFU_BUTTON, INPUT);
    pinMode(SO_A, INPUT);
    pinMode(SO_B, INPUT);
    pinMode(SO_C, INPUT);

    Serial.begin(115200);

    SimpleFOCDebug::enable(&Serial);
    SimpleFOC_CORDIC_Config();

    sensor.init(&SPI_1);
    lastVin = readSupplyVoltage();

    digitalWrite(LED_BUILTIN, HIGH);

    // `Serial` (USB CDC) reports ready as soon as the port enumerates, long
    // before a monitor is actually connected, so a timed wait loses the
    // banner. Instead, nag until the user sends anything. The power stage is
    // untouched while waiting, and the DFU button still works.
    uint32_t lastPrompt = 0;
    while (!Serial.available()) {
        checkDfuButton();
        if (millis() - lastPrompt >= 2000) {
            lastPrompt = millis();
            Serial.println(F("BLDC test firmware ready - press Enter to start"));
        }
    }
    while (Serial.available()) Serial.read();   // swallow the keypress(es)

    printBanner();
}

void loop() {
    checkDfuButton();
    pollSerial();

    switch (mode) {
        case Mode::WATCH:      serviceWatch();   break;
        case Mode::OPENLOOP:
        case Mode::CLOSEDLOOP: serviceRunning(); break;
        case Mode::HOLD:       serviceHold();    break;
        case Mode::IDLE:
        default:
            if (powerStageReady) motor.loopFOC();  // keeps sensor rotation tracking alive
            else sensor.update();
            break;
    }
}

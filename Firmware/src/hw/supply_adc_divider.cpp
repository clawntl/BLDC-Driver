// Supply sensing through a resistor divider on an ADC1 pin (BLDC-Driver):
//   VIN -> R17 100k -> PA3 (V_SENS) -> R16 10k -> GND   =>  V_PA3 = VIN / 11
//
// V_SENS is on ADC1 together with the SOA/SOB/SOC current-sense pins, so it is
// read in two different ways:
//  - before the current sense has been initialised: analogRead() (it
//    initialises and de-initialises ADC1 for every read, which is fine while
//    nothing else uses ADC1);
//  - afterwards: a software-triggered REGULAR conversion on ADC1. The current
//    sense uses INJECTED conversions triggered by the PWM timer, which have
//    priority and simply interrupt a regular conversion, so the two coexist.
//    analogRead() must never be used again from that point: it would
//    HAL_ADC_DeInit() ADC1 and silently break current sensing. That is why
//    readSupplyVoltage() returns false once supplyCurrentSenseStarted() has
//    been called.
//
// The regular conversion is started in one call and collected in a later one
// (no busy-waiting in the FOC loop).

#include "supply.h"

#if defined(SUPPLY_SENSE_ADC_DIVIDER)

#include <Arduino.h>
#include <PeripheralPins.h>
#include <stm32yyxx_ll_adc.h>

namespace {

// Each live sample is one ADC conversion (~7 us), so sampling often costs
// nothing noticeable in the FOC loop; a first-order low-pass smooths it.
constexpr uint32_t SAMPLE_INTERVAL_MS = 10;
constexpr uint32_t FILTER_TAU_MS      = 200;

float filtered = 0;
bool haveValue = false;
uint32_t lastSampleMs = 0;

// Regular-conversion mode (after the current sense owns ADC1).
bool useRegular = false;
bool regularReady = false;   // channel configured
bool converting = false;
uint32_t channel = 0;
uint32_t conversionStartMs = 0;

float rawToVolts(float raw) {
    return raw * (VSENS_ADC_REF_V / 4095.0f) * VSENS_DIVIDER_RATIO * VSENS_CALIBRATION;
}

// Averaged analogRead(). Only valid before the current sense starts.
float readAveraged(uint16_t samples) {
    if (samples == 0) samples = 1;
    analogReadResolution(12);
    analogRead(V_SENS);   // discard first conversion after (re)configuring the pin
    uint32_t sum = 0;
    for (uint16_t i = 0; i < samples; i++) sum += analogRead(V_SENS);
    return rawToVolts(sum / (float)samples);
}

void addSample(float v) {
    if (!haveValue) {
        filtered = v;
        haveValue = true;
        return;
    }
    // First-order low-pass: alpha = dt / tau.
    constexpr float alpha = (float)SAMPLE_INTERVAL_MS / (float)FILTER_TAU_MS;
    filtered += alpha * (v - filtered);
}

// Points the ADC1 regular sequence at V_SENS with a long sample time (the
// ~9 kohm divider has no filter capacitor, so the default 2.5-cycle sample
// time would read low). SMPRx and SQRx may only be written while no
// conversion is ongoing (ADSTART = 0 and JADSTART = 0, RM0440 21.4.12), so the
// injected (current sense) conversions are stopped for the few microseconds
// this takes and then re-armed. They are hardware-triggered by the PWM timer,
// so re-setting JADSTART simply resumes them on the next trigger.
bool configureRegularChannel() {
    const PinName pin = digitalPinToPinName(V_SENS);
    const uint32_t function = pinmap_function(pin, PinMap_ADC);
    if (function == (uint32_t)NC) {
        Serial.println("Supply monitor: V_SENS is not an ADC pin");
        return false;
    }
    if (pinmap_peripheral(pin, PinMap_ADC) != (void *)ADC1) {
        Serial.println("Supply monitor: V_SENS is not on ADC1");
        return false;
    }
    if (!LL_ADC_IsEnabled(ADC1)) {
        Serial.println("Supply monitor: ADC1 not enabled (current sense not running?)");
        return false;
    }
    channel = __LL_ADC_DECIMAL_NB_TO_CHANNEL(STM_PIN_CHANNEL(function));
    pinmap_pinout(pin, PinMap_ADC); // GPIO to analog mode

    const bool injectedRunning = LL_ADC_INJ_IsConversionOngoing(ADC1);
    if (injectedRunning) {
        LL_ADC_INJ_StopConversion(ADC1);
        const uint32_t t0 = micros();
        while (LL_ADC_INJ_IsStopConversionOngoing(ADC1)) {
            if (micros() - t0 > 100) break; // a conversion takes < 1 us here
        }
    }
    // A stray regular conversion (shouldn't be one) would block the writes too.
    if (LL_ADC_REG_IsConversionOngoing(ADC1)) {
        LL_ADC_REG_StopConversion(ADC1);
        while (LL_ADC_REG_IsStopConversionOngoing(ADC1)) { }
    }

    LL_ADC_SetChannelSamplingTime(ADC1, channel, LL_ADC_SAMPLINGTIME_247CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, channel, LL_ADC_SINGLE_ENDED);
    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, channel);
    LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
    LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);

    if (injectedRunning) LL_ADC_INJ_StartConversion(ADC1);

    LL_ADC_ClearFlag_EOC(ADC1);
    LL_ADC_ClearFlag_OVR(ADC1);
    return true;
}

void sampleRegular(uint32_t now) {
    if (!regularReady) return;
    if (!converting) {
        LL_ADC_ClearFlag_EOC(ADC1);
        LL_ADC_REG_StartConversion(ADC1);
        converting = true;
        conversionStartMs = now;
        return;
    }
    if (LL_ADC_IsActiveFlag_EOC(ADC1)) {
        const uint32_t raw = LL_ADC_REG_ReadConversionData12(ADC1); // also clears EOC
        converting = false;
        lastSampleMs = now;
        addSample(rawToVolts(raw));
    } else if (now - conversionStartMs > 10) {
        // ~7 us expected; give up on this one and start again.
        converting = false;
    }
}

} // namespace

const char *supplySensorName() { return "V_SENS"; }

void supplyBegin() {
    // The first sample (one interval from now) seeds the filter. Not seeded
    // from measuredSupplyVoltageOrDefault(): that returns the 12 V fallback
    // when VIN is out of range (e.g. USB power only), which would be
    // reported to the host as a real reading.
    lastSampleMs = millis();
}

bool readSupplyVoltage(float &volts) {
    if (useRegular) return false; // analogRead() would break the current sense
    volts = readAveraged(64);
    return true;
}

bool readSupplyCurrent(float &) { return false; }

void supplyCurrentSenseStarted() {
    useRegular = true;
    converting = false;
    regularReady = configureRegularChannel();
    if (regularReady) Serial.println("Supply monitor: live VIN via ADC1 regular conversions");
}

void updateSupplyMonitor() {
    const uint32_t now = millis();
    if (useRegular) {
        // Start a conversion once the interval has passed; collect it on a
        // later call.
        if (converting || now - lastSampleMs >= SAMPLE_INTERVAL_MS) sampleRegular(now);
        return;
    }
    if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;
    lastSampleMs = now;
    // Short burst: analogRead() takes ~10 us each at the long sample time.
    addSample(readAveraged(4));
}

float supplyVoltageNow() { return haveValue ? filtered : 0.0f; }

#endif // SUPPLY_SENSE_ADC_DIVIDER

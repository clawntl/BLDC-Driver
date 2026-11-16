#include "supply_monitor.h"

#include <Arduino.h>
#include <PeripheralPins.h>
#include <stm32yyxx_ll_adc.h>

#include "pins.h"
#include "motor_config.h"
#include "supply_voltage.h"

namespace {

float filtered = 0;
bool haveValue = false;
uint32_t lastSampleMs = 0;

// Regular-conversion mode (after the current sense owns ADC1).
bool useRegular = false;
bool regularReady = false;   // channel configured
bool converting = false;
uint32_t channel = 0;
uint32_t conversionStartMs = 0;

float rawToVolts(uint32_t raw) {
    return raw * (VSENS_ADC_REF_V / 4095.0f) * VSENS_DIVIDER_RATIO * VSENS_CALIBRATION;
}

void addSample(float v) {
    if (!haveValue) {
        filtered = v;
        haveValue = true;
        return;
    }
    // First-order low-pass: alpha = dt / tau.
    constexpr float alpha = (float)SUPPLY_SAMPLE_INTERVAL_MS / (float)SUPPLY_FILTER_TAU_MS;
    filtered += alpha * (v - filtered);
}

// Points the ADC1 regular sequence at V_SENS with a long sample time.
// SMPRx and SQRx may only be written while no conversion is ongoing
// (ADSTART = 0 and JADSTART = 0, RM0440 21.4.12), so the injected (current
// sense) conversions are stopped for the few microseconds this takes and then
// re-armed. They are hardware-triggered by the PWM timer, so re-setting
// JADSTART simply resumes them on the next trigger.
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

void initSupplyMonitor() {
    // The first sample (one interval from now) seeds the filter. Not seeded
    // from measuredSupplyVoltageOrDefault(): that returns the 12 V fallback
    // when VIN is out of range (e.g. USB power only), which would be
    // reported to the host as a real reading.
    lastSampleMs = millis();
}

void supplyMonitorCurrentSenseStarted() {
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
        if (converting || now - lastSampleMs >= SUPPLY_SAMPLE_INTERVAL_MS) sampleRegular(now);
        return;
    }
    if (now - lastSampleMs < SUPPLY_SAMPLE_INTERVAL_MS) return;
    lastSampleMs = now;
    // Short burst: analogRead() takes ~10 us each at the long sample time.
    addSample(readSupplyVoltage(4));
}

float supplyVoltageNow() { return haveValue ? filtered : 0.0f; }

uint16_t supplyMillivolts() {
    if (!haveValue) return 0;
    const float mv = filtered * 1000.0f;
    if (mv < 1.0f) return 1;
    if (mv > 65535.0f) return 65535;
    return (uint16_t)lroundf(mv);
}

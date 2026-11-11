#include "supply_voltage.h"

#include <Arduino.h>

#include "pins.h"
#include "motor_config.h"

float readSupplyVoltage(uint16_t samples) {
    if (samples == 0) samples = 1;
    analogReadResolution(12);
    analogRead(V_SENS);   // discard first conversion after (re)configuring the pin
    uint32_t sum = 0;
    for (uint16_t i = 0; i < samples; i++) sum += analogRead(V_SENS);
    const float vPin = (sum / (float)samples) * (VSENS_ADC_REF_V / 4095.0f);
    return vPin * VSENS_DIVIDER_RATIO * VSENS_CALIBRATION;
}

float measuredSupplyVoltageOrDefault() {
    const float vin = readSupplyVoltage();
    Serial.print("Supply voltage (V_SENS): ");
    Serial.print(vin, 2);
    Serial.println(" V");
    if (vin < SUPPLY_VOLTAGE_MIN || vin > SUPPLY_VOLTAGE_MAX) {
        Serial.print("WARNING: outside ");
        Serial.print(SUPPLY_VOLTAGE_MIN, 1);
        Serial.print("-");
        Serial.print(SUPPLY_VOLTAGE_MAX, 1);
        Serial.print(" V sanity range, using default ");
        Serial.print(MOTOR_VOLTAGE_POWER_SUPPLY, 1);
        Serial.println(" V");
        return MOTOR_VOLTAGE_POWER_SUPPLY;
    }
    return vin;
}

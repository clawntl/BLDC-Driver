// Parts of supply.h shared by every supply sensor.

#include "supply.h"

#include <Arduino.h>
#include <cmath>

#include "motor_config.h"

float measuredSupplyVoltageOrDefault() {
    float vin = 0;
    if (!readSupplyVoltage(vin)) {
        Serial.print("WARNING: couldn't read the supply voltage (");
        Serial.print(supplySensorName());
        Serial.print("), using default ");
        Serial.print(MOTOR_VOLTAGE_POWER_SUPPLY, 1);
        Serial.println(" V");
        return MOTOR_VOLTAGE_POWER_SUPPLY;
    }
    Serial.print("Supply voltage (");
    Serial.print(supplySensorName());
    Serial.print("): ");
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

uint16_t supplyMillivolts() {
    const float v = supplyVoltageNow();
    if (!(v > 0.0f)) return 0;
    const float mv = v * 1000.0f;
    if (mv < 1.0f) return 1;
    if (mv > 65535.0f) return 65535;
    return (uint16_t)lroundf(mv);
}

int16_t supplyMilliamps() {
    float a;
    if (!supplyCurrentNow(a)) return INT16_MIN;
    const float ma = a * 1000.0f;
    if (ma >  32767.0f) return 32767;
    if (ma < -32767.0f) return -32767;
    return (int16_t)lroundf(ma);
}

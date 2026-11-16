#include "power_monitor.h"

#include <Arduino.h>
#include <Wire.h>

#include "pins.h"
#include "motor_config.h"

namespace {

constexpr uint8_t REG_CONFIG   = 0x00;
constexpr uint8_t REG_SHUNT_V  = 0x01; // signed, 2.5 uV/LSB
constexpr uint8_t REG_BUS_V    = 0x02; // 1.25 mV/LSB
constexpr uint8_t REG_MANUF_ID = 0xFE; // 0x5449 ("TI")
constexpr uint8_t REG_DIE_ID   = 0xFF; // 0x2260

// RESET=0, AVG=16 (010), VBUSCT=1.1 ms (100), VSHCT=1.1 ms (100),
// MODE=shunt+bus continuous (111). One averaged result every ~35 ms.
constexpr uint16_t CONFIG_VALUE = 0x4527;

constexpr float SHUNT_LSB_V = 2.5e-6f;
constexpr float BUS_LSB_V   = 1.25e-3f;

bool present = false;

bool writeReg(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(INA226_I2C_ADDR);
    Wire.write(reg);
    Wire.write(static_cast<uint8_t>(value >> 8));
    Wire.write(static_cast<uint8_t>(value & 0xFF));
    return Wire.endTransmission() == 0;
}

bool readReg(uint8_t reg, uint16_t &value) {
    Wire.beginTransmission(INA226_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(static_cast<uint8_t>(INA226_I2C_ADDR), static_cast<uint8_t>(2)) != 2) return false;
    const uint8_t hi = Wire.read();
    const uint8_t lo = Wire.read();
    value = (static_cast<uint16_t>(hi) << 8) | lo;
    return true;
}

} // namespace

bool powerMonitorBegin() {
    Wire.setSDA(I2C1_SDA);
    Wire.setSCL(I2C1_SCL);
    Wire.begin();
    Wire.setClock(400'000);

    uint16_t manuf = 0, die = 0;
    present = readReg(REG_MANUF_ID, manuf) && readReg(REG_DIE_ID, die)
           && manuf == 0x5449 && die == 0x2260
           && writeReg(REG_CONFIG, CONFIG_VALUE);
    if (!present) {
        Serial.println("WARNING: INA226 power monitor not found at 0x40");
        return false;
    }
    // Let the first averaged conversion finish before anyone reads it.
    delay(40);
    return true;
}

bool powerMonitorPresent() { return present; }

bool readSupplyVoltage(float &volts) {
    uint16_t raw;
    if (!present || !readReg(REG_BUS_V, raw)) return false;
    volts = raw * BUS_LSB_V;
    return true;
}

bool readSupplyCurrent(float &amps) {
    uint16_t raw;
    if (!present || !readReg(REG_SHUNT_V, raw)) return false;
    amps = static_cast<int16_t>(raw) * SHUNT_LSB_V / INA226_SHUNT_OHMS;
    return true;
}

float measuredSupplyVoltageOrDefault() {
    float vin = 0;
    if (!readSupplyVoltage(vin)) {
        Serial.print("WARNING: couldn't read the supply voltage, using default ");
        Serial.print(MOTOR_VOLTAGE_POWER_SUPPLY, 1);
        Serial.println(" V");
        return MOTOR_VOLTAGE_POWER_SUPPLY;
    }
    Serial.print("Supply voltage (INA226): ");
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

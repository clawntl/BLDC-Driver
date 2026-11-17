// Supply sensing with an INA226 power monitor on I2C1 (MiniFOC):
//   V_IN -> shunt (INA226_SHUNT_OHMS) -> V_M (DRV8316 + 5 V buck)
// Bus voltage is measured on the V_IN side. Current is positive when the
// board draws from the supply and negative when the motor regenerates.
//
// Unlike an ADC divider this goes over I2C, so it can be read at any time,
// including while FOC is running (each read blocks for roughly 100 us).

#include "supply.h"

#if defined(SUPPLY_SENSE_INA226)

#include <Arduino.h>
#include <Wire.h>

namespace {

// How often the live voltage for the heartbeat is read. Each read is an I2C
// transfer that blocks for ~100 us, so keep this well above the FOC loop
// period. No software filter: the INA226 already averages (CONFIG_VALUE).
constexpr uint32_t SAMPLE_INTERVAL_MS = 100;

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

float liveVolts = 0;
uint32_t lastSampleMs = 0;
uint8_t failedReads = 0;

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

const char *supplySensorName() { return "INA226"; }

void supplyBegin() {
    Wire.setSDA(I2C1_SDA);
    Wire.setSCL(I2C1_SCL);
    Wire.begin();
    Wire.setClock(400'000);

    uint16_t manuf = 0, die = 0;
    present = readReg(REG_MANUF_ID, manuf) && readReg(REG_DIE_ID, die)
           && manuf == 0x5449 && die == 0x2260
           && writeReg(REG_CONFIG, CONFIG_VALUE);
    if (!present) {
        Serial.print("WARNING: INA226 power monitor not found at 0x");
        Serial.println(INA226_I2C_ADDR, HEX);
        return;
    }
    // Let the first averaged conversion finish before anyone reads it.
    delay(40);
}

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

void supplyCurrentSenseStarted() { }

void updateSupplyMonitor() {
    if (!present) return;
    const uint32_t now = millis();
    if (lastSampleMs != 0 && now - lastSampleMs < SAMPLE_INTERVAL_MS) return;
    lastSampleMs = now ? now : 1;
    float v;
    if (readSupplyVoltage(v)) {
        liveVolts = v;
        failedReads = 0;
    } else if (++failedReads >= 3) {
        liveVolts = 0;  // stop reporting a stale value
    }
}

float supplyVoltageNow() { return liveVolts; }

#endif // SUPPLY_SENSE_INA226

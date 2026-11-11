#include "board_settings.h"

#include <Arduino.h>
#include <stm32g4xx_ll_utils.h> // LL_GetFlashSize()
#include <EEPROM.h> // brings in the buffered eeprom_* API from stm32_eeprom.h
#include <cmath>
#include <cstring>

#include "can_protocol.h"

namespace {

// Layout written to flash. Bump SETTINGS_VERSION whenever BoardSettings
// changes shape: an old page then fails validation and the board reports
// itself unconfigured, instead of misreading stale bytes.
constexpr uint32_t SETTINGS_MAGIC   = 0x53425244; // "SBRD"
constexpr uint16_t SETTINGS_VERSION = 1;

struct StoredSettings {
    uint32_t      magic;
    uint16_t      version;
    uint16_t      size;      // sizeof(StoredSettings), extra layout check
    BoardSettings settings;
    uint32_t      crc;       // CRC-32 of every byte before this field
};

BoardSettings active;
BoardSettings pending;

// Plain bitwise CRC-32 (IEEE 802.3, reflected). Tiny and only run at boot/save.
uint32_t crc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

void readStored(StoredSettings &out) {
    eeprom_buffer_fill(); // copy the whole flash page into the RAM buffer
    uint8_t *dst = reinterpret_cast<uint8_t *>(&out);
    for (size_t i = 0; i < sizeof(out); i++) {
        dst[i] = eeprom_buffered_read_byte(i);
    }
}

bool settingsAreSane(const BoardSettings &s) {
    if (s.side != MotorSide::Unset && s.side != MotorSide::Left && s.side != MotorSide::Right) return false;
    if (s.calibrationValid) {
        if (s.sensorDirection != 1 && s.sensorDirection != -1) return false;
        if (!std::isfinite(s.zeroElectricAngle)) return false;
        if (s.zeroElectricAngle < 0.0f || s.zeroElectricAngle > 6.2832f) return false;
    }
    return true;
}

bool storedIsValid(const StoredSettings &st) {
    if (st.magic != SETTINGS_MAGIC) return false;
    if (st.version != SETTINGS_VERSION) return false;
    if (st.size != sizeof(StoredSettings)) return false;
    const uint32_t crc = crc32(reinterpret_cast<const uint8_t *>(&st), offsetof(StoredSettings, crc));
    if (crc != st.crc) return false;
    return settingsAreSane(st.settings);
}

} // namespace

static_assert(sizeof(StoredSettings) <= E2END + 1, "settings do not fit in the EEPROM emulation page");

const BoardSettings &activeSettings() { return active; }
BoardSettings &pendingSettings() { return pending; }

bool loadBoardSettings() {
    StoredSettings st;
    readStored(st);
    if (!storedIsValid(st)) {
        active  = BoardSettings{};
        pending = BoardSettings{};
        return false;
    }
    active  = st.settings;
    pending = st.settings;
    return true;
}

// End of the firmware image in flash (initialised data is stored after .text).
extern "C" uint32_t _sidata, _sdata, _edata;

bool firmwareOverlapsSettings() {
    const uint32_t imageEnd = reinterpret_cast<uint32_t>(&_sidata)
                            + (reinterpret_cast<uint32_t>(&_edata) - reinterpret_cast<uint32_t>(&_sdata));
    return imageEnd > settingsFlashAddress();
}

bool saveBoardSettings() {
    // Never erase a page that holds our own code.
    if (firmwareOverlapsSettings()) return false;

    StoredSettings st;
    memset(static_cast<void *>(&st), 0, sizeof(st)); // deterministic padding bytes -> deterministic CRC
    st.magic    = SETTINGS_MAGIC;
    st.version  = SETTINGS_VERSION;
    st.size     = sizeof(StoredSettings);
    st.settings = pending;
    st.crc      = crc32(reinterpret_cast<const uint8_t *>(&st), offsetof(StoredSettings, crc));

    // Buffered API: one page erase + program for the whole struct. (The
    // per-byte EEPROM.write()/put() API erases and rewrites the page for
    // EVERY byte, which is slow and burns flash endurance.)
    eeprom_buffer_fill();
    const uint8_t *src = reinterpret_cast<const uint8_t *>(&st);
    for (size_t i = 0; i < sizeof(st); i++) {
        eeprom_buffered_write_byte(i, src[i]);
    }
    eeprom_buffer_flush();

    // Verify by reading the flash back.
    StoredSettings check;
    readStored(check);
    return memcmp(&check, &st, sizeof(st)) == 0;
}

bool boardConfigured() {
    return active.side == MotorSide::Left || active.side == MotorSide::Right;
}

uint32_t thisMotorCanAddr() {
    return active.side == MotorSide::Left ? LEFT_MOTOR_CAN_ADDR : RIGHT_MOTOR_CAN_ADDR;
}

uint32_t telemetryCanAddr() {
    return active.side == MotorSide::Left ? LEFT_MOTOR_SEND_TO : RIGHT_MOTOR_SEND_TO;
}

uint8_t broadcastFloatOffset() {
    // Left motor reads the first 4 bytes, right motor the last 4.
    return active.side == MotorSide::Left ? 0 : 4;
}

const char *sideName(MotorSide side) {
    switch (side) {
        case MotorSide::Left:  return "left";
        case MotorSide::Right: return "right";
        default:               return "unset";
    }
}

uint32_t settingsFlashAddress() {
    // Same calculation as stm32_eeprom.c's default FLASH_BASE_ADDRESS (last page).
    return FLASH_BASE + LL_GetFlashSize() * 1024 - FLASH_PAGE_SIZE;
}

#include "board_settings.h"

#include <Arduino.h>
#include <stm32g4xx_ll_utils.h> // LL_GetFlashSize()
#include <EEPROM.h> // brings in the buffered eeprom_* API from stm32_eeprom.h
#include <cmath>
#include <cstring>

#include "can_protocol.h"
#include "cogging.h"

namespace {

// Layout written to flash. Bump SETTINGS_VERSION whenever BoardSettings
// changes shape: an old page then fails validation and the board reports
// itself unconfigured, instead of misreading stale bytes.
constexpr uint32_t SETTINGS_MAGIC   = 0x53425244; // "SBRD"
constexpr uint16_t SETTINGS_VERSION = 3;

struct StoredSettings {
    uint32_t      magic;
    uint16_t      version;
    uint16_t      size;      // sizeof(StoredSettings), extra layout check
    BoardSettings settings;
    uint32_t      crc;       // CRC-32 of every byte before this field
};

// Layout 1 (firmware 1.0-1.3): BoardSettings without the current loop. Read
// so that updating the firmware keeps a board's node ID and alignment.
struct BoardSettingsV1 {
    uint8_t nodeId;
    bool    invertDirection;
    bool    calibrationValid;
    int8_t  sensorDirection;
    float   zeroElectricAngle;
};
struct StoredSettingsV1 {
    uint32_t        magic;
    uint16_t        version;
    uint16_t        size;
    BoardSettingsV1 settings;
    uint32_t        crc;
};
static_assert(sizeof(StoredSettingsV1) == 20, "layout 1 must match what firmware 1.0-1.3 wrote");

// Layout 2 (firmware 1.4-1.6): without cogComp.
struct BoardSettingsV2 {
    uint8_t nodeId;
    bool    invertDirection;
    bool    calibrationValid;
    int8_t  sensorDirection;
    float   zeroElectricAngle;
    float   currentP;
    float   currentI;
    float   currentTf;
};
struct StoredSettingsV2 {
    uint32_t        magic;
    uint16_t        version;
    uint16_t        size;
    BoardSettingsV2 settings;
    uint32_t        crc;
};
static_assert(sizeof(StoredSettingsV2) == 32, "layout 2 must match what firmware 1.4-1.6 wrote");

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

template <typename T>
void readStored(T &out) {
    eeprom_buffer_fill(); // copy the whole flash page into the RAM buffer
    uint8_t *dst = reinterpret_cast<uint8_t *>(&out);
    for (size_t i = 0; i < sizeof(out); i++) {
        dst[i] = eeprom_buffered_read_byte(i);
    }
}

bool settingsAreSane(const BoardSettings &s) {
    if (s.nodeId != 0 && !validNodeId(s.nodeId)) return false;
    if (s.calibrationValid) {
        if (s.sensorDirection != 1 && s.sensorDirection != -1) return false;
        if (!std::isfinite(s.zeroElectricAngle)) return false;
        if (s.zeroElectricAngle < 0.0f || s.zeroElectricAngle > 6.2832f) return false;
    }
    if (s.cogComp > 2) return false;
    return validCurrentP(s.currentP) && validCurrentI(s.currentI) && validCurrentTf(s.currentTf);
}

bool storedV1IsValid(const StoredSettingsV1 &st) {
    if (st.magic != SETTINGS_MAGIC || st.version != 1 || st.size != sizeof(StoredSettingsV1)) return false;
    return crc32(reinterpret_cast<const uint8_t *>(&st), offsetof(StoredSettingsV1, crc)) == st.crc;
}

bool storedV2IsValid(const StoredSettingsV2 &st) {
    if (st.magic != SETTINGS_MAGIC || st.version != 2 || st.size != sizeof(StoredSettingsV2)) return false;
    return crc32(reinterpret_cast<const uint8_t *>(&st), offsetof(StoredSettingsV2, crc)) == st.crc;
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

static_assert(sizeof(StoredSettings) <= COG_TABLE_OFFSET, "settings would overlap the anti-cogging table");

const BoardSettings &activeSettings() { return active; }
BoardSettings &pendingSettings() { return pending; }

bool loadBoardSettings() {
    StoredSettings st;
    readStored(st);
    if (storedIsValid(st)) {
        active  = st.settings;
        pending = st.settings;
        return true;
    }
    // Saved by firmware 1.4-1.6? Keep it, with anti-cogging off. (Older
    // layouts are rewritten in the new one at the next save.)
    BoardSettings s{};
    StoredSettingsV2 v2;
    readStored(v2);
    if (storedV2IsValid(v2)) {
        s.nodeId            = v2.settings.nodeId;
        s.invertDirection   = v2.settings.invertDirection;
        s.calibrationValid  = v2.settings.calibrationValid;
        s.sensorDirection   = v2.settings.sensorDirection;
        s.zeroElectricAngle = v2.settings.zeroElectricAngle;
        s.currentP          = v2.settings.currentP;
        s.currentI          = v2.settings.currentI;
        s.currentTf         = v2.settings.currentTf;
        if (settingsAreSane(s)) {
            active  = s;
            pending = s;
            return true;
        }
        s = BoardSettings{};
    }
    // Saved by firmware 1.0-1.3? Keep it; the current loop gets its defaults.
    StoredSettingsV1 v1;
    readStored(v1);
    if (storedV1IsValid(v1)) {
        s.nodeId            = v1.settings.nodeId;
        s.invertDirection   = v1.settings.invertDirection;
        s.calibrationValid  = v1.settings.calibrationValid;
        s.sensorDirection   = v1.settings.sensorDirection;
        s.zeroElectricAngle = v1.settings.zeroElectricAngle;
        if (settingsAreSane(s)) {
            active  = s;
            pending = s;
            return true;
        }
    }
    active  = BoardSettings{};
    pending = BoardSettings{};
    return false;
}

// End of the firmware image in flash (initialised data is stored after .text).
extern "C" uint32_t _sidata, _sdata, _edata;

uint32_t firmwareImageBytes() {
    const uint32_t imageEnd = reinterpret_cast<uint32_t>(&_sidata)
                            + (reinterpret_cast<uint32_t>(&_edata) - reinterpret_cast<uint32_t>(&_sdata));
    return imageEnd - FLASH_BASE;
}

bool firmwareOverlapsSettings() {
    return FLASH_BASE + firmwareImageBytes() > settingsFlashAddress();
}

bool settingsUnsaved() {
    return active.nodeId != pending.nodeId
        || active.invertDirection != pending.invertDirection
        || active.calibrationValid != pending.calibrationValid
        || active.sensorDirection != pending.sensorDirection
        || active.zeroElectricAngle != pending.zeroElectricAngle
        || active.currentP != pending.currentP
        || active.currentI != pending.currentI
        || active.currentTf != pending.currentTf
        || active.cogComp != pending.cogComp
        || Cogging::tableUnsaved();
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
    // The anti-cogging table goes in the same page, after the settings.
    Cogging::writeToEepromBuffer();
    eeprom_buffer_flush();

    // Verify by reading the flash back (readStored() refills the buffer).
    StoredSettings check;
    readStored(check);
    const bool tableOk = Cogging::verifyEepromBuffer();
    return memcmp(&check, &st, sizeof(st)) == 0 && tableOk;
}

bool boardConfigured() {
    return validNodeId(active.nodeId);
}

bool validNodeId(int id) {
    return id >= NODE_ID_MIN && id <= NODE_ID_MAX;
}

bool validCurrentP(float v)  { return std::isfinite(v) && v >= 0.0f && v <= CURRENT_LOOP_P_MAX; }
bool validCurrentI(float v)  { return std::isfinite(v) && v >= 0.0f && v <= CURRENT_LOOP_I_MAX; }
bool validCurrentTf(float v) { return std::isfinite(v) && v >= 0.0f && v <= CURRENT_LOOP_TF_MAX; }

uint32_t commandCanId()      { return COMMAND_ID_BASE + active.nodeId; }
uint32_t telemetryCanId()    { return TELEMETRY_ID_BASE + active.nodeId; }
uint32_t currentFrameCanId() { return CURRENT_FRAME_BASE + (active.nodeId - 1) / 2; }
uint8_t  currentFloatOffset() { return (active.nodeId % 2) ? 0 : 4; } // odd: bytes 0-3, even: 4-7

uint32_t settingsFlashAddress() {
    // Same calculation as stm32_eeprom.c's default FLASH_BASE_ADDRESS (last page).
    return FLASH_BASE + LL_GetFlashSize() * 1024 - FLASH_PAGE_SIZE;
}

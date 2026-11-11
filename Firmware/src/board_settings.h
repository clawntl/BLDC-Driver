#pragma once

#include <cstdint>

// Per-board settings kept in flash, so a single firmware image can be flashed
// to both motor drivers and then configured over USB serial (see
// serial_console.h for the commands).
//
// Storage: STM32duino's EEPROM emulation, which uses the LAST 2 KB flash page
// of the STM32G431CB (0x0801F800). The firmware image must therefore stay
// below 126 KB. A DFU upload only erases the pages it writes, so the settings
// survive reflashing; a full-chip erase (STM32CubeProgrammer / ST-Link) wipes
// them and the board comes back up unconfigured.

enum class MotorSide : uint8_t {
    Unset = 0,
    Left  = 1,
    Right = 2,
};

struct BoardSettings {
    MotorSide side = MotorSide::Unset;

    // Flip the sign of the commanded current and of the reported velocity, so
    // the host can use the same sign convention for both (mirrored) wheels.
    // NOTE: only do this here OR in the host code, not both.
    bool invertDirection = false;

    // Encoder/motor alignment found by SimpleFOC's initFOC(). When valid,
    // initFOC() skips the sensor alignment (no wheel jolt, faster start).
    // Must be cleared (`cal clear`) if the motor, magnet or encoder is
    // remounted, otherwise FOC will run with a wrong electrical angle.
    bool    calibrationValid  = false;
    int8_t  sensorDirection   = 0;     // SimpleFOC Direction: CW = 1, CCW = -1
    float   zeroElectricAngle = 0.0f;  // rad, [0, 2*pi)
};

// Settings the firmware is running with (loaded once at boot, never changed
// at runtime - CAN filters etc. are configured from these).
const BoardSettings &activeSettings();

// Working copy edited by the serial console. Written to flash by
// saveBoardSettings(); takes effect after the reboot that follows a save.
BoardSettings &pendingSettings();

// Reads flash into both copies. Returns false (and leaves defaults, i.e.
// "unconfigured") if the page is blank, corrupt or from an incompatible
// layout version.
bool loadBoardSettings();

// Writes pendingSettings() to flash and reads it back to verify. Stalls the
// CPU for tens of ms while the page is erased, so the motor must NOT be
// running when this is called. Returns true on a verified write; refuses (and
// returns false) if the firmware image has grown into the settings page.
bool saveBoardSettings();

// True if the firmware image extends into the settings flash page, i.e. the
// image is over 126 KB. Checked at boot and before every save.
bool firmwareOverlapsSettings();

// True once a motor side has been saved; CAN is only started when true.
bool boardConfigured();

// CAN identifiers derived from activeSettings().side. Only meaningful when
// boardConfigured() is true.
uint32_t thisMotorCanAddr();
uint32_t telemetryCanAddr();
// Byte offset of this motor's float in the 8-byte BOTH_MOTOR_CAN_ADDR frame.
uint8_t  broadcastFloatOffset();

const char *sideName(MotorSide side);

// Address of the flash page used for storage (for diagnostics).
uint32_t settingsFlashAddress();

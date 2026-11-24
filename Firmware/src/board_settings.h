#pragma once

#include <cstdint>

#include "motor_config.h" // CURRENT_LOOP_*_DEFAULT

// Per-board settings kept in flash, so a single firmware image can be flashed
// to both motor drivers and then configured over USB serial (see
// serial_console.h for the commands).
//
// Storage: STM32duino's EEPROM emulation, which uses the LAST 2 KB flash page
// of the STM32G431CB (0x0801F800). The firmware image must therefore stay
// below 126 KB. A DFU upload only erases the pages it writes, so the settings
// survive reflashing; a full-chip erase (STM32CubeProgrammer / ST-Link) wipes
// them and the board comes back up unconfigured.

struct BoardSettings {
    // CAN node ID, NODE_ID_MIN..NODE_ID_MAX; 0 = not set (CAN stays off).
    // Stored in the byte that held left (1) / right (2) in firmware 1.0/1.1,
    // so a board set to left comes up as node 1 and right as node 2.
    uint8_t nodeId = 0;

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

    // Current loop tuning (settings layout 2, firmware 1.4). Unlike the
    // fields above these also apply at once when changed (see
    // applyCurrentLoopSettings() in motor_control.h); saving keeps them.
    float currentP  = CURRENT_LOOP_P_DEFAULT;   // V/A
    float currentI  = CURRENT_LOOP_I_DEFAULT;   // V/(A s)
    float currentTf = CURRENT_LOOP_TF_DEFAULT;  // s

    // Anti-cogging compensation (settings layout 3, firmware 1.7): 0 off,
    // 1 on, 2 slots only (COG_MODE_* in cogging.h). Like the current loop:
    // applies at once, saving keeps it. The table itself is stored after
    // these settings, see cogging.h.
    uint8_t cogComp = 0;
};

// Range checks for the current loop values (see CURRENT_LOOP_*_MAX).
bool validCurrentP(float v);
bool validCurrentI(float v);
bool validCurrentTf(float v);

// Settings the firmware is running with (loaded once at boot, never changed
// at runtime - CAN filters etc. are configured from these).
const BoardSettings &activeSettings();

// Working copy edited by the serial console. Written to flash by
// saveBoardSettings(); takes effect after the reboot that follows a save.
BoardSettings &pendingSettings();

// Reads flash into both copies. Returns false (and leaves defaults, i.e.
// "unconfigured") if the page is blank, corrupt or from an incompatible
// layout version. Settings saved by older firmware are read too: 1.0-1.3
// (layout 1) with the current loop at its defaults, 1.4-1.6 (layout 2) with
// anti-cogging off.
bool loadBoardSettings();

// Writes pendingSettings() and the anti-cogging table to flash and reads
// them back to verify. Stalls the
// CPU for tens of ms while the page is erased, so the motor must NOT be
// running when this is called. Returns true on a verified write; refuses (and
// returns false) if the firmware image has grown into the settings page.
bool saveBoardSettings();

// True if the firmware image extends into the settings flash page, i.e. the
// image is over 126 KB. Checked at boot and before every save.
bool firmwareOverlapsSettings();

// True once a node ID has been saved; CAN is only started when true.
bool boardConfigured();

// CAN identifiers derived from activeSettings().nodeId (see can_protocol.h).
// Only meaningful when boardConfigured() is true.
uint32_t commandCanId();
uint32_t telemetryCanId();
uint32_t currentFrameCanId();
// Byte offset of this board's float in its current frame (0 or 4).
uint8_t  currentFloatOffset();

bool validNodeId(int id);

// True if the pending settings differ from the active ones, or the
// anti-cogging table has been calibrated / cleared since the last save.
bool settingsUnsaved();

// Size of the firmware image in flash, in bytes.
uint32_t firmwareImageBytes();

// Address of the flash page used for storage (for diagnostics).
uint32_t settingsFlashAddress();

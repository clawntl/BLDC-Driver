#include "serial_console.h"

#include <Arduino.h>
#include <IWatchdog.h>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <strings.h>

#include "board_config.h"
#include "board_settings.h"
#include "hw/supply.h"
#include "hw/board_info.h"
#include "can_bus.h"
#include "can_protocol.h"
#include "dfu.h"
#include "motor_control.h"

static bool settingsEqual(const BoardSettings &a, const BoardSettings &b) {
    return a.nodeId == b.nodeId
        && a.invertDirection == b.invertDirection
        && a.calibrationValid == b.calibrationValid
        && (!a.calibrationValid
            || (a.sensorDirection == b.sensorDirection && a.zeroElectricAngle == b.zeroElectricAngle))
        && a.currentP == b.currentP && a.currentI == b.currentI && a.currentTf == b.currentTf;
}

static void printSettings(const char *label, const BoardSettings &s) {
    Serial.print(label);
    Serial.print("node=");
    if (s.nodeId) Serial.print(s.nodeId); else Serial.print("not set");
    Serial.print(" invert=");
    Serial.print(s.invertDirection ? "on" : "off");
    Serial.print(" calibration=");
    if (s.calibrationValid) {
        Serial.print("dir ");
        Serial.print(s.sensorDirection);
        Serial.print(", zero angle ");
        Serial.println(s.zeroElectricAngle, 4);
    } else {
        Serial.println("none");
    }
    Serial.print("         current loop P ");
    Serial.print(s.currentP, 3);
    Serial.print(" V/A, I ");
    Serial.print(s.currentI, 1);
    Serial.print(" V/(A s), filter ");
    Serial.print(s.currentTf * 1000.0f, 3);
    Serial.println(" ms");
}

static void printCurrentLoop() {
    Serial.print("Current loop in use: P ");
    Serial.print(currentLoopP(), 3);
    Serial.print(" V/A, I ");
    Serial.print(currentLoopI(), 1);
    Serial.print(" V/(A s), filter ");
    Serial.print(currentLoopTf() * 1000.0f, 3);
    Serial.print(" ms; FOC loop ");
    const float hz = focLoopHz();
    if (hz > 0) { Serial.print(hz, 0); Serial.println(" Hz"); }
    else Serial.println("not running");
    Serial.print("Defaults: P ");
    Serial.print(CURRENT_LOOP_P_DEFAULT, 1);
    Serial.print(", I ");
    Serial.print(CURRENT_LOOP_I_DEFAULT, 0);
    Serial.print(", filter ");
    Serial.print(CURRENT_LOOP_TF_DEFAULT * 1000.0f, 2);
    Serial.println(" ms  (`set curp|curi|curtf <value>`, `cur defaults`)");
}

static const char *stateName(uint8_t state) {
    switch (state) {
        case DRIVER_STATE_IDLE:    return "idle (waiting for START_FOC)";
        case DRIVER_STATE_RUNNING: return "running";
        case DRIVER_STATE_FAULT:   return "fault (FOC start failed)";
        default:                   return "unconfigured";
    }
}

void printConfig() {
    Serial.print("Firmware: ");
    Serial.print(FIRMWARE_VERSION_MAJOR); Serial.print('.');
    Serial.print(FIRMWARE_VERSION_MINOR); Serial.print('.');
    Serial.print(FIRMWARE_VERSION_PATCH);
    Serial.print(" (CAN protocol ");
    Serial.print(CAN_PROTOCOL_VERSION);
    Serial.print("), image ");
    Serial.print(firmwareImageBytes());
    Serial.println(" bytes");
    Serial.print("Board:   ");
    Serial.print(BOARD_MODEL);
    Serial.print(" (hardware rev ");
    Serial.print(BOARD_HW_REV);
    Serial.print(", capabilities 0x");
    Serial.print(boardCapabilities(), HEX);
    Serial.println(")");
    const BoardSettings &active  = activeSettings();
    const BoardSettings &pending = pendingSettings();

    printSettings("Active:  ", active);
    if (!settingsEqual(active, pending)) {
        printSettings("Pending: ", pending);
        Serial.println("         (unsaved - type `save` to write and reboot, `revert` to discard)");
    }

    Serial.print("State:   ");
    Serial.println(stateName(driverState()));

    if (boardConfigured()) {
        Serial.print("CAN:     current frame 0x");
        Serial.print(currentFrameCanId(), HEX);
        Serial.print(" bytes ");
        Serial.print(currentFloatOffset());
        Serial.print("-");
        Serial.print(currentFloatOffset() + 3);
        Serial.print(", commands 0x");
        Serial.print(commandCanId(), HEX);
        Serial.print(", telemetry 0x");
        Serial.print(telemetryCanId(), HEX);
        if (!canStarted())    Serial.print("  [NOT STARTED]");
        else if (canBusOff()) Serial.print("  [BUS-OFF]");
        Serial.println();
    } else {
        Serial.println("CAN:     off (board not configured - `set node 1-15`, then `save`)");
    }

    const uint8_t flags = statusFlags();
    Serial.print("Supply:  ");
    Serial.print(supplyVoltageNow(), 2);
    Serial.print(" V now");
    float iin;
    if (supplyCurrentNow(iin)) {
        Serial.print(", ");
        Serial.print(iin, 3);
        Serial.print(" A drawn");
    }
    Serial.print(", ");
    Serial.print(measuredSupplyVoltage(), 2);
    Serial.print(" V at boot, ");
    Serial.print(trackedSupplyVoltage(), 2);
    Serial.print(" V used for PWM scaling (");
    Serial.print(supplySensorName());
    Serial.println(")");
    Serial.print("Motor voltage: phases 0-");
    Serial.print(phaseVoltageLimit(), 2);
    Serial.print(" V, up to ");
    Serial.print(phaseVoltageLimit() * _1_SQRT3, 2);
    Serial.println(" V on the q axis (Space Vector PWM)");
    Serial.print("Current: ");
    Serial.print(commandedCurrent(), 3);
    Serial.print(" A commanded");
    if (flags & STATUS_FLAG_COMMAND_TIMEOUT) Serial.print("  [COMMAND TIMEOUT]");
    if (flags & STATUS_FLAG_DRIVER_FAULT)    Serial.print("  [DRV8316 FAULT]");
    Serial.println();
    if (flags & STATUS_FLAG_CALIBRATION_USED) Serial.println("Align:   stored alignment used (sensor alignment skipped)");

    printCurrentLoop();

    Serial.print("Flash:   settings page at 0x");
    Serial.println(settingsFlashAddress(), HEX);
}

static void printHelp() {
    Serial.println("Commands:");
    Serial.println("  config                  show settings and state");
    Serial.println("  set node 1-15           this board's CAN node ID (sets its CAN IDs)");
    Serial.println("  set side left|right     shorthand for node 1 / node 2 (a robot's wheels)");
    Serial.println("  set invert on|off       flip command/telemetry sign");
    Serial.println("  set curp <V/A>          current loop P (default 3)");
    Serial.println("  set curi <V/(A s)>      current loop I (default 300)");
    Serial.println("  set curtf <ms>          current filter time constant (default 0.53 ms)");
    Serial.println("                          (the current loop applies at once; `save` keeps it)");
    Serial.println("  cur                     show the current loop in use and the FOC loop rate");
    Serial.println("  cur defaults            current loop back to the defaults (applies at once)");
    Serial.println("  cal store               keep this boot's FOC alignment (FOC must be running)");
    Serial.println("  cal clear               forget the stored alignment");
    Serial.println("  save                    write settings to flash and reboot (stops the motor)");
    Serial.println("  revert                  discard unsaved changes");
    Serial.println("  start                   start FOC (bench use; normally sent over CAN)");
    Serial.println("  reboot                  reset the board");
    Serial.println("  dfu                     reboot into the USB DFU bootloader");
    Serial.println("Changes are pending until `save` (the current loop also applies at once).");
}

static bool parseOnOff(const char *arg, bool &out) {
    if (!strcasecmp(arg, "on") || !strcmp(arg, "1") || !strcasecmp(arg, "true"))   { out = true;  return true; }
    if (!strcasecmp(arg, "off") || !strcmp(arg, "0") || !strcasecmp(arg, "false")) { out = false; return true; }
    return false;
}

static void cmdSet(char *args) {
    char *key = strtok(args, " ");
    char *val = strtok(nullptr, " ");
    if (!key || !val) {
        Serial.println("usage: set node 1-15  |  set side left|right  |  set invert on|off  |  set curp|curi|curtf <value>");
        return;
    }
    BoardSettings &p = pendingSettings();
    // Current loop: applied at once as well as pending.
    if (!strcasecmp(key, "curp") || !strcasecmp(key, "curi") || !strcasecmp(key, "curtf")) {
        char *end = nullptr;
        const float v = strtof(val, &end);
        if (end == val || *end) { Serial.println("not a number"); return; }
        if (!strcasecmp(key, "curp")) {
            if (!validCurrentP(v)) { Serial.print("curp must be 0-"); Serial.println(CURRENT_LOOP_P_MAX, 0); return; }
            p.currentP = v;
        } else if (!strcasecmp(key, "curi")) {
            if (!validCurrentI(v)) { Serial.print("curi must be 0-"); Serial.println(CURRENT_LOOP_I_MAX, 0); return; }
            p.currentI = v;
        } else {
            const float s = v / 1000.0f; // typed in ms
            if (!validCurrentTf(s)) { Serial.print("curtf must be 0-"); Serial.print(CURRENT_LOOP_TF_MAX * 1000.0f, 0); Serial.println(" ms"); return; }
            p.currentTf = s;
        }
        applyCurrentLoopSettings(p);
        printCurrentLoop();
        Serial.println("Applied now; type `save` to keep it (or `revert`).");
        return;
    }
    if (!strcasecmp(key, "node")) {
        const int id = atoi(val);
        if (!validNodeId(id)) { Serial.println("node must be 1-15"); return; }
        p.nodeId = (uint8_t)id;
    } else if (!strcasecmp(key, "side")) {
        if (!strcasecmp(val, "left"))       p.nodeId = 1;
        else if (!strcasecmp(val, "right")) p.nodeId = 2;
        else { Serial.println("side must be left or right"); return; }
    } else if (!strcasecmp(key, "invert")) {
        if (!parseOnOff(val, p.invertDirection)) { Serial.println("invert must be on or off"); return; }
    } else {
        Serial.print("unknown setting: ");
        Serial.println(key);
        return;
    }
    printSettings("Pending: ", p);
    Serial.println("Type `save` to write and reboot.");
}

static void cmdCal(char *args) {
    char *sub = strtok(args, " ");
    BoardSettings &p = pendingSettings();
    if (sub && !strcasecmp(sub, "store")) {
        if (!motorCalibrated()) {
            Serial.println("No valid alignment yet - start FOC first (`start`, or CMD_START_FOC over CAN).");
            return;
        }
        p.calibrationValid  = true;
        p.sensorDirection   = static_cast<int8_t>(motor.sensor_direction);
        p.zeroElectricAngle = motor.zero_electric_angle;
    } else if (sub && !strcasecmp(sub, "clear")) {
        p.calibrationValid  = false;
        p.sensorDirection   = 0;
        p.zeroElectricAngle = 0;
    } else {
        Serial.println("usage: cal store | cal clear");
        return;
    }
    printSettings("Pending: ", p);
    Serial.println("Type `save` to write and reboot.");
}

static void cmdSave() {
    if (focRunning()) {
        Serial.println("Stopping motor for the flash write...");
        stopMotor();
    }
    // Erase + program takes tens of ms; make sure the watchdog (if running)
    // has a full period available.
    if (IWatchdog.isEnabled()) IWatchdog.reload();

    const bool ok = saveBoardSettings();
    Serial.println(ok ? "Settings saved and verified. Rebooting..." : "ERROR: flash verify failed! Rebooting...");
    softwareReset();
}

static void handleLine(char *line) {
    // Split off the command word; the rest is passed on as arguments.
    char *args = strchr(line, ' ');
    if (args) {
        *args++ = '\0';
        while (*args == ' ') args++;
    } else {
        args = line + strlen(line); // empty string
    }

    if (!strcasecmp(line, "help") || !strcmp(line, "?")) printHelp();
    else if (!strcasecmp(line, "config") || !strcasecmp(line, "status")) printConfig();
    else if (!strcasecmp(line, "set"))    cmdSet(args);
    else if (!strcasecmp(line, "cal"))    cmdCal(args);
    else if (!strcasecmp(line, "save"))   cmdSave();
    else if (!strcasecmp(line, "revert")) {
        pendingSettings() = activeSettings();
        applyCurrentLoopSettings(activeSettings());
        Serial.println("Pending changes discarded.");
    }
    else if (!strcasecmp(line, "cur")) {
        if (!strcasecmp(args, "defaults")) {
            BoardSettings &p = pendingSettings();
            p.currentP  = CURRENT_LOOP_P_DEFAULT;
            p.currentI  = CURRENT_LOOP_I_DEFAULT;
            p.currentTf = CURRENT_LOOP_TF_DEFAULT;
            applyCurrentLoopSettings(p);
            Serial.println("Current loop back to the defaults (applied; `save` to keep).");
        } else if (*args) {
            Serial.println("usage: cur  |  cur defaults");
            return;
        }
        printCurrentLoop();
    }
    else if (!strcasecmp(line, "start"))  requestStartFOC();
    else if (!strcasecmp(line, "reboot")) { Serial.println("Rebooting..."); softwareReset(); }
    else if (!strcasecmp(line, "dfu"))    { Serial.println("Entering DFU mode..."); Serial.flush(); rebootToDFU(); }
    else {
        Serial.print("Unknown command: ");
        Serial.print(line);
        Serial.println("  (type `help`)");
    }
}

// Bytes are accumulated one at a time (Serial.readStringUntil() would stall
// the FOC loop for up to its timeout on a partial line).
void pollSerialConsole() {
    static char buf[48];
    static uint8_t len = 0;
    static bool overflow = false;

    while (Serial.available()) {
        const char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (overflow) {
                Serial.println("Line too long - ignored");
            } else if (len > 0) {
                buf[len] = '\0';
                // trim trailing spaces
                while (len > 0 && buf[len - 1] == ' ') buf[--len] = '\0';
                char *start = buf;
                while (*start == ' ') start++;
                if (*start) handleLine(start);
            }
            len = 0;
            overflow = false;
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = c;
        } else {
            overflow = true;
        }
    }
}

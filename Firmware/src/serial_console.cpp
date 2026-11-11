#include "serial_console.h"

#include <Arduino.h>
#include <IWatchdog.h>
#include <cstring>
#include <strings.h>

#include "board_settings.h"
#include "can_bus.h"
#include "can_protocol.h"
#include "dfu.h"
#include "motor_control.h"

static bool settingsEqual(const BoardSettings &a, const BoardSettings &b) {
    return a.side == b.side
        && a.invertDirection == b.invertDirection
        && a.calibrationValid == b.calibrationValid
        && (!a.calibrationValid
            || (a.sensorDirection == b.sensorDirection && a.zeroElectricAngle == b.zeroElectricAngle));
}

static void printSettings(const char *label, const BoardSettings &s) {
    Serial.print(label);
    Serial.print("side=");
    Serial.print(sideName(s.side));
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
        Serial.print("CAN:     commands 0x");
        Serial.print(thisMotorCanAddr(), HEX);
        Serial.print(", telemetry 0x");
        Serial.print(telemetryCanAddr(), HEX);
        Serial.print(", broadcast bytes ");
        Serial.print(broadcastFloatOffset());
        Serial.print("-");
        Serial.print(broadcastFloatOffset() + 3);
        if (!canStarted())    Serial.print("  [NOT STARTED]");
        else if (canBusOff()) Serial.print("  [BUS-OFF]");
        Serial.println();
    } else {
        Serial.println("CAN:     off (board not configured - `set side left|right`, then `save`)");
    }

    const uint8_t flags = statusFlags();
    Serial.print("Supply:  ");
    Serial.print(measuredSupplyVoltage(), 2);
    Serial.println(" V (measured at boot)");
    Serial.print("Current: ");
    Serial.print(commandedCurrent(), 3);
    Serial.print(" A commanded");
    if (flags & STATUS_FLAG_COMMAND_TIMEOUT) Serial.print("  [COMMAND TIMEOUT]");
    if (flags & STATUS_FLAG_DRIVER_FAULT)    Serial.print("  [DRV8316 FAULT]");
    Serial.println();
    if (flags & STATUS_FLAG_CALIBRATION_USED) Serial.println("Align:   stored alignment used (sensor alignment skipped)");

    Serial.print("Flash:   settings page at 0x");
    Serial.println(settingsFlashAddress(), HEX);
}

static void printHelp() {
    Serial.println("Commands:");
    Serial.println("  config                  show settings and state");
    Serial.println("  set side left|right     which motor this board drives (sets CAN IDs)");
    Serial.println("  set invert on|off       flip command/telemetry sign");
    Serial.println("  cal store               keep this boot's FOC alignment (FOC must be running)");
    Serial.println("  cal clear               forget the stored alignment");
    Serial.println("  save                    write settings to flash and reboot (stops the motor)");
    Serial.println("  revert                  discard unsaved changes");
    Serial.println("  start                   start FOC (bench use; normally sent over CAN)");
    Serial.println("  reboot                  reset the board");
    Serial.println("  dfu                     reboot into the USB DFU bootloader");
    Serial.println("Changes are pending until `save`.");
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
        Serial.println("usage: set side left|right  |  set invert on|off");
        return;
    }
    BoardSettings &p = pendingSettings();
    if (!strcasecmp(key, "side")) {
        if (!strcasecmp(val, "left"))       p.side = MotorSide::Left;
        else if (!strcasecmp(val, "right")) p.side = MotorSide::Right;
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
    else if (!strcasecmp(line, "revert")) { pendingSettings() = activeSettings(); Serial.println("Pending changes discarded."); }
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

#include "can_bus.h"

#include <Arduino.h>

#include <ACANFD_STM32.h>          // -> fdcan1 object, pins PB8(RX)/PB9(TX)
#include <ACANFD_STM32_Settings.h> // -> ACANFD_STM32_Settings, DataBitRateFactor

#include "board_config.h"
#include "can_protocol.h"
#include "motor_control.h"
#include "motor_config.h"
#include "board_settings.h"
#include "hw/supply.h"
#include "hw/board_info.h"
#include "dfu.h"
#include "cogging.h"

#include <IWatchdog.h>
#include <cmath>

static void float2bytesMSB(float val, uint8_t *bytes) {
    // Copy the raw bits of val into an integer so can perform
    // bitwise operations
    uint32_t valAsInt;
    memcpy(&valAsInt, &val, sizeof(float));

    // Extract the bytes in big-endian (MSB-first) order
    bytes[0] = (valAsInt >> 24) & 0xFF;
    bytes[1] = (valAsInt >> 16) & 0xFF;
    bytes[2] = (valAsInt >> 8)  & 0xFF;
    bytes[3] = (valAsInt >> 0)  & 0xFF;
}

static float bytes2floatMSB(const uint8_t *bytes) {
    // Read the bytes in big-endian (MSB-first) order
    uint32_t valAsInt =
        ((uint32_t)bytes[0] << 24) |
        ((uint32_t)bytes[1] << 16) |
        ((uint32_t)bytes[2] << 8)  |
        ((uint32_t)bytes[3]);

    // Copy the raw bits to a float and return
    float val;
    memcpy(&val, &valAsInt, sizeof(float));

    return val;
}

static bool can_started = false;

// Rate-limited reporting of transmit errors: with the bus down, every
// telemetry frame fails, and printing each one floods USB serial and slows
// the control loop.
static uint32_t txErrorCount = 0;
static uint32_t lastTxErrorStatus = 0;
static uint32_t lastTxErrorReportMs = 0;

static uint32_t lastStatusSendMs = 0;
static uint32_t lastBusCheckMs = 0;
static bool busOffReported = false;

// Accept only this board's current frame (shared with one other node) and
// its own command ID.
void configureAndStartCAN() {
    // DataBitRateFactor::x1 -> data phase == arbitration phase (we never set BRS,
    // so only the arbitration-phase sample point below actually matters).
    ACANFD_STM32_Settings settings(CAN_BIT_RATE, CAN_SAMPLE_POINT, DataBitRateFactor::x1, CAN_SAMPLE_POINT);
    settings.mModuleMode = ACANFD_STM32_Settings::NORMAL_FD;
    settings.mNonMatchingStandardFrameReception = ACANFD_STM32_FilterAction::REJECT;

    if (!settings.mBitSettingOk) {
        Serial.println("CAN bit rate/sample point could not be matched within tolerance!");
    }

    ACANFD_STM32_StandardFilters stdFilters;
    stdFilters.addDual(currentFrameCanId(), commandCanId(), ACANFD_STM32_FilterAction::FIFO0);

    const uint32_t errorCode = fdcan1.beginFD(settings, stdFilters);

    Serial.print("CAN bit rate: ");
    Serial.print(settings.actualArbitrationBitRate());
    Serial.print(" bit/s, sample point: ");
    Serial.print(settings.arbitrationSamplePointFromBitStart());
    Serial.println(" %");
    Serial.print("CAN node ");
    Serial.print(activeSettings().nodeId);
    Serial.print(": current frame 0x");
    Serial.print(currentFrameCanId(), HEX);
    Serial.print(", commands 0x");
    Serial.print(commandCanId(), HEX);
    Serial.print(", telemetry 0x");
    Serial.println(telemetryCanId(), HEX);

    if (errorCode != 0) {
        Serial.print("fdcan1 configuration error: 0x");
        Serial.println(errorCode, HEX);
        return;
    }
    can_started = true;
}

bool canStarted() { return can_started; }

static void sendFrame(const uint8_t *data, uint8_t len, uint32_t canID) {
    if (!can_started) return;

    CANFDMessage txMsg;
    txMsg.id   = canID;
    txMsg.ext  = false;
    txMsg.type = CANFDMessage::CAN_DATA; // classic (non-FD) data frame
    txMsg.len  = len;
    memcpy(txMsg.data, data, len);

    const uint32_t sendStatus = fdcan1.tryToSendReturnStatusFD(txMsg);
    if (sendStatus != 0) {
        txErrorCount++;
        lastTxErrorStatus = sendStatus;
    }
}

void sendFloat(uint8_t cmd, float dat, uint32_t canID) {
    // cmd byte + 4 big-endian float bytes
    uint8_t frame[5];
    frame[0] = cmd;
    float2bytesMSB(dat, &frame[1]);
    sendFrame(frame, sizeof(frame), canID);
}

void sendVelocityTelemetry(float radPerS, float iqAmps, float idAmps) {
    uint8_t frame[8];
    frame[0] = CMD_SHAFT_VELOCITY;
    float2bytesMSB(radPerS, &frame[1]);
    const float ma = std::isfinite(iqAmps) ? iqAmps * 1000.0f : 0.0f;
    const int16_t q = (int16_t)constrain(lroundf(ma), -32767L, 32767L);
    frame[5] = (uint8_t)((uint16_t)q >> 8);
    frame[6] = (uint8_t)((uint16_t)q & 0xFF);
    const float dc = std::isfinite(idAmps) ? idAmps * 1000.0f / D_CURRENT_LSB_MA : 0.0f;
    frame[7] = (uint8_t)(int8_t)constrain(lroundf(dc), -127L, 127L);
    sendFrame(frame, sizeof(frame), telemetryCanId());
}

void sendError(uint8_t errorCode) {
    sendFloat(CMD_ERROR, (float)errorCode, telemetryCanId());
}

void sendStartResult(bool ok) {
    sendFloat(CMD_START_FOC, ok ? 1.0f : 0.0f, telemetryCanId());
}

static void sendStatus() {
    // Bytes 3-4: supply voltage in mV, big-endian (0 = no reading yet).
    // Bytes 5-6 (CAP_MEASURES_INPUT_CURRENT only): input current in mA, int16 big-endian.
    const uint16_t mv = supplyMillivolts();
    const uint16_t ma = (uint16_t)supplyMilliamps();
    const uint8_t frame[7] = { CMD_DRIVER_STATUS, driverState(), statusFlags(),
                               (uint8_t)(mv >> 8), (uint8_t)(mv & 0xFF),
                               (uint8_t)(ma >> 8), (uint8_t)(ma & 0xFF) };
    const uint8_t len = (boardCapabilities() & CAP_MEASURES_INPUT_CURRENT) ? 7 : 5;
    sendFrame(frame, len, telemetryCanId());
}

// Work that has to wait until the reply has gone out (it stops the CPU or
// resets it): run from pollCAN() a few ms after the command.
enum class DeferredAction : uint8_t { None, SaveAndReboot, Reboot };
static DeferredAction deferredAction = DeferredAction::None;
static uint32_t deferredAtMs = 0;

static void deferAction(DeferredAction a) {
    deferredAction = a;
    deferredAtMs = millis();
}

static void replyStatus(uint8_t cmd, uint8_t status) {
    const uint8_t frame[2] = { cmd, status };
    sendFrame(frame, sizeof(frame), telemetryCanId());
}

static void replyGetInfo() {
    const uint32_t kb = (firmwareImageBytes() + 1023) / 1024;
    const uint8_t frame[8] = { CMD_GET_INFO, CAN_PROTOCOL_VERSION,
                               FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR, FIRMWARE_VERSION_PATCH,
                               (uint8_t)min<uint32_t>(kb, 255), 0, 0 };
    sendFrame(frame, sizeof(frame), telemetryCanId());
}

static void replyGetBoardInfo(uint8_t page) {
    uint8_t frame[8] = { CMD_GET_BOARD_INFO, page, 0, 0, 0, 0, 0, 0 };
    if (page == 0) {
        const uint32_t caps = boardCapabilities();
        frame[2] = (uint8_t)(caps >> 24);
        frame[3] = (uint8_t)(caps >> 16);
        frame[4] = (uint8_t)(caps >> 8);
        frame[5] = (uint8_t)caps;
        frame[6] = BOARD_HW_REV;   // frame[7] reserved, 0
    } else if (page <= 5) {
        // Characters (page-1)*6 .. +5 of the model name, NUL-padded.
        const char *model = boardModel();
        const size_t len = strlen(model);
        for (size_t i = 0; i < 6; i++) {
            const size_t at = (size_t)(page - 1) * 6 + i;
            frame[2 + i] = at < len ? (uint8_t)model[at] : 0;
        }
    } else if (page <= 7) {
        // MCU unique ID (serial number), 6 bytes per page.
        uint8_t uid[12];
        boardSerialBytes(uid);
        memcpy(&frame[2], &uid[(page - 6) * 6], 6);
    } else {
        const uint8_t nack[3] = { CMD_NACK, CMD_GET_BOARD_INFO, CONFIG_ERR_BAD_VALUE };
        sendFrame(nack, sizeof(nack), telemetryCanId());
        return;
    }
    sendFrame(frame, sizeof(frame), telemetryCanId());
}

static void replyGetConfig(uint8_t which) {
    const BoardSettings &s = which ? pendingSettings() : activeSettings();
    uint8_t flags = 0;
    if (s.invertDirection)                         flags |= CONFIG_FLAG_INVERT;
    if (s.calibrationValid)                        flags |= CONFIG_FLAG_CALIBRATION;
    if (s.calibrationValid && s.sensorDirection < 0) flags |= CONFIG_FLAG_SENSOR_DIR_CCW;
    if (settingsUnsaved())                         flags |= CONFIG_FLAG_UNSAVED;
    uint8_t frame[8] = { CMD_GET_CONFIG, which, s.nodeId, flags };
    float2bytesMSB(s.calibrationValid ? s.zeroElectricAngle : 0.0f, &frame[4]);
    sendFrame(frame, sizeof(frame), telemetryCanId());
}

static void replySetParam(uint8_t param, uint8_t status, float now) {
    uint8_t frame[7] = { CMD_SET_PARAM, param, status };
    float2bytesMSB(now, &frame[3]);
    sendFrame(frame, sizeof(frame), telemetryCanId());
}

static void handleSetParam(const CANFDMessage &rxMsg) {
    if (rxMsg.len < 6) { replySetParam(rxMsg.len > 1 ? rxMsg.data[1] : 0, CONFIG_ERR_BAD_VALUE, 0); return; }
    const uint8_t param = rxMsg.data[1];
    const float v = bytes2floatMSB(rxMsg.data + 2);
    BoardSettings &p = pendingSettings();
    if (!std::isfinite(v)) { replySetParam(param, CONFIG_ERR_BAD_VALUE, 0); return; }
    switch (param) {
        case PARAM_NODE_ID: {
            const int id = (int)lroundf(v);
            if (!validNodeId(id)) {
                replySetParam(param, CONFIG_ERR_BAD_VALUE, (float)p.nodeId);
                return;
            }
            p.nodeId = (uint8_t)id;
            Serial.print("CAN: pending node ID = ");
            Serial.println(id);
            replySetParam(param, CONFIG_OK, (float)id);
            return;
        }
        case PARAM_INVERT:
            p.invertDirection = v > 0.5f;
            Serial.print("CAN: pending invert = ");
            Serial.println(p.invertDirection ? "on" : "off");
            replySetParam(param, CONFIG_OK, p.invertDirection ? 1.0f : 0.0f);
            return;
        case PARAM_VELOCITY_INTERVAL_MS: {
            const int ms = (int)lroundf(v);
            if (ms < 1 || ms > 100) {
                replySetParam(param, CONFIG_ERR_BAD_VALUE, (float)velocityTelemetryIntervalMs());
                return;
            }
            setVelocityTelemetryIntervalMs((uint8_t)ms);
            replySetParam(param, CONFIG_OK, (float)ms);
            return;
        }
        // Current loop: pending (kept by CMD_SAVE) and applied at once.
        case PARAM_CURRENT_P:
        case PARAM_CURRENT_I:
        case PARAM_CURRENT_TF: {
            float &field = param == PARAM_CURRENT_P ? p.currentP
                         : param == PARAM_CURRENT_I ? p.currentI : p.currentTf;
            const bool ok = param == PARAM_CURRENT_P ? validCurrentP(v)
                          : param == PARAM_CURRENT_I ? validCurrentI(v) : validCurrentTf(v);
            if (!ok) {
                replySetParam(param, CONFIG_ERR_BAD_VALUE, field);
                return;
            }
            field = v;
            applyCurrentLoopSettings(p);
            Serial.print("CAN: current loop P ");
            Serial.print(p.currentP, 3);
            Serial.print(", I ");
            Serial.print(p.currentI, 1);
            Serial.print(", filter ");
            Serial.print(p.currentTf * 1000.0f, 3);
            Serial.println(" ms (applied, unsaved)");
            replySetParam(param, CONFIG_OK, field);
            return;
        }
        // Anti-cogging on/off: pending (kept by CMD_SAVE) and applied at once.
        case PARAM_COG_COMP: {
            const long m = lroundf(v);
            if (m < 0 || m > 2) { replySetParam(param, CONFIG_ERR_BAD_VALUE, p.cogComp); return; }
            p.cogComp = (uint8_t)m;
            Serial.print("CAN: anti-cogging compensation mode ");
            Serial.print(p.cogComp);
            Serial.println(" (applied, unsaved)");
            replySetParam(param, CONFIG_OK, p.cogComp);
            return;
        }
        default:
            replySetParam(param, CONFIG_ERR_UNKNOWN, 0);
            return;
    }
}

static void handleGetParam(const CANFDMessage &rxMsg) {
    const uint8_t param = rxMsg.len >= 2 ? rxMsg.data[1] : 0;
    const BoardSettings &p = pendingSettings();
    uint8_t status = CONFIG_OK;
    float v = 0;
    switch (param) {
        case PARAM_NODE_ID:              v = p.nodeId; break;
        case PARAM_INVERT:               v = p.invertDirection ? 1.0f : 0.0f; break;
        case PARAM_VELOCITY_INTERVAL_MS: v = velocityTelemetryIntervalMs(); break;
        case PARAM_CURRENT_P:            v = currentLoopP(); break;
        case PARAM_CURRENT_I:            v = currentLoopI(); break;
        case PARAM_CURRENT_TF:           v = currentLoopTf(); break;
        case PARAM_FOC_LOOP_HZ:          v = focLoopHz(); break;
        case PARAM_COG_COMP:             v = p.cogComp; break;
        default:                         status = CONFIG_ERR_UNKNOWN; break;
    }
    uint8_t frame[7] = { CMD_GET_PARAM, param, status };
    float2bytesMSB(v, &frame[3]);
    sendFrame(frame, sizeof(frame), telemetryCanId());
}

static void handleCalibration(const CANFDMessage &rxMsg) {
    const uint8_t op = rxMsg.len >= 2 ? rxMsg.data[1] : 0;
    BoardSettings &p = pendingSettings();
    uint8_t status = CONFIG_OK;
    if (op == CAL_STORE) {
        if (!motorCalibrated()) {
            status = CONFIG_ERR_NOT_ALIGNED;
        } else {
            p.calibrationValid  = true;
            p.sensorDirection   = static_cast<int8_t>(motor.sensor_direction);
            p.zeroElectricAngle = motor.zero_electric_angle;
            Serial.println("CAN: pending calibration = this boot's alignment");
        }
    } else if (op == CAL_CLEAR) {
        p.calibrationValid  = false;
        p.sensorDirection   = 0;
        p.zeroElectricAngle = 0;
        Serial.println("CAN: pending calibration cleared");
    } else {
        status = CONFIG_ERR_UNKNOWN;
    }
    const uint8_t frame[3] = { CMD_CALIBRATION, op, status };
    sendFrame(frame, sizeof(frame), telemetryCanId());
}

static uint8_t cogFlags() {
    uint8_t f = 0;
    if (Cogging::tableValid())   f |= COG_FLAG_TABLE;
    if (pendingSettings().cogComp) f |= COG_FLAG_ENABLED;
    if (Cogging::tableUnsaved()) f |= COG_FLAG_UNSAVED;
    if (Cogging::tableValid() && motorCalibrated() && !Cogging::tableMatchesMotor()) f |= COG_FLAG_MISMATCH;
    if (focRunning() && pendingSettings().cogComp && Cogging::tableMatchesMotor() && !Cogging::calibrating())
        f |= COG_FLAG_APPLIED;
    if (Cogging::tableRefined()) f |= COG_FLAG_REFINED;
    if (pendingSettings().cogComp == COG_MODE_SLOTS) f |= COG_FLAG_SLOTS_ONLY;
    return f;
}

static uint8_t clampUnits(float v, float unit) {
    if (!(v > 0)) return 0;
    return (uint8_t)min(255L, lroundf(v / unit));
}

static void handleCogging(const CANFDMessage &rxMsg) {
    const uint8_t op = rxMsg.len >= 2 ? rxMsg.data[1] : COG_STATUS;
    uint8_t status = CONFIG_OK;
    switch (op) {
        case COG_STATUS:
        case COG_RESULT:
            break;
        case COG_READ: {
            // Four table counts per frame; bin hi 0xFF = no table / bad bin.
            const int bin = rxMsg.len >= 4 ? (rxMsg.data[2] << 8) | rxMsg.data[3] : -1;
            uint8_t f[8] = { CMD_COGGING, COG_READ, 0xFF, 0xFF, 0, 0, 0, 0 };
            if (Cogging::tableValid() && bin >= 0 && bin < COG_BINS) {
                f[2] = (uint8_t)(bin >> 8);
                f[3] = (uint8_t)bin;
                for (int i = 0; i < 4; i++) f[4 + i] = (uint8_t)Cogging::tableCount(bin + i);
            }
            sendFrame(f, sizeof(f), telemetryCanId());
            return;
        }
        case COG_INFO: {
            uint8_t f[8] = { CMD_COGGING, COG_INFO, Cogging::tableValid() ? (uint8_t)CONFIG_OK : (uint8_t)CONFIG_ERR_BAD_VALUE };
            float2bytesMSB(Cogging::tableScale(), &f[3]);
            f[7] = COG_BINS / 16;
            sendFrame(f, sizeof(f), telemetryCanId());
            return;
        }
        case COG_CALIBRATE:
        case COG_REFINE:
            if (!focRunning() || !motorCalibrated()) status = CONFIG_ERR_NOT_RUNNING;
            else if (op == COG_REFINE && !Cogging::tableMatchesMotor()) status = CONFIG_ERR_BAD_VALUE;
            else if (!Cogging::startCalibration(op == COG_REFINE)) status = CONFIG_ERR_NOT_RUNNING;
            break;
        case COG_ABORT:
            Cogging::abort();
            break;
        case COG_CLEAR:
            Cogging::clear();
            Serial.println("CAN: anti-cogging table cleared (unsaved)");
            break;
        default: {
            const uint8_t nack[3] = { CMD_NACK, CMD_COGGING, CONFIG_ERR_UNKNOWN };
            sendFrame(nack, sizeof(nack), telemetryCanId());
            return;
        }
    }
    uint8_t frame[8] = { CMD_COGGING, op, status, 0, 0, 0, 0, 0 };
    if (op == COG_RESULT) {
        frame[3] = Cogging::tableCoverage();
        frame[4] = Cogging::tableQuality();
        frame[5] = clampUnits(Cogging::lastMaxTrackingError(), 0.0001f);
        frame[6] = clampUnits(Cogging::lastFilteredNoise(), 0.0005f);
    } else {
        const Cogging::State st = Cogging::state();
        frame[3] = (uint8_t)st;
        frame[4] = st == Cogging::State::Failed ? (uint8_t)Cogging::failure() : Cogging::progress();
        frame[5] = cogFlags();
        frame[6] = clampUnits(Cogging::tableAmplitude(), 0.004f);
        frame[7] = clampUnits(Cogging::tableFriction(), 0.002f);
    }
    sendFrame(frame, sizeof(frame), telemetryCanId());
}

static void handleCommandMessage(const CANFDMessage &rxMsg) {
    if (rxMsg.len < 1) return;

    switch (rxMsg.data[0]) {
        case CMD_START_FOC: {
            Serial.println("Received CMD_START_FOC");
            requestStartFOC();
            break;
        }
        case CMD_GET_INFO:
            replyGetInfo();
            break;
        case CMD_GET_BOARD_INFO:
            replyGetBoardInfo(rxMsg.len >= 2 ? rxMsg.data[1] : 0);
            break;
        case CMD_GET_CONFIG:
            replyGetConfig(rxMsg.len >= 2 && rxMsg.data[1] ? 1 : 0);
            break;
        case CMD_SET_PARAM:
            handleSetParam(rxMsg);
            break;
        case CMD_GET_PARAM:
            handleGetParam(rxMsg);
            break;
        case CMD_CALIBRATION:
            handleCalibration(rxMsg);
            break;
        case CMD_COGGING:
            handleCogging(rxMsg);
            break;
        case CMD_SAVE:
            if (firmwareOverlapsSettings()) {
                replyStatus(CMD_SAVE, CONFIG_ERR_FLASH);
            } else {
                Serial.println("CAN: save requested - stopping the motor, saving and rebooting");
                replyStatus(CMD_SAVE, CONFIG_OK);
                deferAction(DeferredAction::SaveAndReboot);
            }
            break;
        case CMD_REVERT:
            pendingSettings() = activeSettings();
            applyCurrentLoopSettings(activeSettings());
            Cogging::revert();
            Serial.println("CAN: pending changes discarded");
            replyStatus(CMD_REVERT, CONFIG_OK);
            break;
        case CMD_REBOOT:
            Serial.println("CAN: reboot requested");
            replyStatus(CMD_REBOOT, CONFIG_OK);
            deferAction(DeferredAction::Reboot);
            break;
        default: {
            // CMD_SHAFT_VELOCITY / CMD_DRIVER_STATUS / CMD_ERROR are only ever
            // sent by the drivers; anything else is a command we don't know.
            if (rxMsg.data[0] != CMD_SHAFT_VELOCITY && rxMsg.data[0] != CMD_DRIVER_STATUS
                && rxMsg.data[0] != CMD_ERROR) {
                const uint8_t frame[3] = { CMD_NACK, rxMsg.data[0], CONFIG_ERR_UNKNOWN };
                sendFrame(frame, sizeof(frame), telemetryCanId());
            }
            break;
        }
    }
}

// Runs a deferred save/reboot once the reply has had time to go out (a frame
// takes ~0.1 ms at 1 Mbit/s; 20 ms covers a queue of telemetry ahead of it).
static void runDeferredAction() {
    if (deferredAction == DeferredAction::None) return;
    if (millis() - deferredAtMs < 20) return;
    const DeferredAction a = deferredAction;
    deferredAction = DeferredAction::None;
    if (focRunning()) stopMotor();
    if (a == DeferredAction::SaveAndReboot) {
        if (IWatchdog.isEnabled()) IWatchdog.reload();
        const bool ok = saveBoardSettings();
        Serial.println(ok ? "Settings saved and verified. Rebooting..." : "ERROR: flash verify failed! Rebooting...");
    } else {
        Serial.println("Rebooting...");
    }
    softwareReset();
}

static void handleFrame(const CANFDMessage &rxMsg) {
    // Only standard (non-extended) ID messages are used by this protocol.
    if (rxMsg.ext) return;

    if (rxMsg.id == currentFrameCanId()) {
        // Exactly two floats expected; anything shorter would leave stale
        // bytes in rxMsg.data.
        if (rxMsg.len != 8) return;
        // NaN in our slot = no command for this node (setTargetCurrent ignores
        // it, so it doesn't count as a command for the timeout either).
        setTargetCurrent(bytes2floatMSB(rxMsg.data + currentFloatOffset()));
    } else if (rxMsg.id == commandCanId()) {
        handleCommandMessage(rxMsg);
    }
}

// FDCAN goes bus-off after too many transmit errors (e.g. no other node
// acking, wiring fault) and sets CCCR.INIT, which stops it until software
// clears INIT; the peripheral then waits for 128 x 11 recessive bits and
// rejoins the bus. The ACANFD library doesn't do this itself.
static void checkBusOff() {
    const uint32_t now = millis();
    if (now - lastBusCheckMs < 100) return;
    lastBusCheckMs = now;

    if (fdcan1.getStatus().isBusOff()) {
        if (!busOffReported) {
            Serial.println("CAN bus-off - attempting recovery");
            busOffReported = true;
        }
        FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
    } else if (busOffReported) {
        Serial.println("CAN bus recovered");
        busOffReported = false;
    }
}

static void reportTxErrors() {
    if (txErrorCount == 0) return;
    const uint32_t now = millis();
    if (now - lastTxErrorReportMs < 1000) return;
    lastTxErrorReportMs = now;

    Serial.print("CAN send errors in the last second: ");
    Serial.print(txErrorCount);
    Serial.print(" (last status 0x");
    Serial.print(lastTxErrorStatus, HEX);
    Serial.println(")");
    txErrorCount = 0;
}

void pollCAN() {
    if (!can_started) return;

    // Drain everything that has arrived, so the newest current command is
    // applied straight away instead of one queued frame per loop.
    CANFDMessage rxMsg;
    while (fdcan1.receiveFD0(rxMsg)) {
        handleFrame(rxMsg);
    }

    const uint32_t now = millis();
    if (now - lastStatusSendMs >= CAN_STATUS_INTERVAL_MS) {
        lastStatusSendMs = now;
        sendStatus();
    }

    checkBusOff();
    reportTxErrors();
    runDeferredAction();
}

void discardPendingCAN() {
    if (!can_started) return;
    CANFDMessage rxMsg;
    uint32_t discarded = 0;
    while (fdcan1.receiveFD0(rxMsg)) discarded++;
    if (discarded > 0) {
        Serial.print("Discarded ");
        Serial.print(discarded);
        Serial.println(" CAN frame(s) received during FOC start");
    }
}

bool canBusOff() {
    return can_started && fdcan1.getStatus().isBusOff();
}

#include "can_bus.h"

#include <Arduino.h>

#include <ACANFD_STM32.h>          // -> fdcan1 object, pins PB8(RX)/PB9(TX)
#include <ACANFD_STM32_Settings.h> // -> ACANFD_STM32_Settings, DataBitRateFactor

#include "can_protocol.h"
#include "motor_control.h"
#include "motor_config.h"
#include "board_settings.h"

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

// Accept only frames addressed to both motors (broadcast) or to this motor.
// Equivalent to the old SimpleCAN mask filter (BOTH_MOTOR_CAN_ADDR / CAN_MASK),
// but expressed directly as the two identifiers it actually let through.
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
    stdFilters.addDual(BOTH_MOTOR_CAN_ADDR, thisMotorCanAddr(), ACANFD_STM32_FilterAction::FIFO0);

    const uint32_t errorCode = fdcan1.beginFD(settings, stdFilters);

    Serial.print("CAN bit rate: ");
    Serial.print(settings.actualArbitrationBitRate());
    Serial.print(" bit/s, sample point: ");
    Serial.print(settings.arbitrationSamplePointFromBitStart());
    Serial.println(" %");
    Serial.print("CAN IDs: commands 0x");
    Serial.print(thisMotorCanAddr(), HEX);
    Serial.print(", telemetry 0x");
    Serial.println(telemetryCanAddr(), HEX);

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

void sendError(uint8_t errorCode) {
    sendFloat(CMD_ERROR, (float)errorCode, telemetryCanAddr());
}

void sendStartResult(bool ok) {
    sendFloat(CMD_START_FOC, ok ? 1.0f : 0.0f, telemetryCanAddr());
}

static void sendStatus() {
    const uint8_t frame[5] = { CMD_DRIVER_STATUS, driverState(), statusFlags(), 0, 0 };
    sendFrame(frame, sizeof(frame), telemetryCanAddr());
}

static void handleCommandMessage(const CANFDMessage &rxMsg) {
    if (rxMsg.len < 1) return;

    switch (rxMsg.data[0]) {
        case CMD_START_FOC: {
            Serial.println("Received CMD_START_FOC");
            requestStartFOC();
            break;
        }
        default: {
            // CMD_SHAFT_VELOCITY / CMD_DRIVER_STATUS / CMD_ERROR are only ever sent
            // by the drivers, so there's nothing to do if one is received.
            break;
        }
    }
}

static void handleFrame(const CANFDMessage &rxMsg) {
    // Only standard (non-extended) ID messages are used by this protocol.
    if (rxMsg.ext) return;

    if (rxMsg.id == BOTH_MOTOR_CAN_ADDR) {
        // Exactly two floats expected; anything shorter would leave stale
        // bytes in rxMsg.data.
        if (rxMsg.len != 8) return;
        setTargetCurrent(bytes2floatMSB(rxMsg.data + broadcastFloatOffset()));
    } else if (rxMsg.id == thisMotorCanAddr()) {
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

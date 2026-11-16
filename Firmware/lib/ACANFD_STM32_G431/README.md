## ACANFD_STM32_G431

A CAN FD driver for a single custom **STM32G431CBU6** board, using **FDCAN1** on pins **PB8 (RX)** / **PB9 (TX)**.

This is a stripped-down fork of Pierre Molinaro's [ACANFD-STM32](https://github.com/pierremolinaro/acanfd-stm32) library. All other STM32 board variants (Nucleo-G431KB, Nucleo-G474RE, Nucleo-H743ZI2, etc.), their pin tables, and the programmable-RAM-section code paths used by other chips have been removed. Everything else — the driver core, bit timing calculator, message classes, filters, FIFOs — is unmodified from upstream.

### Why this fork exists

Upstream ACANFD-STM32 supports many boards and lets you pick pins/peripheral at construction time from board-specific pin arrays. This project only ever targets one board, so the fork hardcodes:

* the peripheral (`FDCAN1`), its RAM base address, and its two interrupts (`FDCAN1_IT0_IRQn` / `FDCAN1_IT1_IRQn`),
* the pin mapping (`PB_9` AF9 for TX, `PB_8` AF9 for RX) — matching the `HAL_FDCAN_MspInit` generated for this board in `Core/Src/stm32g4xx_hal_msp.c`,
* the FDCAN kernel clock source (`RCC_FDCANCLKSOURCE_PCLK1`, i.e. PCLK1) to match this board's `SystemClock_Config`,
* `HAS_PROGRAMMABLE_FDCAN_RAM_SECTIONS = false`, since the G4 FDCAN peripheral has fixed-size message RAM sections (unlike H7).

Bit rate, sample point, filters, FIFO sizes, and mode are all still fully configurable at **runtime** via `ACANFD_STM32_Settings` — nothing about the CAN protocol configuration was hardcoded, only the board wiring.

If your board revision ever routes FDCAN1 to different pins, add the new entries to `fdcan1_tx_pin_array` / `fdcan1_rx_pin_array` in [`src/ACANFD_STM32_G431CBU6-objects.h`](src/ACANFD_STM32_G431CBU6-objects.h) and select them at runtime with `ACANFD_STM32_Settings::mTxPin` / `mRxPin` (index into those arrays).

### Compatibility with other ACAN libraries

Same API and the same `CANFDMessage` / `CANMessage` classes as the other ACAN/ACANFD libraries (ACAN2515, ACAN2517FD, ACANFD-STM32, etc.), so code written against those ports over with minimal changes.

### Files

| File | Purpose |
|---|---|
| `ACANFD_STM32.h` | Top-level include. Pulls in the board definition and declares the global `fdcan1` object. |
| `ACANFD_STM32_G431CBU6-objects.h` | Board-specific: pin arrays, the `fdcan1` instance, and the two IRQ handlers wired to `fdcan1.isr0()` / `isr1()`. |
| `ACANFD_STM32_G431CBU6-settings.h` | Board-specific: `HAS_PROGRAMMABLE_FDCAN_RAM_SECTIONS` and `fdcanClock()` (enables the FDCAN clock, selects PCLK1 as source, returns its frequency). |
| `ACANFD_STM32_Settings.h` / `.cpp` | `ACANFD_STM32_Settings` — bit timing calculator and all runtime-configurable driver settings. |
| `ACANFD_STM32_from_cpp.h` → `ACANFD-STM32-fixed-ram-sections.h` / `.cpp` | The `ACANFD_STM32` driver class itself (fixed-RAM-section variant, as used by G4). |
| `ACANFD_STM32_CANFDMessage.h`, `ACANFD_STM32_CANMessage.h` | `CANFDMessage` (up to 64 data bytes) and classic `CANMessage` (up to 8 data bytes) frame classes. |
| `ACANFD_STM32_Filters.h` / `.cpp` | `ACANFD_STM32_StandardFilters` / `ACANFD_STM32_ExtendedFilters` acceptance filter builders. |
| `ACANFD_STM32_FIFO.h` / `.cpp`, `ACANFD_STM32_DynamicArray.h` | Internal driver FIFO and dynamic array helpers. |

### Quick start

```cpp
#include <ACANFD_STM32.h>          // -> fdcan1 object, pins PB8(RX)/PB9(TX)
#include <ACANFD_STM32_Settings.h> // -> ACANFD_STM32_Settings, DataBitRateFactor

void setup () {
  Serial.begin (115200) ;

  // Arbitration 500 kbit/s @ 80% sample point, data 4x (2 Mbit/s) @ 80% sample point
  ACANFD_STM32_Settings settings (500 * 1000, 80, DataBitRateFactor::x4, 80) ;
  settings.mModuleMode = ACANFD_STM32_Settings::NORMAL_FD ;

  const uint32_t errorCode = fdcan1.beginFD (settings) ;
  if (errorCode != 0) {
    Serial.print ("fdcan1 configuration error: 0x") ;
    Serial.println (errorCode, HEX) ;
    while (true) { /* halt */ }
  }
}

void loop () {
  //--- Send a CAN FD frame with bit rate switch every second
  static uint32_t lastSend = 0 ;
  if (millis () - lastSend >= 1000) {
    lastSend = millis () ;

    CANFDMessage frame ;
    frame.id  = 0x123 ;
    frame.ext = false ;
    frame.type = CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH ;
    frame.len = 16 ;
    for (uint8_t i = 0 ; i < frame.len ; i++) frame.data [i] = i ;

    fdcan1.tryToSendReturnStatusFD (frame) ;
  }

  //--- Receive
  CANFDMessage rxFrame ;
  if (fdcan1.receiveFD0 (rxFrame)) {
    Serial.print ("RX id 0x") ;
    Serial.println (rxFrame.id, HEX) ;
  }
}
```

A fuller, runtime-configurable version of this example lives in [`src/working_canfd_example.txt`](../../src/working_canfd_example.txt) at the project root.

### API overview

#### `fdcan1` (global object, declared in `ACANFD_STM32_G431CBU6-objects.h`)

The single driver instance for this board. All CAN operations go through it.

* `uint32_t beginFD (const ACANFD_STM32_Settings & settings, const ACANFD_STM32_StandardFilters & std = {}, const ACANFD_STM32_ExtendedFilters & ext = {})`
  Configures and starts the peripheral. Returns `0` on success, otherwise a bitmask of `kXxx` error constants declared in `ACANFD_STM32` (e.g. `kMessageRamOverflow`, `kTooManyStandardFilters`, `kInvalidTxPin`). If the requested bit rate/sample point can't be achieved, `mBitSettingOk` on the settings will be `false` and `beginFD` will not configure the hardware.
* `void end (void)` — stops the peripheral.
* `uint32_t tryToSendReturnStatusFD (const CANFDMessage & message)` — enqueues a frame for transmission. Returns `0` on success, or `kInvalidMessage` / `kTransmitBufferIndexTooLarge` / `kTransmitBufferOverflow`.
* `bool availableFD0 (void)` / `bool receiveFD0 (CANFDMessage & outMessage)` — poll / pop from driver receive FIFO 0.
* `bool availableFD1 (void)` / `bool receiveFD1 (CANFDMessage & outMessage)` — same, for FIFO 1 (used for frames matched to `ACANFD_STM32_FilterAction::FIFO1`).
* `bool dispatchReceivedMessage (void)`, `dispatchReceivedMessageFIFO0 (void)`, `dispatchReceivedMessageFIFO1 (void)` — pop a message and invoke its filter's callback (see `mCallBack` on filters) instead of handling it manually.
* `void poll (void)` — call periodically from `loop()` if you are not relying purely on interrupts, to move data between the hardware FIFOs and the driver FIFOs.
* `uint32_t statusFlags (void) const` — bit 0/1: hardware/driver RxFIFO0 overflow, bit 2/3: hardware/driver RxFIFO1 overflow, bit 4: bus off.
* `BusStatus getStatus (void) const` — snapshot of the FDCAN error counters/protocol status (`txErrorCount()`, `rxErrorCount()`, `isBusOff()`, `transceiverDelayCompensationOffset()`).
* FIFO introspection: `transmitFIFOSize/Count/PeakCount()`, `driverReceiveFIFO0Size/Count/PeakCount()`, `driverReceiveFIFO1Size/Count/PeakCount()`, and matching `resetDriverReceiveFIFOxPeakCount()`.
* `void isr0 (void)` / `void isr1 (void)` — called from the `FDCAN1_IT0_IRQHandler` / `FDCAN1_IT1_IRQHandler` handlers already wired up in `ACANFD_STM32_G431CBU6-objects.h`. You should not need to call these yourself.

#### `ACANFD_STM32_Settings`

Constructed with your desired bit rate(s); computes the actual timing register values.

```cpp
// Sample point chosen automatically:
ACANFD_STM32_Settings (uint32_t arbitrationBitRate, DataBitRateFactor dataFactor, uint32_t tolerancePPM = 1000) ;

// Explicit sample points for arbitration and data phase:
ACANFD_STM32_Settings (uint32_t arbitrationBitRate, uint32_t arbitrationSamplePoint,
                       DataBitRateFactor dataFactor, uint32_t dataSamplePoint,
                       uint32_t tolerancePPM = 1000) ;
```

`DataBitRateFactor` is `x1`...`x10` — the data-phase bit rate is this multiple of the arbitration bit rate (e.g. 500 kbit/s arbitration with `x4` gives 2 Mbit/s data).

Useful accessors after construction: `actualArbitrationBitRate()`, `actualDataBitRate()`, `exactArbitrationBitRate()`, `exactDataBitRate()`, `ppmFromWishedBitRate()`, `arbitrationSamplePointFromBitStart()`, `dataSamplePointFromBitStart()`, `checkBitSettingConsistency()`, and the `mBitSettingOk` flag.

Notable settings you can override before calling `beginFD`:

* `mModuleMode` — `NORMAL_FD` (default), `INTERNAL_LOOP_BACK`, `EXTERNAL_LOOP_BACK`, `BUS_MONITORING`.
* `mDriverReceiveFIFO0Size` / `mDriverReceiveFIFO1Size` (default 60) and `mDriverTransmitFIFOSize` (default 10).
* `mDiscardReceivedStandardRemoteFrames` / `mDiscardReceivedExtendedRemoteFrames`.
* `mNonMatchingStandardFrameReception` / `mNonMatchingExtendedFrameReception` (`ACANFD_STM32_FilterAction::FIFO0` / `FIFO1` / `REJECT`) and matching `mNonMatchingStandardMessageCallBack` / `mNonMatchingExtendedMessageCallBack`.
* `mEnableRetransmission`, `mOpenCollectorOutput`, `mInputPullup`, `mTransceiverDelayCompensation`.
* `mTxPin` / `mRxPin` — index into `fdcan1_tx_pin_array` / `fdcan1_rx_pin_array` (defaults to `255`, meaning "use the first/only entry").

#### Filters

`ACANFD_STM32_StandardFilters` (11-bit IDs) and `ACANFD_STM32_ExtendedFilters` (29-bit IDs) are built up before calling `beginFD` and passed in:

```cpp
ACANFD_STM32_StandardFilters stdFilters ;
stdFilters.addSingle (0x123, ACANFD_STM32_FilterAction::FIFO0) ;
stdFilters.addRange  (0x200, 0x2FF, ACANFD_STM32_FilterAction::FIFO1, myCallback) ;

fdcan1.beginFD (settings, stdFilters) ;
```

Each supports `addSingle`, `addDual`, `addRange`, and `addClassic` (mask-based), each optionally taking a `void (*)(const CANFDMessage &)` callback invoked by `dispatchReceivedMessage()`.

#### `CANFDMessage` / `CANMessage`

`CANFDMessage` carries up to 64 data bytes (`len` is the real length: 0-8, 12, 16, 20, 24, 32, 48, or 64 — use `.pad()` to round up, `.isValid()` to check). `type` is one of `CAN_REMOTE`, `CAN_DATA`, `CANFD_NO_BIT_RATE_SWITCH`, `CANFD_WITH_BIT_RATE_SWITCH`. `CANMessage` is the classic (non-FD, ≤8 byte) equivalent and converts implicitly to `CANFDMessage`.

### Bit rate / sample point

The driver supports standard rates (62.5 kbit/s, 125 kbit/s, 250 kbit/s, 500 kbit/s, 1 Mbit/s) as well as non-standard ones (e.g. 833 kbit/s) via an exhaustive bit-timing search. If the requested combination of bit rate and sample point isn't achievable within `tolerancePPM`, `ACANFD_STM32_Settings::mBitSettingOk` is `false` and `beginFD` refuses to configure the hardware (returns non-zero).

### Notes / limitations of this fork

* Single target only: STM32G431CBU6, FDCAN1, PB8/PB9. There is no board-selection macro — to reuse this on different hardware, edit `ACANFD_STM32_G431CBU6-objects.h` / `-settings.h` directly or add pins to the existing arrays.
* No `examples` or `extras` directory (upstream's PDF API reference and per-board example sketches were dropped with the other boards). Refer to this README and to `src/working_canfd_example.txt` in the main project for usage.
* Only FDCAN1 is wired up — this chip's other CAN instances (if any) are not exposed.

# Firmware guide

The firmware is a [PlatformIO](https://platformio.org/) project in [`Firmware/`](../Firmware). It uses the STM32duino Arduino core, [SimpleFOC](https://github.com/simplefoc/Arduino-FOC) 2.4.0 and [SimpleFOCDrivers](https://github.com/simplefoc/Arduino-FOC-drivers) 1.0.9. Library versions are pinned in `platformio.ini`.

The same source tree builds for several boards that share the STM32G431 + DRV8316 design but differ in pins and parts. Each board has its own build environment (see [Boards](#boards)).

One firmware image serves every board of a type. Per-board settings are stored in flash and changed over the USB serial console or CAN, so you don't need a separate build for each motor. These settings are the board's node ID (and so its CAN IDs), direction inversion, and a stored FOC alignment.

## Build environments

| Environment | Board | Purpose |
|---|---|---|
| `bldc_driver_v02` (default) | BLDC-Driver v0.2 | The real firmware: FOC current control commanded over CAN |
| `bldc_driver_v02_test` | BLDC-Driver v0.2 | Bench and bring-up firmware, driven from the USB serial console only (no CAN) |
| `minifoc_mk2` | MiniFOC Mk2 | The real firmware |
| `minifoc_mk2_test` | MiniFOC Mk2 | Bench and bring-up firmware |

Every board has a `<name>` and a `<name>_test` environment, defined in `boards/<name>/board.ini`.

In the rest of this guide, "`driver`" means a board's firmware environment and "`motor_test`" its `_test` environment.

```bash
cd Firmware
```

```bash
pio run -e bldc_driver_v02                 # build
```

```bash
pio run -e bldc_driver_v02 -t upload       # flash over USB DFU
```

```bash
pio device monitor                # serial console, 115200 baud
```

## Flashing

`upload_protocol = dfu` is the default. It uses the STM32 ROM bootloader over USB-C, so no programmer is needed. You can enter DFU mode in several ways:

- Hold **BOOT** while you press **RST** or plug in USB.
- Type `dfu` in the serial console (works in both firmwares).
- BLDC-Driver only: press the **USER** button (PB12) while the `driver` firmware is running. (On MiniFOC the BOOT button is the CAN RX pin and can't be read by the firmware.)

To use an ST-Link on the SWD header instead, switch to `upload_protocol = stlink` in `platformio.ini`.

> [!NOTE]
> A DFU upload erases only the flash pages it writes, so the saved board settings survive a reflash. A **full-chip erase** (for example from STM32CubeProgrammer) wipes them, and the board then comes back up unconfigured.

## First-time setup

A freshly flashed board is **unconfigured**: CAN stays off, and the LED blinks quickly. Open the serial monitor and run:

```text
set node 1           # 1-15. Selects the CAN IDs (see can-protocol.md); `set side left|right` = node 1 / 2
set invert on|off    # optional
save                 # write to flash, verify, reboot
```

### Storing the FOC alignment (optional)

By default, every FOC start runs SimpleFOC's sensor alignment, which nudges the motor for 1–3 seconds. To skip the alignment on later boots:

1. Start FOC once with `start` in the console or `CMD_START_FOC` over CAN.
2. Type `cal store`, then `save`.

From then on, the stored sensor direction and zero electric angle are used, and the status heartbeat sets `STATUS_FLAG_CALIBRATION_USED`.

> [!WARNING]
> If you remount the motor, magnet or encoder, run `cal clear` and then `save`. Otherwise FOC runs with the wrong electrical angle.

## Serial console reference (`driver`)

The console runs at 115200 baud. Lines end in LF or CR, and commands are case-insensitive.

| Command | Description |
|---|---|
| `help`, `?` | List commands |
| `config`, `status` | Show active and pending settings, driver state, CAN IDs, supply voltage, commanded current |
| `set node 1-15` | Set the board's CAN node ID (sets its CAN IDs) |
| `set side left\|right` | Shorthand for node 1 / node 2 |
| `set invert on\|off` | Flip the sign of commanded current and reported velocity |
| `cal store` | Keep this boot's FOC alignment (FOC must already be running) |
| `cal clear` | Forget the stored alignment |
| `save` | Stop the motor, write settings to flash, verify and reboot |
| `revert` | Discard unsaved changes |
| `start` | Start FOC from the console (bench use; normally sent over CAN) |
| `reboot` | Reset the MCU |
| `dfu` | Reboot into the USB DFU bootloader |

`set` and `cal` only change the **pending** settings. Nothing takes effect until `save`.

## Runtime behaviour

1. **Boot.** The firmware waits up to 2 s for a serial terminal, loads settings, starts CAN (if the board is configured), measures VIN and configures the DRV8316. The motor is **not** energised yet.
2. **Idle.** The board waits for `CMD_START_FOC` over CAN (or `start` on the console) and sends a status heartbeat every 100 ms, which includes the live supply voltage.
3. **Start.** The board writes the DRV8316 registers again, in case the board booted on USB power before the motor supply was connected. It then runs `motor.init()`, current-sense initialisation and `initFOC()`, which block for about 1–3 s. CAN frames received during the start are discarded. The result is reported as a `CMD_START_FOC` reply, and a `CMD_ERROR` frame is sent on failure. A failed start can be retried.
4. **Running.** The board runs FOC current (torque) control. Target current comes from the board's current frame and is clamped to ±`MOTOR_CURRENT_LIMIT`. Shaft velocity is sent every 10 ms.

### Safety features

| Feature | Behaviour |
|---|---|
| Command timeout | If no current command arrives for 100 ms while running, the target drops to 0 A (the motor freewheels) until commands resume |
| Watchdog | Independent watchdog (250 ms), started once FOC is running |
| DRV8316 fault polling | Status is read every 500 ms. A new fault is printed, reported over CAN (`ERR_DRIVER_FAULT`) and cleared |
| CAN bus-off recovery | The firmware detects bus-off and rejoins the bus automatically |
| Input validation | NaN or Inf current commands are ignored (NaN = "no command for this node"), and short current frames are rejected |

### Status LED

| Pattern | Meaning |
|---|---|
| Fast blink (5 Hz) | Unconfigured: set a node ID and save |
| Short blip every second | Idle, waiting for `CMD_START_FOC` |
| Solid on | FOC running |
| Double blink every second | FOC start failed (send `CMD_START_FOC` to retry) |

## Configuration (`Firmware/include/motor_config.h`)

| Define | Default | Description |
|---|---|---|
| `MOTOR_POLE_PAIRS` | 11 | Rotor pole pairs. **Set this for your motor** |
| `MOTOR_CURRENT_LIMIT` | 2.0 A | Commanded current clamp |
| `MOTOR_VOLTAGE_LIMIT` | 10.0 V | Driver voltage limit (also capped at the supply voltage) |
| `MOTOR_VOLTAGE_SENSOR_ALIGN` | 3.0 V | Voltage used during sensor alignment |
| `MOTOR_VOLTAGE_POWER_SUPPLY` | 12.0 V | Fallback when the VIN measurement is out of range |
| `SUPPLY_VOLTAGE_MIN` / `_MAX` | 6 / 30 V | Sanity window for the measured VIN |
| `SHAFT_VELOCITY_TELEMETRY_INTERVAL_US` | 10 000 | Velocity telemetry period |
| `COMMAND_TIMEOUT_MS` | 100 | Current command timeout |
| `CAN_STATUS_INTERVAL_MS` | 100 | Heartbeat period |
| `DRV_STATUS_POLL_INTERVAL_MS` | 500 | DRV8316 fault poll period |
| `WATCHDOG_TIMEOUT_US` | 250 000 | Watchdog timeout |
| `BOOT_SERIAL_WAIT_MS` | 2 000 | How long to wait for a serial terminal at boot |

The CAN IDs and bit rate are in [`Firmware/include/can_protocol.h`](../Firmware/include/can_protocol.h). Hardware constants (pins, current-sense gain, `VSENS_CALIBRATION` and so on) are in the board headers, see [Boards](#boards).

## Boards

Each board is a folder in [`Firmware/boards/`](../Firmware/boards):

```
Firmware/boards/
  bldc_driver_v02/
    board_config.h      pins, parts and hardware constants
    board.ini    build environments bldc_driver_v02 and bldc_driver_v02_test
  minifoc_mk2/
    board_config.h
    board.ini
```

`platformio.ini` pulls in every `boards/*/board.ini`, and each board's environments add `-I boards/<name>`, so the code finds that board's header as `"board_config.h"`. Nothing outside the folder lists the boards: the CI workflow also builds whatever folders it finds.

| | [BLDC-Driver v0.2](boards/bldc_driver_v02.md) | [MiniFOC Mk2](boards/minifoc_mk2.md) |
|---|---|---|
| Folder | `bldc_driver_v02` | `minifoc_mk2` |
| Model / hardware rev | BLDC-Driver v0.2 / 2 | MiniFOC Mk2 / 2 |
| Encoder | AS5048A, 5 MHz SPI | AS5047P, 8 MHz SPI |
| Phase current sense | DRV8316 low-side CSAs, 600 mV/A | Inline 3 mΩ shunts + INA241A3 (50 V/V) |
| Supply sensing | V_SENS divider on ADC1 | INA226 on I2C1 (voltage and input current) |
| 5 V supply | DRV8316 buck | Separate AP63205 (DRV8316 buck off) |
| DFU button | USER (PB12) | None (BOOT is CAN RX) |
| Capabilities (`CMD_GET_BOARD_INFO`) | `CAP_MEASURES_SUPPLY_VOLTAGE` | `CAP_MEASURES_SUPPLY_VOLTAGE`, `CAP_MEASURES_INPUT_CURRENT` |

### Parts

Each kind of part has an interface in [`src/hw/`](../Firmware/src/hw) and one file per part in [`src/parts/`](../Firmware/src/parts):

| Interface | Parts | Selected by |
|---|---|---|
| `hw/encoder.h` | `encoder_as5048a.cpp`, `encoder_as5047p.cpp` | `ENCODER_AS5048A`, `ENCODER_AS5047P` |
| `hw/current_sense.h` | `current_drv8316_lowside.cpp`, `current_inline.cpp` | `CURRENT_SENSE_DRV8316_LOWSIDE`, `CURRENT_SENSE_INLINE` |
| `hw/supply.h` | `supply_vsens.cpp`, `supply_ina226.cpp` | `SUPPLY_SENSE_ADC_DIVIDER`, `SUPPLY_SENSE_INA226` |

Every part file is compiled for every board, but its contents are inside `#if defined(<its macro>)`, so only the parts the board header selects end up in the build. Choosing no part of a kind gives an "undefined reference" link error, and choosing two gives a "multiple definition" error.

The rest of the firmware only uses the interfaces, so it never checks which board or part it has. The DRV8316 setup (`hw/drv8316_setup.cpp`) is shared too: it turns the DRV8316 buck on if the board defines `DRV8316_BUCK_VOLTAGE` and off otherwise, and asks the current-sense part for any settings it needs (the CSA gain for the DRV8316's own amplifiers).

### What a board header defines

- `BOARD_MODEL` (up to 30 characters) and `BOARD_HW_REV`. The host reads these with `CMD_GET_BOARD_INFO`.
- The pin map: `LED_BUILTIN`, SPI1, `DRV_CS`, `ENC_CS`, `CAN_EN`, `CAN_RX_BOOT0`, the six PWM pins and `SO_A/B/C`.
- One part of each kind, plus that part's constants:
  - an encoder with `ENCODER_SPI_HZ`;
  - a current sense, either `CURRENT_SENSE_MV_PER_A` and `DRV8316_CSA_GAIN`, or `CURRENT_SENSE_SHUNT_OHMS` and `CURRENT_SENSE_GAIN`, plus `CURRENT_SENSE_OFFSET_HINT` for `motor_test`;
  - a supply sensor with its pins and constants (`V_SENS`, `VSENS_*`, or `I2C1_*`, `INA226_*`).
- Optional: `DRV8316_BUCK_VOLTAGE`, and `HAS_DFU_BUTTON` with `DFU_BUTTON`.

Build flags that the STM32 core reads, rather than our code, go in `board.ini`: `USB_PRODUCT`, and on BLDC-Driver `ADC_SAMPLINGTIME`.

### Variants

A variant that differs in a few values, such as other shunts, another encoder or one moved pin, is a new folder whose `board_config.h` includes the base board and overrides what changes:

```cpp
// boards/minifoc_mk2_2mohm/board_config.h
#pragma once
#include "../minifoc_mk2/board_config.h"

#undef  BOARD_MODEL
#define BOARD_MODEL "MiniFOC Mk2 2mOhm"
#undef  CURRENT_SENSE_SHUNT_OHMS
#define CURRENT_SENSE_SHUNT_OHMS 0.002f
```

Its `board.ini` is a copy of the base board's, with the folder name changed. Give a variant its own `BOARD_MODEL`, so a host can tell it from its base.

### Adding a board

1. Copy the folder of the closest board in `boards/`, or make it a variant as above.
2. Edit `board_config.h`: `BOARD_MODEL`, `BOARD_HW_REV`, pins, parts and constants.
3. Edit `board.ini`: rename both environments and the `-I boards/...` path, and set `USB_PRODUCT`.
4. If the board has a part the firmware doesn't know yet, add a file in `src/parts/` that implements the interface in `src/hw/`, guarded by a new macro (copy a similar part). If the part gives the host something new, such as an IMU, give it a capability bit in `can_protocol.h` and add it to `boardCapabilities()` in `hw/board_info.cpp`.
5. Build the new environments, then bring the board up with its `_test` environment and the [test routines](test-routines.md).
6. Outside the firmware, use the same folder name for the board's KiCad project (`Hardware/<name>/`), its pictures (`Images/<name>/`) and its page (`docs/boards/<name>.md`, copy an existing one). Add it to the boards table in the README.

The host tools need no changes: they show the model name and use the capability bits that the board reports.

## Source layout

| File | Responsibility |
|---|---|
| `src/main.cpp` | `setup()` / `loop()`, status LED |
| `src/motor_control.*` | SimpleFOC motor, driver, current sense and encoder; FOC start, command timeout, telemetry, fault polling |
| `src/can_bus.*` | FDCAN setup and filters, frame encoding and decoding, heartbeat, bus-off recovery |
| `src/board_settings.*` | Per-board settings in flash (EEPROM emulation, CRC-32 and version checked) |
| `src/serial_console.*` | Non-blocking USB serial command console |
| `src/hw/` | Part interfaces (`encoder.h`, `current_sense.h`, `supply.h`), the shared DRV8316 setup, `board_info` (capabilities, model) and the supply code every sensor shares |
| `src/parts/` | One file per part, compiled in when the board header selects it |
| `src/dfu.*` | Reboot into the ROM DFU bootloader (also with the watchdog running), DFU button |
| `src/clock_config.cpp` | 160 MHz clock tree from the 16 MHz HSE |
| `src/motor_test/` | Bench-test firmware (`_test` environments only) |
| `boards/<name>/` | Each board's `board_config.h` (pins, parts) and `board.ini` (build environments) |
| `lib/ACANFD_STM32_G431/` | FDCAN driver, a fork of ACANFD-STM32 trimmed to this board |

### Flash size limit

Settings live in the **last 2 KB flash page** (0x0801F800), so the firmware image must stay below **126 KB**. The firmware checks this at boot and before every save, and prints an error if the image overlaps the settings page.

## Bench-test firmware (`motor_test`)

`motor_test` brings up a new board or motor one subsystem at a time. It shares the board header, `motor_config.h`, `src/hw/` and `src/parts/` (DRV8316 setup, encoder, current sense, VIN measurement), the clock configuration and DFU with the real firmware.

```bash
pio run -e bldc_driver_v02_test -t upload       # or minifoc_mk2_test
```

```bash
pio device monitor
```

Type `help` in the monitor. For a full step-by-step procedure with expected readings and troubleshooting for each step, see **[Test routines](test-routines.md)**. In summary:

| Step | Command | What it checks |
|---|---|---|
| 1 | `s`, `n [N]`, `w` | Encoder only (power stage off): angle, AGC, magnitude and field flags; noise and glitches at rest; angle while turning by hand |
| 2 | `drv` | Configures the DRV8316 and prints its register and fault status |
| 3 | `vin` | Supply voltage (on BLDC-Driver, run it before `cs`); input current too on MiniFOC |
| 4 | `v <volts>`, `ol <rad/s>` | Open-loop spin at a low voltage, compared against the sensor speed |
| 5 | `rev [n]` | Open-loop revolutions: verifies pole pairs and direction |
| 6 | `cs`, `hold [volts]` | Current-sense offsets, and DC phase currents with a locked field |
| 7 | `foc`, `t <volts>` | SimpleFOC alignment, then closed-loop voltage torque |

`x` or an empty line stops the motor, and every motor-on mode stops on its own after 30 s. The test voltage is capped at 4 V.

> [!CAUTION]
> Open-loop modes push current continuously even when the rotor is stalled. Keep the voltage low, keep wheels off the ground and use a current-limited supply.

When you've finished, type `dfu` and flash the board's firmware environment again.

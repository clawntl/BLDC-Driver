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

1. **Boot.** The firmware waits up to 2 s for a serial terminal, loads settings, starts CAN (if the board is configured), measures VIN and configures the DRV8316. The motor is **not** energised yet. If the DRV8316 doesn't answer over SPI (no motor supply yet, or firmware built for another board), a warning is printed.
2. **Idle.** The board waits for `CMD_START_FOC` over CAN (or `start` on the console) and sends a status heartbeat every 100 ms, which includes the live supply voltage.
3. **Start.** The board checks the DRV8316 answers over SPI (writing its registers again if it didn't at boot), then runs `motor.init()`, current-sense initialisation and `initFOC()`, which block for about 1–3 s. CAN frames received during the start are discarded. The result is reported as a `CMD_START_FOC` reply, and a `CMD_ERROR` frame is sent on failure. A failed start can be retried.
4. **Running.** The board runs FOC current (torque) control. Target current comes from the board's current frame and is clamped to ±`MOTOR_CURRENT_LIMIT`. Shaft velocity is sent every 10 ms.

### Safety features

| Feature | Behaviour |
|---|---|
| Command timeout | If no current command arrives for 100 ms while running, the target drops to 0 A (the motor freewheels) until commands resume |
| Watchdog | Independent watchdog (250 ms), started once FOC is running |
| DRV8316 fault polling | Status is read every 500 ms. A new fault is printed, reported over CAN (`ERR_DRIVER_FAULT`) and cleared |
| CAN bus-off recovery | The firmware detects bus-off and rejoins the bus automatically |
| Input validation | NaN or Inf current commands are ignored (NaN = "no command for this node"), and short current frames are rejected |
| Wrong-board check | Start FOC refuses to run (`ERR_DRIVER_SPI`) unless the DRV8316 reads back the registers written to it, so firmware built for another board can't drive the bridge with the wrong current sense |

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

The CAN IDs and bit rate are in [`Firmware/include/can_protocol.h`](../Firmware/include/can_protocol.h). Hardware constants (pins, current-sense gain, `VSENS_CALIBRATION` and so on) are in the board headers, see below.

## Boards

Everything that differs between boards lives in one header per board in [`Firmware/include/boards/`](../Firmware/include/boards). The environment's `-D BOARD_...` flag picks it through `include/board.h`.

| | BLDC-Driver v0.2 | MiniFOC Mk2 |
|---|---|---|
| Header | `bldc_driver_v02.h` | `minifoc_mk2.h` |
| Board ID / hardware rev (`CMD_GET_INFO`) | 1 / 2 | 2 / 2 |
| Encoder | AS5048A, 5 MHz SPI | AS5047P, 8 MHz SPI |
| Phase current sense | DRV8316 low-side CSAs, 600 mV/A | Inline 3 mΩ shunts + INA241A3 (50 V/V) |
| Supply sensing | V_SENS divider on ADC1 | INA226 on I2C1 (voltage and input current) |
| 5 V supply | DRV8316 buck | Separate AP63205 (DRV8316 buck off) |
| DFU button | USER (PB12) | None (BOOT is CAN RX) |

A board header defines:

- `BOARD_ID`, `BOARD_HW_REV`, `BOARD_MODEL`: reported by `CMD_GET_INFO` and the `config` command. IDs are listed in `include/board_ids.h`.
- The pin map: `LED_BUILTIN`, SPI1, `DRV_CS`, `ENC_CS`, `CAN_EN`, the six PWM pins and `SO_A/B/C`.
- Exactly one of each part type, plus that part's constants:
  - `ENCODER_AS5048A` or `ENCODER_AS5047P`, with `ENCODER_SPI_HZ`;
  - `CURRENT_SENSE_DRV8316_LOWSIDE` (with `CURRENT_SENSE_MV_PER_A`, `DRV8316_CSA_GAIN`) or `CURRENT_SENSE_INLINE` (with `CURRENT_SENSE_SHUNT_OHMS`, `CURRENT_SENSE_GAIN`);
  - `SUPPLY_SENSE_ADC_DIVIDER` (`V_SENS`, `VSENS_*`) or `SUPPLY_SENSE_INA226` (`I2C1_*`, `INA226_*`);
  - `DRV8316_BUCK_ENABLED` 0/1 (and `DRV8316_BUCK_VOLTAGE`);
  - `HAS_DFU_BUTTON` with `DFU_BUTTON`, if there is one.

`board.h` stops the build with an error if one is missing. The code in [`Firmware/src/hw/`](../Firmware/src/hw) turns these into the SimpleFOC objects, the DRV8316 setup and the supply sensing, so the rest of the firmware never checks which board it is.

### Adding a board

1. Copy the closest header in `include/boards/`, rename it and update the pins and parts. Give it a new number in `include/board_ids.h` (never reuse one) and set `BOARD_HW_REV`.
2. Add it to the `#if` list in `include/board.h`.
3. In `platformio.ini`, copy a board's `[name]` section and its two `[env:...]` sections, and change the `-D BOARD_...` flag and `USB_PRODUCT`.
4. If the board has a part the firmware doesn't know yet (a different encoder, current-sense amplifier or supply monitor), add a new capability macro and handle it in `src/hw/`: `board_hw.h` for encoders and current sense, or a new `supply_*.cpp` that implements `hw/supply.h`.
5. Build every environment (`pio run`), then bring the board up with its `_test` environment and the [test routines](test-routines.md).

## Source layout

| File | Responsibility |
|---|---|
| `src/main.cpp` | `setup()` / `loop()`, status LED |
| `src/motor_control.*` | SimpleFOC motor, driver, current sense and encoder; FOC start, command timeout, telemetry, fault polling |
| `src/can_bus.*` | FDCAN setup and filters, frame encoding and decoding, heartbeat, bus-off recovery |
| `src/board_settings.*` | Per-board settings in flash (EEPROM emulation, CRC-32 and version checked) |
| `src/serial_console.*` | Non-blocking USB serial command console |
| `src/hw/board_hw.h` | Encoder and current-sense types for the board, encoder diagnostics adapter |
| `src/hw/drv8316_setup.cpp` | DRV8316 register setup and the read-back check (shared with `motor_test`) |
| `src/hw/supply.h` | Supply sensing interface: boot measurement, live VIN for the heartbeat, input current |
| `src/hw/supply_adc_divider.cpp` | V_SENS divider: `analogRead()` before the current sense starts, regular ADC1 conversions afterwards |
| `src/hw/supply_ina226.cpp` | INA226 power monitor over I2C |
| `src/dfu.*` | Reboot into the ROM DFU bootloader (also with the watchdog running), DFU button |
| `src/clock_config.cpp` | 160 MHz clock tree from the 16 MHz HSE |
| `src/motor_test/` | Bench-test firmware (`_test` environments only) |
| `include/board.h`, `include/boards/` | Board selection, pin maps and fitted parts |
| `lib/ACANFD_STM32_G431/` | FDCAN driver, a fork of ACANFD-STM32 trimmed to this board |

### Flash size limit

Settings live in the **last 2 KB flash page** (0x0801F800), so the firmware image must stay below **126 KB**. The firmware checks this at boot and before every save, and prints an error if the image overlaps the settings page.

## Bench-test firmware (`motor_test`)

`motor_test` brings up a new board or motor one subsystem at a time. It shares the board header, `motor_config.h`, `src/hw/` (DRV8316 setup, encoder, current sense, VIN measurement), the clock configuration and DFU with the real firmware.

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

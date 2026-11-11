# Firmware guide

The firmware is a [PlatformIO](https://platformio.org/) project in [`Firmware/`](../Firmware). It uses the STM32duino Arduino core, [SimpleFOC](https://github.com/simplefoc/Arduino-FOC) 2.4.0 and [SimpleFOCDrivers](https://github.com/simplefoc/Arduino-FOC-drivers) 1.0.9. Library versions are pinned in `platformio.ini`.

One firmware image serves every board. Per-board settings are stored in flash and changed over the USB serial console, so you don't need a separate build for each motor. These settings are which motor the board drives (and so its CAN IDs), direction inversion, and a stored FOC alignment.

## Build environments

| Environment | Purpose |
|---|---|
| `driver` (default) | The real firmware: FOC current control commanded over CAN |
| `motor_test` | Bench and bring-up firmware, driven from the USB serial console only (no CAN) |

```bash
cd Firmware
```

```bash
pio run -e driver                 # build
```

```bash
pio run -e driver -t upload       # flash over USB DFU
```

```bash
pio device monitor                # serial console, 115200 baud
```

## Flashing

`upload_protocol = dfu` is the default. It uses the STM32 ROM bootloader over USB-C, so no programmer is needed. You can enter DFU mode in several ways:

- Hold **BOOT** while you press **RST** or plug in USB.
- Type `dfu` in the serial console (works in both firmwares).
- Press the **USER** button (PB12) while the `driver` firmware is running.

To use an ST-Link on the SWD header instead, switch to `upload_protocol = stlink` in `platformio.ini`.

> [!NOTE]
> A DFU upload erases only the flash pages it writes, so the saved board settings survive a reflash. A **full-chip erase** (for example from STM32CubeProgrammer) wipes them, and the board then comes back up unconfigured.

## First-time setup

A freshly flashed board is **unconfigured**: CAN stays off, and the LED blinks quickly. Open the serial monitor and run:

```text
set side left        # or right. Selects the CAN IDs (see can-protocol.md)
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
| `set side left\|right` | Choose the motor side (sets the CAN IDs) |
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

1. **Boot.** The firmware waits up to 2 s for a serial terminal, measures VIN, loads settings, starts CAN (if the board is configured) and configures the DRV8316. The motor is **not** energised yet.
2. **Idle.** The board waits for `CMD_START_FOC` over CAN (or `start` on the console) and sends a status heartbeat every 100 ms.
3. **Start.** The board runs `motor.init()`, current-sense initialisation and `initFOC()`, which block for about 1–3 s. CAN frames received during the start are discarded. The result is reported as a `CMD_START_FOC` reply, and a `CMD_ERROR` frame is sent on failure. A failed start can be retried.
4. **Running.** The board runs FOC current (torque) control. Target current comes from the broadcast frame and is clamped to ±`MOTOR_CURRENT_LIMIT`. Shaft velocity is sent every 10 ms.

### Safety features

| Feature | Behaviour |
|---|---|
| Command timeout | If no current command arrives for 100 ms while running, the target drops to 0 A (the motor freewheels) until commands resume |
| Watchdog | Independent watchdog (250 ms), started once FOC is running |
| DRV8316 fault polling | Status is read every 500 ms. A new fault is printed, reported over CAN (`ERR_DRIVER_FAULT`) and cleared |
| CAN bus-off recovery | The firmware detects bus-off and rejoins the bus automatically |
| Input validation | NaN or Inf current commands are ignored, and short broadcast frames are rejected |

### Status LED

| Pattern | Meaning |
|---|---|
| Fast blink (5 Hz) | Unconfigured: set a side and save |
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
| `VSENS_CALIBRATION` | 1.0 | Trim factor: (multimeter VIN) / (reported VIN) |
| `SUPPLY_VOLTAGE_MIN` / `_MAX` | 6 / 30 V | Sanity window for the measured VIN |
| `SHAFT_VELOCITY_TELEMETRY_INTERVAL_US` | 10 000 | Velocity telemetry period |
| `COMMAND_TIMEOUT_MS` | 100 | Current command timeout |
| `CAN_STATUS_INTERVAL_MS` | 100 | Heartbeat period |
| `DRV_STATUS_POLL_INTERVAL_MS` | 500 | DRV8316 fault poll period |
| `WATCHDOG_TIMEOUT_US` | 250 000 | Watchdog timeout |
| `BOOT_SERIAL_WAIT_MS` | 2 000 | How long to wait for a serial terminal at boot |

The CAN IDs and bit rate are in [`Firmware/include/can_protocol.h`](../Firmware/include/can_protocol.h).

## Source layout

| File | Responsibility |
|---|---|
| `src/main.cpp` | `setup()` / `loop()`, status LED |
| `src/motor_control.*` | SimpleFOC motor, driver, current sense and encoder; FOC start, command timeout, telemetry, fault polling |
| `src/can_bus.*` | FDCAN setup and filters, frame encoding and decoding, heartbeat, bus-off recovery |
| `src/board_settings.*` | Per-board settings in flash (EEPROM emulation, CRC-32 and version checked) |
| `src/serial_console.*` | Non-blocking USB serial command console |
| `src/supply_voltage.*` | VIN measurement through the V_SENS divider |
| `src/dfu.*` | Reboot into the ROM DFU bootloader (also with the watchdog running), USER button |
| `src/clock_config.cpp` | 160 MHz clock tree from the 16 MHz HSE |
| `src/motor_test/` | Bench-test firmware (`motor_test` environment only) |
| `include/pins.h` | Board pin mapping |
| `lib/ACANFD_STM32_G431/` | FDCAN driver, a fork of ACANFD-STM32 trimmed to this board |

### Flash size limit

Settings live in the **last 2 KB flash page** (0x0801F800), so the firmware image must stay below **126 KB**. The firmware checks this at boot and before every save, and prints an error if the image overlaps the settings page.

## Bench-test firmware (`motor_test`)

`motor_test` brings up a new board or motor one subsystem at a time. It shares `pins.h`, `motor_config.h`, the clock configuration, DFU and the VIN measurement code with the real firmware.

```bash
pio run -e motor_test -t upload
```

```bash
pio device monitor
```

Type `help` in the monitor. The recommended order:

| Step | Command | What it checks |
|---|---|---|
| 1 | `s`, `n [N]`, `w` | Encoder only (power stage off): angle, AGC, magnitude and field flags; noise and glitches at rest; angle while turning by hand |
| 2 | `drv` | Configures the DRV8316 and prints its register and fault status |
| 3 | `vin` | Supply voltage (must run before `cs`) |
| 4 | `v <volts>`, `ol <rad/s>` | Open-loop spin at a low voltage, compared against the sensor speed |
| 5 | `rev [n]` | Open-loop revolutions: verifies pole pairs and direction |
| 6 | `cs`, `hold [volts]` | Current-sense offsets, and DC phase currents with a locked field |
| 7 | `foc`, `t <volts>` | SimpleFOC alignment, then closed-loop voltage torque |

`x` or an empty line stops the motor, and every motor-on mode stops on its own after 30 s. The test voltage is capped at 4 V.

> [!CAUTION]
> Open-loop modes push current continuously even when the rotor is stalled. Keep the voltage low, keep wheels off the ground and use a current-limited supply.

When you've finished, type `dfu` and flash the `driver` environment again.

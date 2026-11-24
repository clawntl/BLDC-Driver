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
| `config`, `status` | Show the firmware, board and serial number, active and pending settings, driver state, CAN IDs, supply voltage, commanded current |
| `set node 1-15` | Set the board's CAN node ID (sets its CAN IDs) |
| `set side left\|right` | Shorthand for node 1 / node 2 |
| `set invert on\|off` | Flip the sign of commanded current and reported velocity |
| `set curp <V/A>` | Current loop P (default 3). Applies at once |
| `set curi <V/(A·s)>` | Current loop I (default 300). Applies at once |
| `set curtf <ms>` | Current filter time constant, in ms (default 0.53). Applies at once |
| `cur` | Show the current loop in use and the FOC loop rate |
| `cur defaults` | Current loop back to the defaults (applies at once) |
| `cal store` | Keep this boot's FOC alignment (FOC must already be running) |
| `cal clear` | Forget the stored alignment |
| `cog` | Anti-cogging status |
| `cog on\|slots\|off` | Anti-cogging compensation: on, slots only (the lowest harmonics left out), off. Applies at once |
| `cog cal [kp kd speed]` | Measure the cogging (FOC running, wheel free, ~55 s). Optional position loop gains (A/rad, A/(rad/s)) and sweep speed (rad/s) |
| `cog refine [kp kd speed]` | Measure again with the table applied and add what is left |
| `cog abort` / `cog clear` | Stop a calibration / forget the table |
| `cog dump` | Print the table (and the last calibration's forward and backward curves) as CSV. Over CAN: `COG_READ` / `COG_INFO` |
| `save` | Stop the motor, write settings to flash, verify and reboot |
| `revert` | Discard unsaved changes |
| `start` | Start FOC from the console (bench use; normally sent over CAN) |
| `reboot` | Reset the MCU |
| `dfu` | Reboot into the USB DFU bootloader |

`set` and `cal` only change the **pending** settings. Nothing takes effect until `save`, except the current loop (`set curp/curi/curtf`, `cur defaults`), which also applies at once so it can be tuned on a running motor; `save` keeps it and `revert` goes back to the saved values. The same parameters can be set over CAN (`PARAM_CURRENT_*`, see the [CAN protocol](can-protocol.md)).

### Current loop

The board runs SimpleFOC's FOC current control: a PI controller per axis turns the error between the commanded and the measured current into a voltage. Its defaults (P = 3 V/A, I = 300 V/(A·s), filter 0.53 ms) suit low-resistance motors. On a higher-resistance gimbal motor the loop is slow: with R = 2.8 Ω per phase (GM3506; its datasheet's 5.6 Ω is line-to-line) the actual current follows a step in the command with a time constant of roughly (R + P) / I ≈ 20 ms, and it falls behind whenever the motor speeds up, because the back-EMF rises faster than the integral can follow. A balancing robot sees that as torque lag.

Raising **I** speeds the loop up roughly in proportion; that's the main knob. **P** adds a direct response, but too much for the motor's inductance and the FOC loop rate makes it ring. A starting point for a 5–6 Ω motor is P = 3 and I = 1500–3000 (about 3–6 ms). Check the FOC loop rate with `cur`: keep the loop's response well below it (I / (R + P) under about a tenth of the loop rate in rad/s). Settings saved by firmware 1.0–1.3 are kept when updating; the current loop then starts at the defaults.

### Motor voltage (firmware 1.5)

The drivers use **Space Vector PWM**. The PWM scaling follows the **live supply voltage** (filtered over about 0.5 s), not just the reading at boot.

**Phase voltage range.** The phases swing between 0 and `MOTOR_MAX_DUTY` × the supply, capped at `MOTOR_VOLTAGE_LIMIT`. The 0.92 duty limit leaves the DRV8316's low-side current-sense amplifiers 4 µs of low-side on-time per 20 kHz period.

**Motor voltage.** The most the motor gets on the q axis without distortion is the range / √3. That's also the current loop's output limit.

| Supply | q-axis voltage | GM3506 (Ke ≈ 0.036 V/(rad/s)): no-current speed | full 1.5 A up to (R = 2.8 Ω) |
|---|---|---|---|
| 12.6 V (3S full) | 6.7 V | ~185 rad/s | ~70 rad/s |
| 11.1 V (3S nominal) | 5.9 V | ~164 rad/s | ~47 rad/s |
| 9.6 V (EBS-SBR cutoff) | 5.1 V | ~142 rad/s | ~25 rad/s |
| Firmware 1.4 (Sine PWM, fixed 10 V) | 5.0 V | ~137 rad/s | ~20 rad/s |

**Why it changed.** Sine PWM centres the phases on half the range and can only reach range / 2. Firmware 1.4 also scaled the PWM by the supply measured at boot, so as the battery ran down the real voltages fell below the commanded ones. The serial console's `status` shows the supply in use and the motor voltage.

### Anti-cogging (firmware 1.7)

A gimbal motor's magnets pull towards the stator teeth, so even with no current the rotor has preferred positions: on a 24-slot, 22-pole motor such as the GM3506, 264 of them per turn (every 1.36°). That **cogging** torque acts like a bumpy spring. A balancing robot holding still sits right in it, and its controller sees the bumps as disturbances: a small stick-slip sway. Motors differ, and new ones tend to cog more.

Anti-cogging measures the cogging once and then cancels it: each FOC loop adds the current that counters the cogging at the rotor's angle.

**Calibration** (`cog cal`, or `CMD_COGGING` calibrate). With FOC running and the wheel free to turn (robot on its back or on a stand), the board turns the rotor slowly with its own stiff position loop, one turn forwards and one back (~55 s), and records the average current that loop needs in each of 1920 angle bins (0.19° each). Friction pushes against the motion in each direction, so the mean of the two passes is the cogging and half their difference the friction.

Only the repeatable part goes into the table: the harmonics 1–64 per turn, plus 264 and 528 (the slot/pole cogging of a 24-slot, 22-pole motor). On the first GM3506s measured (robot e4a7d0), each bin's value also carried ~20 mA of noise that differed between passes, while the cogging itself was almost all at 2, 22–26 and 42–48 per turn, and hardly anything at 264; the bump you feel turning the wheel by hand is the 24-per-turn one, every 15°. Without the filter the compensation played that noise back as fake cogging. The calculation takes ~25 ms at the end, with the rotor standing still.

The result is reported as:

- **peak-to-peak**: the size of the cogging, in mA;
- **friction**: what it took to keep turning, in mA;
- **agreement**: how alike the forward and backward curves are over the kept harmonics (above ~85 % is good; low means the rotor slipped, or friction that changes with the angle);
- **noise left out**: rms of what the filter removed (shown after a calibration, until reboot);
- **largest position error**: how far the rotor lagged its set point. Above ~5 mrad the cogging is strong compared with the position loop, and part of it gets smoothed away; run `cog refine` afterwards (it measures again with the table applied and adds what is left), or calibrate with a larger kp.

During a calibration the current loop's I is raised to at least 3000 V/(A·s) and the velocity filter shortened to 1 ms; both go back afterwards. A current command over 50 mA from the host aborts it. The new table is pending until `save` (which reboots), and `revert` brings the stored one back.

**Compensation** (`cog on`, `PARAM_COG_COMP`; off by default, a tuning setting like the current loop). The table is read ahead of the rotor by the time the current loop takes to follow, (`MOTOR_PHASE_RESISTANCE` + P) / I (1.9 ms with P 3, I 3000; 3.9 ms with I 1500), and fades out between 6 and 15 rad/s: by then the 48-per-turn part comes round more than 100 times a second, faster than the current loop can follow, and the wheel's inertia smooths it out anyway. The measured current reported in the velocity telemetry includes the compensation, so turn it off for spin tests.

**Slots only** (`cog slots`, `PARAM_COG_COMP` 2) leaves harmonics 1–3 per turn out of the compensation. On e4a7d0, full compensation halved the 0.3–1 Hz sway while standing (tilt 0.17° → 0.09° rms in that band), but the robot started to wander slowly (position 0.02 → 0.15 rad rms below 0.3 Hz). The likely cause is the large 2-per-turn part (~25 mA): it probably depends on the load on the bearings, which differs between lying down (where it's measured) and standing. The bumps that cause the sway are the 20–48-per-turn ones.

**Storage.** The table (1920 bytes and a 28-byte header with its own CRC) is kept in the settings flash page, after the settings. It belongs to the motor and magnet it was measured with: recalibrate after changing either (or after the stored FOC alignment changes direction, which the board detects: the table is then not applied).

**How the numbers were chosen.** The defaults (kp 20 A/rad, kd 0.08 A/(rad/s), 0.25 rad/s) come from a simulation of a GM3506 wheel (inertia 1.4×10⁻⁵ kg·m², Kt 0.054 N·m/A, 1.5 mN·m friction) with a 2 ms current loop and the AS5048A's 14-bit resolution; the firmware's calibration code was run against the same model. With cogging of 3 mN·m amplitude at 264 per turn the compensation cut the cogging current from 42 to 3.8 mA rms (2.6 after a refine) and the speed ripple of a PI speed loop at 1 rad/s from 66 % to 10 %; with the shape measured on e4a7d0, from 36 to 0.3 mA rms and from 110 % to 2 %. (The model doesn't have the real measurement noise; the filter is what deals with that.) Much stiffer position loops oscillate; much softer ones let the rotor snap from step to step instead of following.

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
| `MOTOR_VOLTAGE_LIMIT` | 12.0 V | Cap on the phase voltage range (driver.voltage_limit). With a 3S pack the duty limit below applies instead |
| `MOTOR_MAX_DUTY` | 0.92 | Largest PWM duty: the phase voltage range is this × the supply, followed live. Leaves low-side on-time for the DRV8316's current sense |
| `SUPPLY_TRACK_INTERVAL_MS` / `SUPPLY_TRACK_FILTER_S` | 20 ms / 0.5 s | How the supply voltage is followed while FOC runs |
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
| `src/cogging.*` | Anti-cogging: calibration, the table (stored after the settings) and the compensation |
| `src/serial_console.*` | Non-blocking USB serial command console |
| `src/hw/` | Part interfaces (`encoder.h`, `current_sense.h`, `supply.h`), the shared DRV8316 setup, `board_info` (capabilities, model) and the supply code every sensor shares |
| `src/parts/` | One file per part, compiled in when the board header selects it |
| `src/dfu.*` | Reboot into the ROM DFU bootloader (also with the watchdog running), DFU button |
| `src/clock_config.cpp` | 160 MHz clock tree from the 16 MHz HSE |
| `src/motor_test/` | Bench-test firmware (`_test` environments only) |
| `boards/<name>/` | Each board's `board_config.h` (pins, parts) and `board.ini` (build environments) |
| `lib/ACANFD_STM32_G431/` | FDCAN driver, a fork of ACANFD-STM32 trimmed to this board |

### Flash size limit

Settings live in the **last 2 KB flash page** (0x0801F800), so the firmware image must stay below **126 KB**. The firmware checks this at boot and before every save, and prints an error if the image overlaps the settings page. The page holds the settings (bytes 0–63) and the anti-cogging table (from byte 64).

`lib_archive = false` links every object file of every library, and some unused ones stay in the image because they create global objects at start-up. `scripts/skip_unused_lib_sources.py` leaves SimpleFOCDrivers' `comms/` folder out of the build (~2.9 KB). Firmware 1.7 is then about 123 KB (more with some toolchains: a macOS build came out ~0.7 KB bigger).

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

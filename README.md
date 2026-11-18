# BLDC-Driver: compact FOC motor drivers

Small brushless motor drivers built around an **STM32G431** microcontroller and a **TI DRV8316** three-phase driver with integrated MOSFETs. They run field-oriented control (FOC) with [SimpleFOC](https://github.com/simplefoc) and take current (torque) commands over CAN. They suit gimbal motors and small direct-drive wheels.

This repository holds the hardware for each board and one firmware that runs on all of them. Every board speaks the same CAN protocol, so different boards can share a bus and one controller.

## Boards

| | [BLDC-Driver v0.2](docs/boards/bldc_driver_v02.md) | [MiniFOC Mk2](docs/boards/minifoc_mk2.md) |
|---|---|---|
| | <img src="Images/bldc_driver_v02/pinout.png" width="300" alt="BLDC-Driver v0.2"> | <img src="Images/minifoc_mk2/render_top.png" width="220" alt="MiniFOC Mk2"> |
| **Form factor** | 39 × 36 mm rectangle, with the encoder on a break-away tab | 40 mm round, mounts on the back of the motor |
| **Encoder** | AS5048A | AS5047P |
| **Phase current sense** | DRV8316 low-side amplifiers | Inline shunts + INA241A3 |
| **Supply sensing** | Voltage (resistor divider) | Voltage and current (INA226) |
| **5 V supply** | DRV8316 buck | AP63205 buck; can run from USB alone |
| **Power input** | 2-pin, 5–35 V | XT30, 5–35 V |
| **CAN** | TJA1051T/3, 2 × JST-PH, jumper termination | ATA6561, JST-GH, solder-jumper termination |
| **Extras** | User button (DFU), SPI3 header | I²C and SPI3 headers, RGB LEDs, footprints for an IMU and a BME280 |
| **Firmware environment** | `bldc_driver_v02` | `minifoc_mk2` |

Each board page covers its features, pin mapping, connectors, jumpers and manufacturing files.

## Repository layout

| Path | Contents |
|---|---|
| [`Hardware/<board>/`](Hardware) | KiCad project, schematic PDF and production files (Gerbers, BOM, pick-and-place for JLCPCB) for each board |
| [`Images/<board>/`](Images) | Pinout diagram and renders |
| [`Firmware/`](Firmware) | PlatformIO firmware for every board: the CAN motor driver and a bench-test build |
| [`Firmware/boards/<board>/`](Firmware/boards) | Each board's pins, parts and build environments |
| [`Firmware/tools/can_test/`](Firmware/tools/can_test) | CAN bench test tools for the PC (GUI and command line) |
| [`docs/`](docs) | Documentation (see below) |

Folders are named after the board and revision (`bldc_driver_v02`, `minifoc_mk2`) in all of these places.

## Documentation

- **Boards:** [BLDC-Driver v0.2](docs/boards/bldc_driver_v02.md), [MiniFOC Mk2](docs/boards/minifoc_mk2.md). Features, pin mapping, connectors, jumpers, buttons, LEDs and manufacturing notes.
- **[Firmware guide](docs/firmware.md)**: building, flashing, first-time configuration, the serial console, tuning parameters, the bench-test firmware, and how boards and parts are defined (including adding a new board).
- **[Test routines](docs/test-routines.md)**: step-by-step bench testing of a board, motor and encoder with the `motor_test` firmware, with expected readings and troubleshooting.
- **[CAN protocol](docs/can-protocol.md)**: message IDs, frame formats, status and error codes, protocol versions, plus an example host program.
- **[CAN test tools](docs/can-tools.md)**: a GUI and a command-line tool to drive, test, monitor and configure boards over CAN from a PC.

## Quick start

1. **Install [PlatformIO](https://platformio.org/)** (the VS Code extension or the CLI). In VS Code, open the `Firmware/` folder.
2. **Put the board in DFU mode.** Hold **BOOT**, then plug in USB-C or tap **RST**, then release **BOOT**. On a board that's already running the firmware, you can type `dfu` in the serial console instead.
3. **Build and flash** the environment for your board from the `Firmware/` directory:
   ```bash
   cd Firmware
   ```
   ```bash
   pio run -e bldc_driver_v02 -t upload      # BLDC-Driver v0.2
   pio run -e minifoc_mk2 -t upload          # MiniFOC Mk2
   ```
   In VS Code, pick the environment in the status bar first. The default environment is `bldc_driver_v02`.
4. **Configure the board** over USB serial. The board boots unconfigured, with CAN disabled:
   ```bash
   pio device monitor
   ```
   Then type:
   ```text
   set node 1         # 1-15, sets the CAN IDs (or: set side left / right = node 1 / 2)
   set invert off     # optional: flip the sign of commands/telemetry for a mirrored motor
   save               # writes to flash and reboots
   ```
5. **Drive it over CAN** at 1 Mbit/s:
   - Send `0x10` to the board's command ID (`0x020` + node, e.g. `0x021` for node 1) to align and start FOC.
   - Wait for the `[0x10][1.0f]` reply.
   - Stream target currents in amps as two big-endian floats on ID `0x010` (nodes 1 and 2; `0x011` for 3 and 4, and so on) at least every 100 ms.

   See [docs/can-protocol.md](docs/can-protocol.md) for the full protocol, or try it from a PC with the [CAN test tools](docs/can-tools.md).

> [!CAUTION]
> Test new motors with the wheels off the ground and a current-limited supply. The `motor_test` bench firmware ([docs/firmware.md](docs/firmware.md#bench-test-firmware-motor_test)) checks the encoder, power stage and current sense step by step before you run closed-loop control.

The firmware's default motor parameters (11 pole pairs, 2 A current limit, 10 V voltage limit) live in [`Firmware/include/motor_config.h`](Firmware/include/motor_config.h). Change them to match your motor.

## License

This project is shared under the terms of the [GNU Affero General Public License Version 3](https://choosealicense.com/licenses/agpl-3.0/) as published by the Free Software Foundation.

The bundled [`ACANFD_STM32_G431`](Firmware/lib/ACANFD_STM32_G431) library is a stripped-down fork of Pierre Molinaro's [ACANFD-STM32](https://github.com/pierremolinaro/acanfd-stm32) and keeps its own MIT license.

The CAN test tools bundle ElmüSoft's Candlelight Python classes in [`Firmware/tools/can_test/elmue`](Firmware/tools/can_test/elmue), unmodified, under their own BSD 3-Clause license.

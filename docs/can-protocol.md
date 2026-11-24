# CAN protocol

The definitions live in [`Firmware/include/can_protocol.h`](../Firmware/include/can_protocol.h). To try the protocol from a PC, use the [CAN test tools](can-tools.md).

## Protocol versions

The protocol version is reported by `CMD_GET_INFO`, separately from the firmware version:

| Protocol | Firmware | Adds |
|---|---|---|
| 1 | 1.0 | Start FOC, current commands, velocity telemetry, status heartbeat, errors. Left/right boards |
| 2 | 1.1 | Configuration over CAN (`CMD_GET_INFO` … `CMD_REBOOT`); the supply voltage in the heartbeat |
| 3 | 1.2 | Node IDs 1–15 instead of left/right (the IDs below) |
| 4 | 1.3 | `CMD_GET_BOARD_INFO` (model name, capabilities); input current in the heartbeat on boards with `CAP_MEASURES_INPUT_CURRENT` |
| 5 | 1.4 | `CMD_GET_PARAM`; current loop tuning (`PARAM_CURRENT_P/I/TF`) and the FOC loop rate (`PARAM_FOC_LOOP_HZ`) |
| 6 | 1.6 | The measured q and d currents in `CMD_SHAFT_VELOCITY` (8 bytes) |

## Basics


- **Bus:** classic CAN 2.0, standard 11-bit IDs, **1 Mbit/s**, 87.5 % sample point (no FD or bit-rate switching).
- **Floats:** IEEE-754 single precision, **big-endian** (most significant byte first).
- **Units:** current in amps, velocity in rad/s (shaft).
- Each board has a **node ID** from 1 to 15, so up to 15 boards can share a bus. A board starts CAN only after its node ID has been saved, over the [serial console](firmware.md#first-time-setup) or (once it has one) over CAN.

## Identifiers

For node *n*:

| ID | Direction | Content |
|---|---|---|
| `0x010 + (n−1)/2` | Host → boards | Target currents: 8 bytes = two floats, one for an odd node and one for the next even node |
| `0x020 + n` | Host → board | Command frame `[CMD][payload…]` |
| `0x100 + n` | Board → host | Telemetry, status and reply frames `[CMD][payload…]` |

(`(n−1)/2` is integer division.)

| Node | Current frame (bytes) | Commands | Telemetry |
|---|---|---|---|
| 1 | `0x010` (0–3) | `0x021` | `0x101` |
| 2 | `0x010` (4–7) | `0x022` | `0x102` |
| 3 | `0x011` (0–3) | `0x023` | `0x103` |
| 4 | `0x011` (4–7) | `0x024` | `0x104` |
| … | … | … | … |
| 15 | `0x017` (0–3) | `0x02F` | `0x10F` |

The current frames have the lowest IDs, so they win arbitration over every other frame on the bus. The hardware filter on each board accepts only its own current frame and its own command ID.

**Upgrading from firmware 1.0/1.1:** those versions used left/right instead of node IDs, with the currents on `0x020` and the right board's telemetry on `0x100`. A board saved as left comes up as node 1 and right as node 2 (the saved byte is the same). Host code needs the new current frame (`0x010`) and right telemetry (`0x102`) IDs.

## Host → board

### Target current, ID `0x010 + (n−1)/2` (8 bytes)

| Bytes | Content |
|---|---|
| 0–3 | Target current for the odd node (float, A) |
| 4–7 | Target current for the even node (float, A) |

- **NaN means "no command" for that node.** Use it to drive one node of a pair without touching the other: the board ignores it, and it doesn't count as a command for the timeout.
- Frames whose length isn't exactly 8 bytes are ignored, and so are NaN and Inf values.
- Values are clamped to ±`MOTOR_CURRENT_LIMIT` (2 A by default).
- If the board's `invert` setting is on, the sign is flipped on the driver.
- Send this frame **at least every 100 ms** while running. Otherwise the command timeout applies 0 A until frames resume.

### Configuration commands (protocol version 2 and later)

These are sent on the board's command ID (`0x021` or `0x022`). The reply comes back on its telemetry ID with the same command byte. Firmware 1.0 (protocol 1) ignores them and doesn't reply, so a host can use `CMD_GET_INFO` to find out which it's talking to.

| CMD | Name | Request | Reply |
|---|---|---|---|
| `0x20` | `CMD_GET_INFO` | `[0x20]` | `[0x20][protocol][fw major][fw minor][fw patch][image KB][0][0]` |
| `0x21` | `CMD_GET_CONFIG` | `[0x21][0 = active, 1 = pending]` | `[0x21][which][node ID, 0 = not set][config flags][zero electric angle, float]` |
| `0x22` | `CMD_SET_PARAM` | `[0x22][param][float value]` | `[0x22][param][status][float value now]` |
| `0x23` | `CMD_CALIBRATION` | `[0x23][1 = store, 2 = clear]` | `[0x23][op][status]` |
| `0x24` | `CMD_SAVE` | `[0x24]` | `[0x24][status]`, then (if OK) the board stops the motor, writes flash and reboots |
| `0x25` | `CMD_REVERT` | `[0x25]` | `[0x25][status]`: unsaved changes discarded |
| `0x26` | `CMD_REBOOT` | `[0x26]` | `[0x26][status]`, then the board resets (the motor stops) |
| `0x27` | `CMD_GET_BOARD_INFO` (protocol 4) | `[0x27][page]` | see [Board description](#board-description-cmd_get_board_info-protocol-4) |
| `0x28` | `CMD_GET_PARAM` (protocol 5) | `[0x28][param]` | `[0x28][param][status][float value]`: saved parameters read the pending value, the others the value in use |
| | any other | | `[0xFE][command][1]` (`CMD_NACK`: not understood) |

**Parameters (`CMD_SET_PARAM`, `CMD_GET_PARAM`):**

| ID | Name | Values | Kind |
|---|---|---|---|
| 1 | `PARAM_NODE_ID` | 1–15 | Saved. **The board's CAN IDs change after the reboot.** Don't give two boards the same ID: they can then no longer be told apart (fix it over USB with `set node`) |
| 2 | `PARAM_INVERT` | 0 / 1 | Saved |
| `0x10` | `PARAM_VELOCITY_INTERVAL_MS` | 1–100 (default 10) | Runtime: applies at once and goes back to the default after a reset |
| `0x20` | `PARAM_CURRENT_P` | 0–50 V/A (default 3) | Tuning (protocol 5): current loop P, both axes |
| `0x21` | `PARAM_CURRENT_I` | 0–100 000 V/(A·s) (default 300) | Tuning: current loop I, both axes |
| `0x22` | `PARAM_CURRENT_TF` | 0–0.01 s (default 0.00053) | Tuning: time constant of the filter on the measured currents |
| `0x30` | `PARAM_FOC_LOOP_HZ` | read only | FOC loop rate while running, Hz (0 = not running). `CMD_SET_PARAM` answers status 1 |

**Tuning** parameters apply at once (also while the motor runs) **and** change the pending settings: `CMD_SAVE` keeps them, `CMD_REVERT` goes back to the saved values. See [Current loop](firmware.md#current-loop).

Saved parameters and `CMD_CALIBRATION` change the **pending** settings, exactly like the serial console's `set` and `cal`. They take effect after `CMD_SAVE`. `CMD_CALIBRATION` store needs FOC to be running (it keeps this boot's alignment); clear makes the next start run the sensor alignment.

**Status byte:** 0 = OK, 1 = unknown command, parameter or operation, 2 = value out of range, 3 = no alignment to store (FOC not running), 4 = flash: the firmware image overlaps the settings page.

**Config flags (`CMD_GET_CONFIG`):** bit 0 = invert, bit 1 = an alignment is stored, bit 2 = stored sensor direction is CCW, bit 3 = unsaved changes (pending differs from active).

`CMD_SAVE` and `CMD_REBOOT` stop the motor. A host must not send them while the robot is balancing.

#### Re-running the sensor alignment

1. Send `CMD_CALIBRATION` clear, then `CMD_SAVE`. The board reboots.
2. Start FOC as normal. With no stored alignment it runs the sensor alignment, so the wheel moves a little.
3. Send `CMD_CALIBRATION` store, then `CMD_SAVE`. The board reboots and from then on uses the new alignment.

### `CMD_START_FOC`, `0x10`, on the board's command ID

Payload: `[0x10]` (any extra bytes are ignored).

This frame starts the motor: initialisation plus sensor alignment, which blocks for about 1–3 s. Alignment is skipped if a calibration is stored. The driver replies with a `CMD_START_FOC` frame. Sending the command again while FOC is already running simply replies "ok" again.

## Driver → host

All frames are sent on the board's telemetry ID.

| CMD | Name | Payload | When |
|---|---|---|---|
| `0x02` | `CMD_SHAFT_VELOCITY` | float rad/s, then (protocol 6) the measured q current as int16 mA and the measured d current as int8 in 16 mA steps (±2.03 A) | Every 10 ms while FOC runs (`PARAM_VELOCITY_INTERVAL_MS` changes this) |
| `0x03` | `CMD_DRIVER_STATUS` | `[state][flags][supply mV, 2 bytes]`, then `[input mA, 2 bytes]` on boards with `CAP_MEASURES_INPUT_CURRENT` | Every 100 ms, in every state |
| `0x10` | `CMD_START_FOC` | float `1.0` = started, `0.0` = failed | After each start request |
| `0xFF` | `CMD_ERROR` | float error code | On an error |

These frames are 5 bytes long (one command byte plus four payload bytes), with two exceptions:
- the status frame of a board with `CAP_MEASURES_INPUT_CURRENT` is 7 bytes;
- the velocity frame is 8 bytes from protocol 6.

Hosts should check the length; anything past the bytes they know about can be ignored.

**Velocity frame currents (protocol 6).** These are the measured currents the current loop works with, after its filter. They are not the commands. The velocity and the q current follow the invert setting, so they have the same sign as the current commands; the d current is as measured. With a good current loop, q follows the command until the motor runs out of voltage and d stays near 0. A noisy or offset reading shows up here first. Configuration replies (see above) are also sent on the telemetry ID.

### Driver state (status byte 1)

| Value | State | Meaning |
|---|---|---|
| 0 | `UNCONFIGURED` | Never sent: CAN is off until the board is configured |
| 1 | `IDLE` | Waiting for `CMD_START_FOC` |
| 2 | `RUNNING` | FOC running and accepting current commands |
| 3 | `FAULT` | FOC start failed. Send `CMD_START_FOC` to retry |

### Supply voltage (status bytes 3–4)

The driver's supply (battery) voltage in millivolts, as an unsigned 16-bit big-endian integer. `0` means there is no reading yet; firmware from before this field was added always sends `0`. Both drivers on a robot normally share one battery, so the host can average the two readings.

How it's measured depends on the board's supply sensor (see the [firmware guide](firmware.md#boards)). BLDC-Driver uses the V_SENS divider, low-pass filtered with a time constant of about 200 ms; its accuracy depends on `VSENS_CALIBRATION` in the board header. MiniFOC uses an INA226, read every 100 ms.

### Input current (status bytes 5–6, `CAP_MEASURES_INPUT_CURRENT` only)

The current the board draws from its supply in milliamps, as a signed 16-bit big-endian integer. It is positive when the board is drawing current and negative when the motor regenerates into the supply. `-32768` (`0x8000`) means there is no reading. Boards without `CAP_MEASURES_INPUT_CURRENT` send a 5-byte status frame without these bytes.

### Board description (`CMD_GET_BOARD_INFO`, protocol 4)

With this command a host can show and use any board without keeping its own list of boards. Send `[0x27][page]` to the board's command ID:

| Page | Reply |
|---|---|
| 0 | `[0x27][0][capabilities, uint32 big-endian][hardware rev][0]` |
| 1–5 | `[0x27][page][6 characters of the model name]`: characters 0–5, 6–11 and so on, NUL-padded after the end |
| other | `[0xFE][0x27][2]` (bad value) |

To read the model name, request pages 1, 2, … and stop at the first page that contains a NUL. A name is at most 30 characters.

Capability bits:

| Bit | Name | Meaning |
|---|---|---|
| 0 | `CAP_MEASURES_SUPPLY_VOLTAGE` | The heartbeat supply voltage is a measurement |
| 1 | `CAP_MEASURES_INPUT_CURRENT` | The heartbeat carries the input current (bytes 5–6) |

New bits are added when a board gains something a host can use. A host should ignore bits it doesn't know. It should decide what to expect from the capability bits, never from the model name. Firmware before protocol 4 answers `CMD_GET_BOARD_INFO` with a NACK.

### Status flags (status byte 2)

| Bit | Flag | Meaning |
|---|---|---|
| 0 | `COMMAND_TIMEOUT` | No current command for 100 ms, so 0 A is applied |
| 1 | `CALIBRATION_USED` | The stored alignment was used and `initFOC` alignment was skipped |
| 2 | `INVERTED` | The direction invert setting is on |
| 3 | `DRIVER_FAULT` | The DRV8316 reported a fault at the last poll |

### Error codes (`CMD_ERROR` payload)

| Code | Name | Meaning |
|---|---|---|
| 1 | `ERR_MOTOR_INIT` | SimpleFOC `motor.init()` failed |
| 2 | `ERR_CURRENT_SENSE` | Current-sense initialisation failed |
| 3 | `ERR_FOC_INIT` | `initFOC()` failed (sensor or current-sense alignment) |
| 4 | `ERR_DRIVER_FAULT` | DRV8316 fault flag set (over-current, over-temperature, undervoltage and so on; the details are printed on the serial console) |

## Typical sequence

1. Wait for a `CMD_DRIVER_STATUS` frame with state `IDLE` from each board.
2. Send `[0x10]` to each board's command ID (`0x021`, `0x022`, …).
3. Wait for the `[0x10][1.0]` reply from each board.
4. Stream the current frames (`0x010` for nodes 1 and 2) with target currents (for example at 100–1000 Hz). Read `CMD_SHAFT_VELOCITY` for feedback.
5. To stop, send 0 A or stop sending. After 100 ms each driver applies 0 A on its own.

## Example host (Python, `python-can`)

This example uses Linux SocketCAN, set up first with `ip link set can0 up type can bitrate 1000000`.

```python
import struct
import time

import can

# Nodes 1 and 2: commands 0x020 + n, telemetry 0x100 + n, currents in frame 0x010.
LEFT_CMD, RIGHT_CMD, CURRENTS = 0x021, 0x022, 0x010
LEFT_TLM, RIGHT_TLM = 0x101, 0x102
CMD_SHAFT_VELOCITY, CMD_DRIVER_STATUS, CMD_START_FOC, CMD_ERROR = 0x02, 0x03, 0x10, 0xFF

bus = can.Bus(interface="socketcan", channel="can0")

def send(arbitration_id, data):
    bus.send(can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False))

# Start FOC on both drivers and wait for their replies.
send(LEFT_CMD, [CMD_START_FOC])
send(RIGHT_CMD, [CMD_START_FOC])
started = set()
deadline = time.time() + 5
while len(started) < 2 and time.time() < deadline:
    msg = bus.recv(timeout=0.1)
    if msg and msg.arbitration_id in (LEFT_TLM, RIGHT_TLM) and msg.data[0] == CMD_START_FOC:
        ok = struct.unpack(">f", bytes(msg.data[1:5]))[0] == 1.0
        print(hex(msg.arbitration_id), "started" if ok else "FAILED")
        started.add(msg.arbitration_id)

# Command 0.3 A on both motors at 200 Hz and print the velocities.
while True:
    send(CURRENTS, struct.pack(">ff", 0.3, 0.3))  # use float("nan") to leave a node alone
    msg = bus.recv(timeout=0.005)
    if msg and msg.data[0] == CMD_SHAFT_VELOCITY:
        side = "L" if msg.arbitration_id == LEFT_TLM else "R"
        print(side, struct.unpack(">f", bytes(msg.data[1:5]))[0], "rad/s")
    elif msg and msg.data[0] == CMD_DRIVER_STATUS:
        side = "L" if msg.arbitration_id == LEFT_TLM else "R"
        print(side, "supply", struct.unpack(">H", bytes(msg.data[3:5]))[0] / 1000, "V")
    time.sleep(0.005)
```

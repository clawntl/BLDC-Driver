# CAN protocol

The definitions live in [`Firmware/include/can_protocol.h`](../Firmware/include/can_protocol.h).

- **Bus:** classic CAN 2.0, standard 11-bit IDs, **1 Mbit/s**, 87.5 % sample point (no FD or bit-rate switching).
- **Floats:** IEEE-754 single precision, **big-endian** (most significant byte first).
- **Units:** current in amps, velocity in rad/s (shaft).
- A board starts CAN only after its side (left or right) has been saved over the [serial console](firmware.md#first-time-setup).

## Identifiers

| ID | Direction | Used by | Content |
|---|---|---|---|
| `0x020` | Host → both drivers | Left and right | Target currents: 8 bytes = two floats |
| `0x021` | Host → driver | Left | Command frame `[CMD][payload…]` |
| `0x022` | Host → driver | Right | Command frame |
| `0x101` | Driver → host | Left | Telemetry and status frames `[CMD][payload…]` |
| `0x100` | Driver → host | Right | Telemetry and status frames |

The hardware filters on each driver accept only `0x020` and its own command ID.

## Host → driver

### Target current, ID `0x020` (8 bytes)

| Bytes | Content |
|---|---|
| 0–3 | Left motor target current (float, A) |
| 4–7 | Right motor target current (float, A) |

- Frames whose length isn't exactly 8 bytes are ignored, and so are NaN and Inf values.
- Values are clamped to ±`MOTOR_CURRENT_LIMIT` (2 A by default).
- If the board's `invert` setting is on, the sign is flipped on the driver.
- Send this frame **at least every 100 ms** while running. Otherwise the command timeout applies 0 A until frames resume.

### `CMD_START_FOC`, `0x10`, on the board's command ID

Payload: `[0x10]` (any extra bytes are ignored).

This frame starts the motor: initialisation plus sensor alignment, which blocks for about 1–3 s. Alignment is skipped if a calibration is stored. The driver replies with a `CMD_START_FOC` frame. Sending the command again while FOC is already running simply replies "ok" again.

## Driver → host

All frames are sent on the board's telemetry ID.

| CMD | Name | Payload | When |
|---|---|---|---|
| `0x02` | `CMD_SHAFT_VELOCITY` | float rad/s (sign follows the invert setting) | Every 10 ms while FOC runs |
| `0x03` | `CMD_DRIVER_STATUS` | `[state][flags][0][0]` | Every 100 ms, in every state |
| `0x10` | `CMD_START_FOC` | float `1.0` = started, `0.0` = failed | After each start request |
| `0xFF` | `CMD_ERROR` | float error code | On an error |

All frames are 5 bytes long: one command byte plus four payload bytes.

### Driver state (status byte 1)

| Value | State | Meaning |
|---|---|---|
| 0 | `UNCONFIGURED` | Never sent: CAN is off until the board is configured |
| 1 | `IDLE` | Waiting for `CMD_START_FOC` |
| 2 | `RUNNING` | FOC running and accepting current commands |
| 3 | `FAULT` | FOC start failed. Send `CMD_START_FOC` to retry |

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
2. Send `[0x10]` to `0x021` and `0x022`.
3. Wait for the `[0x10][1.0]` reply from each board.
4. Stream `0x020` frames with target currents (for example at 100–1000 Hz). Read `CMD_SHAFT_VELOCITY` for feedback.
5. To stop, send 0 A or stop sending. After 100 ms each driver applies 0 A on its own.

## Example host (Python, `python-can`)

This example uses Linux SocketCAN, set up first with `ip link set can0 up type can bitrate 1000000`.

```python
import struct
import time

import can

LEFT_CMD, RIGHT_CMD, BOTH = 0x021, 0x022, 0x020
LEFT_TLM, RIGHT_TLM = 0x101, 0x100
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
    send(BOTH, struct.pack(">ff", 0.3, 0.3))
    msg = bus.recv(timeout=0.005)
    if msg and msg.data[0] == CMD_SHAFT_VELOCITY:
        side = "L" if msg.arbitration_id == LEFT_TLM else "R"
        print(side, struct.unpack(">f", bytes(msg.data[1:5]))[0], "rad/s")
    time.sleep(0.005)
```

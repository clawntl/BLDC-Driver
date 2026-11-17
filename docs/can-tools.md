# CAN test tools

[`Firmware/tools/can_test`](../Firmware/tools/can_test) holds two bench tools that talk to boards running this firmware over CAN. They work with any board (BLDC-Driver, MiniFOC and later ones), because each board reports its own model name and capabilities.

| Tool | Use it for |
|---|---|
| `cantest_gui.py` | Driving two boards side by side, watching a bus with the robot's controller on it, and changing board settings |
| `cantest.py` | Automatic pass/fail tests, scripted settings changes, and decoding the bus in a terminal |

Both need only Python 3 with tkinter (included in the python.org installers). They don't need any pip packages unless you use the python-can backend.

> [!CAUTION]
> The motors **will** spin. Clamp them or lift the wheels before you start FOC or run a test.

## Adapters

| Backend | Adapter | Setup |
|---|---|---|
| `elmue` (default) | A Candlelight adapter with ElmüSoft's CANable 2.5 firmware, e.g. a WeAct USB2CANFD v2 | None. ElmüSoft's Python classes are bundled in `elmue/`. Only one program can open the adapter at a time |
| `python-can` | Any adapter [python-can](https://python-can.readthedocs.io/) supports (slcan, socketcan, …) | `pip install python-can`, then `--backend python-can --interface slcan --channel COM5` |

The bit timing defaults to what the firmware uses: 1 Mbit/s with an 87.5 % sample point, classic CAN.

## GUI

```bash
cd Firmware/tools/can_test
python cantest_gui.py
```

1. Press **Refresh** and pick the adapter.
2. Set the node IDs of the two panels: by default node 1 on the left and node 2 on the right.
3. Choose a mode, then press **Connect**:
   - **Control**: the GUI is the controller, so unplug the robot's controller first. Each panel has **Start FOC**, **Send** and a current slider; **Start both** and **Link sliders** drive both motors together. A panel with **Send** off sends NaN ("no command"), so that board times out to 0 A. **STOP** (or Esc) sends 0 A to both boards and stops sending.
   - **Monitor**: the robot's controller stays on the bus and in charge, and the GUI sends nothing. It shows what the controller commands each node next to that board's state, flags, supply and velocity. **Listen-only** keeps the adapter completely silent; it doesn't even acknowledge frames.
4. **Info** reads the board's model name, firmware and protocol version. On boards that measure input current (capability `CAP_INPUT_CURRENT`), the current is shown next to the supply voltage.

**Auto test** and **Bus-off test** run the same sequences as the command line, for one panel at a time.

### Changing a board's settings

Press **Settings...** on the board's panel (Control mode):

- **Set node**, **Set invert** and the **Alignment** buttons change the *pending* settings.
- **Save & reboot** writes them to flash. If the node ID changed, the panel follows the board to its new ID.
- **Revert** discards unsaved changes.
- **Re-run alignment...** walks you through clearing the stored alignment, letting Start FOC align, and storing the result.

## Command line

```bash
python cantest.py --node 2 auto                   # automatic test sequence, PASS / FAIL
python cantest.py --node 1 info                   # board, firmware, protocol, settings
python cantest.py --side left run --current 0.5   # hold 0.5 A until Ctrl+C
python cantest.py monitor                         # decode every frame on the bus
python cantest.py monitor --listen-only           # ... without acknowledging frames
python cantest.py --node 1 busoff                 # bus-off recovery test (asks you to unplug CAN)
python cantest.py list                            # connected adapters
```

`--node 1-15` selects the board, and `--side left|right` is shorthand for node 1 / 2. `python cantest.py -h` lists every option.

> [!NOTE]
> `auto` also sends current to the board's partner node (the other half of its current frame: 1/2, 3/4, …) to check that the board ignores it. Don't have that partner on the bus while `auto` runs.

### Changing settings

```bash
python cantest.py --node 1 config --set-node 3 --save      # node 1 becomes node 3
python cantest.py --node 3 config --set-invert on --save
python cantest.py --node 3 config --cal clear --save       # the next Start FOC aligns...
python cantest.py --node 3 config --cal store --save       # ...and this keeps that alignment
python cantest.py --node 3 config --revert                 # discard unsaved changes
```

Changes stay pending until `--save`, which stops the motor, writes flash and reboots the board. A board that has never been configured has CAN switched off, so give it its first node ID over USB serial (see the [firmware guide](firmware.md#first-time-setup)).

Never give two boards on one bus the same node ID: their frames can't be told apart, and only USB serial can fix it. Both tools refuse a node ID that is already in use.

## Older firmware

| Firmware (protocol) | What works |
|---|---|
| 1.3+ (4) | Everything, including the model name, capabilities and input current |
| 1.2 (3) | Everything except the board description. Info shows "board not reported" |
| 1.1 (2) | Uses left/right CAN IDs, which these tools no longer speak. Update the firmware |
| 1.0 (1) | No configuration over CAN. Update the firmware over USB |

## License

ElmüSoft's Candlelight classes in `elmue/` are bundled unmodified and keep their own BSD 3-Clause license (`elmue/License.txt`).

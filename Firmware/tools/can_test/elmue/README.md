# ElmüSoft Candlelight Python classes (third-party)

These files are ElmüSoft's Python sample application for the CANable 2.5 firmware. They're bundled here so that `minifoc_cantest.py` and `minifoc_cantest_gui.py` work without any extra setup.

- **Source:** [Elmue/CANable-2.5-firmware-Slcan-and-Candlelight](https://github.com/Elmue/CANable-2.5-firmware-Slcan-and-Candlelight), folder `SampleApplication Python/Source`, commit `12aa9fa` (2026-10-03).
- **License:** BSD 3-Clause, Copyright ElmüSoft. See [License.txt](License.txt). This license applies to the files in this folder only. The rest of the MiniFOC repository is AGPL-3.0.
- **Changes:** none. The files are as published. Only their line endings may differ.

## Known issues

The test tool works around two issues in `OsLibrary_Windows.py` at runtime, so the files in this folder stay unmodified:

1. **64-bit Python, adapter listing:** `EnumDevices()` fails with "API Error 6: The handle is invalid". The SetupAPI functions are called without ctypes `restype`/`argtypes`, so the `HDEVINFO` handle is truncated to 32 bits. The tool's fix is `_fix_elmue_setupapi_types()` in `minifoc_cantest.py`.
2. **Crash after closing the adapter:** `Close()` lets the receive thread exit while its overlapped `WinUsb_ReadPipe()` is still pending. Cancelling that read then writes into the thread's freed `OVERLAPPED` structure, and the process crashes with an access violation on a later open or soon after. The tool's fix is `ElmueBus._stop_pipe_thread()` in `minifoc_cantest.py`.

To update these files, copy the newer versions from upstream and update the commit above. If upstream fixes either issue, the corresponding workaround can be removed.

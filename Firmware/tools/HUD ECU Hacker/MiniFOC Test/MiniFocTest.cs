/*******************************************************************************
Macro script for HUD ECU Hacker - CAN Raw Terminal
MiniFOC motor driver bench test over CAN.

Install: copy the whole "MiniFOC Test" folder into
    C:\Program Files (x86)\HUD ECU Hacker\Macros\
then restart HUD ECU Hacker. The macro appears in the CAN Raw Terminal.

Before running:
  - Open the CAN Raw Terminal at 1 Mbit/s, classic CAN (no data bitrate),
    87.5 % sample point, 11 bit IDs.
  - "Send always 8 data bytes" must be OFF (the short-frame test needs DLC 4).
  - Show all packets / no filter, or a filter that passes 0x100 and 0x101.
  - Only ONE MiniFOC on the bus for "Auto test": it sends current to the
    other motor's half of 0x020 to check that this board ignores it.
  - Clamp the motor or lift the wheel: it WILL spin.

Modes:
  Auto test       Runs the CAN test sequence and prints PASS / FAIL lines.
  Manual current  Starts FOC if needed, then sends the entered current every
                  <Interval> ms until Stop is pressed. Prints the mean
                  velocity, state and flags once per second.

Protocol: see Firmware/include/can_protocol.h in the MiniFOC repo.
*******************************************************************************/

// ===============================================================================
//                            INTERNAL CLASS Packet
// ===============================================================================

public class Packet
{
    public int    CanID;  // the CAN bus ID (11 bit or 29 bite)
    public bool   b29Bit; // true --> ID has 29 bit, false --> ID has 11 bit.
    public bool   bRTR;   // Remote Transmission Request: a frame without data, rarely used. Only valid if bFDF = false
    public bool   bFDF;   // Flexible Data Frame: (CAN FD) allow up to 64 data bytes and use a longer CRC
    public bool   bBRS;   // Bit Rate Switching:  (CAN FD) use a higher baudrate for the data bytes. Only valid if bFDF = true
    public byte[] Data;   // the data bytes (mostly 8 bytes)

    // Constructor
    public Packet(int p_CanID, bool p_b29Bit, params byte[] p_Data)
    {
        b29Bit = p_b29Bit;
        CanID  = p_CanID;
        Data   = p_Data;
    }
}

// ===============================================================================
//                          MiniFOC CAN protocol
// ===============================================================================

const int BOTH_MOTOR_ID      = 0x020;

// initFOC() blocks: motor init, current sense offset calibration, encoder
// alignment and current sense alignment. Can take well over 5 s on Mk2.
const int START_TIMEOUT_MS   = 20000;

const int CMD_SHAFT_VELOCITY = 0x02;
const int CMD_DRIVER_STATUS  = 0x03;
const int CMD_START_FOC      = 0x10;
const int CMD_ERROR          = 0xFF;

const int STATE_UNCONFIGURED = 0;
const int STATE_IDLE         = 1;
const int STATE_RUNNING      = 2;
const int STATE_FAULT        = 3;

const int FLAG_TIMEOUT       = 1;
const int FLAG_CALIBRATION   = 2;
const int FLAG_INVERTED      = 4;
const int FLAG_DRV_FAULT     = 8;

enum TxMode { Normal, None, Short, NaN }

class AbortedException : Exception { }

// ===============================================================================
//                                  State
// ===============================================================================

static int       s32_CmdID;
static int       s32_TeleID;
static int       s32_Offset;
static int       s32_Interval;
static Stopwatch i_Clock;

// Updated by ProcessRx()
static int   s32_LastState;
static int   s32_LastFlags;
static int   s32_FlagsSeen;      // OR of all flags since ResetWindow()
static int   s32_StatusCount;    // since ResetWindow()
static int   s32_VelCount;       // since ResetWindow()
static float f_VelSum;           // since ResetWindow()
static float f_LastVel;
static float f_StartReply;       // NaN until a CMD_START_FOC reply arrives
static List<int> i_Errors;
static bool  b_DrvFaultSeen;

static int s32_Pass;
static int s32_Fail;

// ===============================================================================
//                     PUBLIC FUNCTIONS (called from HUD ECU Hacker)
// ===============================================================================

public static void MiniFocTest(object[] Parameters)
{
    string s_Side     = (string)Parameters[0];
    string s_Mode     = (string)Parameters[1];
    string s_Current  = (string)Parameters[2];
    int    s32_Ms     = (int)   Parameters[3];

    float f_Current;
    if (!float.TryParse(s_Current.Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out f_Current))
        throw new Exception("Current must be a number in A, for example 0.3");
    if (Math.Abs(f_Current) > 5.0f)
        throw new Exception("Current must be between -5 and 5 A (the firmware clamps to 2 A anyway).");
    if (s32_Ms < 1 || s32_Ms > 90)
        throw new Exception("Interval must be 1..90 ms (the driver's command timeout is 100 ms).");

    bool b_Left  = (s_Side == "Left");
    s32_CmdID    = b_Left ? 0x021 : 0x022;
    s32_TeleID   = b_Left ? 0x101 : 0x100;
    s32_Offset   = b_Left ? 0 : 4;
    s32_Interval = s32_Ms;

    s32_LastState  = -1;
    s32_LastFlags  = 0;
    f_LastVel      = 0;
    f_StartReply   = float.NaN;
    i_Errors       = new List<int>();
    b_DrvFaultSeen = false;
    s32_Pass       = 0;
    s32_Fail       = 0;
    ResetWindow();
    i_Clock = Stopwatch.StartNew();

    PrintTrace("==== MiniFOC " + s_Side + ": commands 0x" + s32_CmdID.ToString("X3") +
               ", telemetry 0x" + s32_TeleID.ToString("X3") + ", " + s_Mode + " ====");

    try
    {
        if (s_Mode == "Auto test") AutoTest(f_Current);
        else                       ManualCurrent(f_Current);
    }
    catch (AbortedException)
    {
        PrintTrace("Stopped by user.");
    }

    // Leave the motor at 0 A (the driver's command timeout would do this too).
    try { SendCurrent(0, 0, TxMode.Normal); } catch (Exception) { }
}

// ===============================================================================
//                                  Tests
// ===============================================================================

static void AutoTest(float f_Current)
{
    float I = Math.Abs(f_Current);
    if (I < 0.05f) I = 0.3f;

    // ---- 1. Heartbeat ----
    ResetWindow();
    Run(1000, 0, 0, TxMode.None, null);
    if (s32_StatusCount == 0)
    {
        Check(false, "Status heartbeat", "nothing received on 0x" + s32_TeleID.ToString("X3") +
              ". Check: side setting (`config` on serial), bitrate/sample point, termination, GND, CAN Raw (not Sniff) terminal.");
        Summary();
        return;
    }
    Check(s32_StatusCount >= 8 && s32_StatusCount <= 12, "Status heartbeat",
          s32_StatusCount + " frames in 1 s (expect 10)");
    PrintTrace("Driver state: " + StateName(s32_LastState) + ", flags: " + FlagNames(s32_LastFlags));

    // ---- 2. Start FOC ----
    if (!StartFoc(START_TIMEOUT_MS, "Start FOC"))
    {
        Summary();
        return;
    }

    ResetWindow();
    Run(300, 0, 0, TxMode.None, null);
    Check(s32_LastState == STATE_RUNNING, "State is running after start", StateName(s32_LastState));
    Check((s32_FlagsSeen & FLAG_TIMEOUT) != 0, "Timeout flag set while no commands are sent", FlagNames(s32_FlagsSeen));

    // ---- 3. Zero current clears the timeout ----
    ResetWindow();
    Run(1000, 0, 0, TxMode.Normal, null);
    Check((s32_LastFlags & FLAG_TIMEOUT) == 0, "Current commands clear the timeout flag", FlagNames(s32_LastFlags));
    Check(s32_VelCount >= 80 && s32_VelCount <= 110, "Velocity telemetry rate",
          s32_VelCount + " frames in 1 s (expect ~100)");

    // ---- 4. Positive current ----
    Run(2000, I, 0, TxMode.Normal, null);
    ResetWindow();
    Run(1000, I, 0, TxMode.Normal, null);
    float f_Pos = VelAvg();
    Check(f_Pos > 2.0f, "Positive current spins forward",
          F(I) + " A -> mean " + F(f_Pos) + " rad/s (expect > 2)");

    // ---- 5. Negative current ----
    Run(2000, -I, 0, TxMode.Normal, null);
    ResetWindow();
    Run(1000, -I, 0, TxMode.Normal, null);
    float f_Neg = VelAvg();
    Check(f_Neg < -2.0f, "Negative current spins backward",
          F(-I) + " A -> mean " + F(f_Neg) + " rad/s (expect < -2)");

    // ---- 6. Other motor's half of 0x020 is ignored ----
    Run(3000, 0, 0, TxMode.Normal, null); // coast
    float f_V0 = Math.Abs(f_LastVel);
    Run(2000, 0, I, TxMode.Normal, null);
    float f_V1 = Math.Abs(f_LastVel);
    Check(f_V1 <= f_V0 + 1.0f, "Other motor's half of 0x020 is ignored",
          "|vel| " + F(f_V0) + " -> " + F(f_V1) + " rad/s while " + F(I) + " A sent to the other side");

    // ---- 7. Command timeout while spinning ----
    Run(2000, I, 0, TxMode.Normal, null);
    ResetWindow();
    Run(400, 0, 0, TxMode.None, null);
    Check((s32_FlagsSeen & FLAG_TIMEOUT) != 0, "Command timeout after 400 ms of silence", FlagNames(s32_FlagsSeen));
    ResetWindow();
    Run(500, 0, 0, TxMode.Normal, null);
    Check((s32_LastFlags & FLAG_TIMEOUT) == 0, "Timeout clears when commands resume", FlagNames(s32_LastFlags));

    // ---- 8. Short (4 byte) frames are rejected ----
    ResetWindow();
    Run(400, 0, 0, TxMode.Short, null);
    Check((s32_FlagsSeen & FLAG_TIMEOUT) != 0, "4-byte 0x020 frames are ignored",
          FlagNames(s32_FlagsSeen) + ((s32_FlagsSeen & FLAG_TIMEOUT) != 0 ? "" :
          " - is \"Send always 8 data bytes\" switched off?"));
    Run(500, 0, 0, TxMode.Normal, null);

    // ---- 9. NaN current is rejected ----
    ResetWindow();
    Run(400, 0, 0, TxMode.NaN, null);
    Check((s32_FlagsSeen & FLAG_TIMEOUT) != 0, "NaN current is ignored", FlagNames(s32_FlagsSeen));
    Run(500, 0, 0, TxMode.Normal, null);

    // ---- 10. START_FOC while running replies at once ----
    f_StartReply = float.NaN;
    long s64_T0 = i_Clock.ElapsedMilliseconds;
    SendPacket(new Packet(s32_CmdID, false, (byte)CMD_START_FOC));
    Run(1000, 0, 0, TxMode.Normal, delegate { return !float.IsNaN(f_StartReply); });
    long s64_Dt = i_Clock.ElapsedMilliseconds - s64_T0;
    Check(f_StartReply == 1.0f && s64_Dt < 200, "START_FOC while running replies immediately",
          (float.IsNaN(f_StartReply) ? "no reply" : "reply " + F(f_StartReply)) + " after " + s64_Dt + " ms");

    // ---- 11. Whole run ----
    Check(i_Errors.Count == 0, "No CMD_ERROR frames during the test", i_Errors.Count + " received");
    Check(!b_DrvFaultSeen, "No DRV8316 fault flag during the test", b_DrvFaultSeen ? "fault flag seen" : "none");

    Run(300, 0, 0, TxMode.Normal, null);
    Summary();
}

static void ManualCurrent(float f_Current)
{
    ResetWindow();
    Run(300, 0, 0, TxMode.None, null);
    if (s32_LastState != STATE_RUNNING)
    {
        if (!StartFoc(START_TIMEOUT_MS, "Start FOC"))
            return;
    }

    PrintTrace("Sending " + F(f_Current) + " A every " + s32_Interval + " ms. Press Stop to end.");
    while (true)
    {
        ResetWindow();
        Run(1000, f_Current, 0, TxMode.Normal, null);
        PrintTrace("vel " + F(VelAvg()) + " rad/s (" + s32_VelCount + " frames/s), state " +
                   StateName(s32_LastState) + ", flags " + FlagNames(s32_LastFlags));
    }
}

// Sends CMD_START_FOC and waits for the reply. Returns true if it reported success.
static bool StartFoc(int s32_TimeoutMs, string s_Name)
{
    f_StartReply = float.NaN;
    int s32_ErrBefore = i_Errors.Count;
    long s64_T0 = i_Clock.ElapsedMilliseconds;
    PrintTrace("Sending START_FOC, waiting up to " + s32_TimeoutMs / 1000 +
               " s (the driver sends nothing on CAN while it aligns)...");
    SendPacket(new Packet(s32_CmdID, false, (byte)CMD_START_FOC));
    Run(s32_TimeoutMs, 0, 0, TxMode.None, delegate { return !float.IsNaN(f_StartReply); });
    long s64_Dt = i_Clock.ElapsedMilliseconds - s64_T0;

    if (float.IsNaN(f_StartReply))
    {
        Check(false, s_Name, "no reply within " + s32_TimeoutMs + " ms");
        return false;
    }
    bool b_Ok = (f_StartReply == 1.0f);
    string s_Detail = "reply " + F(f_StartReply) + " after " + s64_Dt + " ms";
    if (!b_Ok && i_Errors.Count > s32_ErrBefore)
        s_Detail += ", error " + ErrorName(i_Errors[i_Errors.Count - 1]);
    Check(b_Ok, s_Name, s_Detail);
    return b_Ok;
}

// ===============================================================================
//                                  Helpers
// ===============================================================================

// Runs for s32_Ms milliseconds: sends the current command every s32_Interval ms
// (unless e_Mode is None), and processes everything received. Returns early if
// f_StopWhen returns true.
static void Run(int s32_Ms, float f_Mine, float f_Other, TxMode e_Mode, Func<bool> f_StopWhen)
{
    long s64_End    = i_Clock.ElapsedMilliseconds + s32_Ms;
    long s64_NextTx = i_Clock.ElapsedMilliseconds;
    while (i_Clock.ElapsedMilliseconds < s64_End)
    {
        if (UserAborted())
            throw new AbortedException();

        long s64_Now = i_Clock.ElapsedMilliseconds;
        if (e_Mode != TxMode.None && s64_Now >= s64_NextTx)
        {
            int s32_Free = CalcTxFreeSlots(); // -1 if the adapter can't tell
            if (s32_Free < 0 || s32_Free > 2)
                SendCurrent(f_Mine, f_Other, e_Mode);

            s64_NextTx += s32_Interval;
            if (s64_NextTx < s64_Now) s64_NextTx = s64_Now + s32_Interval;
        }

        ProcessRx();
        if (f_StopWhen != null && f_StopWhen())
            return;

        Thread.Sleep(1);
    }
}

static void SendCurrent(float f_Mine, float f_Other, TxMode e_Mode)
{
    byte[] u8_Data;
    if (e_Mode == TxMode.Short)
    {
        u8_Data = new byte[4];
        Array.Copy(FloatToBE(f_Mine), 0, u8_Data, 0, 4);
    }
    else
    {
        u8_Data = new byte[8];
        Array.Copy(FloatToBE(e_Mode == TxMode.NaN ? float.NaN : f_Mine), 0, u8_Data, s32_Offset,     4);
        Array.Copy(FloatToBE(f_Other),                                   0, u8_Data, 4 - s32_Offset, 4);
    }
    SendPacket(new Packet(BOTH_MOTOR_ID, false, u8_Data));
}

// Drains the Rx queue (HUD shows each packet in the Trace) and updates the state.
static void ProcessRx()
{
    while (true)
    {
        Packet i_Rx = ReceivePacket(0, false);
        if (i_Rx == null)
            break;
        if (i_Rx.b29Bit || i_Rx.CanID != s32_TeleID || i_Rx.Data == null || i_Rx.Data.Length < 1)
            continue;

        byte[] u8_D = i_Rx.Data;
        switch (u8_D[0])
        {
            case CMD_DRIVER_STATUS:
                if (u8_D.Length < 3) break;
                s32_LastState  = u8_D[1];
                s32_LastFlags  = u8_D[2];
                s32_FlagsSeen |= u8_D[2];
                s32_StatusCount++;
                if ((u8_D[2] & FLAG_DRV_FAULT) != 0) b_DrvFaultSeen = true;
                break;

            case CMD_SHAFT_VELOCITY:
                if (u8_D.Length < 5) break;
                f_LastVel = BEToFloat(u8_D, 1);
                f_VelSum += f_LastVel;
                s32_VelCount++;
                break;

            case CMD_START_FOC:
                if (u8_D.Length < 5) break;
                f_StartReply = BEToFloat(u8_D, 1);
                break;

            case CMD_ERROR:
                if (u8_D.Length < 5) break;
                int s32_Code = (int)BEToFloat(u8_D, 1);
                i_Errors.Add(s32_Code);
                PrintTrace("ERROR: driver reported " + ErrorName(s32_Code));
                break;
        }
    }
}

static void ResetWindow()
{
    s32_FlagsSeen   = 0;
    s32_StatusCount = 0;
    s32_VelCount    = 0;
    f_VelSum        = 0;
}

static float VelAvg()
{
    return s32_VelCount > 0 ? f_VelSum / s32_VelCount : 0;
}

static void Check(bool b_Ok, string s_Name, string s_Detail)
{
    if (b_Ok)
    {
        s32_Pass++;
        PrintTrace("PASS  " + s_Name + " - " + s_Detail);
    }
    else
    {
        s32_Fail++;
        PrintTrace("ERROR: FAIL  " + s_Name + " - " + s_Detail);
    }
}

static void Summary()
{
    PrintTrace("==== " + s32_Pass + " passed, " + s32_Fail + " failed ====");
}

static byte[] FloatToBE(float f_Val)
{
    byte[] u8_B = BitConverter.GetBytes(f_Val);
    if (BitConverter.IsLittleEndian) Array.Reverse(u8_B);
    return u8_B;
}

static float BEToFloat(byte[] u8_Data, int s32_Pos)
{
    byte[] u8_B = new byte[4];
    Array.Copy(u8_Data, s32_Pos, u8_B, 0, 4);
    if (BitConverter.IsLittleEndian) Array.Reverse(u8_B);
    return BitConverter.ToSingle(u8_B, 0);
}

static string F(float f_Val)
{
    return f_Val.ToString("0.00", CultureInfo.InvariantCulture);
}

static string StateName(int s32_State)
{
    switch (s32_State)
    {
        case STATE_UNCONFIGURED: return "unconfigured";
        case STATE_IDLE:         return "idle";
        case STATE_RUNNING:      return "running";
        case STATE_FAULT:        return "fault (FOC start failed)";
        case -1:                 return "unknown (no status received)";
        default:                 return "invalid (" + s32_State + ")";
    }
}

static string FlagNames(int s32_Flags)
{
    if (s32_Flags == 0) return "none";
    string s_Out = "";
    if ((s32_Flags & FLAG_TIMEOUT)     != 0) s_Out += "timeout ";
    if ((s32_Flags & FLAG_CALIBRATION) != 0) s_Out += "stored-alignment ";
    if ((s32_Flags & FLAG_INVERTED)    != 0) s_Out += "inverted ";
    if ((s32_Flags & FLAG_DRV_FAULT)   != 0) s_Out += "DRV-fault ";
    return s_Out.Trim();
}

static string ErrorName(int s32_Code)
{
    switch (s32_Code)
    {
        case 1:  return "error 1 (motor init failed)";
        case 2:  return "error 2 (current sense init failed)";
        case 3:  return "error 3 (FOC init / sensor alignment failed)";
        case 4:  return "error 4 (DRV8316 fault)";
        default: return "error " + s32_Code + " (unknown)";
    }
}

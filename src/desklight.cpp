// ============================================================
//  desklight.cpp  —  Desk Light Robot Controller
//  Hardware: Arduino MEGA ADK
//
//  Changes from original:
//    • PlatformIO project (this file is src/desklight.cpp)
//    • Animation storage: fixed-timestep stream → delta-time events
//    • MIDI knobs: position offset  →  per-servo P-gain
//    • MIDI faders 5/6/7: LED ring colour / brightness / pattern
//      LED ring is on ID 25; regs 30/32/34 = colour/brightness/pattern
//      Fader 0-127 maps linearly to 0..LED_MAX_VAL (no angleOffset applied)
//    • Auto-ranging: on startup each servo's CW/CCW angle limits are read
//      from Dynamixel registers 6/8; fader 0-127 is then mapped to that
//      range so full travel always uses the full fader.
//    • Standalone mode: no MIDI panel -> radar-triggered playback
//      (DFRobot SEN0192 10.525 GHz microwave sensor)
//
//  Pin assignments:
//    UART0 (pins 0/1)  -- USB serial debug @ 115 200 bd
//    UART2 (pins 16/17)-- RS-485 bus @ 1 000 000 bd
//    pin 22            -- RS-485 TX-enable (HIGH = transmit)
//    pin 2             -- SEN0192 radar output (active-LOW pulse)
//    USB Host Shield   -- Korg nanoKONTROL (USB MIDI)
// ============================================================

#include <Arduino.h>
#include <Usb.h>
#include <usbh_midi.h>
#include <avr/pgmspace.h>

#include "DynamixelReader.h"
#include "Animations.h"

// ============================================================
//  Configuration -- edit these to tune behaviour
// ============================================================

#define NUM_SERVOS      5           // MX-28 servos, IDs 20..24
#define FIRST_SERVO_ID  20

// Dynamixel register addresses (MX-28)
#define REG_CW_LIMIT    6           // 0x06  angle limit CW  (2 bytes)
#define REG_CCW_LIMIT   8           // 0x08  angle limit CCW (2 bytes)
#define REG_P_GAIN      28          // 0x1C  proportional gain
#define REG_GOAL_POS    30          // 0x1E  goal position (2 bytes)

// LED ring: Arduino Micro on ID 25, same RS-485 bus, Dynamixel protocol
// Two parameters written as consecutive 2-byte registers:
//   reg 30 = colour, reg 32 = brightness
//   (pattern not yet implemented in LED firmware -- reg 34 not written)
//
// Channel assignment on the nanoKONTROL:
//   faders 0-4  -> servo 0-4 goal position
//   fader  5    -> LED colour     (CC 5)
//   fader  6    -> LED brightness (CC 6)
//   fader  7    -> live P-gain in STOP/RECORD mode; fixed at WORKING_GAIN during PLAY
//
// LED ring firmware register ranges (from desklightRing.ino):
//   reg 30 = angleSetpoint -> Wheel(val/4, ...) -> hue 0-255 needs val 0-1023
//   reg 32 = speedSetpoint -> brightness = 255 - val/4
//             INVERTED: fader 127 -> val=0 -> brightness=255 (full bright)
//                       fader 0   -> val=1020 -> brightness=0 (off)
//   reg 34 = torqueSetpoint -> lit pixels = val/64, not used yet (firmware bug)
#define LED_RING_ID       25
#define LED_REG_COLOR     30
#define LED_REG_BRIGHT    32
#define LED_COLOR_MAX     1023   // fader 0-127 -> 0..1023 (full colour wheel)
#define LED_BRIGHT_MAX    1020   // fader 0-127 -> 1020..0 (inverted: up=brighter)

// Loop timing
#define LOOP_DELAY_MS   100         // main loop period (10 Hz) -- doubles record capacity

// Maximum steps recordable in RAM
#define MAX_STEPS       600

// Number of flash animation tracks (0 = PROGMEM, 1..N = RAM)
#define MAX_TRACKS      3

// ---- P-gain ------------------------------------------------
// Default P-gains per servo applied after ranging (MX-28 scale 0-254).
#define DEFAULT_P_GAIN  { 2, 6, 6, 6, 6 }

// Range of P-gain selectable via fader 7 (0-127 mapped to this range).
// P-gain during playback: fixed target reached via ramp at animation start.
// Also the gain held throughout playback and ramped back down at end.
// Adjust this single constant to tune stiffness during animation.
#define WORKING_GAIN       4

// Live P-gain range for fader 8 in STOP/RECORD mode (0-127 -> P_GAIN_MIN..P_GAIN_MAX)
#define P_GAIN_MIN         1
#define P_GAIN_MAX         16

// ---- Gain ramp at animation start ---------------------------
// When moving to the first frame of an animation the gain starts at
// P_GAIN_RAMP_START and steps up to the user-selected value in
// P_GAIN_RAMP_STEPS increments, with P_GAIN_RAMP_STEP_MS between each.
// Total ramp time = P_GAIN_RAMP_STEPS * P_GAIN_RAMP_STEP_MS ms.
// The move itself is given P_GAIN_MOVE_MS ms at low gain before the ramp
// begins, so the arm can travel to the target position compliantly.
#define P_GAIN_RAMP_START    1      // gain during initial low-compliance move
#define P_GAIN_MOVE_MS       1500   // time allowed for the slow approach move
#define P_GAIN_RAMP_STEPS    20     // number of gain steps in the ramp
#define P_GAIN_RAMP_STEP_MS  50     // ms between each ramp step (~1 s total)

// ---- Home positions ----------------------------------------
// Used as fallback if auto-ranging fails for a servo.
// Also used as the init target during soft-start.
#define ANGLE_HOME      { 2048, 3172, 3072, 2048, 2400 }

// ---- Auto-ranging ------------------------------------------
// Timeout (ms) waiting for each servo to respond to a limit read request.
#define RANGE_READ_TIMEOUT_MS   200

// ---- Radar (SEN0192) standalone mode -----------------------
#define RADAR_PIN                   2

// How many seconds to wait after power-on before the radar starts listening.
// Gives time for the arm to settle and avoids false triggers at startup.
#define RADAR_STARTUP_DELAY_MS      10000UL  // 10 seconds

// Minimum pulse rate to count as a trigger, expressed as pulses-per-second.
// The SEN0192 outputs more pulses when motion is stronger/closer.
// Raise this to require more definite motion; lower it for higher sensitivity.
//   1 = very sensitive (any faint movement triggers)
//   3 = moderate (someone walking past at normal distance)
//   6 = low sensitivity (requires close or fast movement)
#define RADAR_MIN_PULSES_PER_SEC    3

// Counting window -- increase for slower/smoother averaging,
// decrease for faster response. 500ms is a good default.
#define RADAR_WINDOW_MS             500UL

// How long to wait after an animation finishes before the radar
// can trigger again. Prevents retriggering while the arm resets.
#define TRIGGER_COOLDOWN_MS         60000UL  // 1 minute

// ============================================================
//  State
// ============================================================

USB       Usb;
USBH_MIDI Midi(&Usb);

// ---- MIDI panel state --------------------------------------
byte  faderBuffer[8];
byte  lastFaderBuffer[8];
// Live P-gain fader (CC 7): controls gain in STOP/RECORD mode only.
// During PLAY the gain is fixed at WORKING_GAIN.
byte  pGainFader = 64;    // default: maps to ~mid of P_GAIN_MIN..P_GAIN_MAX
byte  recordEnable[8];
byte  muteEnable[8];
byte  soloChannel;          // 1-based; 0 = none
byte  lastSoloChannel;
bool  cycle;
bool  blinkState;
unsigned long blinkTimer;

// ---- Transport state ---------------------------------------
#define MODE_STOP    0
#define MODE_PLAY    1
#define MODE_RECORD  2
int   mode;
int   memoryCounter;
int   trackNumber;
unsigned long startTime;

// ---- Per-servo angle limits (populated by autoRange()) -----
int  servoMin[NUM_SERVOS];   // CW  limit (encoder counts)
int  servoMax[NUM_SERVOS];   // CCW limit (encoder counts)
int  servoHome[NUM_SERVOS];  // home / init position
int  currentPGain[NUM_SERVOS];

// Working setpoints (encoder counts for servos; raw 0-LED_MAX_VAL for LED)
int  angleSetpoint[8];

// ---- RAM animation recording --------------------------------
struct RamEvent {
    uint16_t deltaMs;
    uint8_t  channelMask;
    uint8_t  values[7];    // channels 0-6 (channel 7 = P-gain, never recorded)
};
RamEvent ramAnim[MAX_STEPS];

// ---- Standalone / radar state ------------------------------
bool  midiConnected    = false;
bool  standalonePrimed = false;
unsigned long lastTriggerTime  = 0;
unsigned long radarWindowStart = 0;
uint8_t  radarPulseCount       = 0;
bool     lastRadarPin          = HIGH;
bool     radarArmed            = false;  // becomes true after RADAR_STARTUP_DELAY_MS

// ---- Auto-ranging receive buffer ---------------------------
// ProcessDynamixelData() stores incoming limit responses here.
volatile bool  rangeResponseReady  = false;
volatile uint8_t rangeResponseID   = 0;
volatile int   rangeResponseCW     = 0;
volatile int   rangeResponseCCW    = 0;

// ============================================================
//  Forward declarations
// ============================================================
void autoRange();
void initialize();
void softMoveToFrame(const uint8_t* values, uint8_t mask);
void dumpAnimationToSerial();
void pollSerial();
void directAction();
void setPosition();
void applyChannels(const uint8_t* values, uint8_t mask);
void playFromRam(int step);
void loopFlashPlay();
void start();
void stop();
void record();
void MIDI_poll();
void pollRadar();
void sendLedRing(int color, int brightness);
int  mapFaderToGoal(int channel, byte faderVal);
int  mapFaderToColor(byte faderVal);
int  mapFaderToBrightness(byte faderVal);
int  mapFaderToGlobalPGain(byte faderVal);
void applyGlobalPGain(int gain);

// ============================================================
//  ProcessDynamixelData -- called by DynamixelReader.cpp
//
//  Status packet layout (after header FF FF ID are stripped):
//    dataLength = LENGTH byte from packet = (num_params + 2)
//    Data[0]  = error byte
//    Data[1]  = first payload byte (CW limit lo)
//    Data[2]  = second payload byte (CW limit hi)
//    Data[3]  = third payload byte  (CCW limit lo)
//    Data[4]  = fourth payload byte (CCW limit hi)
//    Data[dataLength-2] = checksum (already verified by DynamixelPoll)
//
//  We only act on responses during autoRange() when rangeResponseReady==false.
// ============================================================
void ProcessDynamixelData(const unsigned char ID,
                          const int dataLength,
                          const unsigned char* const Data) {
    // Only capture if we are actively waiting for a response
    if (rangeResponseReady) return;

    // Must be from one of our servos
    if (ID < FIRST_SERVO_ID || ID >= FIRST_SERVO_ID + NUM_SERVOS) return;

    // A READ response for 4 bytes has LENGTH=6: error + 4 data + checksum
    if (dataLength < 6) return;

    // Data[0] is the error byte -- accept even if non-fatal (bit 6 = instruction error etc.)
    // Bit 2 (0x04) = range error, bit 4 (0x10) = checksum -- these would corrupt data,
    // so reject those.
    uint8_t errByte = Data[0];
    if (errByte & 0x14) {   // range error or checksum error bits
        Serial.print(F("    ! error byte 0x"));
        Serial.println(errByte, HEX);
        return;
    }

    rangeResponseCW  = (int)Data[1] | ((int)Data[2] << 8);
    rangeResponseCCW = (int)Data[3] | ((int)Data[4] << 8);
    rangeResponseID  = ID;
    rangeResponseReady = true;
}

// ============================================================
//  autoRange() -- read CW/CCW angle limits from each servo
//
//  MX-28 status return level (reg 16):
//    0 = never respond (default on many configs -- will cause timeout here)
//    1 = respond to READ commands only  <-- what we set temporarily
//    2 = respond to all commands
//
//  Procedure per servo:
//    1. Write reg 16 = 1  (enable READ responses)
//    2. Request 4 bytes from reg 6 (CW lo/hi, CCW lo/hi)
//    3. Poll for response with timeout
//    4. Validate received values
//    5. Write reg 16 = 0  (silence bus again)
//    6. On failure: apply safe fallback = home +- FADER_RANGE_COUNTS
//
//  Safe fallback uses the same ±508 count range as the original firmware
//  (original formula: goal = home - 4 * fader, fader max = 127 => delta = 508).
// ============================================================

// How many encoder counts correspond to full fader travel in the safe fallback.
// 4 (original MIDI_SCALE) x 127 (max fader) = 508.
#define FALLBACK_RANGE_COUNTS   508

// Register for status return level
#define REG_STATUS_RETURN_LEVEL  16

void autoRange() {
    const int homeDefault[] = ANGLE_HOME;

    Serial.println(F("============================================"));
    Serial.println(F("Auto-ranging servos"));
    Serial.println(F("============================================"));

    for (int n = 0; n < NUM_SERVOS; n++) {
        int id = FIRST_SERVO_ID + n;
        servoHome[n] = homeDefault[n];

        Serial.print(F("Servo "));
        Serial.print(n);
        Serial.print(F("  ID="));
        Serial.print(id);
        Serial.print(F("  home="));
        Serial.print(homeDefault[n]);
        Serial.print(F("  ... "));

        // Step 1: enable READ responses on this servo
        DynamixelWriteByte(id, REG_STATUS_RETURN_LEVEL, 1);
        delay(5);  // short settle time

        // Step 2: request 4 bytes from reg 6
        rangeResponseReady = false;
        getValueFrom(id, REG_CW_LIMIT, 4);

        // Step 3: wait for response
        unsigned long t = millis();
        while (!rangeResponseReady && (millis() - t) < RANGE_READ_TIMEOUT_MS) {
            DynamixelPoll();
        }

        // Step 5: silence this servo again regardless of outcome
        DynamixelWriteByte(id, REG_STATUS_RETURN_LEVEL, 0);
        delay(5);

        // Step 4 & 6: validate or apply fallback
        bool ok = false;
        if (rangeResponseReady && rangeResponseID == (uint8_t)id) {
            int cw  = rangeResponseCW;
            int ccw = rangeResponseCCW;

            // Sanity checks:
            //   - Both zero means "wheel mode" (no limit) or unset -- reject
            //   - CW must be <= CCW (CW is the smaller / lower-angle limit)
            //   - Both must be within the MX-28 hardware range 0-4095
            //   - Range must be at least FALLBACK_RANGE_COUNTS wide to be useful
            if (cw == 0 && ccw == 0) {
                Serial.print(F("WARN both limits=0 (wheel mode?) "));
            } else if (cw > ccw) {
                Serial.print(F("WARN CW>CCW ("));
                Serial.print(cw); Serial.print('>'); Serial.print(ccw);
                Serial.print(F(") "));
            } else if (cw > 4095 || ccw > 4095) {
                Serial.print(F("WARN out-of-range "));
            } else if ((ccw - cw) < FALLBACK_RANGE_COUNTS / 2) {
                Serial.print(F("WARN range too narrow "));
            } else {
                ok = true;
            }
        } else {
            Serial.print(F("TIMEOUT "));
        }

        if (ok) {
            servoMin[n]  = rangeResponseCW;
            servoMax[n]  = rangeResponseCCW;
            servoHome[n] = constrain(homeDefault[n], servoMin[n], servoMax[n]);
            Serial.print(F("OK  CW="));
            Serial.print(servoMin[n]);
            Serial.print(F("  CCW="));
            Serial.print(servoMax[n]);
            Serial.print(F("  home(clamped)="));
            Serial.println(servoHome[n]);
        } else {
            // Safe fallback: mirror the original firmware's ±508 count range,
            // clamped to the MX-28 hardware limits 0-4095.
            servoMin[n] = max(0,    homeDefault[n] - FALLBACK_RANGE_COUNTS);
            servoMax[n] = min(4095, homeDefault[n] + FALLBACK_RANGE_COUNTS);
            servoHome[n] = homeDefault[n];
            Serial.print(F("FALLBACK  CW="));
            Serial.print(servoMin[n]);
            Serial.print(F("  CCW="));
            Serial.println(servoMax[n]);
        }
    }

    Serial.println(F("--------------------------------------------"));
    Serial.println(F("Ranging complete. Summary:"));
    for (int n = 0; n < NUM_SERVOS; n++) {
        Serial.print(F("  ["));
        Serial.print(n);
        Serial.print(F("] ID="));
        Serial.print(FIRST_SERVO_ID + n);
        Serial.print(F("  range="));
        Serial.print(servoMin[n]);
        Serial.print(F(".."));
        Serial.print(servoMax[n]);
        Serial.print(F("  (span="));
        Serial.print(servoMax[n] - servoMin[n]);
        Serial.print(F(")  home="));
        Serial.println(servoHome[n]);
    }
    Serial.println(F("============================================"));
}

// ============================================================
//  initialize() -- cold-start only: move to home at low gain.
//  Called once from setup(). Does NOT ramp gain -- leaves servos
//  at P_GAIN_RAMP_START so the arm is compliant until start() is called.
// ============================================================
void initialize() {
    Serial.println(F("Moving to home position (low gain)..."));
    for (int n = 0; n < NUM_SERVOS; n++) {
        currentPGain[n] = P_GAIN_RAMP_START;
        DynamixelWriteByte(FIRST_SERVO_ID + n, REG_P_GAIN, P_GAIN_RAMP_START);
        DynamixelWrite   (FIRST_SERVO_ID + n, REG_GOAL_POS, servoHome[n]);
        angleSetpoint[n] = servoHome[n];
        delay(10);
    }
    delay(P_GAIN_MOVE_MS);
    sendLedRing(0, 0);
    Serial.println(F("Home reached."));
}

// ============================================================
//  softMoveToFrame() -- move compliantly to a target frame, then
//  ramp gain up to the user-selected value.
//
//  Sequence:
//    1. Set all servos to P_GAIN_RAMP_START (very compliant)
//    2. Send goal positions from the frame
//    3. Wait P_GAIN_MOVE_MS for the arm to arrive
//    4. Ramp gain from P_GAIN_RAMP_START to target in P_GAIN_RAMP_STEPS
//
//  This is called by start() before playback begins, so the arm
//  always arrives softly at frame 0 regardless of where it was.
// ============================================================
void softMoveToFrame(const uint8_t* values, uint8_t mask) {
    int targetGain = WORKING_GAIN;

    Serial.print(F("Soft-move to frame 0, target gain="));
    Serial.println(targetGain);

    // Step 1: set very low gain on all servos
    for (int n = 0; n < NUM_SERVOS; n++) {
        DynamixelWriteByte(FIRST_SERVO_ID + n, REG_P_GAIN, P_GAIN_RAMP_START);
        currentPGain[n] = P_GAIN_RAMP_START;
        delay(2);
    }

    // Step 2: send goal positions for servo channels in the frame
    for (int n = 0; n < NUM_SERVOS; n++) {
        if (mask & (1 << n)) {
            angleSetpoint[n] = mapFaderToGoal(n, values[n]);
        }
        // Always write -- servo must move even if this channel not in mask
        DynamixelWrite(FIRST_SERVO_ID + n, REG_GOAL_POS, angleSetpoint[n]);
        delay(2);
    }
    // Apply LED channels if present
    if (mask & (1 << 5)) angleSetpoint[5] = mapFaderToColor(values[5]);
    if (mask & (1 << 6)) angleSetpoint[6] = mapFaderToBrightness(values[6]);
    sendLedRing(angleSetpoint[5], angleSetpoint[6]);

    Serial.print(F("  Moving ("));
    Serial.print(P_GAIN_MOVE_MS);
    Serial.println(F(" ms)..."));
    delay(P_GAIN_MOVE_MS);

    // Step 3: ramp gain from P_GAIN_RAMP_START to targetGain
    // Use integer linear ramp; avoid going above targetGain
    Serial.print(F("  Ramping gain "));
    Serial.print(P_GAIN_RAMP_START);
    Serial.print(F(" -> "));
    Serial.println(targetGain);

    for (int step = 1; step <= P_GAIN_RAMP_STEPS; step++) {
        int g = P_GAIN_RAMP_START +
                ((targetGain - P_GAIN_RAMP_START) * step) / P_GAIN_RAMP_STEPS;
        g = constrain(g, P_GAIN_RAMP_START, targetGain);
        for (int n = 0; n < NUM_SERVOS; n++) {
            currentPGain[n] = g;
            DynamixelWriteByte(FIRST_SERVO_ID + n, REG_P_GAIN, (byte)g);
        }
        delay(P_GAIN_RAMP_STEP_MS);
    }
    Serial.println(F("  Gain ramp complete."));
}

// ============================================================
//  setup()
// ============================================================
void setup() {
    Serial.begin(115200);
    Serial2.begin(1000000);
    pinMode(22, OUTPUT);
    pinMode(RADAR_PIN, INPUT);

    // USB Host Shield workaround for MEGA ADK
    pinMode(7, OUTPUT);
    digitalWrite(7, HIGH);

    if (Usb.Init() == -1) {
        Serial.println(F("USB Host init failed -- standalone mode"));
        standalonePrimed = true;
    }

    delay(200);

    // Read servo limits before moving anything
    autoRange();

    // Move to home at low gain (arm may be anywhere at power-on)
    initialize();

    trackNumber = 0;
    cycle       = false;   // single-shot for radar-triggered play
    mode        = MODE_STOP;
    // Arm stays at home in STOP mode.
    // MIDI mode: operator presses Play to begin.
    // Standalone mode: radar arms after RADAR_STARTUP_DELAY_MS, then triggers.
    Serial.println(F("Ready. Waiting for input."));
}

// ============================================================
//  Main loop
// ============================================================
static unsigned long loopTime = 0;

// Event-playback cursor for PROGMEM animation
static uint16_t      flashCursor   = 0;
static AnimEvent     flashNextEv;
static bool          flashHasNext  = false;
static unsigned long flashEventDue = 0;

void loop() {
    Usb.Task();
    DynamixelPoll();     // keep receiving RS-485 traffic

    // Detect MIDI connection
    if (!standalonePrimed && Usb.getUsbTaskState() == USB_STATE_RUNNING) {
        midiConnected = true;
    }

    // Radar check when no MIDI panel
    if (standalonePrimed || !midiConnected) {
        pollRadar();
    }

    // Flash animation runs at full resolution, not rate-limited
    loopFlashPlay();

    // Fixed-rate section (LOOP_DELAY_MS)
    if (millis() < loopTime + LOOP_DELAY_MS) return;
    loopTime = millis();

    // Check for changed faders/knobs (MIDI mode only)
    if (midiConnected) {
        bool changed = false;
        for (int i = 0; i < 8; i++) {
            if (faderBuffer[i] != lastFaderBuffer[i]) changed = true;
            lastFaderBuffer[i] = faderBuffer[i];
        }
        if (changed && mode == MODE_STOP) directAction();
    }

    // Record
    if (mode == MODE_RECORD) {
        if (memoryCounter < MAX_STEPS) {
            ramAnim[memoryCounter].deltaMs     = (uint16_t)(millis() - startTime);
            ramAnim[memoryCounter].channelMask = 0x7F; // bits 0-6; bit 7 (P-gain) excluded
            for (int i = 0; i < 7; i++)   // channels 0-6 only; skip channel 7 (P-gain)
                ramAnim[memoryCounter].values[i] = faderBuffer[i];
            // Drive servos and LED ring in real time while recording
            applyChannels(ramAnim[memoryCounter].values, ramAnim[memoryCounter].channelMask);
            memoryCounter++;
        }
        // Blink REC LED near end of buffer
        if (memoryCounter > MAX_STEPS - 5 * (1000 / LOOP_DELAY_MS)) {
            if (millis() > blinkTimer) {
                blinkState = !blinkState;
                byte buf[] = { 0xB0, 45, (byte)(blinkState ? 127 : 0) };
                Midi.SendData(buf, 0);
                blinkTimer = millis() + 500;
            }
        }
        if (memoryCounter >= MAX_STEPS - 1) stop();
    }

    // Play RAM track
    if (mode == MODE_PLAY && trackNumber > 0) {
        if (memoryCounter < MAX_STEPS &&
            ramAnim[memoryCounter].deltaMs <= (millis() - startTime)) {
            playFromRam(memoryCounter);
            memoryCounter++;
            if (memoryCounter >= MAX_STEPS ||
                ramAnim[memoryCounter].channelMask == 0) {
                if (!cycle) stop(); else start();
            }
        }
    }

    // MIDI poll
    if (midiConnected && Usb.getUsbTaskState() == USB_STATE_RUNNING) {
        MIDI_poll();
    }

    // Serial command input
    pollSerial();

    delay(1);
}

// ============================================================
//  loopFlashPlay -- event-driven PROGMEM animation playback
//  Called every loop() iteration for sub-tick timing accuracy.
// ============================================================
void loopFlashPlay() {
    if (mode != MODE_PLAY || trackNumber != 0) return;

    unsigned long now = millis();

    if (!flashHasNext) {
        if (!animNextEvent(animation, flashCursor, flashNextEv)) {
            if (!cycle) stop(); else start();
            return;
        }
        flashHasNext  = true;
        flashEventDue = startTime + flashNextEv.deltaMs;
    }

    while (flashHasNext && now >= flashEventDue) {
        applyChannels(flashNextEv.values, flashNextEv.channelMask);

        AnimEvent nextEv;
        if (animNextEvent(animation, flashCursor, nextEv)) {
            flashNextEv   = nextEv;
            flashEventDue += nextEv.deltaMs;
        } else {
            flashHasNext = false;
            if (!cycle) stop(); else start();
            return;
        }
    }
}

// ============================================================
//  sendLedRing
//  Writes colour/brightness/pattern to LED ring (ID 25).
//  All three values go to a single device at consecutive registers,
//  exactly as in the original setPosition() i==5 block.
// ============================================================
void sendLedRing(int color, int brightness) {
    DynamixelWrite(LED_RING_ID, LED_REG_COLOR,  color);
    DynamixelWrite(LED_RING_ID, LED_REG_BRIGHT, brightness);
}

// ============================================================
//  setPosition -- push current angleSetpoint[] to all devices
// ============================================================
void setPosition() {
    for (int i = 0; i < NUM_SERVOS; i++) {
        DynamixelWrite(FIRST_SERVO_ID + i, REG_GOAL_POS, angleSetpoint[i]);
    }
    // LED ring channels stored in setpoint slots 5/6
    sendLedRing(angleSetpoint[5], angleSetpoint[6]);
}

// ============================================================
//  applyChannels -- decode raw fader bytes and drive hardware
// ============================================================
void applyChannels(const uint8_t* values, uint8_t mask) {
    for (int i = 0; i < 8; i++) {
        if (!(mask & (1 << i))) continue;
        if (i < NUM_SERVOS) {
            // Channels 0-4: servo goal position
            angleSetpoint[i] = mapFaderToGoal(i, values[i]);
            DynamixelWrite(FIRST_SERVO_ID + i, REG_GOAL_POS, angleSetpoint[i]);
        } else if (i == 5) {
            // Channel 5: LED colour (0->1023, full hue wheel)
            angleSetpoint[5] = mapFaderToColor(values[5]);
        } else if (i == 6) {
            // Channel 6: LED brightness (inverted: fader up = brighter)
            angleSetpoint[6] = mapFaderToBrightness(values[6]);
        }
        // Channel 7: P-gain -- never stored in animation data, intentionally skipped
    }
    sendLedRing(angleSetpoint[5], angleSetpoint[6]);
}

// ============================================================
//  directAction -- live control from MIDI panel (STOP mode)
// ============================================================
void directAction() {
    Serial.print(F("srv:"));
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (muteEnable[i]) {
            Serial.print(F("-\t"));
        } else {
            angleSetpoint[i] = mapFaderToGoal(i, faderBuffer[i]);
            Serial.print(angleSetpoint[i]);
            Serial.print('\t');
        }
    }
    // LED channels
    angleSetpoint[5] = mapFaderToColor(faderBuffer[5]);
    angleSetpoint[6] = mapFaderToBrightness(faderBuffer[6]);
    Serial.print(F(" led:"));
    Serial.print(angleSetpoint[5]);
    Serial.print('/');
    Serial.print(angleSetpoint[6]);
    Serial.print(F(" pgain:"));
    Serial.println(mapFaderToGlobalPGain(faderBuffer[7]));
    setPosition();
    // Live P-gain from fader 7 (STOP mode only)
    {
        int gain = mapFaderToGlobalPGain(faderBuffer[7]);
        pGainFader = faderBuffer[7];
        applyGlobalPGain(gain);
    }
}

// ============================================================
//  playFromRam
// ============================================================
void playFromRam(int step) {
    if (soloChannel > 0 && soloChannel <= 8) {
        int ch = soloChannel - 1;
        if (ramAnim[step].channelMask & (1 << ch)) {
            uint8_t vals[8] = {};
            vals[ch] = ramAnim[step].values[ch];
            applyChannels(vals, (uint8_t)(1 << ch));
        }
    } else {
        applyChannels(ramAnim[step].values, ramAnim[step].channelMask);
    }
    for (int i = 0; i < 8; i++) {
        if (muteEnable[i]) Serial.print('-');
        else Serial.print(ramAnim[step].values[i]);
        if (i < 7) Serial.print(',');
    }
    Serial.println();
}

// ============================================================
//  Transport control
// ============================================================
void stop() {
    // Ramp gain DOWN from WORKING_GAIN to P_GAIN_RAMP_START before stopping,
    // so the arm goes limp rather than holding stiffly in its last position.
    if (mode == MODE_PLAY) {
        Serial.print(F("Gain ramp down "));
        Serial.print(WORKING_GAIN);
        Serial.print(F(" -> "));
        Serial.println(P_GAIN_RAMP_START);
        for (int step = P_GAIN_RAMP_STEPS; step >= 0; step--) {
            int g = P_GAIN_RAMP_START +
                    ((WORKING_GAIN - P_GAIN_RAMP_START) * step) / P_GAIN_RAMP_STEPS;
            g = constrain(g, P_GAIN_RAMP_START, WORKING_GAIN);
            for (int n = 0; n < NUM_SERVOS; n++) {
                currentPGain[n] = g;
                DynamixelWriteByte(FIRST_SERVO_ID + n, REG_P_GAIN, (byte)g);
            }
            delay(P_GAIN_RAMP_STEP_MS);
        }
        // Restore live fader gain now that we are back in STOP mode
        int liveGain = mapFaderToGlobalPGain(pGainFader);
        applyGlobalPGain(liveGain);
        Serial.print(F("Restored live gain = "));
        Serial.println(liveGain);
    }

    mode = MODE_STOP;
    Serial.print(F("STOP at step: "));
    Serial.println(memoryCounter);
    if (midiConnected) {
        byte buf[] = { 0xB0, 45, 0 };
        Midi.SendData(buf, 0);
        buf[1] = 41; Midi.SendData(buf, 0);
    }
}

void start() {
    mode          = MODE_PLAY;
    flashCursor   = 0;
    flashHasNext  = false;
    flashEventDue = 0;
    memoryCounter = 0;

    if (midiConnected) {
        byte buf[] = { 0xB0, 41, 127 };
        Midi.SendData(buf, 0);
    }

    // ---- Peek frame 0 and do a compliant move to it -----------
    // For PROGMEM track (0): read first event without consuming the cursor,
    // call softMoveToFrame(), then advance cursor past frame 0 so playback
    // starts at frame 1 (frame 0 position is already applied).
    //
    // For RAM tracks: use ramAnim[0] the same way, then start at step 1.

    if (trackNumber == 0) {
        // Peek at PROGMEM frame 0 (cursor is already reset to 0 above)
        AnimEvent frame0;
        uint16_t peekCursor = 0;
        if (animNextEvent(animation, peekCursor, frame0)) {
            softMoveToFrame(frame0.values, frame0.channelMask);
            // Advance the real cursor past frame 0
            flashCursor = peekCursor;
            // Schedule next event relative to now
            flashHasNext  = false;  // will be loaded fresh in loopFlashPlay
        }
    } else {
        // RAM track: peek frame 0
        if (MAX_STEPS > 0 && ramAnim[0].channelMask != 0) {
            softMoveToFrame(ramAnim[0].values, ramAnim[0].channelMask);
            memoryCounter = 1;      // skip frame 0, already applied
        }
    }

    // Start the clock AFTER the soft-move so timing is correct
    startTime = millis();
}

void record() {
    mode          = MODE_RECORD;
    startTime     = millis();
    memoryCounter = 0;
    if (midiConnected) {
        byte buf[] = { 0xB0, 45, 127 };
        Midi.SendData(buf, 0);
    }
}

// ============================================================
//  Mapping helpers
// ============================================================

// Fader 0-127 mapped to [servoMin..servoMax] for servo i.
// Uses the auto-ranged limits so full fader travel = full servo range.
int mapFaderToGoal(int channel, byte faderVal) {
    return map((int)faderVal, 0, 127, servoMin[channel], servoMax[channel]);
}

// Fader 0-127 -> 0..1023  (full colour wheel)
int mapFaderToColor(byte faderVal) {
    return map((int)faderVal, 0, 127, 0, LED_COLOR_MAX);
}

// Fader 0-127 -> 1020..0  (inverted: fader up = brighter)
// Ring firmware: brightness = 255 - speedSetpoint/4
// At faderVal=127 -> speedSetpoint=0   -> brightness=255 (full bright)
// At faderVal=0   -> speedSetpoint=1020 -> brightness=0  (off)
int mapFaderToBrightness(byte faderVal) {
    return map((int)faderVal, 0, 127, LED_BRIGHT_MAX, 0);
}

// Fader 0-127 mapped to P_GAIN_MIN..P_GAIN_MAX for live STOP mode control.
int mapFaderToGlobalPGain(byte faderVal) {
    return P_GAIN_MIN + ((int)faderVal * (P_GAIN_MAX - P_GAIN_MIN)) / 127;
}

void applyGlobalPGain(int gain) {
    for (int i = 0; i < NUM_SERVOS; i++) {
        currentPGain[i] = gain;
        DynamixelWriteByte(FIRST_SERVO_ID + i, REG_P_GAIN, (byte)gain);
    }
    Serial.print(F("P-gain all = "));
    Serial.println(gain);
}

// ============================================================
//  Radar standalone trigger (SEN0192)
// ============================================================
void pollRadar() {
    unsigned long now = millis();

    // ---- Startup delay: do nothing until arm has settled ----
    if (!radarArmed) {
        if (now >= RADAR_STARTUP_DELAY_MS) {
            radarArmed = true;
            Serial.println(F("Radar armed -- listening for motion."));
        } else {
            return;  // still in startup dead time
        }
    }

    // ---- Edge detection: count falling edges (active-LOW pulses) ----
    bool pinNow = digitalRead(RADAR_PIN);
    if (lastRadarPin == HIGH && pinNow == LOW) {
        // Start a fresh window on the first pulse after the previous window closed
        if (now - radarWindowStart > RADAR_WINDOW_MS) {
            radarPulseCount  = 0;
            radarWindowStart = now;
        }
        radarPulseCount++;
    }
    lastRadarPin = pinNow;

    // ---- Evaluate window when it expires ----
    if (now - radarWindowStart >= RADAR_WINDOW_MS) {
        // Convert RADAR_MIN_PULSES_PER_SEC to required pulses in this window
        // e.g. 3 pulses/sec * (500ms / 1000ms) = 1.5 -> need >= 2 pulses
        uint8_t threshold = max(1,
            (int)(RADAR_MIN_PULSES_PER_SEC * RADAR_WINDOW_MS / 1000UL));

        if (radarPulseCount >= threshold) {
            Serial.print(F("Radar: "));
            Serial.print(radarPulseCount);
            Serial.print(F(" pulses in "));
            Serial.print(RADAR_WINDOW_MS);
            Serial.print(F("ms (threshold="));
            Serial.print(threshold);
            Serial.print(F(")"));

            if (mode != MODE_STOP) {
                Serial.println(F(" -- ignored, not in STOP mode"));
            } else if (lastTriggerTime > 0 &&
                       now - lastTriggerTime < TRIGGER_COOLDOWN_MS) {
                unsigned long remaining = (TRIGGER_COOLDOWN_MS -
                                           (now - lastTriggerTime)) / 1000UL;
                Serial.print(F(" -- cooldown, "));
                Serial.print(remaining);
                Serial.println(F("s remaining"));
            } else {
                Serial.println(F(" -- TRIGGER"));
                lastTriggerTime = now;
                trackNumber     = 0;    // PROGMEM animation
                cycle           = false; // single-shot
                start();
            }
        }
        // Reset window
        radarPulseCount  = 0;
        radarWindowStart = now;
    }
}


// ============================================================
//  pollSerial -- handle single-character commands from serial monitor
//
//  Commands:
//    D  dump current RAM animation as animation_data.h to serial
//    ?  print command help
// ============================================================
void pollSerial() {
    if (!Serial.available()) return;
    char cmd = Serial.read();
    // Flush any trailing newline/CR
    while (Serial.available()) Serial.read();

    switch (cmd) {
    case 'D': case 'd':
        dumpAnimationToSerial();
        break;
    case '?':
        Serial.println(F("Serial commands:"));
        Serial.println(F("  D  dump RAM animation as animation_data.h"));
        Serial.println(F("  ?  this help"));
        break;
    default:
        Serial.print(F("Unknown command '"));
        Serial.print(cmd);
        Serial.println(F("'  -- send ? for help"));
        break;
    }
}

// ============================================================
//  dumpAnimationToSerial
//
//  Converts the RAM recording (absolute-timestamp, all-channels-each-step)
//  into the compact PROGMEM event format (delta-time, changed-channels only)
//  and prints a complete animation_data.h to the serial monitor.
//
//  To make a permanent animation:
//    1. Record your motion (MIDI record button)
//    2. Press stop
//    3. Open serial monitor (115200), send 'D'
//    4. Copy everything between the ===BEGIN=== and ===END=== markers
//    5. Save as  include/animation_data.h  (overwrite the existing file)
//    6. Recompile and flash
//
//  The output is delta-compressed: only changed channels per tick are
//  stored, so sparse animations compress significantly.
// ============================================================
void dumpAnimationToSerial() {
    if (memoryCounter == 0) {
        Serial.println(F("No animation recorded in RAM. Record first, then dump."));
        return;
    }

    Serial.println(F(""));
    Serial.println(F("===BEGIN animation_data.h==="));

    // Header
    Serial.println(F("// AUTO-GENERATED by desklight serial dump."));
    Serial.println(F("// Copy this entire block to include/animation_data.h"));
    Serial.println(F("#pragma once"));
    Serial.println(F("#include <avr/pgmspace.h>"));
    Serial.println(F(""));

    // We build the event stream in RAM as bytes, then print.
    // To avoid a second large buffer, we do two passes:
    //   Pass 1: count output bytes (to know animation_size)
    //   Pass 2: print the actual hex values
    //
    // Event format per record:
    //   [deltaMs hi][deltaMs lo][channelMask][value per set bit in ch order]
    // End marker: 0xFF 0xFF

    // ---- Pass 1: count bytes ----
    uint16_t totalBytes = 0;
    uint8_t  prevValues[7] = {};        // previous fader values per channel
    bool     firstEvent = true;
    uint16_t prevAbsMs  = 0;

    for (int step = 0; step < memoryCounter; step++) {
        uint8_t mask = 0;
        for (int ch = 0; ch < 7; ch++) {
            if (firstEvent || ramAnim[step].values[ch] != prevValues[ch])
                mask |= (1 << ch);
        }
        if (mask == 0 && !firstEvent) continue; // nothing changed, skip

        uint16_t absMs   = ramAnim[step].deltaMs;
        uint16_t deltaMs = firstEvent ? 0 : (uint16_t)(absMs - prevAbsMs);
        deltaMs = min(deltaMs, (uint16_t)0xFFFE);

        totalBytes += 2 + 1;  // deltaMs hi+lo + mask
        for (int ch = 0; ch < 7; ch++)
            if (mask & (1 << ch)) totalBytes++;

        for (int ch = 0; ch < 7; ch++)
            if (mask & (1 << ch)) prevValues[ch] = ramAnim[step].values[ch];
        prevAbsMs  = absMs;
        firstEvent = false;
    }
    totalBytes += 2; // end marker 0xFFFF

    // ---- Print size constant ----
    Serial.print(F("static const uint16_t animation_size PROGMEM = "));
    Serial.print(totalBytes);
    Serial.println(F(";"));
    Serial.println(F("static const byte animation[] PROGMEM = {"));

    // ---- Pass 2: print hex bytes ----
    memset(prevValues, 0, sizeof(prevValues));
    firstEvent = true;
    prevAbsMs  = 0;
    uint8_t  col     = 0;       // bytes printed on current line
    uint16_t printed = 0;

    auto printByte = [&](uint8_t b) {
        if (col == 0) Serial.print(F("    "));
        Serial.print(F("0x"));
        if (b < 0x10) Serial.print('0');
        Serial.print(b, HEX);
        Serial.print(',');
        printed++;
        col++;
        if (col >= 16) { Serial.println(); col = 0; }
    };

    for (int step = 0; step < memoryCounter; step++) {
        uint8_t mask = 0;
        for (int ch = 0; ch < 7; ch++) {
            if (firstEvent || ramAnim[step].values[ch] != prevValues[ch])
                mask |= (1 << ch);
        }
        if (mask == 0 && !firstEvent) continue;

        uint16_t absMs   = ramAnim[step].deltaMs;
        uint16_t deltaMs = firstEvent ? 0 : (uint16_t)(absMs - prevAbsMs);
        deltaMs = min(deltaMs, (uint16_t)0xFFFE);

        printByte((uint8_t)(deltaMs >> 8));
        printByte((uint8_t)(deltaMs & 0xFF));
        printByte(mask);
        for (int ch = 0; ch < 7; ch++)
            if (mask & (1 << ch)) printByte(ramAnim[step].values[ch]);

        for (int ch = 0; ch < 7; ch++)
            if (mask & (1 << ch)) prevValues[ch] = ramAnim[step].values[ch];
        prevAbsMs  = absMs;
        firstEvent = false;
    }
    // End marker
    printByte(0xFF);
    printByte(0xFF);
    if (col > 0) Serial.println();

    Serial.println(F("};"));
    Serial.println(F(""));
    Serial.print(F("// Steps recorded: "));
    Serial.print(memoryCounter);
    Serial.print(F("  ->  Event bytes: "));
    Serial.print(printed);
    Serial.print(F("  (was "));
    Serial.print((uint16_t)memoryCounter * 7);
    Serial.println(F(" bytes uncompressed)"));
    Serial.println(F("===END animation_data.h==="));
}

// ============================================================
//  MIDI_poll -- Korg nanoKONTROL
// ============================================================
//  CC map:
//    0-4   faders       -- servo 0-4 goal position
//    5     fader        -- LED colour
//    6     fader        -- LED brightness
//    7     fader        -- live P-gain in STOP/RECORD mode (ignored during PLAY)
//    16-23 knobs        -- not used (ignored)
//    32-39 solo buttons
//    41    play
//    42    stop
//    43    rewind/step back
//    44    fast-fwd/step fwd
//    45    record
//    46    cycle
//    48-55 mute buttons
//    58    track prev
//    59    track next
//    64-71 record-arm buttons
// ============================================================
void MIDI_poll() {
    uint8_t size;
    byte    buf[3];

    do {
        if ((size = Midi.RcvData(buf)) > 0) {
            midiConnected = true;

            // Faders 0-6: servo position / LED colour+brightness
            // Fader 7:   global P-gain (applied immediately, even during play)
            for (int i = 0; i < 8; i++) {
                if (buf[1] == i) {
                    faderBuffer[i] = buf[2];
                    if (i == 7) {
                        // Live gain control -- only active outside PLAY mode
                        pGainFader = buf[2];
                        if (mode != MODE_PLAY) {
                            applyGlobalPGain(mapFaderToGlobalPGain(pGainFader));
                        }
                    }
                }
            }
            // Knobs (CC 16-23) are no longer used -- nanoKONTROL knobs ignored
            // Mute buttons
            for (int i = 48; i < 56; i++) {
                if (buf[1] == i && buf[2] == 127) {
                    muteEnable[i - 48] ^= 1;
                    byte out[] = { 0xB0, (byte)i,
                                   (byte)(muteEnable[i-48] ? 127 : 0) };
                    Midi.SendData(out, 0);
                }
            }
            // Solo buttons
            for (int i = 32; i < 40; i++) {
                if (buf[1] == i && buf[2] == 127) {
                    byte ch = i - 31;
                    soloChannel = (lastSoloChannel == ch && ch > 0) ? 0 : ch;
                    byte out[] = { 0xB0, (byte)i, 127 };
                    Midi.SendData(out, 0);
                    out[1] = lastSoloChannel + 31; out[2] = 0;
                    Midi.SendData(out, 0);
                    lastSoloChannel = soloChannel;
                }
            }
            // Record-arm buttons
            for (int i = 64; i < 72; i++) {
                if (buf[1] == i && buf[2] == 127) {
                    recordEnable[i - 64] ^= 1;
                    byte out[] = { 0xB0, (byte)i,
                                   (byte)(recordEnable[i-64] ? 127 : 0) };
                    Midi.SendData(out, 0);
                }
            }
            // Transport
            if (buf[1] == 45 && buf[2] == 127) record();
            if (buf[1] == 41 && buf[2] == 127) start();
            if (buf[1] == 42 && buf[2] == 127) stop();
            if (buf[1] == 43 && buf[2] == 127) {
                stop();
                if (memoryCounter > 0) memoryCounter--;
                playFromRam(memoryCounter);
            }
            if (buf[1] == 44 && buf[2] == 127) {
                stop();
                if (memoryCounter < MAX_STEPS - 1) memoryCounter++;
                playFromRam(memoryCounter);
            }
            if (buf[1] == 46 && buf[2] == 127) {
                cycle = !cycle;
                byte out[] = { 0xB0, 46, (byte)(cycle ? 127 : 0) };
                Midi.SendData(out, 0);
            }
            if (buf[1] == 58 && buf[2] == 127) {
                if (trackNumber > 0) trackNumber--;
                Serial.print(F("track: ")); Serial.println(trackNumber);
            }
            if (buf[1] == 59 && buf[2] == 127) {
                if (trackNumber < MAX_TRACKS) trackNumber++;
                Serial.print(F("track: ")); Serial.println(trackNumber);
            }
        }
    } while (size > 0);
}

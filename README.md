# Desklight Robot — Firmware

PlatformIO/VSCode project for the desk-light robot arm with MX-28 servos and
an Arduino-Micro LED ring on a shared RS-485 Dynamixel bus.

## Hardware

| Item | Detail |
|------|--------|
| MCU | Arduino MEGA ADK (ATmega2560) |
| RS-485 bus | UART2 @ 1 000 000 bd, pin 22 = TX-enable |
| Servos | 5 × MX-28, IDs 20–24 |
| LED ring | Arduino Micro speaking Dynamixel protocol, ID 25 |
| MIDI panel | Korg nanoKONTROL via USB Host Shield 2.0 |
| Radar | DFRobot SEN0192 10.525 GHz microwave sensor, pin D2 (active-LOW) |

## Building

```bash
# Open in VSCode with the PlatformIO extension, or use the CLI:
pio run              # compile
pio run -t upload    # flash
pio device monitor   # serial monitor @ 115200 baud
```

## MIDI Mapping (Korg nanoKONTROL)

| Control | CC | Function |
|---------|-----|---------|
| Fader 1–5 | 0–4 | Servo 1–5 goal position (full auto-ranged travel) |
| Fader 6 | 5 | LED ring colour (0–127 → full hue wheel) |
| Fader 7 | 6 | LED ring brightness (up = brighter) |
| Fader 8 | 7 | Global P-gain for all servos (STOP/RECORD mode only) |
| Knobs 1–8 | 16–23 | Not used |
| Play (▶) | 41 | Start playback from flash animation |
| Stop (■) | 42 | Stop |
| Record (●) | 45 | Start recording to RAM |
| Cycle | 46 | Toggle loop on/off |
| ◀◀ | 43 | Step back one frame |
| ▶▶ | 44 | Step forward one frame |
| Mute 1–8 | 48–55 | Mute channel (suppress output) |
| Solo 1–8 | 32–39 | Solo one channel |
| Rec-arm 1–8 | 64–71 | Arm channel for recording |
| Track ◀ | 58 | Previous track (track 0 = flash, 1+ = RAM) |
| Track ▶ | 59 | Next track |

## Startup Behaviour

On power-on the firmware always:

1. Reads CW/CCW angle limits from each servo (auto-ranging, see below)
2. Moves all servos to their home positions at low gain
3. Enters **STOP mode** — the arm waits, no animation plays automatically

With the MIDI panel connected, press **Play (▶)** to begin.
In standalone mode (no panel), the radar controls triggering (see below).

## Auto-Ranging

At startup the firmware reads the programmed CW and CCW angle limits directly
from each servo's EEPROM (registers 6/8). Fader travel is then mapped across
that exact range, so full fader stroke always equals full servo range regardless
of how the limits are set in the servo.

If a servo does not respond (status return level not set), a safe fallback range
of ±508 counts around the home position is used — identical to the original
firmware behaviour. The serial monitor shows the result for every servo:

```
Servo 0  ID=20  home=2048  ... OK  CW=1024  CCW=3072  home(clamped)=2048
Servo 1  ID=21  home=3172  ... FALLBACK  CW=2664  CCW=3680
```

If all servos show FALLBACK, set their status return level to 1 permanently
using the Dynamixel Wizard tool (register 16 = 1).

## Recording a New Animation

Animations are recorded to RAM and then dumped as a C header over serial.

**Recording:**
1. Select track 1 or higher with **Track ▶** (track 0 is the flash animation)
2. Press **Record (●)** and perform the motion — servos and LED ring move in
   real time as you move the faders
3. Press **Stop (■)** when done
4. Maximum recording time: **60 seconds** at 10 Hz (600 steps)

**Making it permanent (saving to flash):**
1. Open the serial monitor at 115200 baud
2. Send the character `D` (capital or lowercase)
3. The firmware prints a complete `animation_data.h` between markers:
   ```
   ===BEGIN animation_data.h===
   ...
   ===END animation_data.h===
   ```
4. Copy everything between the markers and save as `include/animation_data.h`
   (overwrite the existing file)
5. Recompile and flash: `pio run -t upload`

The dump performs delta-compression — only changed channels per tick are stored,
so sparse animations are significantly smaller than the raw recording.

**Serial commands:**

| Command | Effect |
|---------|--------|
| `D` | Dump current RAM animation as `animation_data.h` |
| `?` | Print command help |

## Animation Format

Animations are stored in PROGMEM as a stream of variable-length event records:

```
[deltaMs hi][deltaMs lo][channelMask][value per set bit, channels 0–6]
```

A record with `deltaMs == 0xFFFF` marks end-of-animation.

| Channel | Content |
|---------|---------|
| 0–4 | Servo goal position (raw fader value 0–127, mapped at playback) |
| 5 | LED colour |
| 6 | LED brightness |
| 7 | Not recorded (P-gain is a live parameter, never stored) |

### Converting an old stream-format animation

If you have an animation in the original fixed-timestep format (flat byte array,
8 bytes per frame), convert it with the included tool:

```bash
python3 tools/convert_animation.py \
    --input  animations_original.h \
    --output include/animation_data.h \
    --timestep 50
```

## P-Gain

P-gain controls servo stiffness. The arm uses a fixed working gain during
playback, with a soft ramp at the start and end of each animation.

| Phase | Gain |
|-------|------|
| Soft-move to frame 0 | Ramps 1 → `WORKING_GAIN` over ~1 second |
| Animation playing | Fixed at `WORKING_GAIN` |
| Animation ends | Ramps `WORKING_GAIN` → 1 over ~1 second, then restores fader position |
| STOP / RECORD mode | Live control via fader 8 (range 1–`P_GAIN_MAX`) |

Key constants at the top of `src/desklight.cpp`:

```cpp
#define WORKING_GAIN    4    // gain during playback (4 works well for this arm)
#define P_GAIN_MAX      16   // ceiling for live fader control in STOP mode
```

## Standalone Mode (no MIDI panel)

When the USB Host Shield fails to initialise, the firmware enters standalone
mode and uses the SEN0192 radar sensor on pin **D2** to trigger playback.

### Timing

```
Power-on
    |
    +-- RADAR_STARTUP_DELAY_MS (10 s)   <- arm settles, radar deaf
    |
    +-- Radar armed
           |
           +-- Motion detected -> animation plays (single-shot)
           |
           +-- TRIGGER_COOLDOWN_MS (60 s)  <- radar deaf after playback
                  |
                  +-- Radar armed again
```

### Sensitivity

The sensor outputs active-LOW pulses; the rate increases with motion intensity.
The firmware counts pulses in a sliding window and triggers when the rate
exceeds a threshold:

```cpp
#define RADAR_MIN_PULSES_PER_SEC    3    // required pulse rate to trigger
#define RADAR_WINDOW_MS             500  // counting window in ms
```

`RADAR_MIN_PULSES_PER_SEC` is the main tuning constant:

| Value | Sensitivity |
|-------|-------------|
| 1 | Very sensitive — any faint movement |
| 3 | Moderate — someone walking past at normal distance (default) |
| 6 | Low — requires close or fast movement |

The serial monitor shows pulse count and threshold on every window containing
motion, and reports cooldown time remaining when a trigger is blocked:

```
Radar armed -- listening for motion.
Radar: 4 pulses in 500ms (threshold=2) -- TRIGGER
Radar: 3 pulses in 500ms (threshold=2) -- cooldown, 47s remaining
```

### All radar constants

```cpp
#define RADAR_PIN                   2         // digital input pin (active-LOW)
#define RADAR_STARTUP_DELAY_MS      10000UL   // dead time after power-on
#define RADAR_MIN_PULSES_PER_SEC    3         // sensitivity (pulses/second)
#define RADAR_WINDOW_MS             500UL     // pulse-counting window
#define TRIGGER_COOLDOWN_MS         60000UL   // dead time after animation ends
```

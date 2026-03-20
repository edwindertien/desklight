#pragma once
// ============================================================
//  Animations.h  —  event-based animation storage
// ============================================================
//
//  OLD format (stream):  every 50 ms one row of 8 bytes = 8 bytes/frame
//  NEW format (events):  only changed channels are stored
//
//  Each event record in PROGMEM:
//    byte  deltaMs_hi   — high byte of uint16 delta-time in ms
//    byte  deltaMs_lo   — low  byte of uint16 delta-time in ms
//    byte  channelMask  — bit n set  → channel n value follows
//    byte  values[]     — one byte per set bit, in channel order (0..7)
//
//  A record with deltaMs == 0xFFFF marks end-of-animation.
//
//  Channels 0–4 : MX-28 servos (position 0-127 → maps to angleOffset ± range)
//  Channel  5   : LED ring colour   (0-127)
//  Channel  6   : LED ring brightness (0-127)
//  Channel  7   : LED ring pattern  (0-127)
// ============================================================

#include <avr/pgmspace.h>

// ---- iterator helper ---------------------------------------
struct AnimEvent {
    uint16_t deltaMs;
    uint8_t  channelMask;
    uint8_t  values[8];   // values[i] valid iff bit i set in channelMask
};

// Read next event from PROGMEM stream; returns false at end-of-animation.
// 'cursor' is updated to point past the consumed bytes.
inline bool animNextEvent(const byte* base, uint16_t& cursor, AnimEvent& ev) {
    uint16_t d = ((uint16_t)pgm_read_byte_near(base + cursor) << 8)
               |  (uint16_t)pgm_read_byte_near(base + cursor + 1);
    cursor += 2;
    if (d == 0xFFFF) return false;         // end marker
    ev.deltaMs    = d;
    ev.channelMask = pgm_read_byte_near(base + cursor++);
    for (uint8_t ch = 0; ch < 8; ch++) {
        if (ev.channelMask & (1 << ch))
            ev.values[ch] = pgm_read_byte_near(base + cursor++);
    }
    return true;
}

// ---- generated animation data (see tools/convert_animation.py) ----
#include "animation_data.h"

// ============================================================
// TAS5806M / TAS5806MD (TSSOP-38, marked TAS5807M) + ESP32 A2DP
// v2
//
// Arduino-ESP32 core 3.x (ESP-IDF 5.x), ESP32-A2DP library 1.8+
//
// Features:
//  - Phone volume -> TAS digital volume (no software scaling)
//  - EQ: Samsung EQ on/off + bass / mid / treble + presets
//  - Buttons (vol+/-, play/pause, next, prev) and status LED
//  - Channel mode (stereo / mono / swap), balance
//  - Analog gain limit
//  - Fault monitor with automatic restart
//  - Track title / artist in Serial
//  - All settings saved in flash (NVS)
//
// Serial Monitor: 115200, any line ending. Type "help".
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>

#include "BluetoothA2DPSink.h"

#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================================================
// PINS
// ============================================================

#define TAS_ADDR       0x2C

#define SDA_PIN        21
#define SCL_PIN        22

#define I2S_BCLK_PIN   26
#define I2S_LRCLK_PIN  25
#define I2S_DATA_PIN   27

// Buttons: between pin and GND (internal pull-up). -1 = not used.
// Do not use GPIO 34..39 without external pull-up resistors.
#define BTN_VOL_UP     32
#define BTN_VOL_DOWN   33
#define BTN_PLAY       13   // short: play/pause, long: next EQ preset
#define BTN_NEXT       14   // short: next track
#define BTN_PREV        4   // short: previous track

#define LED_PIN         2   // on-board LED on most DevKits, -1 = none
#define LED_ON_LEVEL   HIGH

// ============================================================
// SETTINGS
// ============================================================

#define BT_NAME        "TAS5807 Speaker"
#define OUT_RATE       48000
#define USE_APLL       1

// Phone volume 127 -> this DIG_VOL level (Samsung uses 0x3D = -6.5 dB).
// Can be changed at runtime: "maxvol -10"
#define VOL_MAX_DB_DEFAULT  -6.5f
// Phone volume 1 -> max - VOL_RANGE_DB, phone volume 0 -> mute
#define VOL_RANGE_DB        50.0f
#define VOL_DEFAULT         80     // 0..127
#define VOL_BUTTON_STEP     6      // ~2.4 dB per click

// 0: buttons change volume locally (always works, phone slider may not follow)
// 1: buttons send AVRCP vol+/- to the phone (phone slider follows,
//    but some phones ignore these commands)
#define BUTTON_VOL_VIA_PHONE 0

// Samsung BQ1 has +16 dB gain. When Samsung EQ is off,
// volume is raised by this amount so loudness stays similar.
#define SAMSUNG_EQ_GAIN_DB  16.0f

// User EQ bands
#define EQ_BASS_HZ     100.0
#define EQ_MID_HZ      1000.0
#define EQ_MID_Q       0.7
#define EQ_TREBLE_HZ   8000.0
#define EQ_LIMIT_DB    12

// Free biquads (pass-through in Samsung config) used for user EQ
#define BQ_USER_BASS   11
#define BQ_USER_MID    13
#define BQ_USER_TREBLE 14

// Also write right-channel biquads (left/right are normally ganged,
// writing both is harmless and works in both cases)
#define EQ_WRITE_RIGHT 1

// DSP_MISC (0x66) values taken from the Samsung dump:
// 0x05 while coefficients are written, 0x04 afterwards
#define DSP_MISC_WRITE 0x05
#define DSP_MISC_RUN   0x04

// Samsung dump writes DEVICE_CTRL_1 (0x02) = 0x51:
// FSW_SEL = 101, 1SPW, BTL. Code 101 is NOT listed in the
// TAS5806M(D) datasheet (000=768k, 001=384k, 011=480k, 100=576k).
// Leave commented unless you see faults/heat/distortion.
//   0x01 = 768 kHz, 1SPW, BTL  (TI example for TAS5806MD)
//   0x11 = 384 kHz, 1SPW, BTL
// #define OVERRIDE_CTRL1 0x01

// ============================================================
// TYPES
// (declared before any function: Arduino IDE inserts
//  function prototypes above the first function)
// ============================================================

struct Tx {
  uint8_t reg;
  uint8_t len;
  uint8_t d[4];
};

struct Stereo16 {
  int16_t l;
  int16_t r;
};

// Biquad in TI convention:
// y = b0*x + b1*x1 + b2*x2 + a1*y1 + a2*y2
struct BQ {
  double b0, b1, b2, a1, a2;
};

#define CFG_VERSION 1

struct Settings {
  uint8_t version;
  uint8_t phoneVol;    // 0..127
  int8_t  maxDbX2;     // max volume in 0.5 dB
  int8_t  bass;        // dB
  int8_t  mid;
  int8_t  treble;
  int8_t  preset;      // -1 = custom
  uint8_t samsungEq;   // 0/1
  uint8_t mode;        // 0 stereo, 1 mono, 2 swap
  int8_t  balance;     // -100 (left) .. +100 (right)
  uint8_t again;       // 0..31, 0 = 0 dB, step -0.5 dB
};

struct Preset {
  const char *name;
  int8_t bass;
  int8_t mid;
  int8_t treble;
};

struct Button {
  int8_t   pin;
  bool     repeat;       // true: fires on press + auto-repeat
  bool     raw;
  bool     pressed;
  bool     longDone;
  uint32_t rawChangedAt;
  uint32_t pressedAt;
  uint32_t lastRepeatAt;
};

enum BtnEvent : uint8_t {
  EV_NONE,
  EV_SHORT,
  EV_LONG,
  EV_REPEAT
};

uint32_t fifoPopBlock(Stereo16 *dst, uint32_t maxFrames);

// ============================================================
// GLOBALS
// ============================================================

BluetoothA2DPSink   a2dp_sink;
A2DPNoVolumeControl noSoftwareVolume;   // volume is done by the TAS
Preferences         prefs;

i2s_chan_handle_t txHandle = nullptr;

Settings cfg;
bool     cfgDirty   = false;
uint32_t cfgDirtyAt = 0;

float    eqHeadroomDb = 0;     // input attenuation for EQ boost
uint8_t  curVolReg    = 0xFF;

volatile bool     tasIsPlaying = false;
volatile bool     playRequest  = false;
volatile bool     muteRequest  = false;
volatile bool     btConnected  = false;
volatile bool     newConnection = false;
volatile int      pendingPhoneVol = -1;
volatile uint32_t lastPcmMillis = 0;

bool     forcedPlay     = false;
bool     faultLatched   = false;
uint32_t lastPlayCmdAt  = 0;

const Preset PRESETS[] = {
  {"flat",   0,  0,  0},
  {"bass",   6,  0,  1},
  {"rock",   4, -2,  3},
  {"vocal", -2,  3,  1},
  {"night", -6,  2, -2},
};
const int PRESET_COUNT = sizeof(PRESETS) / sizeof(PRESETS[0]);

const char *MODE_NAMES[] = {"stereo", "mono", "swap"};

Button btnUp   = {BTN_VOL_UP,   true,  false, false, false, 0, 0, 0};
Button btnDown = {BTN_VOL_DOWN, true,  false, false, false, 0, 0, 0};
Button btnPlay = {BTN_PLAY,     false, false, false, false, 0, 0, 0};
Button btnNext = {BTN_NEXT,     false, false, false, false, 0, 0, 0};
Button btnPrev = {BTN_PREV,     false, false, false, false, 0, 0, 0};

// ============================================================
// SAMSUNG CONFIG (captured, with 2 missing coefficients restored)
//
// Register 0x7F selects the book ONLY on page 0.
// A 4-byte write to 0x7C on other pages is a coefficient.
// ============================================================

#define W1(r,a)       {r,1,{a,0,0,0}}
#define W2(r,a,b)     {r,2,{a,b,0,0}}
#define W4(r,a,b,c,d) {r,4,{a,b,c,d}}

static const Tx initSeq[] = {

  W1(0x00,0x00),
  W1(0x7F,0x00),
  W1(0x78,0x80),
  W1(0x03,0x02),
  W1(0x01,0x11),          // soft reset (registers + DSP) -> long delay below

  W1(0x00,0x00),
  W1(0x7F,0x00),
  W1(0x46,0x01),
  W2(0x4E,0xFB,0xF0),
  W1(0x02,0x51),          // FSW_SEL=101 (undocumented), 1SPW, BTL
  W2(0x53,0x60,0x00),     // loop BW 175 kHz; AGAIN = 0 dB
  W1(0x50,0x00),

  W1(0x00,0x00),
  W1(0x7F,0x00),
  W1(0x66,0x05),
  W1(0x4C,0x3D),
  W1(0x30,0x00),
  W1(0x77,0x00),          // TAS5806MD: headphone driver gain 0, muted, shut down
  W1(0x03,0x0E),

  // BOOK 8C PAGE 2A
  W1(0x00,0x00),
  W1(0x7F,0x8C),
  W1(0x00,0x2A),

  W4(0x30,0x00,0x71,0x94,0x9A),

  // BOOK 8C PAGE 29
  W1(0x00,0x00),
  W1(0x7F,0x8C),
  W1(0x00,0x29),

  W4(0x18,0x00,0x80,0x00,0x00),
  W4(0x1C,0x00,0x00,0x00,0x00),
  W4(0x20,0x00,0x00,0x00,0x00),
  W4(0x24,0x00,0x80,0x00,0x00),

  // BOOK 8C PAGE 2A (DSP volume, temporarily +15 dB)
  W1(0x00,0x00),
  W1(0x7F,0x8C),
  W1(0x00,0x2A),

  W4(0x24,0x02,0xC4,0x89,0x2E),
  W4(0x28,0x02,0xC4,0x89,0x2E),

  // BOOK AA PAGE 2A
  W1(0x00,0x00),
  W1(0x7F,0xAA),
  W1(0x00,0x2A),

  W4(0x34,0x7F,0xFF,0xFF,0xFF),
  W4(0x38,0x00,0x00,0x00,0x00),
  W4(0x3C,0x00,0x00,0x00,0x00),
  W4(0x40,0x00,0x00,0x00,0x00),
  W4(0x44,0x00,0x00,0x00,0x00),

  W4(0x48,0x7F,0xFF,0xFF,0xFF),
  W4(0x4C,0x00,0x00,0x00,0x00),
  W4(0x50,0x00,0x00,0x00,0x00),
  W4(0x54,0x00,0x00,0x00,0x00),
  W4(0x58,0x00,0x00,0x00,0x00),

  // BOOK AA PAGE 2B
  W1(0x00,0x2B),

  W4(0x0C,0x7F,0xFF,0xFF,0xFF),
  W4(0x10,0x00,0x00,0x00,0x00),
  W4(0x14,0x00,0x00,0x00,0x00),
  W4(0x18,0x00,0x00,0x00,0x00),
  W4(0x1C,0x00,0x00,0x00,0x00),

  W4(0x20,0x7F,0xFF,0xFF,0xFF),
  W4(0x24,0x00,0x00,0x00,0x00),
  W4(0x28,0x00,0x00,0x00,0x00),
  W4(0x2C,0x00,0x00,0x00,0x00),
  W4(0x30,0x00,0x00,0x00,0x00),

  // BOOK 8C PAGE 2B
  W1(0x00,0x00),
  W1(0x7F,0x8C),
  W1(0x00,0x2B),

  W4(0x34,0x00,0x44,0x32,0x13),
  W4(0x38,0x00,0x44,0x32,0x13),
  W4(0x3C,0x00,0x03,0x69,0xC5),
  W4(0x40,0x00,0x00,0x00,0x00),
  W4(0x44,0x00,0x00,0x00,0x00),
  W4(0x48,0x00,0x00,0x00,0x00),
  W4(0x4C,0xF9,0xDA,0xBC,0x21),
  W4(0x50,0xFE,0x01,0xC0,0x79),
  W4(0x54,0x00,0x00,0x00,0x00),
  W4(0x58,0x00,0x00,0x00,0x00),

  // BOOK 8C PAGE 2C
  W1(0x00,0x2C),

  W4(0x0C,0x00,0x00,0x00,0x00),
  W4(0x10,0x00,0x00,0x00,0x00),
  W4(0x14,0x00,0x80,0x00,0x00),
  W4(0x18,0x00,0x00,0x00,0x00),
  W4(0x1C,0x00,0x80,0x00,0x00),
  W4(0x20,0x00,0x00,0x00,0x00),

  W4(0x28,0x00,0x00,0x00,0x00),
  W4(0x2C,0x00,0x80,0x00,0x00),

  W4(0x34,0x00,0x80,0x00,0x00),
  W4(0x38,0x00,0x00,0x00,0x00),

  W4(0x48,0x00,0x00,0x00,0x00),
  W4(0x4C,0x00,0x80,0x00,0x00),

  W4(0x5C,0x00,0x00,0x71,0x99),
  W4(0x60,0x00,0x23,0x05,0x53),
  W4(0x64,0x06,0xCB,0x9A,0x26),
  W4(0x68,0xC0,0x00,0x00,0x00),
  W4(0x6C,0x11,0xB2,0xB9,0xA8),

  // BOOK 8C PAGE 2D
  W1(0x00,0x2D),

  W4(0x18,0x6E,0x4D,0x46,0x58),
  W4(0x1C,0x00,0x00,0x57,0x62),
  W4(0x20,0x00,0x00,0x00,0x00),
  W4(0x24,0x00,0x00,0x00,0x00),
  W4(0x28,0x00,0x00,0x00,0x00),
  W4(0x2C,0x00,0x80,0x00,0x00),

  W4(0x58,0x00,0x44,0x32,0x13),
  W4(0x5C,0x00,0x44,0x32,0x13),
  W4(0x60,0x00,0x03,0x69,0xC5),
  W4(0x64,0x00,0x00,0x00,0x00),
  W4(0x68,0x00,0x00,0x00,0x00),
  W4(0x6C,0x00,0x00,0x00,0x00),

  W4(0x70,0xF9,0xDA,0xBC,0x21),
  W4(0x74,0xFE,0x01,0xC0,0x79),
  W4(0x78,0x00,0x00,0x00,0x00),
  W4(0x7C,0x00,0x00,0x00,0x00),   // coefficient 0x7C..0x7F, book unchanged

  // BOOK 8C PAGE 2E (still book 8C)
  W1(0x00,0x2E),

  W4(0x08,0x00,0x4C,0xCC,0xCD),
  W4(0x10,0x00,0x4C,0xCC,0xCD),
  W4(0x18,0x00,0x80,0x00,0x00),
  W4(0x1C,0x40,0x00,0x00,0x00),
  W4(0x20,0x40,0x00,0x00,0x00),

  W1(0x00,0x00),
  W1(0x7F,0x00),
  W1(0x03,0x0E),

  // BOOK 8C PAGE 00
  W1(0x00,0x00),
  W1(0x7F,0x8C),
  W1(0x00,0x00),

  W2(0x7D,0x00,0x00),

  // ----------------------------------------------------------
  // BOOK AA PAGE 24 : biquads, format 5.27, order b0 b1 b2 a1 a2
  // ----------------------------------------------------------
  W1(0x00,0x00),
  W1(0x7F,0xAA),
  W1(0x00,0x24),

  // BQ1 (high-pass ~35 Hz, +16 dB)
  // FIX: b0 was MISSING in the dump. b1 = -2*b2 exactly -> b0 = b2
  W4(0x18,0x32,0x50,0x36,0x45),
  W4(0x1C,0x9B,0x5F,0x93,0x76),
  W4(0x20,0x32,0x50,0x36,0x45),
  W4(0x24,0x0F,0xF2,0xBB,0x0B),
  W4(0x28,0xF8,0x0D,0x39,0xFD),

  // BQ2
  W4(0x2C,0x07,0xFF,0x8E,0x36),
  W4(0x30,0xF0,0x01,0xD6,0x9D),
  W4(0x34,0x07,0xFE,0xA9,0x8C),
  W4(0x38,0x0F,0xFE,0x29,0x63),
  W4(0x3C,0xF8,0x01,0xC8,0x3F),

  // BQ3 (peaking)
  // FIX: b2 was MISSING. For peaking EQ: b0 + b2 = 1 - a2
  W4(0x40,0x08,0x03,0x4C,0x60),
  W4(0x44,0xF0,0x06,0xB9,0x83),
  W4(0x48,0x07,0xF6,0x12,0xD5),
  W4(0x4C,0x0F,0xF9,0x46,0x7D),
  W4(0x50,0xF8,0x06,0xA0,0xCB),

  // BQ4
  W4(0x54,0x07,0xF8,0xB0,0x84),
  W4(0x58,0xF0,0x1B,0x09,0x4D),
  W4(0x5C,0x07,0xEC,0xE2,0xE1),
  W4(0x60,0x0F,0xE4,0xF6,0xB3),
  W4(0x64,0xF8,0x1A,0x6C,0x9B),

  // BQ5
  W4(0x68,0x07,0xF3,0x97,0x96),
  W4(0x6C,0xF0,0x2D,0xDA,0x22),
  W4(0x70,0x07,0xDF,0x8F,0x0D),
  W4(0x74,0x0F,0xD2,0x25,0xDE),
  W4(0x78,0xF8,0x2C,0xD9,0x5D),

  // BQ6 b0 (continues on page 25)
  W4(0x7C,0x07,0xF7,0x3C,0x75),

  // BOOK AA PAGE 25 (still book AA)
  W1(0x00,0x25),

  W4(0x08,0xF0,0x32,0x4F,0xDB),
  W4(0x0C,0x07,0xD9,0x44,0xD4),
  W4(0x10,0x0F,0xCD,0xB0,0x25),
  W4(0x14,0xF8,0x2F,0x7E,0xB6),
  W4(0x18,0x07,0xE5,0x28,0x16),
  W4(0x1C,0xF0,0xAA,0x4C,0x5C),
  W4(0x20,0x07,0x78,0xF7,0xE9),
  W4(0x24,0x0F,0x55,0xB3,0xA4),
  W4(0x28,0xF8,0xA1,0xE0,0x01),
  W4(0x2C,0x07,0x96,0x6D,0x4E),
  W4(0x30,0xF1,0xAD,0xD4,0xE0),
  W4(0x34,0x06,0xFC,0x75,0xCC),
  W4(0x38,0x0E,0x52,0x2B,0x20),
  W4(0x3C,0xF9,0x6D,0x1C,0xE7),
  W4(0x40,0x07,0xC4,0x61,0xF9),
  W4(0x44,0xF2,0x23,0xD1,0xAE),
  W4(0x48,0x06,0xF8,0x86,0x91),
  W4(0x4C,0x0D,0xDC,0x2E,0x52),
  W4(0x50,0xF9,0x43,0x17,0x76),
  W4(0x54,0x09,0x11,0xCE,0x57),
  W4(0x58,0xF4,0xFB,0x56,0x90),
  W4(0x5C,0x04,0x2E,0x93,0x3D),
  W4(0x60,0x0B,0x04,0xA9,0x70),
  W4(0x64,0xFA,0xBF,0x9E,0x6B),

  W4(0x68,0x08,0x00,0x00,0x00),
  W4(0x6C,0x00,0x00,0x00,0x00),
  W4(0x70,0x00,0x00,0x00,0x00),
  W4(0x74,0x00,0x00,0x00,0x00),
  W4(0x78,0x00,0x00,0x00,0x00),

  W4(0x7C,0x08,0x00,0x00,0x00),   // coefficient, book unchanged

  // BOOK AA PAGE 26 (still book AA)
  W1(0x00,0x26),

  W4(0x08,0x00,0x00,0x00,0x00),
  W4(0x0C,0x00,0x00,0x00,0x00),
  W4(0x10,0x00,0x00,0x00,0x00),
  W4(0x14,0x00,0x00,0x00,0x00),

  W4(0x18,0x08,0x00,0x00,0x00),

  W4(0x1C,0x00,0x00,0x00,0x00),
  W4(0x20,0x00,0x00,0x00,0x00),
  W4(0x24,0x00,0x00,0x00,0x00),
  W4(0x28,0x00,0x00,0x00,0x00),

  W4(0x2C,0x08,0x00,0x00,0x00),

  W4(0x30,0x00,0x00,0x00,0x00),
  W4(0x34,0x00,0x00,0x00,0x00),
  W4(0x38,0x00,0x00,0x00,0x00),
  W4(0x3C,0x00,0x00,0x00,0x00),

  // FINAL: DSP volume back to 0 dB
  W1(0x00,0x00),
  W1(0x7F,0x8C),
  W1(0x00,0x2A),

  W4(0x24,0x00,0x80,0x00,0x00),
  W4(0x28,0x00,0x80,0x00,0x00),

  W1(0x00,0x00),
  W1(0x7F,0x00),

  W1(0x00,0x00),
  W1(0x66,0x04),

  W1(0xF7,0x00),   // from the dump (odd register, kept as captured)

  W1(0x00,0x00),
  W1(0x7F,0x00),
  W1(0x4C,0x3D),

  W1(0x00,0x00),
  W1(0x7F,0x00),

  W1(0x03,0x0E),
  W1(0x03,0x0E),

  W1(0x00,0x00),
  W1(0x7F,0x00)
};

// Samsung EQ, left BQ1..BQ10, 5.27 format (b0 b1 b2 a1 a2).
// Same values as in initSeq, used to switch the EQ at runtime.
static const uint32_t SAMSUNG_BQ[10][5] = {
  {0x32503645, 0x9B5F9376, 0x32503645, 0x0FF2BB0B, 0xF80D39FD},  // BQ1
  {0x07FF8E36, 0xF001D69D, 0x07FEA98C, 0x0FFE2963, 0xF801C83F},  // BQ2
  {0x08034C60, 0xF006B983, 0x07F612D5, 0x0FF9467D, 0xF806A0CB},  // BQ3
  {0x07F8B084, 0xF01B094D, 0x07ECE2E1, 0x0FE4F6B3, 0xF81A6C9B},  // BQ4
  {0x07F39796, 0xF02DDA22, 0x07DF8F0D, 0x0FD225DE, 0xF82CD95D},  // BQ5
  {0x07F73C75, 0xF0324FDB, 0x07D944D4, 0x0FCDB025, 0xF82F7EB6},  // BQ6
  {0x07E52816, 0xF0AA4C5C, 0x0778F7E9, 0x0F55B3A4, 0xF8A1E001},  // BQ7
  {0x07966D4E, 0xF1ADD4E0, 0x06FC75CC, 0x0E522B20, 0xF96D1CE7},  // BQ8
  {0x07C461F9, 0xF223D1AE, 0x06F88691, 0x0DDC2E52, 0xF9431776},  // BQ9
  {0x0911CE57, 0xF4FB5690, 0x042E933D, 0x0B04A970, 0xFABF9E6B},  // BQ10
};

// ============================================================
// I2C LOW LEVEL (with book/page cache)
// ============================================================

uint8_t cacheBook = 0xFF;
uint8_t cachePage = 0xFF;

void invalidatePageCache()
{
  cacheBook = 0xFF;
  cachePage = 0xFF;
}

bool tw(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(TAS_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool tr(uint8_t reg, uint8_t &value)
{
  Wire.beginTransmission(TAS_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0)
    return false;

  if (Wire.requestFrom((uint8_t)TAS_ADDR, (uint8_t)1) != 1)
    return false;

  value = Wire.read();
  return true;
}

bool writeTx(const Tx &t)
{
  Wire.beginTransmission(TAS_ADDR);
  Wire.write(t.reg);
  for (uint8_t i = 0; i < t.len; i++)
    Wire.write(t.d[i]);
  return Wire.endTransmission() == 0;
}

bool selectBookPage(uint8_t book, uint8_t page)
{
  if (cacheBook == book && cachePage == page)
    return true;

  bool ok = tw(0x00, 0x00) && tw(0x7F, book) && tw(0x00, page);

  if (ok)
  {
    cacheBook = book;
    cachePage = page;
  }
  else
  {
    invalidatePageCache();
  }
  return ok;
}

bool book0()
{
  return selectBookPage(0x00, 0x00);
}

bool dspWrite32(uint8_t book, uint8_t page, uint8_t reg, int32_t value)
{
  if (!selectBookPage(book, page))
    return false;

  uint32_t v = (uint32_t)value;

  Wire.beginTransmission(TAS_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(v >> 24));
  Wire.write((uint8_t)(v >> 16));
  Wire.write((uint8_t)(v >> 8));
  Wire.write((uint8_t)(v));
  return Wire.endTransmission() == 0;
}

void show(uint8_t reg, const char *name)
{
  uint8_t v;
  Serial.printf("R%02X %-10s = ", reg, name);
  if (tr(reg, v))
    Serial.printf("%02X\n", v);
  else
    Serial.println("ERROR");
}

void waitTas()
{
  Serial.println("Waiting TAS...");

  while (true)
  {
    Wire.beginTransmission(TAS_ADDR);
    if (Wire.endTransmission() == 0)
      break;
    delay(100);
  }

  Serial.println("TAS ACK OK");
}

// ============================================================
// LOAD SAMSUNG CONFIG
// ============================================================

bool loadSamsung()
{
  size_t count = sizeof(initSeq) / sizeof(initSeq[0]);

  Serial.printf("Loading %u Samsung writes...\n", (unsigned)count);

  invalidatePageCache();

  uint8_t curPage = 0;
  uint8_t curBook = 0;

  for (size_t i = 0; i < count; i++)
  {
    const Tx &t = initSeq[i];

    if (t.reg == 0x00 && t.len == 1)
      curPage = t.d[0];
    else if (t.reg == 0x7F && t.len == 1 && curPage == 0)
      curBook = t.d[0];

    if (!writeTx(t))
    {
      Serial.printf("INIT FAIL #%u BOOK=%02X PAGE=%02X REG=%02X\n",
                    (unsigned)(i + 1), curBook, curPage, t.reg);
      invalidatePageCache();
      return false;
    }

    delayMicroseconds(500);

    if (curPage == 0 && curBook == 0)
    {
      if (t.reg == 0x01)
        delay(20);
      else if (t.reg == 0x03)
        delay(5);
    }
  }

  invalidatePageCache();
  Serial.println("Samsung DSP config loaded");
  return true;
}

// ============================================================
// SETTINGS (NVS)
// ============================================================

void setDefaults(Settings &s)
{
  s.version   = CFG_VERSION;
  s.phoneVol  = VOL_DEFAULT;
  s.maxDbX2   = (int8_t)lround(VOL_MAX_DB_DEFAULT * 2.0f);
  s.bass      = 0;
  s.mid       = 0;
  s.treble    = 0;
  s.preset    = 0;
  s.samsungEq = 1;
  s.mode      = 0;
  s.balance   = 0;
  s.again     = 0;
}

int clampInt(int v, int lo, int hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void sanitize(Settings &s)
{
  s.phoneVol  = (uint8_t)clampInt(s.phoneVol, 0, 127);
  s.maxDbX2   = (int8_t)clampInt(s.maxDbX2, -80, 0);
  s.bass      = (int8_t)clampInt(s.bass,   -EQ_LIMIT_DB, EQ_LIMIT_DB);
  s.mid       = (int8_t)clampInt(s.mid,    -EQ_LIMIT_DB, EQ_LIMIT_DB);
  s.treble    = (int8_t)clampInt(s.treble, -EQ_LIMIT_DB, EQ_LIMIT_DB);
  s.preset    = (int8_t)clampInt(s.preset, -1, PRESET_COUNT - 1);
  s.samsungEq = s.samsungEq ? 1 : 0;
  s.mode      = (uint8_t)clampInt(s.mode, 0, 2);
  s.balance   = (int8_t)clampInt(s.balance, -100, 100);
  s.again     = (uint8_t)clampInt(s.again, 0, 31);
}

void loadSettings()
{
  setDefaults(cfg);

  prefs.begin("tasamp", false);

  if (prefs.getBytesLength("cfg") == sizeof(Settings))
  {
    Settings tmp;
    prefs.getBytes("cfg", &tmp, sizeof(tmp));
    if (tmp.version == CFG_VERSION)
      cfg = tmp;
  }

  sanitize(cfg);
  Serial.println("Settings loaded");
}

void saveSettings()
{
  prefs.putBytes("cfg", &cfg, sizeof(cfg));
  cfgDirty = false;
  Serial.println("Settings saved");
}

void markDirty()
{
  cfgDirty = true;
  cfgDirtyAt = millis();
}

// ============================================================
// BIQUAD DESIGN (RBJ cookbook -> TI 5.27)
// ============================================================

BQ bqFlat()
{
  BQ r = {1.0, 0.0, 0.0, 0.0, 0.0};
  return r;
}

BQ designPeak(double f0, double q, double gainDb)
{
  if (fabs(gainDb) < 0.01)
    return bqFlat();

  double A  = pow(10.0, gainDb / 40.0);
  double w0 = 2.0 * M_PI * f0 / OUT_RATE;
  double cs = cos(w0);
  double al = sin(w0) / (2.0 * q);
  double a0 = 1.0 + al / A;

  BQ r;
  r.b0 = (1.0 + al * A) / a0;
  r.b1 = (-2.0 * cs) / a0;
  r.b2 = (1.0 - al * A) / a0;
  r.a1 = (2.0 * cs) / a0;
  r.a2 = -(1.0 - al / A) / a0;
  return r;
}

BQ designShelf(bool high, double f0, double gainDb)
{
  if (fabs(gainDb) < 0.01)
    return bqFlat();

  double A  = pow(10.0, gainDb / 40.0);
  double w0 = 2.0 * M_PI * f0 / OUT_RATE;
  double cs = cos(w0);
  double al = sin(w0) / 2.0 * sqrt(2.0);   // shelf slope S = 1
  double sq = 2.0 * sqrt(A) * al;

  double b0, b1, b2, a0, a1, a2;

  if (!high)
  {
    b0 = A * ((A + 1) - (A - 1) * cs + sq);
    b1 = 2 * A * ((A - 1) - (A + 1) * cs);
    b2 = A * ((A + 1) - (A - 1) * cs - sq);
    a0 = (A + 1) + (A - 1) * cs + sq;
    a1 = -2 * ((A - 1) + (A + 1) * cs);
    a2 = (A + 1) + (A - 1) * cs - sq;
  }
  else
  {
    b0 = A * ((A + 1) + (A - 1) * cs + sq);
    b1 = -2 * A * ((A - 1) + (A + 1) * cs);
    b2 = A * ((A + 1) + (A - 1) * cs - sq);
    a0 = (A + 1) - (A - 1) * cs + sq;
    a1 = 2 * ((A - 1) - (A + 1) * cs);
    a2 = (A + 1) - (A - 1) * cs - sq;
  }

  BQ r;
  r.b0 = b0 / a0;
  r.b1 = b1 / a0;
  r.b2 = b2 / a0;
  r.a1 = -a1 / a0;   // TI stores negated a1, a2
  r.a2 = -a2 / a0;
  return r;
}

double bqMagDb(const BQ &q, double f)
{
  double w  = 2.0 * M_PI * f / OUT_RATE;
  double c1 = cos(w),     s1 = sin(w);
  double c2 = cos(2 * w), s2 = sin(2 * w);

  double nr = q.b0 + q.b1 * c1 + q.b2 * c2;
  double ni = -(q.b1 * s1 + q.b2 * s2);
  double dr = 1.0 - q.a1 * c1 - q.a2 * c2;
  double di = q.a1 * s1 + q.a2 * s2;

  return 10.0 * log10((nr * nr + ni * ni) / (dr * dr + di * di));
}

// max boost of the whole user EQ over 20 Hz..20 kHz
float computeHeadroom(const BQ *bq, int n)
{
  double worst = 0;

  for (int i = 0; i < 64; i++)
  {
    double f = 20.0 * pow(1000.0, i / 63.0);
    double sum = 0;
    for (int k = 0; k < n; k++)
      sum += bqMagDb(bq[k], f);
    if (sum > worst)
      worst = sum;
  }

  return (float)(ceil(worst * 2.0) / 2.0);
}

int32_t toQ(double x, double scale)
{
  long long v = llround(x * scale);
  if (v >  2147483647LL) v =  2147483647LL;
  if (v < -2147483648LL) v = -2147483648LL;
  return (int32_t)v;
}

void bqToFixed(const BQ &q, int32_t *out)
{
  const double S = 134217728.0;   // 2^27 -> 5.27
  out[0] = toQ(q.b0, S);
  out[1] = toQ(q.b1, S);
  out[2] = toQ(q.b2, S);
  out[3] = toQ(q.a1, S);
  out[4] = toQ(q.a2, S);
}

// ============================================================
// DSP BLOCKS
// Addresses: TI SLOA263 "TAS5805M, TAS5806M, TAS5806MD Process Flows"
// ============================================================

// Book 0xAA: left BQ1 starts at page 0x24 reg 0x18,
// right BQ1 at page 0x26 reg 0x54. 15 BQs x 5 coefficients.
void bqCoefAddr(uint8_t ch, uint8_t bq, uint8_t k, uint8_t &page, uint8_t &reg)
{
  uint32_t idx = (ch ? 75u : 0u) + (uint32_t)(bq - 1) * 5u + k;

  if (idx < 26)
  {
    page = 0x24;
    reg  = (uint8_t)(0x18 + idx * 4);
  }
  else
  {
    idx -= 26;
    page = (uint8_t)(0x25 + idx / 30);
    reg  = (uint8_t)(0x08 + (idx % 30) * 4);
  }
}

bool writeBiquad(uint8_t bq, const int32_t *c)
{
  bool ok = true;
  uint8_t channels = EQ_WRITE_RIGHT ? 2 : 1;

  for (uint8_t ch = 0; ch < channels; ch++)
  {
    for (uint8_t k = 0; k < 5; k++)
    {
      uint8_t page, reg;
      bqCoefAddr(ch, bq, k, page, reg);
      ok &= dspWrite32(0xAA, page, reg, c[k]);
    }
  }
  return ok;
}

// Input mixer: Book 0x8C Page 0x29, format 9.23
//   0x18 L->L, 0x1C R->L, 0x20 L->R, 0x24 R->R
bool writeMixer()
{
  double h  = pow(10.0, -eqHeadroomDb / 20.0);
  double gl = 1.0;
  double gr = 1.0;

  if (cfg.balance > 0) gl = 1.0 - cfg.balance / 100.0;
  if (cfg.balance < 0) gr = 1.0 + cfg.balance / 100.0;

  double ll = 1, rl = 0, lr = 0, rr = 1;

  if (cfg.mode == 1)        // mono
  {
    ll = rl = lr = rr = 0.5;
  }
  else if (cfg.mode == 2)   // swap
  {
    ll = 0; rl = 1; lr = 1; rr = 0;
  }

  const double S = 8388608.0;   // 2^23 -> 9.23
  bool ok = true;
  ok &= dspWrite32(0x8C, 0x29, 0x18, toQ(ll * gl * h, S));
  ok &= dspWrite32(0x8C, 0x29, 0x1C, toQ(rl * gl * h, S));
  ok &= dspWrite32(0x8C, 0x29, 0x20, toQ(lr * gr * h, S));
  ok &= dspWrite32(0x8C, 0x29, 0x24, toQ(rr * gr * h, S));
  return ok;
}

bool writeSamsungEq(bool on)
{
  bool ok = true;
  int32_t c[5];

  for (uint8_t i = 0; i < 10; i++)
  {
    if (on)
    {
      for (uint8_t k = 0; k < 5; k++)
        c[k] = (int32_t)SAMSUNG_BQ[i][k];
    }
    else
    {
      bqToFixed(bqFlat(), c);
    }
    ok &= writeBiquad(i + 1, c);
  }
  return ok;
}

bool writeUserEq()
{
  BQ bq[3];
  bq[0] = designShelf(false, EQ_BASS_HZ, cfg.bass);
  bq[1] = designPeak(EQ_MID_HZ, EQ_MID_Q, cfg.mid);
  bq[2] = designShelf(true, EQ_TREBLE_HZ, cfg.treble);

  eqHeadroomDb = computeHeadroom(bq, 3);

  const uint8_t slots[3] = {BQ_USER_BASS, BQ_USER_MID, BQ_USER_TREBLE};
  bool ok = true;
  int32_t c[5];

  for (int i = 0; i < 3; i++)
  {
    bqToFixed(bq[i], c);
    ok &= writeBiquad(slots[i], c);
  }
  return ok;
}

// ============================================================
// VOLUME
// ============================================================

float volumeDb()
{
  float db = cfg.maxDbX2 / 2.0f
           - (127 - cfg.phoneVol) * VOL_RANGE_DB / 126.0f;

  db += eqHeadroomDb;                    // compensate EQ pre-attenuation
  if (!cfg.samsungEq)
    db += SAMSUNG_EQ_GAIN_DB;            // compensate missing +16 dB

  return db;
}

uint8_t volumeReg()
{
  if (cfg.phoneVol == 0)
    return 0xFF;                         // mute

  // 0x30 = 0 dB, +1 = -0.5 dB, 0x00 = +24 dB
  long r = lround(0x30 - 2.0f * volumeDb());
  return (uint8_t)clampInt((int)r, 0x00, 0xFE);
}

void setVolReg(uint8_t r)
{
  book0();
  tw(0x4C, r);
  curVolReg = r;
}

void rampVolTo(uint8_t target, int steps, int stepMs)
{
  int from = curVolReg;
  book0();

  for (int i = 1; i <= steps; i++)
  {
    int r = from + ((int)target - from) * i / steps;
    tw(0x4C, (uint8_t)r);
    delay(stepMs);
  }
  curVolReg = target;
}

void applyVolume()
{
  setVolReg(volumeReg());
}

void printVolume()
{
  if (cfg.phoneVol == 0)
    Serial.println("Volume 0/127 -> MUTE");
  else
    Serial.printf("Volume %u/127 -> DIG_VOL 0x%02X (%.1f dB)\n",
                  cfg.phoneVol, curVolReg, (0x30 - (int)curVolReg) * 0.5f);
}

void setPhoneVolume(int v, bool tellLibrary)
{
  cfg.phoneVol = (uint8_t)clampInt(v, 0, 127);
  if (tellLibrary)
    a2dp_sink.set_volume(cfg.phoneVol);
  applyVolume();
  printVolume();
  markDirty();
}

void volumeStep(int dir)
{
#if BUTTON_VOL_VIA_PHONE
  if (btConnected)
  {
    if (dir > 0) a2dp_sink.volume_up();
    else         a2dp_sink.volume_down();
    return;
  }
#endif
  setPhoneVolume((int)cfg.phoneVol + dir * VOL_BUTTON_STEP, true);
}

// ============================================================
// DSP UPDATE (volume ramp around coefficient writes -> no clicks)
// ============================================================

bool dspUpdateBegin()
{
  bool wasPlaying = tasIsPlaying;

  if (wasPlaying)
    rampVolTo(0xFF, 8, 5);

  book0();
  tw(0x66, DSP_MISC_WRITE);
  return wasPlaying;
}

void dspUpdateEnd(bool wasPlaying)
{
  book0();
  tw(0x66, DSP_MISC_RUN);

  if (wasPlaying)
  {
    delay(30);
    rampVolTo(volumeReg(), 8, 5);
  }
  else
  {
    applyVolume();
  }
}

void applyAllDsp()
{
  bool w = dspUpdateBegin();
  bool ok = writeSamsungEq(cfg.samsungEq);
  ok &= writeUserEq();
  ok &= writeMixer();
  dspUpdateEnd(w);

  book0();
  tw(0x54, cfg.again);

  if (!ok)
    Serial.println("WARNING: some DSP writes failed");
}

void applyUserEq()
{
  bool w = dspUpdateBegin();
  bool ok = writeUserEq();
  ok &= writeMixer();          // headroom may have changed
  dspUpdateEnd(w);
  if (!ok) Serial.println("WARNING: EQ write failed");
}

void applySamsungEq()
{
  bool w = dspUpdateBegin();
  bool ok = writeSamsungEq(cfg.samsungEq);
  dspUpdateEnd(w);
  if (!ok) Serial.println("WARNING: EQ write failed");
}

void applyMixer()
{
  bool w = dspUpdateBegin();
  bool ok = writeMixer();
  dspUpdateEnd(w);
  if (!ok) Serial.println("WARNING: mixer write failed");
}

void printEq()
{
  Serial.printf("EQ: samsung %s | bass %+d  mid %+d  treble %+d dB | preset %s | headroom %.1f dB\n",
                cfg.samsungEq ? "ON" : "OFF",
                cfg.bass, cfg.mid, cfg.treble,
                cfg.preset >= 0 ? PRESETS[cfg.preset].name : "custom",
                eqHeadroomDb);
}

void setPreset(int idx)
{
  idx = ((idx % PRESET_COUNT) + PRESET_COUNT) % PRESET_COUNT;
  cfg.preset = (int8_t)idx;
  cfg.bass   = PRESETS[idx].bass;
  cfg.mid    = PRESETS[idx].mid;
  cfg.treble = PRESETS[idx].treble;
  applyUserEq();
  printEq();
  markDirty();
}

// ============================================================
// TAS SERIAL FORMAT
// ============================================================

void configureTas()
{
  book0();

#ifdef OVERRIDE_CTRL1
  tw(0x02, OVERRIDE_CTRL1);
#endif

  tw(0x31, 0x00);   // normal SCLK polarity
  tw(0x33, 0x02);   // I2S, 24-bit word (32-bit slots, MSB first -> OK)
  tw(0x34, 0x00);   // no data offset
  tw(0x35, 0x11);   // L->L, R->R
  tw(0x78, 0x80);   // clear faults
  tw(0x03, 0x0E);   // Hi-Z + mute until audio arrives
}

// ============================================================
// TAS PLAY / MUTE
// ============================================================

void tasPlay()
{
  book0();
  tw(0x78, 0x80);
  delay(2);
  tw(0x03, 0x03);   // Play, unmuted
  tasIsPlaying = true;
  lastPlayCmdAt = millis();
  Serial.println(">>> TAS PLAY");
}

void tasMute()
{
  book0();
  tw(0x03, 0x0E);   // Hi-Z + mute
  tasIsPlaying = false;
  forcedPlay = false;
  Serial.println(">>> TAS Hi-Z");
}

// ============================================================
// FAULT MONITOR
// ============================================================

uint32_t lastFaultPoll = 0;
uint8_t  lastFaultSig[4] = {0, 0, 0, 0};
uint8_t  lastClkErr = 0;
uint32_t restartTimes[5] = {0, 0, 0, 0, 0};
uint8_t  restartIdx = 0;
uint32_t totalRestarts = 0;

void pollFaults()
{
  uint32_t now = millis();
  if (now - lastFaultPoll < 1000)
    return;
  lastFaultPoll = now;

  uint8_t f[4] = {0, 0, 0, 0};
  uint8_t clk = 0;
  uint8_t pwr = 0;

  book0();
  tr(0x70, f[0]);
  tr(0x71, f[1]);
  tr(0x72, f[2]);
  tr(0x73, f[3]);
  tr(0x39, clk);
  bool pwrOk = tr(0x68, pwr);

  if (memcmp(f, lastFaultSig, 4) != 0)
  {
    Serial.printf("TAS FAULT REG: CHAN=%02X GLOB1=%02X GLOB2=%02X WARN=%02X\n",
                  f[0], f[1], f[2], f[3]);
    memcpy(lastFaultSig, f, 4);
  }

  // bit 3 of CLKDET = "PLL locked" (normal), other bits = clock errors
  uint8_t clkErr = clk & (uint8_t)~0x08;
  if (clkErr != lastClkErr)
  {
    Serial.printf("TAS CLOCK STATUS: 0x%02X%s\n",
                  clk, clkErr ? "  <-- clock error" : "  (ok)");
    lastClkErr = clkErr;
  }

  if (!tasIsPlaying || !pwrOk || now - lastPlayCmdAt < 1000)
    return;

  if (pwr == 0x03)
    return;

  // Amplifier left PLAY on its own -> protection tripped
  totalRestarts++;
  restartTimes[restartIdx] = now;
  restartIdx = (restartIdx + 1) % 5;

  int recent = 0;
  for (int i = 0; i < 5; i++)
    if (restartTimes[i] != 0 && now - restartTimes[i] < 60000)
      recent++;

  Serial.printf("!!! Amp dropped out (POWER_STATE=%02X), restart #%lu\n",
                pwr, (unsigned long)totalRestarts);

  if (recent >= 5)
  {
    faultLatched = true;
    tasMute();
    Serial.println("!!! 5 faults in 60 s: amplifier stays OFF.");
    Serial.println("    Check power supply / speaker wiring, then send 'p' or reconnect.");
    return;
  }

  tw(0x78, 0x80);
  delay(20);
  tw(0x03, 0x03);
  lastPlayCmdAt = millis();
}

// ============================================================
// AUDIO STATS
// ============================================================

uint32_t statUnderruns = 0;
uint32_t statOverflows = 0;
uint32_t statFill      = 0;
int32_t  statPpm       = 0;

// ============================================================
// STATUS
// ============================================================

void status()
{
  book0();

  Serial.println();
  Serial.println("========== TAS ==========");

  show(0x03, "CTRL2");
  show(0x68, "PWR_STATE");   // 3 = play, 2 = Hi-Z
  show(0x02, "CTRL1");
  show(0x33, "SAP_CTRL1");
  show(0x4C, "DIG_VOL");
  show(0x54, "AGAIN");
  show(0x66, "DSP_MISC");

  Serial.println("CLOCK:");
  show(0x37, "FS_MON");
  show(0x38, "BCK_MON");     // 0x40 = 64 x Fs
  show(0x39, "CLKDET");      // 0x08 = PLL locked (normal)

  Serial.println("FAULT:");
  show(0x70, "CHAN_FLT");
  show(0x71, "GLOB_FLT1");
  show(0x72, "GLOB_FLT2");
  show(0x73, "WARNING");

  Serial.println("---------- STATE ----------");
  Serial.printf("BT: %s | amp: %s%s\n",
                btConnected ? "connected" : "not connected",
                tasIsPlaying ? "PLAY" : "Hi-Z",
                faultLatched ? " (FAULT LATCHED)" : "");
  printVolume();
  Serial.printf("Max volume: %.1f dB | analog gain: -%.1f dB\n",
                cfg.maxDbX2 / 2.0f, cfg.again * 0.5f);
  printEq();
  Serial.printf("Channels: %s, balance %+d\n", MODE_NAMES[cfg.mode], cfg.balance);
  Serial.printf("AUDIO: fifo=%lu underruns=%lu overflows=%lu drift=%ld ppm restarts=%lu\n",
                (unsigned long)statFill, (unsigned long)statUnderruns,
                (unsigned long)statOverflows, (long)statPpm,
                (unsigned long)totalRestarts);

  Serial.println("===========================");
}

void dumpPage0()
{
  book0();
  Serial.println();
  Serial.println("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");
  for (int r = 0; r < 0x80; r++)
  {
    if ((r & 0x0F) == 0) Serial.printf("%02X: ", r);
    uint8_t v;
    if (tr((uint8_t)r, v)) Serial.printf("%02X ", v);
    else                   Serial.printf("-- ");
    if ((r & 0x0F) == 0x0F) Serial.println();
  }
}

void printHelp()
{
  Serial.println();
  Serial.println("---------------- COMMANDS ----------------");
  Serial.println("s / status        full status");
  Serial.println("d / dump          dump TAS page 0 registers");
  Serial.println("+ / -             volume up / down");
  Serial.println("vol <0-127>       set volume (like phone slider)");
  Serial.println("maxvol <dB>       volume at 127, e.g. maxvol -10 (-40..0)");
  Serial.println("again <0-31>      analog gain limit, 0 = max, step -0.5 dB");
  Serial.println("bass <dB>         -12..12  (100 Hz shelf)");
  Serial.println("mid <dB>          -12..12  (1 kHz)");
  Serial.println("treble <dB>       -12..12  (8 kHz shelf)");
  Serial.println("preset <name>     flat, bass, rock, vocal, night");
  Serial.println("eq                show EQ");
  Serial.println("seq on|off        Samsung speaker EQ");
  Serial.println("mode stereo|mono|swap");
  Serial.println("bal <-100..100>   balance (- = left, + = right)");
  Serial.println("play / pause / next / prev   control the phone");
  Serial.println("p                 force amplifier PLAY (clears fault latch)");
  Serial.println("m                 amplifier Hi-Z");
  Serial.println("save              save settings now (auto-save after 3 s)");
  Serial.println("reset             factory defaults");
  Serial.println("------------------------------------------");
}

// ============================================================
// I2S: 48 kHz, Philips, 32-bit data in 32-bit slots
// ESP32 classic: slot width MUST equal data width
// ============================================================

bool startI2S()
{
  i2s_chan_config_t chanCfg =
    I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

  chanCfg.dma_desc_num  = 8;
  chanCfg.dma_frame_num = 256;

  esp_err_t err = i2s_new_channel(&chanCfg, &txHandle, nullptr);
  if (err != ESP_OK)
  {
    Serial.printf("i2s_new_channel=%d\n", err);
    return false;
  }

  i2s_std_config_t cfgI2s;
  memset(&cfgI2s, 0, sizeof(cfgI2s));

  i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(OUT_RATE);
  i2s_std_slot_config_t slot =
    I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                        I2S_SLOT_MODE_STEREO);

  cfgI2s.clk_cfg  = clk;
  cfgI2s.slot_cfg = slot;

#if USE_APLL
  cfgI2s.clk_cfg.clk_src = I2S_CLK_SRC_APLL;
#endif

  cfgI2s.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  cfgI2s.gpio_cfg.bclk = (gpio_num_t)I2S_BCLK_PIN;
  cfgI2s.gpio_cfg.ws   = (gpio_num_t)I2S_LRCLK_PIN;
  cfgI2s.gpio_cfg.dout = (gpio_num_t)I2S_DATA_PIN;
  cfgI2s.gpio_cfg.din  = I2S_GPIO_UNUSED;
  cfgI2s.gpio_cfg.invert_flags.mclk_inv = false;
  cfgI2s.gpio_cfg.invert_flags.bclk_inv = false;
  cfgI2s.gpio_cfg.invert_flags.ws_inv   = false;

  err = i2s_channel_init_std_mode(txHandle, &cfgI2s);

#if USE_APLL
  if (err != ESP_OK)
  {
    Serial.printf("APLL init failed (%d), fallback to default clock\n", err);
    cfgI2s.clk_cfg.clk_src = I2S_CLK_SRC_DEFAULT;
    err = i2s_channel_init_std_mode(txHandle, &cfgI2s);
  }
#endif

  if (err != ESP_OK)
  {
    Serial.printf("i2s_channel_init=%d\n", err);
    return false;
  }

  err = i2s_channel_enable(txHandle);
  if (err != ESP_OK)
  {
    Serial.printf("i2s_enable=%d\n", err);
    return false;
  }

  Serial.println("I2S: 48000 Hz, Philips, 32-bit data / 32-bit slots, BCLK 3.072 MHz");
  return true;
}

// ============================================================
// INPUT PCM FIFO (A2DP -> audio task)
// ============================================================

#define FIFO_FRAMES   8192
#define FIFO_TARGET   (FIFO_FRAMES / 2)

Stereo16 pcmFifo[FIFO_FRAMES];

uint32_t fifoRead  = 0;
uint32_t fifoWrite = 0;

portMUX_TYPE fifoMux = portMUX_INITIALIZER_UNLOCKED;

void fifoClear()
{
  portENTER_CRITICAL(&fifoMux);
  fifoRead = 0;
  fifoWrite = 0;
  portEXIT_CRITICAL(&fifoMux);
}

uint32_t fifoCount()
{
  portENTER_CRITICAL(&fifoMux);
  uint32_t r = fifoRead;
  uint32_t w = fifoWrite;
  portEXIT_CRITICAL(&fifoMux);

  return (w >= r) ? (w - r) : (FIFO_FRAMES - r + w);
}

void fifoPush(const int16_t *samples, uint32_t frames)
{
  portENTER_CRITICAL(&fifoMux);

  for (uint32_t i = 0; i < frames; i++)
  {
    uint32_t next = fifoWrite + 1;
    if (next >= FIFO_FRAMES) next = 0;

    if (next == fifoRead)
    {
      fifoRead++;
      if (fifoRead >= FIFO_FRAMES) fifoRead = 0;
      statOverflows++;
    }

    pcmFifo[fifoWrite].l = samples[i * 2];
    pcmFifo[fifoWrite].r = samples[i * 2 + 1];
    fifoWrite = next;
  }

  portEXIT_CRITICAL(&fifoMux);
}

uint32_t fifoPopBlock(Stereo16 *dst, uint32_t maxFrames)
{
  uint32_t n = 0;

  portENTER_CRITICAL(&fifoMux);
  while (n < maxFrames && fifoRead != fifoWrite)
  {
    dst[n++] = pcmFifo[fifoRead];
    fifoRead++;
    if (fifoRead >= FIFO_FRAMES) fifoRead = 0;
  }
  portEXIT_CRITICAL(&fifoMux);

  return n;
}

// ============================================================
// A2DP CALLBACKS (no I2C here - only flags)
// ============================================================

volatile uint32_t pendingSourceRate = 44100;
volatile bool     sourceRateChanged = false;

portMUX_TYPE metaMux = portMUX_INITIALIZER_UNLOCKED;
char metaTitle[96]  = "";
char metaArtist[96] = "";
char metaAlbum[96]  = "";
volatile bool     metaChanged = false;
volatile uint32_t metaAt = 0;

void btAudio(const uint8_t *data, uint32_t length)
{
  if (!data || length < 4)
    return;

  fifoPush((const int16_t *)data, length / 4);

  lastPcmMillis = millis();

  if (!tasIsPlaying)
    playRequest = true;
}

void btSampleRate(uint16_t rate)
{
  if (rate < 8000 || rate > 48000)
    return;

  pendingSourceRate = rate;
  sourceRateChanged = true;
}

void btAudioState(esp_a2d_audio_state_t state, void *ptr)
{
  Serial.printf("A2DP AUDIO = %s\n", a2dp_sink.to_str(state));

  if (state != ESP_A2D_AUDIO_STATE_STARTED)
    muteRequest = true;
}

void btConnection(esp_a2d_connection_state_t state, void *ptr)
{
  Serial.printf("BT = %s\n", a2dp_sink.to_str(state));

  bool conn = (state == ESP_A2D_CONNECTION_STATE_CONNECTED);
  if (conn && !btConnected)
    newConnection = true;
  btConnected = conn;

  if (!conn)
    muteRequest = true;
}

void btVolume(int v)
{
  pendingPhoneVol = v;
}

void btMetadata(uint8_t id, const uint8_t *text)
{
  char *dst = nullptr;

  if (id == ESP_AVRC_MD_ATTR_TITLE)       dst = metaTitle;
  else if (id == ESP_AVRC_MD_ATTR_ARTIST) dst = metaArtist;
  else if (id == ESP_AVRC_MD_ATTR_ALBUM)  dst = metaAlbum;
  else return;

  portENTER_CRITICAL(&metaMux);
  strncpy(dst, text ? (const char *)text : "", 95);
  dst[95] = 0;
  metaChanged = true;
  metaAt = millis();
  portEXIT_CRITICAL(&metaMux);
}

void printMetadata()
{
  if (!metaChanged || millis() - metaAt < 150)
    return;

  char t[96], a[96], al[96];

  portENTER_CRITICAL(&metaMux);
  memcpy(t,  metaTitle,  sizeof(t));
  memcpy(a,  metaArtist, sizeof(a));
  memcpy(al, metaAlbum,  sizeof(al));
  metaChanged = false;
  portEXIT_CRITICAL(&metaMux);

  if (t[0] == 0 && a[0] == 0)
    return;

  Serial.printf("NOW PLAYING: %s - %s", a, t);
  if (al[0])
    Serial.printf("  [%s]", al);
  Serial.println();
}

// ============================================================
// RESAMPLER: source rate -> 48000 Hz, linear interpolation
// ============================================================

#define OUT_FRAMES 256
#define IN_BUF     1024

Stereo16 inBuf[IN_BUF];
uint32_t inCount = 0;

uint64_t posQ32   = 0;
uint64_t baseStep = 0;
float    fillAvg  = FIFO_TARGET;
bool     running  = false;

int32_t outBuffer[OUT_FRAMES * 2];

void resetResampler(uint32_t rate)
{
  fifoClear();
  inCount = 0;
  posQ32 = 0;
  running = false;
  fillAvg = FIFO_TARGET;
  baseStep = ((uint64_t)rate << 32) / OUT_RATE;

  Serial.printf("Resampler: %lu -> %d Hz\n", (unsigned long)rate, OUT_RATE);
}

void writeOut()
{
  size_t written = 0;
  esp_err_t err = i2s_channel_write(txHandle, outBuffer, sizeof(outBuffer),
                                    &written, portMAX_DELAY);
  if (err != ESP_OK)
    Serial.printf("I2S WRITE ERROR=%d\n", err);
}

void writeSilence()
{
  memset(outBuffer, 0, sizeof(outBuffer));
  writeOut();
}

void audioTask(void *arg)
{
  while (true)
  {
    if (sourceRateChanged)
    {
      sourceRateChanged = false;
      resetResampler(pendingSourceRate);
    }

    uint32_t fill = fifoCount();
    statFill = fill;

    if (!running)
    {
      if (fill < FIFO_TARGET)
      {
        writeSilence();
        continue;
      }
      running = true;
      inCount = 0;
      posQ32 = 0;
      fillAvg = fill;
    }

    // slow clock-drift correction (max +-500 ppm)
    fillAvg += ((float)fill - fillAvg) * 0.02f;
    float errNorm = (fillAvg - FIFO_TARGET) / (float)FIFO_TARGET;
    if (errNorm >  1.0f) errNorm =  1.0f;
    if (errNorm < -1.0f) errNorm = -1.0f;
    int32_t ppm = (int32_t)(errNorm * 500.0f);
    statPpm = ppm;

    uint64_t step = baseStep + (int64_t)((int64_t)baseStep * ppm / 1000000);

    uint32_t need = (uint32_t)((posQ32 + step * OUT_FRAMES) >> 32) + 2;
    if (need > IN_BUF) need = IN_BUF;

    if (inCount < need)
      inCount += fifoPopBlock(inBuf + inCount, need - inCount);

    if (inCount < need)
    {
      statUnderruns++;
      running = false;
      writeSilence();
      continue;
    }

    uint64_t pos = posQ32;

    for (uint32_t n = 0; n < OUT_FRAMES; n++)
    {
      uint32_t idx  = (uint32_t)(pos >> 32);
      uint32_t frac = (uint32_t)pos;

      const Stereo16 &a = inBuf[idx];
      const Stereo16 &b = inBuf[idx + 1];

      int32_t l = a.l + (int32_t)(((int64_t)(b.l - a.l) * frac) >> 32);
      int32_t r = a.r + (int32_t)(((int64_t)(b.r - a.r) * frac) >> 32);

      outBuffer[n * 2]     = l * 65536;
      outBuffer[n * 2 + 1] = r * 65536;

      pos += step;
    }

    uint32_t consumed = (uint32_t)(pos >> 32);
    posQ32 = pos & 0xFFFFFFFFULL;

    if (consumed > 0)
    {
      memmove(inBuf, inBuf + consumed, (inCount - consumed) * sizeof(Stereo16));
      inCount -= consumed;
    }

    writeOut();
  }
}

// ============================================================
// BUTTONS & LED
// ============================================================

void initButton(Button &b)
{
  if (b.pin >= 0)
    pinMode(b.pin, INPUT_PULLUP);
}

BtnEvent pollButton(Button &b)
{
  if (b.pin < 0)
    return EV_NONE;

  uint32_t now = millis();
  bool raw = digitalRead(b.pin) == LOW;

  if (raw != b.raw)
  {
    b.raw = raw;
    b.rawChangedAt = now;
  }

  // no debounced change -> handle "held"
  if (now - b.rawChangedAt < 25 || raw == b.pressed)
  {
    if (b.pressed)
    {
      uint32_t held = now - b.pressedAt;

      if (b.repeat)
      {
        if (held >= 400 && now - b.lastRepeatAt >= 120)
        {
          b.lastRepeatAt = now;
          return EV_REPEAT;
        }
      }
      else if (!b.longDone && held >= 800)
      {
        b.longDone = true;
        return EV_LONG;
      }
    }
    return EV_NONE;
  }

  // debounced edge
  b.pressed = raw;

  if (raw)
  {
    b.pressedAt = now;
    b.lastRepeatAt = now;
    b.longDone = false;
    return b.repeat ? EV_SHORT : EV_NONE;
  }

  if (!b.repeat && !b.longDone)
    return EV_SHORT;

  return EV_NONE;
}

void pollButtons()
{
  BtnEvent e;

  e = pollButton(btnUp);
  if (e == EV_SHORT || e == EV_REPEAT) volumeStep(+1);

  e = pollButton(btnDown);
  if (e == EV_SHORT || e == EV_REPEAT) volumeStep(-1);

  e = pollButton(btnPlay);
  if (e == EV_SHORT)
  {
    if (a2dp_sink.get_audio_state() == ESP_A2D_AUDIO_STATE_STARTED)
    {
      Serial.println("BUTTON: pause");
      a2dp_sink.pause();
    }
    else
    {
      Serial.println("BUTTON: play");
      a2dp_sink.play();
    }
  }
  else if (e == EV_LONG)
  {
    setPreset(cfg.preset < 0 ? 0 : cfg.preset + 1);
  }

  e = pollButton(btnNext);
  if (e == EV_SHORT || e == EV_LONG)
  {
    Serial.println("BUTTON: next");
    a2dp_sink.next();
  }

  e = pollButton(btnPrev);
  if (e == EV_SHORT || e == EV_LONG)
  {
    Serial.println("BUTTON: previous");
    a2dp_sink.previous();
  }
}

void updateLed()
{
  if (LED_PIN < 0)
    return;

  uint32_t t = millis();
  bool on;

  if (faultLatched)       on = (t / 60) % 2;        // fast blink: fault
  else if (!btConnected)  on = (t / 500) % 2;       // slow blink: waiting
  else if (!tasIsPlaying) on = true;                // solid: connected
  else                    on = (t % 2000) > 100;    // short dips: playing

  digitalWrite(LED_PIN, on ? LED_ON_LEVEL : !LED_ON_LEVEL);
}

// ============================================================
// SERIAL CONSOLE
// ============================================================

char     lineBuf[64];
uint8_t  lineLen = 0;
uint32_t lastCharAt = 0;

void handleCommand(char *line)
{
  for (char *p = line; *p; p++)
    *p = (char)tolower((unsigned char)*p);

  char cmd[16] = "";
  char arg[32] = "";
  sscanf(line, "%15s %31s", cmd, arg);

  bool hasArg = arg[0] != 0;
  long n = strtol(arg, nullptr, 10);

  if (!cmd[0])
    return;

  if (!strcmp(cmd, "s") || !strcmp(cmd, "status"))
    status();

  else if (!strcmp(cmd, "d") || !strcmp(cmd, "dump"))
    dumpPage0();

  else if (!strcmp(cmd, "h") || !strcmp(cmd, "help") || !strcmp(cmd, "?"))
    printHelp();

  else if (!strcmp(cmd, "p"))
  {
    faultLatched = false;
    tasPlay();
    forcedPlay = true;
  }

  else if (!strcmp(cmd, "m"))
    tasMute();

  else if (!strcmp(cmd, "+"))
    volumeStep(+1);

  else if (!strcmp(cmd, "-"))
    volumeStep(-1);

  else if (!strcmp(cmd, "vol"))
  {
    if (hasArg) setPhoneVolume((int)n, true);
    else        printVolume();
  }

  else if (!strcmp(cmd, "maxvol"))
  {
    if (hasArg)
    {
      float db = (float)atof(arg);
      if (db < -40) db = -40;
      if (db > 0)   db = 0;
      cfg.maxDbX2 = (int8_t)lround(db * 2.0f);
      applyVolume();
      markDirty();
    }
    Serial.printf("Max volume: %.1f dB\n", cfg.maxDbX2 / 2.0f);
    printVolume();
  }

  else if (!strcmp(cmd, "again"))
  {
    if (hasArg)
    {
      cfg.again = (uint8_t)clampInt((int)n, 0, 31);
      book0();
      tw(0x54, cfg.again);
      markDirty();
    }
    Serial.printf("Analog gain: -%.1f dB (AGAIN=%u)\n", cfg.again * 0.5f, cfg.again);
  }

  else if (!strcmp(cmd, "bass") || !strcmp(cmd, "mid") || !strcmp(cmd, "treble"))
  {
    if (hasArg)
    {
      int8_t v = (int8_t)clampInt((int)n, -EQ_LIMIT_DB, EQ_LIMIT_DB);
      if (cmd[0] == 'b')      cfg.bass = v;
      else if (cmd[0] == 'm') cfg.mid = v;
      else                    cfg.treble = v;
      cfg.preset = -1;
      applyUserEq();
      markDirty();
    }
    printEq();
  }

  else if (!strcmp(cmd, "preset"))
  {
    int found = -1;
    for (int i = 0; i < PRESET_COUNT; i++)
      if (!strcmp(arg, PRESETS[i].name))
        found = i;

    if (found >= 0)
      setPreset(found);
    else
    {
      Serial.print("Presets:");
      for (int i = 0; i < PRESET_COUNT; i++)
        Serial.printf(" %s", PRESETS[i].name);
      Serial.println();
    }
  }

  else if (!strcmp(cmd, "eq"))
    printEq();

  else if (!strcmp(cmd, "seq") || !strcmp(cmd, "samsung"))
  {
    if (!strcmp(arg, "on") || !strcmp(arg, "off"))
    {
      cfg.samsungEq = !strcmp(arg, "on");
      applySamsungEq();
      markDirty();
    }
    printEq();
  }

  else if (!strcmp(cmd, "mode"))
  {
    int found = -1;
    for (int i = 0; i < 3; i++)
      if (!strcmp(arg, MODE_NAMES[i]))
        found = i;

    if (found >= 0)
    {
      cfg.mode = (uint8_t)found;
      applyMixer();
      markDirty();
    }
    Serial.printf("Channels: %s (stereo|mono|swap)\n", MODE_NAMES[cfg.mode]);
  }

  else if (!strcmp(cmd, "bal"))
  {
    if (hasArg)
    {
      cfg.balance = (int8_t)clampInt((int)n, -100, 100);
      applyMixer();
      markDirty();
    }
    Serial.printf("Balance: %+d\n", cfg.balance);
  }

  else if (!strcmp(cmd, "play"))  a2dp_sink.play();
  else if (!strcmp(cmd, "pause")) a2dp_sink.pause();
  else if (!strcmp(cmd, "next"))  a2dp_sink.next();
  else if (!strcmp(cmd, "prev"))  a2dp_sink.previous();

  else if (!strcmp(cmd, "save"))
    saveSettings();

  else if (!strcmp(cmd, "reset"))
  {
    setDefaults(cfg);
    applyAllDsp();
    a2dp_sink.set_volume(cfg.phoneVol);
    saveSettings();
    Serial.println("Defaults restored");
    status();
  }

  else
    Serial.printf("Unknown command '%s'. Type help\n", cmd);
}

void pollSerial()
{
  while (Serial.available())
  {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n')
    {
      if (lineLen)
      {
        lineBuf[lineLen] = 0;
        lineLen = 0;
        handleCommand(lineBuf);
      }
      continue;
    }

    // "+" / "-" work instantly without Enter
    if (lineLen == 0 && (c == '+' || c == '-'))
    {
      volumeStep(c == '+' ? +1 : -1);
      continue;
    }

    if (lineLen < sizeof(lineBuf) - 1)
      lineBuf[lineLen++] = c;

    lastCharAt = millis();
  }

  // Serial Monitor with "No line ending"
  if (lineLen && millis() - lastCharAt > 300)
  {
    lineBuf[lineLen] = 0;
    lineLen = 0;
    handleCommand(lineBuf);
  }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("======================================");
  Serial.println("TAS5806  BT -> 48k / 32-bit I2S   v2");
  Serial.println("======================================");

  loadSettings();

  if (LED_PIN >= 0)
    pinMode(LED_PIN, OUTPUT);

  initButton(btnUp);
  initButton(btnDown);
  initButton(btnPlay);
  initButton(btnNext);
  initButton(btnPrev);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  waitTas();

  // Clocks first: TAS DSP needs valid I2S clocks
  if (!startI2S())
  {
    Serial.println("I2S INIT FAILED");
    while (true) delay(1000);
  }

  resetResampler(44100);

  xTaskCreatePinnedToCore(audioTask, "audio48", 6144, nullptr, 5, nullptr, 1);

  delay(200);

  book0();
  tw(0x03, 0x02);   // Hi-Z
  delay(10);

  if (!loadSamsung())
  {
    Serial.println("TAS CONFIG FAILED");
    while (true) delay(1000);
  }

  configureTas();
  applyAllDsp();    // Samsung EQ (L+R), user EQ, mixer, volume, analog gain

  delay(100);
  status();

  // Bluetooth
  a2dp_sink.set_volume_control(&noSoftwareVolume);
  a2dp_sink.set_stream_reader(btAudio, false);
  a2dp_sink.set_sample_rate_callback(btSampleRate);
  a2dp_sink.set_on_audio_state_changed_post(btAudioState);
  a2dp_sink.set_on_connection_state_changed(btConnection);
  a2dp_sink.set_on_volumechange(btVolume);
  a2dp_sink.set_avrc_rn_volumechange(btVolume);
  a2dp_sink.set_avrc_metadata_attribute_mask(
    ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST | ESP_AVRC_MD_ATTR_ALBUM);
  a2dp_sink.set_avrc_metadata_callback(btMetadata);
  a2dp_sink.set_volume(cfg.phoneVol);

  Serial.println("Starting Bluetooth...");
  a2dp_sink.start(BT_NAME, true);

  Serial.println();
  Serial.printf("Connect to: %s\n", BT_NAME);
  printHelp();
}

// ============================================================
// LOOP  (all I2C here, never in BT callbacks)
// ============================================================

void loop()
{
  if (newConnection)
  {
    newConnection = false;
    faultLatched = false;
  }

  if (pendingPhoneVol >= 0)
  {
    int v = pendingPhoneVol;
    pendingPhoneVol = -1;
    if (v != cfg.phoneVol)
    {
      Serial.print("Phone ");
      setPhoneVolume(v, false);
    }
  }

  if (playRequest)
  {
    playRequest = false;
    if (!tasIsPlaying && !faultLatched)
    {
      delay(30);
      tasPlay();
    }
  }

  if (muteRequest)
  {
    muteRequest = false;
    if (tasIsPlaying)
      tasMute();
  }

  // No PCM for 300 ms -> Hi-Z (not in forced play mode)
  if (tasIsPlaying && !forcedPlay)
  {
    if (millis() - lastPcmMillis > 300)
      tasMute();
  }

  pollFaults();
  pollButtons();
  pollSerial();
  printMetadata();
  updateLed();

  if (cfgDirty && millis() - cfgDirtyAt > 3000)
    saveSettings();

  delay(2);
}

# Samsung TAS5807 ESP32 A2DP

Reuse the Samsung TV TAS58xx amplifier section as a standalone Bluetooth amplifier controlled by a classic ESP32.

The firmware was developed around a Samsung amplifier IC marked **TAS5807M** and uses the TAS5806M / TAS5806MD-style register and DSP architecture used by this board.

## Features

- Bluetooth A2DP audio receiver
- Original Samsung DSP initialization sequence
- Original Samsung speaker EQ on/off
- User 3-band EQ: bass / mid / treble
- EQ presets: `flat`, `bass`, `rock`, `vocal`, `night`
- Phone volume mapped directly to TAS digital volume (no software PCM scaling)
- Configurable maximum output level
- Configurable TAS analog gain limit
- Stereo / mono / channel swap
- Left/right balance
- AVRCP play / pause / next / previous
- Physical buttons
- Status LED
- Track title / artist / album in Serial Monitor
- TAS fault monitoring and automatic restart
- Protection latch after repeated faults
- Persistent settings in ESP32 NVS
- 44.1/48 kHz source -> fixed 48 kHz output resampling
- FIFO prebuffering and slow clock-drift correction
- ESP32 APLL support with fallback

## Hardware

### ESP32

Use a **classic ESP32**, for example ESP32-WROOM-32 / ESP32 DevKit.

The sketch targets:

- Arduino-ESP32 Core 3.x
- ESP-IDF 5.x based core
- ESP32-A2DP library 1.8+

### Amplifier

Default TAS I2C address:

```text
0x2C
```

## Wiring

### I2C

| ESP32 | TAS58xx |
|---|---|
| GPIO21 | SDA |
| GPIO22 | SCL |
| GND | GND |

The firmware uses 100 kHz I2C.

### I2S

| ESP32 | TAS58xx |
|---|---|
| GPIO26 | BCLK / SCLK |
| GPIO25 | LRCLK / WS |
| GPIO27 | SDIN |
| GND | GND |

If the TAS remains connected to the original Samsung SoC, the original Samsung I2C/I2S drivers must not drive the same lines at the same time as the ESP32.

## I2S format

The ESP32 generates:

```text
Sample rate:     48,000 Hz
Protocol:        Philips I2S
Channels:        Stereo
Data width:      32 bit
Slot width:      32 bit
Slots/frame:     2
BCLK/frame:      64
BCLK:            3.072 MHz
```

The TAS serial interface is configured with:

```text
R31 = 0x00
R33 = 0x02
R34 = 0x00
R35 = 0x11
```

The ESP32 sends 32-bit physical slots while the TAS reads the upper 24 bits. Bluetooth PCM is signed 16-bit stereo and is MSB-aligned in each output word.

## Bluetooth

Default device name:

```text
TAS5807 Speaker
```

Change it in the sketch with:

```cpp
#define BT_NAME "TAS5807 Speaker"
```

The firmware receives A2DP PCM through ESP32-A2DP and handles I2S output itself.

## Audio path

```text
Phone
  -> Bluetooth A2DP
  -> signed 16-bit stereo PCM
  -> FIFO
  -> resampler / drift correction
  -> 48 kHz, 32-bit-slot Philips I2S
  -> TAS58xx DSP
  -> amplifier output
```

The output side always runs at 48 kHz. A2DP sources such as 44.1 kHz are resampled to 48 kHz.

## Samsung DSP configuration

The sketch contains a captured Samsung initialization sequence including DSP routing and speaker EQ coefficients.

A critical detail is TAS book/page addressing: register `0x7F` acts as the book selector only when the current page is `0x00`. A multi-byte coefficient write ending at register `0x7F` on a DSP page must not be treated as a book change.

The current configuration also restores two coefficients that were missing from the original capture so the Samsung biquad chain remains valid.

## User EQ

Default bands:

```text
Bass:    100 Hz shelf
Mid:     1 kHz, Q 0.7
Treble:  8 kHz shelf
Range:   -12 ... +12 dB
```

Built-in presets:

| Preset | Bass | Mid | Treble |
|---|---:|---:|---:|
| flat | 0 | 0 | 0 |
| bass | +6 | 0 | +1 |
| rock | +4 | -2 | +3 |
| vocal | -2 | +3 | +1 |
| night | -6 | +2 | -2 |

The firmware automatically applies input headroom compensation when EQ boost is added.

## Buttons

Buttons are connected between the GPIO and GND and use the ESP32 internal pull-up.

| Function | GPIO |
|---|---:|
| Volume + | 32 |
| Volume - | 33 |
| Play/Pause | 13 |
| Next | 14 |
| Previous | 4 |

`BTN_PLAY` short press = play/pause. Long press = next EQ preset.

Set a button pin to `-1` to disable it.

GPIO34..39 require external pull-ups and are not suitable for the default internal-pull-up setup.

## Status LED

Default LED pin:

```cpp
#define LED_PIN 2
```

Set `LED_PIN` to `-1` to disable it.

## Persistent settings

The firmware stores user settings in ESP32 NVS using `Preferences`.

Saved parameters include:

- phone-style volume
- maximum volume
- bass / mid / treble
- preset
- Samsung EQ enabled/disabled
- channel mode
- balance
- analog gain limit

Settings are automatically saved a few seconds after changes, or immediately with the `save` command.

## Serial Monitor

Use:

```text
115200 baud
```

Type:

```text
help
```

for the built-in command list.

### Commands

```text
s / status        full status
d / dump          dump TAS page 0 registers
+ / -             volume up / down
vol <0-127>       set volume
maxvol <dB>       max level at volume 127 (-40..0 dB)
again <0-31>      analog gain limit, 0=max, -0.5 dB/step
bass <dB>         -12..12 dB
mid <dB>          -12..12 dB
treble <dB>       -12..12 dB
preset <name>     flat, bass, rock, vocal, night
eq                show EQ
seq on|off        Samsung speaker EQ
mode stereo|mono|swap
bal <-100..100>   balance
play              AVRCP play
pause             AVRCP pause
next              AVRCP next
prev              AVRCP previous
p                 force amplifier PLAY / clear fault latch
m                 amplifier Hi-Z
save              save settings now
reset             restore factory defaults
```

## Volume architecture

Software PCM scaling is disabled through `A2DPNoVolumeControl`.

Phone volume is mapped to the TAS digital volume register instead, preserving PCM resolution.

Defaults in the sketch:

```text
Phone volume:        80 / 127
Maximum output:      -6.5 dB at 127
Volume range:        50 dB
Button step:         6 / 127
```

`maxvol` can limit the maximum level between -40 dB and 0 dB.

`again` controls the TAS analog gain limit from 0 to 31 in 0.5 dB steps.

## Channel modes

```text
mode stereo
mode mono
mode swap
```

Balance:

```text
bal -100   # left
bal 0      # center
bal 100    # right
```

## Fault monitoring

The firmware periodically monitors TAS power state, clock status and fault registers.

If the amplifier unexpectedly leaves PLAY, the firmware can restart it automatically. Repeated protection events can latch the amplifier off to avoid continuous restart loops.

Use:

```text
p
```

to clear the fault latch and force PLAY.

## Track metadata

AVRCP metadata for title, artist and album is requested from the connected source.

Example Serial output:

```text
NOW PLAYING: Artist - Title  [Album]
```

## Build

1. Install Arduino IDE.
2. Install **esp32 by Espressif Systems**, Core 3.x.
3. Install **ESP32-A2DP** version 1.8 or newer.
4. Open the `.ino` file.
5. Select a classic ESP32 board such as ESP32 Dev Module.
6. Build and upload.
7. Open Serial Monitor at 115200 baud.
8. Pair with `TAS5807 Speaker`.

## Important notes

This firmware contains Samsung-specific DSP coefficients and board configuration captured from one Samsung implementation. Other boards may use different speaker EQ, routing, output-stage configuration or gain settings.

The Samsung dump writes `DEVICE_CTRL_1 (0x02) = 0x51`. The sketch keeps the captured value by default and includes an optional compile-time override for testing other supported switching-frequency configurations.

## Repository

Main firmware:

```text
Samsung_TAS5807_ESP32_A2DP.ino
```

## License

No license has been selected yet. Add a license before redistributing the project if you want to define explicit reuse terms.

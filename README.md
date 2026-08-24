# esp32_ms_iobox3 — MicroSquirt I/O Box (ESP-NOW era)

Actuator + analog-input box for the MS2/Extra dash
([esp32_ms_dash](https://github.com/Carputer-project/esp32_ms_dash)). Runs on an
ESP32 WROOM-32 dev board with two GC9A01A 1.28" round displays.

**Architecture note:** the original v3 design read the MicroSquirt CAN bus
directly (TWAI on GPIO4/5, plus an OBD-II mode). That was stripped in Aug-2026 —
the box now receives the decoded `outpc` buffer **over ESP-NOW from the dash**,
which owns the CAN bus. GPIO4/5 are free and carry the clock-backlight LED bar.
Commands from the dash ride the same link. The only wired connection to the car
is power, grounds, actuator outputs and analog taps.

```
dash ──0xA0 outpc @10Hz──► box ──► fan / IAC PWM / O1-O7 / buzzer / LED bar
box  ──0xB0 status @10Hz──► dash  (indicators, warns, fuel %, per-channel mV)
dash ──0xC0 command──────► box   (same grammar as serial console)
box  ──0xD0 ack/snapshot──► peer (diag module reply channel)
```

## Pin map

Defaults below; everything except the fixed gas-gauge CS/DC is runtime-mappable
via `P` commands and persisted in NVS (`P` alone prints the live map).

| Function | Default GPIO | Notes |
|----------|--------------|-------|
| IAC PWM (LEDC ch0, **250 Hz**, 10-bit) | 19 | MOSFET gate (IRLZ44N), hardwired on this board |
| O1–O7 (ULN2003A low-side) | 13, 12, 14, 27, 26, 25, 33 | |
| Idle display GC9A01A SCLK/MOSI/CS/DC | 15, 16, 17, 18 | soft SPI |
| Gas display CS / DC | **2 / 21 (fixed)** | shares SCLK/MOSI with idle display; DC2 was old I2C SDA — I2C bus dropped |
| Warning buzzer (active piezo) | 32 | **inverted: LOW = sound**, pull-up holds silent through resets |
| LED bar (6× WS2812, RMT) | 4 | brightness capped at 64/255 (~25%) — shares logic buck |
| Analog A1–A4 | 36, 39, 34, 35 | input-only pins, ADC_11db |
| CAN | — | removed; GPIO4/5 repurposed |

`P` refuses unusable GPIOs: 0/1/3 (boot straps + UART0), 2 (gas-gauge CS),
6–11 (flash), 20/24 and 28–31 (dead), 34–39 (input-only). Everything else up
to GPIO 33 is accepted — including the free CAN-era pins 4 and 5.

### Analog front end

All four channels: **100 kΩ series from the car wire → node → 4.6 kΩ to GND**
(rework of 2026-08-22 after two boards were killed by a low-impedance bodge;
worst-case pin voltage ~1.76 V even at 40 V load dump). `readAnalogMv()` reports
**source millivolts**: node × (100k+4.6k)/4.6k ≈ ×22.74. A4 additionally caps at
60000 mV so an unplugged sender reads EMPTY (safe direction), not wrapped-full.

A3/A4 are wired ground-side-switched / passive-resistive respectively:
- A3 high beam: car wire → ~220 Ω → node, 100 kΩ top leg tied to +12 V so the
  GND-switched circuit is visible; firmware runs it in L-mode.
- A4 fuel sender: 3V3 → **220 Ω** → node ← sender → GND (Toyota FSM sender
  family: FULL≈3 Ω … EMPTY≈110 Ω). OEM gauge must be disconnected from the line.

## ESP-NOW protocol

Fixed channel 1, softAP **`iobox3-ms`**, broadcast frames, unencrypted.
RX filter: if neither peer MAC is bound, accept all; otherwise sender must match
a bound MAC (`P DASH <aa:bb:cc:dd:ee:ff>` / `P DIAG <mac>`, each with CLEAR;
stored under their own NVS keys, survive schema changes). Dropped frames count
into `dropped=` in `?`.

| Frame | Layout |
|-------|--------|
| `0xA0` (dash→box, 10 Hz, 75 B) | `[A0][maskLo][maskHi][72B outpc]`. mask=0 = heartbeat (link alive, no fresh ECU data): does **not** refresh the failsafe timer |
| `0xB0` (box→peers, 10 Hz, 15 B) | `[B0][anLatch][0][seq][warnLo][gasPct][a1..a4 src-mV LE][iacDuty%]` — receivers with len≥8 guards stay compatible |
| `0xC0` (dash→box) | `[C0][len][ascii cmd]` → FreeRTOS queue → executed in loop() context, never in the WiFi task (NVS commits / TFT teardown / RMT writes can't race drawing) |
| `0xD0` (box→peer) | `[D0][len][cmd]` ack after every queued command; `[D0][0x01][23B snapshot]` after a remote `?` |

Latch bits: 1=indicator L, 2=indicator R, 4=high beam, 8=gas channel.
Debounce: a latch flips after 30 consecutive agreeing ADC reads (~100 ms);
active-high releases −150 mV, active-low releases +150 mV.

## Engine data & failsafe

`outpc` offsets consumed (big-endian ×10): rpm@6 · map@18 · mat@20 · clt@22 ·
tps@24 · batt@26 · afr@28 · iacstep@54 (groups 0/2/3/6).

Freshness = any group seen within **500 ms** (`FAILSAFE_MS`). On stale:
all outputs off, IAC to fail duty (0%), warnings suppressed. The heartbeat
(mask=0) keeps counts alive without faking freshness.

## Outputs

| Output | Modes (`O<n>…`) |
|--------|-----------------|
| Fan (channel `Y <1-7>`, default 6) | auto hysteresis (`F <on °F>` 100–280 validated / `E <off °F>` 90–270) or manual `F 1/0`; re-pointing `Y` releases the old channel first |
| O1–O7 | `O<n> 0`=off · `O<n> 1`=on · `O<n> T<°F>`=coolant temp trigger · `O<n> R<rpm>`=RPM trigger · plain value = manual state |
| Shift light | `S <rpm>` sets O1 to RPM mode at that speed (hardwired light output path) |
| Warn output | `W warnout <0-7>` blinks a channel at 500 ms while any warning latches (skipped if == fanOut) |

Any enabled analog input can force outputs: `A<n> O<k> <thrV>` targets output k,
`A<n> F<thrV>` targets the fan.

## IAC (Toyota rotary ISC)

250 Hz PWM through one choke point `setIac()` capped at **93% duty** — the rotor
slams its mechanical stop above ~94%. Three modes:

- **AUTO** (`I A`): coolant table interpolation (50°F→60% … 200°F→28%) plus RPM
  trim `clamp((target−rpm)/20, −5…+8)`, duty window 5–93%.
- **FOLLOW** (`I F`): tracks TunerStudio's own idle motor — `duty = iacstep × 100 / 255`
  (group 6 required). This mirrors the ECU exactly with zero local math.
- **MANUAL** (`I <0-100>`): direct duty.
- `T <rpm>` sets the AUTO target (≥500).

## Fuel gauge system (A4)

- 5-point calibration `gasCalMv[FULL, ¾, ½, ¼, EMPTY]`, piecewise-linear
  percent between anchors (mV rises as fuel drops). Defaults encode the Toyota
  FSM sender spec through the 220 Ω/3V3 front end: `{1009, 4800, 9800, 16418, 25014}` reported-mV.
- `Q F/E` record live readings as FULL/EMPTY and **auto re-linearise the three
  midpoints** once both anchors are sane; `Q 1/2/3` refine individual quarters
  afterwards (explicit entries win).
- `Q D<0-15>` EMA damping · `Q W<5-90>` low-fuel % · `Q M<mpg>` economy ·
  `Q T<gal>` tank size · plain `Q` dumps everything.
- Display: needle gauge, 240° sweep, quarter marks; ≤ low-fuel threshold the
  ring/needle/% flash red at 2 Hz with LOW FUEL label. Estimated miles =
  pct × tank × mpg. Open-circuit sender reads empty (60k cap → past EMPTY anchor).

## Engine profile warnings

`W` configures thresholds; evaluation requires fresh engine data:

bit 0 IDLE_LO · 1 IDLE_HI · 2 OVERREV · 3 OVERHEAT · 4 HOTAIR · 5 LOWBATT ·
6 HIBATT · 7 OVERBOOST · 8 LEAN · 9 RICH

Warnings latch and hold for `W hold <ms>` (default 3000) after the condition
clears — that hold is the hysteresis downstream consumers see. AFR checks gate
on throttle (TPS ≥ 5%) and valid range, so decel lean never fires.
Priority name order: OVERHEAT → OVERBOOST → OVERREV → LOWBATT → HIBATT →
LEAN → RICH → HOTAIR → IDLEHI → IDLELO.

Buzzer beeps 100 ms / 900 ms while any warning latches or fuel is low
(`B 0/1` enable, `B T` test beep, `B L` continuous loop mode, `X0-X9` raw pin
probing). Only the **low byte** of warn flags reaches the dash frame.

## Bench tools

- **Simulation `G 1`** — 60 s synthetic drive cycle injected into the real
  decode buffer: pull to 8500 rpm (OVERREV), heat soak (OVERHEAT), rich/lean
  excursions, battery sag, idle hunting. Exercises warnings, fan hysteresis,
  shift light, displays exactly as a live link would. `G 0` clears.
- **Boot self-test `Z 1`** — 1 s settle then 3 s sim sweeping CLT from fan-off
  to fan-on+8°F so the real fan hysteresis clicks. Default **off** (ACC-fed
  installs stay quiet).
- **Analog force `A<n> D1/D0/DA`** — force latches to verify the dash UI with
  zero wiring; `*` marks forced channels in `?`.

## Serial console (115200)

Line-based commands; non-printable bytes poison the whole line (discarded —
GPIO3 noise immunity), lines cap at 128 chars. Send `\n?\n` after garbage to
get a clean status dump.

```
?                    full status (link counters, decoded values, gas, fan, IAC,
                     outputs, A1-A4 volts+latch, engine profile)
M                    link mode report ("proto=ms2 link=espnow")
G 1|0                simulation on/off (+ report)
Z 1|0                boot self-test
F <on°F>|A|1|0       fan on-temp / auto / manual
E <off°F>            fan off-temp
I <duty>|A|F         IAC manual / auto / follow
T <rpm>              AUTO target
Y <1-7|0>            fan output channel
S <rpm>              shift-light output setpoint (O1 RPM mode)
O<n> <mode>          output modes (see table)
A<n> …               analog input: <v>=high thr · H<v>/L<v>=polarity+thr ·
                     O<k> <v>/F<v>=output target · D1/D0/DA=bench force · 0=disable
L <r> <g> <b>|0|?    LED bar color / off / report
W …                  engine profile (see above)
B 0|1|T|L            buzzer disable/enable/beep/loop
X0-X9                buzzer pin manual states (X9 = back to auto)
Q …                  fuel gauge calibration (see above)
P …                  pin map + MAC binding + TFT enable + RESET
```

## Build & flash

```bash
pio run                          # build
pio run --target upload          # flash over USB
pio device monitor -b 115200     # console
```

PlatformIO `esp32dev`, Arduino framework pinned to core **2.0.17** (the shared
package gets replaced by other projects' pioarduino cores; `ledcSetup/ledcAttachPin`
here are core-2 API). Libs: Adafruit GFX + GC9A01A.

## Known limitations (honest list)

- Warn flags truncate to the low byte over the air — LEAN/RICH (bits 8/9) beep
  locally but never appear on the dash banner; the dash computes LEAN itself.
- No read-back channel to the dash: settings steppers track sent values as
  assumptions (the diag module's 0xD0 snapshot is the workaround).
- IDLE-HI has no coast gate — closed-throttle above idle-max will beep (M4).
- Gas EMA advances on every `gasPercent()` call; harmless at damp=0, slightly
  cadence-dependent when damped.

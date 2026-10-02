#include <Arduino.h>
#include <driver/pcnt.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Preferences.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
// esp_task_wdt_init()/esp_task_wdt_status() are not reachable via Arduino.h;
// esp32-hal.h declares only the core helpers (enableLoopWDT et al).
#include <esp_task_wdt.h>

// Old CAN pins (GPIO4/5) are now FREE — the SN65HVD230 transceiver is gone.
// PIN_LED retired: GPIO2 is owned by the gas-gauge TFT CS (GAS_CS below).

static constexpr uint8_t PIN_A1 = 36;
static constexpr uint8_t PIN_A2 = 39;
static constexpr uint8_t PIN_A3 = 34;
static constexpr uint8_t PIN_A4 = 35;

// v3 board free right-side pins (15/16/17/18) — GPIO19 is the MOSFET gate.
static constexpr uint8_t PIN_TFT_SCLK_DEF = 15;
static constexpr uint8_t PIN_TFT_MOSI_DEF = 16;
static constexpr uint8_t PIN_TFT_CS_DEF   = 17;
static constexpr uint8_t PIN_TFT_DC_DEF   = 18;
static Adafruit_GC9A01A* s_tft = nullptr;

// Second round display: gas gauge (same GC9A01A). Shares SCLK/MOSI with
// display1 (software SPI, selected by its own CS). CS2=D2 boots high so the
// screen stays off at boot; DC2=D21 (was I2C SDA -- I2C bus dropped).
static constexpr uint8_t GAS_SCLK = 15;
static constexpr uint8_t GAS_MOSI = 16;
static constexpr uint8_t GAS_CS   = 2;
static constexpr uint8_t GAS_DC   = 21;
static Adafruit_GC9A01A* s_gasTft = nullptr;

// Default pin map (iobox3 / v3 board). Runtime-configurable via 'P' command.
static constexpr uint8_t PIN_IAC_DEF = 19;
static constexpr uint8_t PIN_O1_DEF  = 13;
static constexpr uint8_t PIN_O2_DEF  = 12;
static constexpr uint8_t PIN_O3_DEF  = 14;
static constexpr uint8_t PIN_O4_DEF  = 27;
static constexpr uint8_t PIN_O5_DEF  = 26;
static constexpr uint8_t PIN_O6_DEF  = 25;
static constexpr uint8_t PIN_O7_DEF  = 33;
static constexpr uint8_t PIN_BUZZ_DEF = 32;   // only free clean pin on iobox3
static constexpr uint8_t PIN_LED_DATA_DEF = 4; // old CAN RX pin — free since ESP-NOW migration
static constexpr uint8_t PIN_SPEED = 5;      // ABS speed input (LM393 -> PCNT) — old CAN TX, free

static constexpr uint8_t  CAN_GROUP_COUNT = 9;   // outpc groups (72 bytes) carried in 0xA0
static constexpr uint16_t DASH_TX_MS = 100;
static constexpr uint32_t FAILSAFE_MS = 500;
// Fuel-sender sampling rate, 20 Hz. Fixed (not per-loop-pass) so that gasDamp is
// a real time constant: (gasDamp+1) x GAS_SAMPLE_MS. See the gasSampleMv() call
// in loop() for why per-pass was wrong in both directions.
static constexpr uint16_t GAS_SAMPLE_MS = 50;
// A1-A4 input dividers: 100k top (signal->pin) + 4.6k bottom (pin->GND).
// readAnalogMv() reports SOURCE mV: pin_mV * (Rtop+Rbot)/Rbot.
static constexpr float    ADC_R_TOP_OHM = 100000.0f;   // 2026-08-22: front-end rework — all channels 100k series
static constexpr float    ADC_R_BOT_OHM = 4600.0f;
static constexpr float    ADC_DIVIDER   = (ADC_R_TOP_OHM + ADC_R_BOT_OHM) / ADC_R_BOT_OHM;

enum OutMode : uint8_t { OM_OFF = 0, OM_MAN = 1, OM_TEMP = 2, OM_RPM = 3 };

// ESP-NOW link to the dash (replaces the CAN bus). Frame protocol:
//   0xA0 dash -> iobox3: [0xA0][maskLo][maskHi][72B outpc]  = 75B @10Hz
//   0xB0 iobox3 -> dash: [0xB0][anLatch][spd][seq][warn][gas][a1..a4 mV][iac%][modes] = 19B @10Hz
//   0xC0 dash -> iobox3: [0xC0][len][cmd...]
// Both stay on a fixed channel (1) so no AP association is required.
static constexpr uint8_t  FRAME_ECU    = 0xA0;
static constexpr uint8_t  FRAME_STATUS = 0xB0;
static constexpr uint8_t  FRAME_CMD    = 0xC0;
static constexpr uint8_t  FRAME_REPLY  = 0xD0;   // box -> console: OTA cmd ack + snapshot
static constexpr uint8_t  ESP_NOW_CHANNEL = 1;
static const uint8_t      ESP_NOW_BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

enum WarnBit : uint16_t {
    W_IDLE_LO = 1 << 0,
    W_IDLE_HI = 1 << 1,
    W_OVERREV = 1 << 2,
    W_OVERHEAT = 1 << 3,
    W_HOTAIR = 1 << 4,
    W_LOWBATT = 1 << 5,
    W_HIBATT = 1 << 6,
    W_OVERBOOST = 1 << 7,
    W_LEAN = 1 << 8,
    W_RICH = 1 << 9,
};

struct EngineProfile {
    bool    enabled = true;
    int16_t idleRpmMin = 700;
    int16_t idleRpmMax = 1100;
    int16_t maxRpm = 8000;
    int16_t cltMax = 2300;
    int16_t matMax = 1600;
    int16_t battMin = 110;
    int16_t battMax = 160;
    int16_t mapMax = 2800;
    int16_t afrLow = 100;    // below this = RICH
    int16_t afrHigh = 165;   // above this = LEAN
    uint16_t warnHoldMs = 3000;
    uint8_t  warnOut = 0;
};

struct PinMap {
    uint8_t iac = PIN_IAC_DEF;
    uint8_t out[7] = { PIN_O1_DEF, PIN_O2_DEF, PIN_O3_DEF, PIN_O4_DEF, PIN_O5_DEF, PIN_O6_DEF, PIN_O7_DEF };
    uint8_t tftSclk = PIN_TFT_SCLK_DEF;
    uint8_t tftMosi = PIN_TFT_MOSI_DEF;
    uint8_t tftCs   = PIN_TFT_CS_DEF;
    uint8_t tftDc   = PIN_TFT_DC_DEF;
    uint8_t buzz    = PIN_BUZZ_DEF;   // active piezo, 5V-referenced: LOW = sound, Hi-Z = silent
    uint8_t ledData = PIN_LED_DATA_DEF; // clock backlight LED bar (WS2812-type)
};

static constexpr uint16_t CFG_MAGIC = 0x4971;   // 0x4970→0x4971: wipe poisoned gas table (E=0 inversion), adopt 220R-front-end defaults
// Stock gas anchors (reported mV) for the 220R-from-3V3 A4 front end:
// FULL=3R/1009, EMPTY=110R/25014 (Toyota FSM sender spec). Used by `Q R` to
// clear poisoned calibration and as a safe mapping fallback in gasPctFromFilt()
// when the recorded anchors are degenerate/off-scale — so the needle can
// never pin at 100% on garbage data (see 2026-09-20 gas-stuck-at-full fix).
static constexpr uint16_t kGasStockMv[5] = {1009, 4800, 9800, 16418, 25014};
struct Cfg {
    uint16_t magic = CFG_MAGIC;
    PinMap pin;
    uint8_t proto = 0;               // kept for NVS layout compat (always MS2/UART link)
    bool    tftEnable = true;
    int16_t fanOnTemp = 1800;
    int16_t fanOffTemp = 1700;
    bool    fanAuto = true;
    bool    fanManual = false;
    int16_t iacTargetRpm = 900;
    uint8_t iacFailDuty = 0;
    bool    iacAuto = false;
    bool    iacFollow = true;               /* default idle mode = FOLLOW MS */
    uint8_t iacManualDuty = 30;
    int16_t shiftRpm = 7000;
    uint8_t outMode[7] = { OM_RPM, OM_OFF, OM_OFF, OM_OFF, OM_OFF, OM_OFF, OM_OFF };
    int16_t outTemp[7] = { 0, 0, 0, 0, 0, 0, 0 };
    int16_t outRpm[7]  = { 7000, 0, 0, 0, 0, 0, 0 };
    bool    outManual[7] = { false, false, false, false, false, false, false };
    bool    anEnable[4] = { true, true, true, false };   // AE111 build: indicators+beam on, gas until cal
    uint16_t anThresh[4] = { 2000, 2000, 2000, 0 };      // 2V active-high each (100k/4.6k front end)
    uint8_t anOut[4] = { 0, 0, 0, 0 };
    uint8_t fanOut = 6;
    bool    respEnable = false;
    uint8_t respId = 5;
    uint8_t _pad    = 0;          // was bootTest — kept for NVS layout (sim/boottest removed)
    // Reported-mV anchors for the 220R-from-3V3 A4 front end (reported =
    // node_mV * 22.74). PROVENANCE 2026-08-23: Toyota FSM sender spec for
    // this family (AE92/Corolla + 90-93 Celica, toyotanation FSM quotes):
    // FULL = 3R, EMPTY = 110R -> node 44.4mV / 1100mV -> 1009 / 25014 rep.
    // Cross-checked: user's live quarter reading 16418 rep = 61.6R, right
    // where the known Toyota taper (TA22: 1/2 = 33R, fast rise low) puts a
    // quarter tank. Caveat: 3R full = 44mV raw, below the ESP32 ADC's
    // ~150mV linear knee -> near-FULL readings are mushy until SET F
    // records the anchor through the same path (absorbs the offset).
    uint16_t  gasCalMv[5] = {1009, 4800, 9800, 16418, 25014}; // FULL,3/4,HALF,1/4,EMPTY
    uint8_t  gasDamp = 5;       // EMA smoothing 0(raw)..15 — tames small-signal sender jitter; on-car default 5 (2026-09-21)
    uint8_t  lowFuelPct = 20;   // at/below this % the gas gauge flashes red
    uint8_t  gasMpg = 25;       // assumed fuel economy for est. miles remaining
    uint16_t tankGalX10 = 132;  // tank capacity in tenths-gallons (13.2 gal = 132)
    bool    buzzerEnable = true; // beep on any engine-profile warning or low fuel
    bool    ledOn = false;       // clock backlight bar
    uint8_t ledR = 255, ledG = 120, ledB = 20;  // warm amber default
    EngineProfile eng;
};

static constexpr uint8_t kIacTempF[8] = { 50, 80, 100, 120, 140, 160, 180, 200 };
static constexpr uint8_t kIacDuty[8]  = { 60, 55, 50, 45, 40, 35, 30, 28 };

static Cfg        g_cfg;
static Preferences g_prefs;
// False when NVS failed to mount. Every get then returns a default and every put
// is a silent no-op, which is indistinguishable from "empty" unless the callers
// check this explicitly.
static bool s_nvsOk = true;
static const char* const kPrefsName = "iobox";

static uint8_t  s_outpc[72];
/* Group-seen flags as ONE uint32_t, not bool[9].
 *
 * These were nine separate bytes written outside the critical section that
 * guards the payload, while updateOutputs() read them one at a time from the
 * loop() task. The WiFi task publishing a frame mid-read could therefore leave
 * a consumer holding payload from frame N with a mask that was half N and half
 * N-1 — which is exactly the torn state the per-group gating exists to prevent
 * (a stale g_clt read as present-in-this-frame). A 32-bit aligned load is a
 * single atomic access, so one read of this word is self-consistent.
 *
 * It is written inside the same portENTER_CRITICAL as the payload memcpy, and
 * decodeOutpc() snapshots it under the same mux, so mask and payload always
 * come from the same frame. */
static volatile uint32_t s_groupMask = 0;
static uint32_t           s_groupMaskSnap = 0;  // snapshot of the mask that matches the last decoded payload
static inline bool groupSeen(uint8_t i) { return (s_groupMaskSnap >> i) & 1u; }
static uint32_t s_lastFrameMs = 0;
static uint32_t s_realRxMs = 0;    // any REAL frame received (proves link alive)
static bool     s_canFresh = false;
static bool     s_anyGroupSeen = false;

static constexpr uint8_t ADC_PINS[4] = { PIN_A1, PIN_A2, PIN_A3, PIN_A4 };

static uint16_t rdU16(const uint8_t* buf, uint8_t off) { return (uint16_t)((buf[off] << 8) | buf[off + 1]); }
static int16_t  rdS16(const uint8_t* buf, uint8_t off) { return (int16_t)rdU16(buf, off); }

static uint32_t g_rpm = 0;
static int16_t  g_map = 0, g_mat = 0, g_clt = 0, g_tps = 0, g_batt = 0, g_afr = 0;
static int16_t  g_iacStep = 0;

static void resetData() {
    s_groupMask = 0;
    s_groupMaskSnap = 0;
    s_anyGroupSeen = false;
    s_lastFrameMs = 0;
    s_canFresh = false;
    memset(s_outpc, 0, sizeof(s_outpc));
}

static uint8_t interpolateIac(int16_t cltF) {
    if (cltF <= kIacTempF[0]) return kIacDuty[0];
    for (uint8_t i = 1; i < 8; i++) {
        if (cltF <= kIacTempF[i]) {
            int16_t x0 = kIacTempF[i - 1], x1 = kIacTempF[i];
            int16_t y0 = kIacDuty[i - 1], y1 = kIacDuty[i];
            return (uint8_t)(y0 + (y1 - y0) * (cltF - x0) / (x1 - x0));
        }
    }
    return kIacDuty[7];
}

static void setFan(bool on) {
    if (g_cfg.fanOut >= 1 && g_cfg.fanOut <= 7) digitalWrite(g_cfg.pin.out[g_cfg.fanOut - 1], on ? HIGH : LOW);
}
// Fan run-on: after auto mode drops the fan at fanOffTemp, keep it running a
// few extra seconds to bleed residual radiator heat -> widens the effective
// hysteresis, slows cycling. Applies ONLY to the auto hysteresis path; manual
// off and the overheat/failsafe paths stay instant (see updateOutputs).
static constexpr uint32_t FAN_RUNON_MS = 5000;
static void setOut(uint8_t i, bool on) { digitalWrite(g_cfg.pin.out[i], on ? HIGH : LOW); }
// Measured on the bench (2026-08-22): rotary ISC rotor reaches its mechanical
// full-open stop at ~94% PWM duty; above that it slams the stop and bounces.
// Every duty path funnels through setIac(), so cap here once.
static constexpr uint8_t kIacDutyMax = 93;
static void setIac(uint8_t duty) {
    if (duty > kIacDutyMax) duty = kIacDutyMax;
    ledcWrite(0, (uint32_t)duty * 1023 / 100);
}

static uint16_t readAnalogMv(uint8_t i) {
    // Cap below uint16 wrap: A4's only GND path is the sender itself, so an
    // unplugged sender rails the node to 3V3 -> ~75k reported -> previously
    // wrapped to ~9.5k and the gauge read FULL on an open circuit. 60k keeps
    // A1-A3 (max ~40k at load dump) untouched; gasPercent's 32k filter cap
    // then maps 60k past the EMPTY anchor -> reads empty, the safe direction.
    // Oversample x8: ESP32 ADC jitter is tens of mV per sample, so one bad
    // reading near the threshold could out-vote the latch debounce on a
    // floating/noisy line. 8-read mean kills it; cost is ~0.3ms/ch.
    uint32_t sum = 0;
    for (uint8_t k = 0; k < 8; k++) sum += analogRead(ADC_PINS[i]);
    uint32_t mv = (uint32_t)((sum / 8.0f) * 3300.0f / 4095.0f * ADC_DIVIDER);
    return (uint16_t)(mv > 60000u ? 60000u : mv);
}

static const char* tgtName(uint8_t t) {
    static char buf[8];
    if (t == 0) return "-";
    if (t == 7) return "fan";
    snprintf(buf, sizeof buf, "O%u", t);
    return buf;
}

static bool s_anLatch[4] = { false, false, false, false };

// Bench override: -1 = auto (ADC threshold), 0/1 = forced latch state.
// Set via `A<n> D1|D0|DA`. Lets the dash UI be verified with zero wiring.
static int8_t s_anForce[4] = { -1, -1, -1, -1 };

// Per-input polarity: false = active-HIGH (12V feed, default), true =
// active-LOW (GND-switched — latches when voltage FALLS below threshold).
// 2026-08-21: requested by user for A3 high beam in the ST162 — that tap
// idles at ~12V through the lamp filament and pulls to GND when the beam
// is selected. Indicators stay active-high (12V flasher pulses). Polarity
// is per-channel, stored under its own NVS key ("anpol") so toggling it
// never resets Cfg or gas calibration. Set via `A<n> L<v>` / `A<n> H<v>`.
// NOTE: an UNCONNECTED input reads 0V and will latch in low mode — only
// use low mode on channels whose car wire actually idles at ~12V.
static bool s_anLow[4] = { false, false, false, false };

static void saveAnPol() {
    // Upper nibble = validity marker. Guarantees a firmware update can never
    // resurrect stale/unrecognized polarity from NVS: unknown data -> all-high.
    uint8_t b = 0xA0;
    for (uint8_t i = 0; i < 4; i++) if (s_anLow[i]) b |= (uint8_t)(1u << i);
    g_prefs.putUChar("anpol", b);
}
static void loadAnPol() {
    uint8_t b = g_prefs.getUChar("anpol", 0);
    uint8_t v = (b & 0xF0) == 0xA0 ? (b & 0x0F) : 0;
    for (uint8_t i = 0; i < 4; i++) s_anLow[i] = (v & (1u << i)) != 0;
}

static void updateAnalogLatch() {
    // Debounce: a latch flips only after AN_DEBOUNCE consecutive agreeing
    // reads (~100ms at loop cadence). Kills ADC noise/crosstalk chatter on
    // car-harness-length inputs; far shorter than a 1-2Hz flasher phase.
    static bool    candState[4] = { false, false, false, false };
    static uint8_t candCnt[4]   = { 0, 0, 0, 0 };
    constexpr uint8_t AN_DEBOUNCE = 30;

    for (uint8_t i = 0; i < 4; i++) {
        if (s_anForce[i] >= 0) {          // bench override wins over everything
            s_anLatch[i] = s_anForce[i] == 1;
            candCnt[i] = 0;
            continue;
        }
        if (!g_cfg.anEnable[i]) { s_anLatch[i] = false; candCnt[i] = 0; continue; }
        int16_t t = (int16_t)g_cfg.anThresh[i];
        int16_t mv = (int16_t)min(readAnalogMv(i), (uint16_t)32767);   // clamp: >32767 would cast negative
        bool raw;
        if (s_anLow[i]) {
            // GND-switched line: set at/below threshold, release above +150mV
            raw = s_anLatch[i] ? (mv < t + 150) : (mv <= t);
        } else {
            // Active-high: latch on rise above threshold, release below -150mV
            int16_t lo = t > 150 ? t - 150 : 0;
            raw = s_anLatch[i] ? (mv > lo) : (mv >= t);
        }
        if (raw != candState[i]) { candState[i] = raw; candCnt[i] = 1; }
        else if (candCnt[i] < 255) candCnt[i]++;
        if (candCnt[i] >= AN_DEBOUNCE && s_anLatch[i] != raw) s_anLatch[i] = raw;
    }
}

static bool inputForces(uint8_t target) {
    if (target == 0) return false;
    for (uint8_t i = 0; i < 4; i++) {
        if (g_cfg.anOut[i] == target && g_cfg.anEnable[i] && s_anLatch[i]) return true;
    }
    return false;
}

static void outputsOff() {
    setFan(false);
    for (uint8_t i = 0; i < 7; i++) setOut(i, false);
}

/* Which PinMap field a caller is about to overwrite, so pinAliasFree() can tell
 * "this pin is already mine" from "this pin is already someone else's".
 * Deliberately a separate parameter from skip_idx - see pinAliasFree(). */
enum : uint8_t { PINASSIGN_NONE = 0, PINASSIGN_IAC, PINASSIGN_BZ,
                 PINASSIGN_TFTS, PINASSIGN_TFTM, PINASSIGN_TFTC, PINASSIGN_TFTD };

/* True only if s is a complete, non-empty decimal integer (optional leading
 * minus).
 *
 * This exists because String::toInt() returns 0 for anything it cannot parse,
 * so `Y fan`, `Y on`, `Y -` and `I abc` all arrive as 0 - and 0 is a VALID and
 * destructive value for exactly those two commands:
 *   Y: fanOut = 0 means "no fan output", and setFan() writes nothing at
 *      fanOut == 0, so every downstream path believes cooling is being driven
 *      while no pin is touched. This is the io-C1 regression reopened by a
 *      different door: c2b28de fixed the fanOut == 0 symptom and left the
 *      toInt() -> 0 route into it wide open.
 *   I: 0 is a legal idle-air duty, so garbage silently parks the valve shut.
 *
 * Testing the parsed RESULT cannot distinguish "the operator typed 0" from
 * "the operator typed rubbish", so the string has to be validated first. */
static bool argIsInt(const String& s) {
    if (s.length() == 0) return false;
    for (unsigned i = 0; i < s.length(); i++) {
        char c = s[i];
        if (c == '-' && i == 0 && s.length() > 1) continue;
        if (c < '0' || c > '9') return false;
    }
    return true;
}

/* Float sibling of argIsInt(), for the arguments that are genuinely fractional
 * (coolant setpoints, tank capacity). Same reason: String::toFloat() stops at the
 * first bad character, so "212abc" parses as 212.0, and atof() additionally
 * accepts "nan" and "inf", so `(int)toFloat()` is undefined behaviour on those.
 * Both facts have caused real defects here. Hoisted to file scope so `Q T` and
 * `O<n>T` share one definition rather than each arm growing its own lambda -
 * two copies of a validator is two copies to forget to update. */
static bool argIsFloat(const String& s) {
    if (s.length() == 0) return false;
    bool seenDigit = false, seenDot = false;
    for (unsigned i = 0; i < s.length(); i++) {
        char ch = s[i];
        if ((ch == '-' || ch == '+') && i == 0) continue;
        if (ch == '.' && !seenDot) { seenDot = true; continue; }
        if (ch < '0' || ch > '9') return false;
        seenDigit = true;
    }
    return seenDigit;
}

static void handleCommand(const String& line);
static bool pinOk(uint8_t p);
static bool pinAliasFree(uint8_t p, int skip_idx, uint8_t assigning);
// ---------------------------------------------------------------------------
// Clock backlight LED bar (WS2812-type addressable, 6 LEDs) on GPIO4.
// RMT one-shot TX; sends only on change so ESP-NOW timing is untouched.
// Brightness capped in firmware: 6 LEDs at full white pull ~360mA and share
// the logic buck. Raise LED_BRIGHT_MAX only if the bar gets its own feed.
// ---------------------------------------------------------------------------
#include <driver/rmt.h>

static constexpr uint8_t LED_COUNT      = 6;
static constexpr uint8_t LED_BRIGHT_MAX = 64;   // 0-255 scale cap (~25%)
static constexpr rmt_channel_t LED_RMT_CHAN = RMT_CHANNEL_0;

static rmt_item32_t s_ledItems[LED_COUNT * 24];
static volatile bool s_ledBusy = false;

static void ledPush(uint8_t r, uint8_t g, uint8_t b) {
    if (s_ledBusy) return;              // drop frame on rare simultaneous cmd
    s_ledBusy = true;
    uint8_t rr = (uint16_t)r * LED_BRIGHT_MAX / 255;
    uint8_t gg = (uint16_t)g * LED_BRIGHT_MAX / 255;
    uint8_t bb = (uint16_t)b * LED_BRIGHT_MAX / 255;
    const uint8_t rgb[3] = { gg, rr, bb };          // WS2812 wire order: GRB
    uint16_t k = 0;
    for (uint8_t i = 0; i < LED_COUNT; i++) {
        for (uint8_t ch = 0; ch < 3; ch++) {
            uint8_t byte = rgb[ch];
            for (int8_t bit = 7; bit >= 0; bit--) {
                bool on = byte & (1 << bit);
                s_ledItems[k].level0 = 1;
                // 12.5ns ticks (clk_div=1): T1H=800ns T1L=500ns / T0H=400ns T0L=850ns
                s_ledItems[k].duration0 = on ? 64 : 32;
                s_ledItems[k].level1 = 0;
                s_ledItems[k].duration1 = on ? 40 : 68;
                k++;
            }
        }
    }
    rmt_write_items(LED_RMT_CHAN, s_ledItems, k, true);   // blocking ~0.5ms
    rmt_wait_tx_done(LED_RMT_CHAN, pdMS_TO_TICKS(20));
    delayMicroseconds(60);                                // reset latch >50us
    s_ledBusy = false;
}

static void ledApply() {
    if (g_cfg.ledOn) ledPush(g_cfg.ledR, g_cfg.ledG, g_cfg.ledB);
    else             ledPush(0, 0, 0);
}

static void ledInit() {
    rmt_config_t cfg = {};
    cfg.rmt_mode = RMT_MODE_TX;
    cfg.channel = LED_RMT_CHAN;
    cfg.gpio_num = (gpio_num_t)g_cfg.pin.ledData;
    cfg.clk_div = 1;                      // 80MHz/1 = 80MHz -> 12.5ns tick (WS2812 bit timings)
    cfg.mem_block_num = 1;
    cfg.tx_config.loop_en = false;
    cfg.tx_config.carrier_en = false;
    cfg.tx_config.idle_output_en = true;
    cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    if (rmt_config(&cfg) == ESP_OK && rmt_driver_install(LED_RMT_CHAN, 0, 0) == ESP_OK) {
        ledApply();
        Serial.printf("led bar: %u px on GPIO%u (cap %u/255)\n",
                      LED_COUNT, g_cfg.pin.ledData, LED_BRIGHT_MAX);
    } else {
        Serial.println("led bar: RMT init FAILED");
    }
}

static void applyPinConfig();

// ---------------------------------------------------------------------------
// ESP-NOW link to the dash (replaces CAN + UART).
// Frame protocol:
//   0xA0 dash -> iobox3: [0xA0][maskLo][maskHi][72B outpc]  = 75B @10Hz
//   0xB0 iobox3 -> dash: [0xB0][anLatch][spd][seq][warn][gas][a1..a4 mV][iac%][modes] = 19B @10Hz
//   0xC0 dash -> iobox3: [0xC0][len][cmd...]
// Both devices stay on a fixed channel (1) so no AP association is required.
// ---------------------------------------------------------------------------
static uint8_t  s_anLatchByte = 0;
static uint8_t  s_seq = 0;
static uint16_t s_warnLatched = 0;   // forward decl (real def below)
static volatile float s_speedMph = 0; // ABS speed, mph (0xB0 f[2])
static void gasSampleMv();     // ADC read + EMA advance; ONCE per loop()
static int  gasPctFromFilt();  // pure s_gasFilt -> percent mapping
static void gasAutoCal();
static void saveCfg();

static uint32_t s_rxA0Count = 0;
static uint32_t s_rxC0Count = 0;

// Protects s_outpc: written by espnowRecv (WiFi task), read by decodeOutpc
// (loop task). Short critical sections — ~1-2us at 10Hz.
static portMUX_TYPE s_outpcMux = portMUX_INITIALIZER_UNLOCKED;

// 0xC0 commands are queued here and executed from loop(), never inside the
// ESP-NOW RX callback — handlers do NVS commits, TFT teardown/reinit and RMT
// writes that must not race main-loop drawing or stall the WiFi task.
static QueueHandle_t s_cmdQ = nullptr;
static constexpr UBaseType_t CMD_Q_SLOTS = 6;
static constexpr size_t      CMD_Q_LEN   = 72;

// Bound dash MAC (ESP-NOW RX filter). All-zero = unbound = accept any peer.
// Stored under its own NVS key so binding survives Cfg schema changes
// without a CFG_MAGIC bump. Set via `P DASH <aa:bb:cc:dd:ee:ff> | CLEAR`.
static uint8_t s_dashMac[6] = {0, 0, 0, 0, 0, 0};
static bool     s_dashMacHinted = false;
static uint32_t s_rxDroppedCount = 0;

static void loadDashMac() {
    uint8_t buf[6] = {0};
    size_t len = g_prefs.getBytes("dashmac", buf, sizeof(buf));
    if (len == sizeof(buf)) memcpy(s_dashMac, buf, sizeof(buf));
}
static void saveDashMac() { g_prefs.putBytes("dashmac", s_dashMac, sizeof(s_dashMac)); }
static bool dashMacBound() {
    for (uint8_t i = 0; i < 6; i++) if (s_dashMac[i]) return true;
    return false;
}

// Bound diag-module MAC (second allowed sender). Same NVS-survival scheme as
// dashmac — own key, no CFG_MAGIC coupling. Set via `P DIAG <mac> | CLEAR`.
static uint8_t s_diagMac[6] = {0, 0, 0, 0, 0, 0};
static void loadDiagMac() {
    uint8_t buf[6] = {0};
    size_t len = g_prefs.getBytes("diagmac", buf, sizeof(buf));
    if (len == sizeof(buf)) memcpy(s_diagMac, buf, sizeof(buf));
}
static void saveDiagMac() { g_prefs.putBytes("diagmac", s_diagMac, sizeof(s_diagMac)); }
static bool diagMacBound() {
    for (uint8_t i = 0; i < 6; i++) if (s_diagMac[i]) return true;
    return false;
}
// Filter rule: nothing bound -> accept all; else sender must match a bound MAC.
static bool peerAllowed(const uint8_t* mac) {
    if (!dashMacBound() && !diagMacBound()) return true;
    if (dashMacBound() && memcmp(mac, s_dashMac, 6) == 0) return true;
    if (diagMacBound() && memcmp(mac, s_diagMac, 6) == 0) return true;
    return false;
}

static void espnowRecv(const uint8_t* mac, const uint8_t* data, int len) {
    if (!data || len < 1) return;
    if (!peerAllowed(mac)) {
        s_rxDroppedCount++;              // not our dash/diag — ignore
        return;
    }
    if (!dashMacBound() && !diagMacBound() && !s_dashMacHinted) {
        s_dashMacHinted = true;
        Serial.println("note: peer MAC filter unbound — pin it with P DASH/P DIAG <mac> (see ? )");
    }
    switch (data[0]) {
        case FRAME_ECU: {
            if (len < 3 + 72) break;
            s_rxA0Count++;
            uint16_t mask = (uint16_t)(data[1] | (data[2] << 8));
            // mask==0 = heartbeat: link alive but dash has no fresh ECU data.
            // Do NOT refresh s_lastFrameMs — lets the failsafe trip.
            if (mask == 0) break;
            portENTER_CRITICAL(&s_outpcMux);
            memcpy(s_outpc, &data[3], 72);
            s_groupMask = mask & 0x1FFu;   // 9 groups are live; ignore any stray high bits
            portEXIT_CRITICAL(&s_outpcMux);
            s_anyGroupSeen = mask != 0;
            s_lastFrameMs = millis();
            s_realRxMs = millis();
            break;
        }
        case FRAME_CMD: {
            if (len < 2) break;
            s_rxC0Count++;
            uint8_t n = data[1] < (len - 2) ? data[1] : (uint8_t)(len - 2);
            if (n > CMD_Q_LEN - 1) n = CMD_Q_LEN - 1;
            char buf[CMD_Q_LEN];
            uint8_t j = 0;
            for (uint8_t i = 0; i < n; i++) {
                char ch = (char)data[2 + i];
                if (ch == '\0' || ch == '\n' || ch == '\r') break;
                buf[j++] = ch;
            }
            buf[j] = '\0';
            if (s_cmdQ) xQueueSend(s_cmdQ, buf, 0);   // drop if full
            break;
        }
    }
}

static uint32_t s_txB0Count = 0;

static void espnowSendStatus() {
    // v3 frame: v2 (gas% + per-channel source-mV) + live IAC duty % in [14].
    // Dash reads only [1](latch) and [4](warn) with a len>=8 guard, so the
    // extra bytes are ignored by old receivers — fully backward compatible.
    // Extended v4: +[15]=fanMode(0/1/2) +[16]=iacMode(0/1/2) +[17]=buzzerOn +[18]=reserved (was bootTestOn)
    // v5: [2] (was spare 0) now carries speed mph from the ABS/LM393 input.
    uint8_t f[19] = {0};
    f[0] = FRAME_STATUS;
    uint8_t latch = 0;
    for (uint8_t i = 0; i < 4; i++) if (s_anLatch[i]) latch |= (1u << i);
    f[1] = latch;
    f[2] = (uint8_t)constrain((int)s_speedMph, 0, 255);
    f[3] = ++s_seq;
    f[4] = (uint8_t)(s_warnLatched & 0xFF);
    f[5] = (uint8_t)constrain(gasPctFromFilt(), 0, 100);
    for (uint8_t i = 0; i < 4; i++) {
        uint16_t mv = readAnalogMv(i);
        f[6 + i * 2]     = (uint8_t)(mv & 0xFF);
        f[6 + i * 2 + 1] = (uint8_t)(mv >> 8);
    }
    f[14] = (uint8_t)((uint32_t)ledcRead(0) * 100 / 1023);
    // New mode state bytes
    uint8_t fanMode = 0;
    if (g_cfg.fanAuto) fanMode = 1;
    else if (g_cfg.fanManual) fanMode = 2;
    f[15] = fanMode;
    uint8_t iacMode = 0;
    if (g_cfg.iacAuto) iacMode = 1;
    else if (g_cfg.iacFollow) iacMode = 2;
    f[16] = iacMode;
    f[17] = g_cfg.buzzerEnable ? 1 : 0;
    f[18] = 0;   // reserved (bootTest removed — kept 0 for frame layout compat)
    esp_err_t r = esp_now_send(ESP_NOW_BROADCAST, f, sizeof(f));
    s_txB0Count++;
    if (r != ESP_OK) {
        Serial.printf("espnow send B0 FAILED: %s\n", esp_err_to_name(r));
    }
}

// ---------------------------------------------------------------------------
// Vehicle speed from a 2-wire passive ABS VR sensor (AE111 Corolla). The raw
// AC sine is converted to a clean digital square wave by an LM393 conditioner
// on GPIO5 (old CAN TX pin — free). PCNT counts teeth in hardware; every
// DASH_TX_MS tick the counter delta is accumulated over a 1 s window and
// converted to mph. The mph value rides in 0xB0 f[2] (previously a spare 0).
// PULSES_PER_MPH is a place-holder: ~44-tooth front tone ring + ~1.85 m tyre.
// CALIBRATE after install: drive a measured distance and set the constant
// from pulses-per-mile (= pulses/km * 1.609). See abs_speed_sensor.md.
static constexpr float   PULSES_PER_MPH = 10.6f;
static constexpr pcnt_unit_t SPEED_PCNT_UNIT = (pcnt_unit_t)1;   // free (no other PCNT use)
static constexpr int16_t SPEED_PCNT_H_LIM = 32767;
// No s_pcntLast: with M3 the PCNT counter is cleared after every read and
// re-arms from 0, so each tick's raw counter value IS the pulse delta.
static int32_t           s_pcntAcc = 0;
static uint32_t          s_speedWinMs = 0;

static void speedInit() {
    pcnt_config_t pc = {};
    pc.pulse_gpio_num = (gpio_num_t)PIN_SPEED;
    pc.ctrl_gpio_num  = PCNT_PIN_NOT_USED;
    pc.lctrl_mode     = PCNT_MODE_KEEP;
    pc.hctrl_mode     = PCNT_MODE_KEEP;
    pc.pos_mode       = PCNT_COUNT_INC;   // count rising edges only
    pc.neg_mode       = PCNT_COUNT_DIS;
    pc.counter_h_lim  = SPEED_PCNT_H_LIM;
    pc.counter_l_lim  = 0;
    pc.unit           = (pcnt_unit_t)SPEED_PCNT_UNIT;
    pc.channel        = PCNT_CHANNEL_0;
    pcnt_unit_config(&pc);
    pcnt_set_filter_value(SPEED_PCNT_UNIT, 100);   // reject < ~1.25 us noise
    pcnt_filter_enable(SPEED_PCNT_UNIT);
    pcnt_counter_pause(SPEED_PCNT_UNIT);
    pcnt_counter_clear(SPEED_PCNT_UNIT);
    pcnt_counter_resume(SPEED_PCNT_UNIT);
    Serial.printf("speed: PCNT ch on GPIO%d ready (pulses/mph placeholder %.1f)\n",
                  (int)PIN_SPEED, (double)PULSES_PER_MPH);
}

// dtMs between calls (call from the 10 Hz dash tick). Untouched when the
// window isn't a full second, so the gauge stays frozen on the same value.
static void speedTick(uint32_t dtMs) {
    int16_t cnt = 0;
    pcnt_get_counter_value((pcnt_unit_t)SPEED_PCNT_UNIT, &cnt);
    // The ESP32 PCNT hardware wraps back to 0 when it hits h_lim (32767), NOT
    // through two's complement — at 60 mph (~636 pulses/s) it resets every
    // ~51.5 s ($H_LIM/$10.6). The old uint16 arithmetic only handled the true
    // 2's-complement wrap, so the 32767->0 transition read as a huge negative
    // delta and got swallowed by the ">0" guard -> a whole second of pulses
    // were lost -> recurring mph dip every ~51 s at 60 mph (M3).
    //
    // Fix: byte delta off of the counter restarted from 0 each read. Clear the
    // hardware counter after every sample so it can never reach h_lim, and
    // count the current value directly as "pulses since last read".
    pcnt_counter_clear(SPEED_PCNT_UNIT);   // re-arm; next cnt = pulses since now
    int32_t delta = (int32_t)cnt;          // counter was 0 at the previous clear
    if (delta > 0) {
        s_pcntAcc += delta;
        s_speedWinMs += dtMs;
    } else {
        s_speedWinMs += dtMs;  // keep the window advancing even while stopped
    }
    if (s_speedWinMs >= 1000) {
        float hz = (float)s_pcntAcc * 1000.0f / (float)s_speedWinMs;
        s_speedMph = hz / PULSES_PER_MPH;
        s_pcntAcc = 0;
        s_speedWinMs = 0;
    }
}

// 0xD0 reply channel for the diag module (and any future console peer).
// Sent from loop() context immediately after a queued OTA command executes —
// never inside the ESP-NOW RX callback. Purely additive: A0/B0 formats and
// the outpc receive path are untouched.
//   [D0][len][cmd...]        ack + echo of the command that just ran
//   [D0][0x01][27B snapshot] machine-readable state, sent after a remote '?'
static void espnowSendAck(const char* cmd) {
    size_t n = strlen(cmd);
    if (n > 20) n = 20;
    uint8_t f[22] = {0};
    f[0] = FRAME_REPLY;
    f[1] = (uint8_t)n;
    memcpy(&f[2], cmd, n);
    esp_now_send(ESP_NOW_BROADCAST, f, 2 + n);
}

static void espnowSendSnapshot() {
    // v2 snapshot: [2]=canFresh [3..16]=rpm,map,mat,clt,tps,batt,afr LE [17]=iac%
    // [18]=flags b0 fan b1 follow b2 auto b4 buzzing b5 eng-enabled
    // [19..20]=iacTarget [21]=O1-7 bitmask [22]=warnLatched low byte
    // v3: +[23]=fanMode(0/1/2) +[24]=iacMode(0/1/2) +[25]=reserved (was bootTestOn) +[26]=reserved
    uint8_t f[27] = {0};
    auto wr16 = [&](int o, int v){ f[o] = (uint8_t)(v & 0xFF); f[o+1] = (uint8_t)((v >> 8) & 0xFF); };
    f[0] = FRAME_REPLY; f[1] = 0x01;
    f[2] = s_canFresh ? 1 : 0;
    wr16(3,  (int)g_rpm); wr16(5,  (int)g_map); wr16(7,  (int)g_mat);
    wr16(9,  (int)g_clt); wr16(11, (int)g_tps); wr16(13, (int)g_batt);
    wr16(15, (int)g_afr);
    f[17] = (uint8_t)((uint32_t)ledcRead(0) * 100 / 1023);
    bool fanOn = g_cfg.fanOut >= 1 && g_cfg.fanOut <= 7 &&
                 digitalRead(g_cfg.pin.out[g_cfg.fanOut - 1]);
    f[18] = (uint8_t)((fanOn ? 1 : 0) | (g_cfg.iacFollow ? 2 : 0) | (g_cfg.iacAuto ? 4 : 0)
                    | (digitalRead(g_cfg.pin.buzz) == LOW ? 16 : 0)
                    | (g_cfg.eng.enabled ? 32 : 0));
    wr16(19, (int)g_cfg.iacTargetRpm);
    uint8_t outs = 0;
    for (uint8_t i = 0; i < 7; i++) if (digitalRead(g_cfg.pin.out[i])) outs |= (1u << i);
    f[21] = outs;
    f[22] = (uint8_t)(s_warnLatched & 0xFF);
    // New mode state bytes (v3)
    uint8_t fanMode = 0;
    if (g_cfg.fanAuto) fanMode = 1;
    else if (g_cfg.fanManual) fanMode = 2;
    f[23] = fanMode;
    uint8_t iacMode = 0;
    if (g_cfg.iacAuto) iacMode = 1;
    else if (g_cfg.iacFollow) iacMode = 2;
    f[24] = iacMode;
    f[25] = 0;   // bootTest removed — reserved
    f[26] = 0; // reserved
    esp_now_send(ESP_NOW_BROADCAST, f, sizeof(f));
}

static void espnowInit() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("iobox3-ms", NULL, ESP_NOW_CHANNEL);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init FAILED");
        return;
    }
    esp_now_register_recv_cb(espnowRecv);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, ESP_NOW_BROADCAST, 6);
    peer.channel = ESP_NOW_CHANNEL;
    peer.ifidx = WIFI_IF_AP;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
    Serial.println("ESP-NOW link up (ch1)");
}

static void decodeOutpc() {
    // Snapshot under the mux so a frame arriving mid-decode can't tear a
    // BE16 pair across two different broadcasts. The group mask is snapshotted
    // in the same critical section: taking it afterwards could pair this
    // payload with the NEXT frame's mask, re-introducing the staleness the
    // per-group gating is there to prevent.
    uint8_t  local[72];
    uint32_t mask;
    portENTER_CRITICAL(&s_outpcMux);
    memcpy(local, s_outpc, sizeof(local));
    mask = s_groupMask;
    s_groupMaskSnap = mask;  // consistent with the local[72] copy just snapped under the same mux
    portEXIT_CRITICAL(&s_outpcMux);

    if (mask & (1u << 0)) g_rpm  = rdU16(local, 6);
    if (mask & (1u << 2)) {
        g_map  = rdS16(local, 18);
        g_mat  = rdS16(local, 20);
        g_clt  = rdS16(local, 22);
    }
    if (mask & (1u << 3)) {
        g_tps  = rdS16(local, 24);
        g_batt = rdS16(local, 26);
        g_afr  = rdS16(local, 28);
    }
    if (mask & (1u << 6)) g_iacStep = rdS16(local, 54);
}

// CAN-less simulation and boot self-test REMOVED (never inject synthetic ECU
// data — the box only ever mirrors real dash 0xA0 frames).
static uint16_t s_warnRaw = 0;
static uint32_t s_warnFirstMs = 0;

static const char* kTopWarnOrder[] = {
    "OVERHEAT", "OVERBOOST", "OVERREV", "LOWBATT", "HIBATT",
    "LEAN", "RICH", "HOTAIR", "IDLEHI", "IDLELO"
};

static const char* topWarnName(uint16_t raw) {
    static const uint16_t kPrio[] = { W_OVERHEAT, W_OVERBOOST, W_OVERREV, W_LOWBATT, W_HIBATT,
                                      W_LEAN, W_RICH, W_HOTAIR, W_IDLE_HI, W_IDLE_LO };
    for (uint8_t i = 0; i < 10; i++) {
        if (raw & kPrio[i]) return kTopWarnOrder[i];
    }
    return "";
}

static uint16_t engineWarnFlags() {
    uint16_t raw = 0;
    if (!s_canFresh) return 0;
    bool cltOk = groupSeen(2) && g_clt > 100 && g_clt < 3500;
    bool matOk = groupSeen(2) && g_mat > 0 && g_mat < 3000;
    /* map had no reading sanity gate at all, unlike clt/mat/afr. `g_map > 0`
     * is the minimum: a zero or absent reading is not evidence of overboost.
     * With no gate, a stored threshold of 0 (reachable until the W map range
     * floor was raised) matched every positive MAP - i.e. every running
     * engine - and latched W_OVERBOOST permanently. */
    bool mapOk = groupSeen(2) && g_map > 0 && g_map < 4000;
    bool onThrottle = groupSeen(3) && g_tps >= 50;
    bool afrOk = onThrottle && g_afr >= 100 && g_afr <= 250;

    // Partial-frame hardening: only trust a field if its group was present in
    // the current frame. On a real 0xA0 broadcast all 9 groups arrive together,
    // but if a frame is ever partial, g_* values from a previous frame would be
    // stale here and could misfire a warning / mis-drive an actuator.
    bool rpmOk = groupSeen(0);
    bool tpsOk = groupSeen(3);
    bool battOk = groupSeen(3);

    if (rpmOk && g_rpm >= g_cfg.eng.maxRpm) raw |= W_OVERREV;
    if (rpmOk && g_rpm > 0 && g_rpm < g_cfg.eng.idleRpmMin && tpsOk && g_tps < 200) raw |= W_IDLE_LO;
    if (rpmOk && g_rpm > 0 && g_rpm > g_cfg.eng.idleRpmMax && tpsOk && g_tps < 200) raw |= W_IDLE_HI;
    if (cltOk && g_clt > g_cfg.eng.cltMax) raw |= W_OVERHEAT;
    if (matOk && g_mat > g_cfg.eng.matMax) raw |= W_HOTAIR;
    if (battOk && g_batt > 0 && g_batt < g_cfg.eng.battMin) raw |= W_LOWBATT;
    if (battOk && g_batt > 0 && g_batt > g_cfg.eng.battMax) raw |= W_HIBATT;
    if (mapOk && g_map > g_cfg.eng.mapMax) raw |= W_OVERBOOST;
    if (afrOk && g_afr > g_cfg.eng.afrHigh) raw |= W_LEAN;
    if (afrOk && g_afr < g_cfg.eng.afrLow) raw |= W_RICH;
    return raw;
}

static void updateEngineProfile() {
    /* 'W 0' (eng.enabled == false) must actually silence the warning system.
     * g_cfg.eng.enabled was read in exactly three places -- the '?' report, the
     * 0xD0 snapshot flag, and the two setters -- and never gated
     * engineWarnFlags(). So after W 0 every warning still computed, the buzzer
     * still beeped, and eng.warnOut kept blinking its relay. */
    if (!g_cfg.eng.enabled) {
        s_warnRaw = 0;
        s_warnLatched = 0;
        /* stop any latch-driven buzz via updateBuzzer() below: with no
         * latched warning and eng.enabled false it will not retrigger */
        return;
    }
    s_warnRaw = engineWarnFlags();
    if (s_warnRaw) {
        if (!s_warnLatched) s_warnFirstMs = millis();
        s_warnLatched = s_warnRaw;
    } else if (s_warnLatched && (millis() - s_warnFirstMs) >= g_cfg.eng.warnHoldMs) {
        s_warnLatched = 0;
    }
}

static uint32_t s_fanOffAtMs = 0;            /* when the fan was last commanded off */
static bool     s_fanHeldAfterLinkLoss = false;

static void updateOutputs() {
    /* Link-lost failsafe.
     *
     * s_canFresh goes false 500 ms after the dash stops sending a non-zero
     * mask, and the dash zeroes its mask as soon as ITS CAN receive goes stale
     * for >1 s. So an ordinary ECU reset, a dash reboot, or a second of LVGL
     * task starvation on the dash reaches this path.
     *
     * This path has been rewritten twice and was still wrong both times, in the
     * same way: it tried to handle link loss by RETURNING EARLY, and everything
     * past the return was therefore unreachable in exactly the case it exists
     * for. Three consequences, all real:
     *   1. The emergency overheat override ("a disabled fan can never kill the
     *      engine on a hot day") was stranded whenever fanHoldOn came out
     *      false, which includes a stalled engine at high coolant temp.
     *   2. inputForces() was stranded, and that is the ONLY path that drives
     *      the turn indicators and high beam from A1-A3. Those are the car's
     *      own flasher pulses and headlamp feed on GPIO36/39/34 - they have
     *      nothing to do with the dash. A >=5 s dropout switched off indicators
     *      that were wired, powered and pulsing correctly.
     *   3. It called setIac(g_cfg.iacFailDuty), and iacFailDuty is never
     *      written by anything - grep finds only the "= 0" default and these
     *      two call sites. So it was permanently setIac(0) -> ledcWrite(0,0),
     *      and per the note on setIac() 0% is the CLOSED stop (94% is
     *      mechanical full-open). With the engine idling warm and the link down
     *      for more than FAN_RUNON_MS, that slammed the rotary ISC shut and
     *      re-applied it every pass. Its justifying comment read "there is no
     *      engine to stall" - but engineRunning was TRUE, and true is the only
     *      way to reach that branch. A comment asserting the opposite of the
     *      guard that precedes it.
     *
     * There is now NO early return. Link loss selects a different set of
     * actions; it does not skip the pass:
     *   - cooling keeps its run-on hold and still reaches the overheat override
     *   - the local A1-A3 inputs keep driving their relays
     *   - link-derived output modes (OM_TEMP/OM_RPM) are suppressed, because
     *     the outpc payload is frozen and acting on it would be acting on
     *     stale data. OM_MAN is a local decision and is honoured.
     *   - the IAC is HELD, not closed, whenever the engine is running. Holding
     *     means not writing it: the LEDC register keeps its last commanded
     *     duty, which is the best guess available with no data. Only a stopped
     *     engine gets iacFailDuty.
     */
    bool linkFresh = s_canFresh;
    bool engineRunning = groupSeen(0) && g_rpm > 300;
    bool fanHoldOn    = false;   /* minimum-on hold while the link is down */

    if (!linkFresh) {
        if (!s_fanHeldAfterLinkLoss) { s_fanHeldAfterLinkLoss = true; s_fanOffAtMs = millis(); }
        bool withinRunOn = (millis() - s_fanOffAtMs) < FAN_RUNON_MS;

        /* Cooling on link loss must not depend on the group-seen flag: a frame
         * that omitted group 2 must not be able to switch the fan off when the
         * last known coolant was hot. So this uses raw g_clt, guarded only by a
         * sane range, and NOT the cltReadable gate above - that is correct for
         * the warning path (do not act on absent data) and wrong here.
         *
         * (An earlier `cltHot = cltReadable && ...` local sat alongside this and
         * was dead: the fail path used cltHotForFail, and cltReadable survived
         * only to feed it. GCC -Wall confirms it was never read. Removed.) */
        bool cltHotForFail = (g_clt > 100 && g_clt < 3500 && g_clt >= g_cfg.fanOnTemp);

        fanHoldOn = cltHotForFail || withinRunOn;
    } else {
        s_fanHeldAfterLinkLoss = false;
    }

    int16_t clt = g_clt;
    bool cltOk = groupSeen(2) && clt > 100 && clt < 3500;

    bool fanOn = false;
    // Hoisted to updateOutputs() scope (not the auto+cltOk block) so the
    // manual branch below can clear the mode-continuity latch. Without the
    // clear, a manual->auto round trip kept fanAutoPrev set from the last auto
    // session and the mode-switch guard could never fire — the stale run-on
    // timer then resurrected a spurious ~5 s fan run-on (M2, 2026-09-17).
    static bool fanState = false;
    static bool fanAutoPrev = false;    // tracks auto-mode continuity
    static uint32_t fanRunOnStart = 0;
    if (g_cfg.fanAuto) {
        if (cltOk) {
            // Mode-switch guard: if we left auto and came back, reset the
            // run-on timer so a stale value can't force a spurious run-on.
            if (!fanAutoPrev) fanRunOnStart = 0;
            fanAutoPrev = true;
            if (!fanState && clt >= g_cfg.fanOnTemp) {
                fanState = true;
            } else if (fanState && clt <= g_cfg.fanOffTemp) {
                fanState = false;
                fanRunOnStart = millis();
            }
            fanOn = fanState;
            // Run-on: hold a few seconds after the off trigger (auto cycling
            // only) to bleed residual heat and slow the on/off oscillation.
            if (!fanState && fanRunOnStart && (millis() - fanRunOnStart) < FAN_RUNON_MS) fanOn = true;
        }
    } else {
        // Left auto mode: clear the continuity latch so re-entering auto
        // resets the run-on timer and can't resurrect a stale run-on.
        fanAutoPrev = false;
        fanRunOnStart = 0;
        fanOn = g_cfg.fanManual;
    }
    // Emergency override: if the engine is at fan-on temp, force the fan on
    // regardless of mode (auto/manual/off). Overrides fanManual=false and
    // fanAuto=false so a disabled fan can never kill the engine on a hot day.
    // Fires at the same fixed fanOnTemp (180F) as auto mode — the fan never
    // runs hotter than 180 nominal in any mode.
    if (cltOk && clt >= g_cfg.fanOnTemp) fanOn = true;
    fanOn = fanOn || inputForces(7);
    // Link-lost minimum-on hold: keep the fan driven after the ESP-NOW link
    // drops so a flapping link cannot strobe it, and so a warm engine stays
    // cooled through the dropout. Uses the raw g_clt (not the groupSeen(2)-
    // gated cltOk) because the whole point is to keep cooling when the last
    // frame did not carry coolant data.
    if (fanHoldOn) fanOn = true;
    setFan(fanOn);

    for (uint8_t i = 0; i < 7; i++) {
        if ((i + 1) == g_cfg.fanOut) continue;
        bool on = false;
        /* Link down: OM_TEMP/OM_RPM read a frozen outpc payload, so honouring
         * them means acting on stale data. OM_MAN is a local operator decision
         * with no dependency on the ECU and is kept. */
        if (linkFresh) {
            switch (g_cfg.outMode[i]) {
                case OM_OFF:  on = false; break;
                case OM_MAN:  on = g_cfg.outManual[i]; break;
                case OM_TEMP: on = cltOk && clt >= g_cfg.outTemp[i]; break;
                case OM_RPM:  on = groupSeen(0) && g_rpm >= g_cfg.outRpm[i]; break;
            }
        } else if (g_cfg.outMode[i] == OM_MAN) {
            on = g_cfg.outManual[i];
        }
        /* inputForces() is honoured on BOTH paths and always was the point:
         * A1-A3 are local car wires, not dash data. */
        on = on || inputForces(i + 1);
        setOut(i, on);
    }

    /* Frozen warning data must not drive the warn relay at 2 Hz forever. */
    if (linkFresh && g_cfg.eng.warnOut >= 1 && g_cfg.eng.warnOut <= 7 &&
        g_cfg.eng.warnOut != g_cfg.fanOut && s_warnLatched) {
        bool blink = ((millis() / 500) & 1) == 0;
        setOut(g_cfg.eng.warnOut - 1, blink);
    }

    /* Link down + engine running: HOLD the valve. The correct action with no
     * data is no action - the LEDC register keeps its last commanded duty.
     * Writing iacFailDuty here is what closed a warm idling engine's idle air
     * after a link dropout; the "engine stopped, so closing is safe" branch is
     * the ONLY case where that value is defensible. */
    if (!linkFresh && engineRunning) return;

    uint8_t duty;
    if (!linkFresh) {
        setIac(g_cfg.iacFailDuty);   /* engine stopped: no idle to stall */
        return;
    }
    if (g_cfg.iacFollow && groupSeen(6)) {
        duty = (uint8_t)constrain((int16_t)(g_iacStep * 100 / 255), 0, 100);
    } else if (g_cfg.iacAuto || (g_cfg.iacFollow && !groupSeen(6))) {
        if (!cltOk) {
            duty = interpolateIac(120);
        } else {
            duty = interpolateIac(clt / 10);
            // Trim idle towards the target rpm, but only if the current frame
            // actually carried RPM — otherwise g_rpm is stale and the trim is
            // garbage. Partial-frame hardening (see engineWarnFlags).
            if (groupSeen(0)) {
                int16_t err = g_cfg.iacTargetRpm - (int16_t)g_rpm;
                int16_t trim = constrain((int16_t)(err / 20), -5, 8);
                duty = constrain((int16_t)duty + trim, 5, kIacDutyMax);
            }
        }
    } else {
        duty = g_cfg.iacManualDuty;
    }
    setIac(duty);
}

// Warning buzzer (active piezo, 5V-referenced) on PinMap.buzz (default GPIO32).
// Beeps whenever ANY engine-profile warning is latched or fuel is low:
// 100ms beep / 900ms silence repeating while the condition holds. No latch of
// its own — it stops as soon as the condition clears. B T (test) fires one
// 100ms beep immediately.
static uint32_t s_buzzTestUntilMs = 0;
static bool s_buzzLoop = false;    // B L: backup-truck beep until B 0 / B 1 / B L

static bool s_buzzManual = false;  // X <n>: manual pin test, auto drive paused
/* A manual pin test must not be able to hold the horn on forever. `X 3` drives
 * the pin LOW (continuous sound) and s_buzzManual used to clear only on `X 9` or
 * a `P WIPE` - so an operator who walked away after testing left an unstoppable
 * horn, and silencing it meant erasing the whole configuration.
 *
 * 5 s: long enough to hear and diagnose a pin test, short enough that an
 * unattended box goes quiet on its own. Same magnitude as FAN_RUNON_MS. */
static constexpr uint32_t BUZZ_MANUAL_MS = 5000;
static uint32_t s_buzzManualUntilMs = 0;

/* Give the buzzer pad back to the firmware. SHARED by `X 9` and the manual-test
 * timeout, deliberately: this file already shipped the bug where a path cleared
 * s_buzzManual but left the pad configured, so the pin was still not driven by
 * updateBuzzer() and the buzzer stayed dead until a reboot. Clearing the flag
 * without restoring the pin is the same bug a second time, so there is exactly
 * one implementation of "release" and both callers use it.
 *
 * The `!= iac` guard is not optional: on a config where pin.buzz == pin.iac
 * this pad is the idle-air MOSFET gate and must be left to its LEDC PWM. */
static void buzzReleasePad(uint8_t p) {
    if (p > 39) return;
    if (p == g_cfg.pin.iac) return;   // that pad belongs to the valve's LEDC
    pinMode(p, OUTPUT);
    gpio_pullup_en((gpio_num_t)p);
    digitalWrite(p, HIGH);             // inverted: HIGH = silent
}
static void buzzReleasePin() { buzzReleasePad(g_cfg.pin.buzz); }

static void updateBuzzer() {
    static uint32_t last = 0;
    uint32_t now = millis();
    if (s_buzzManual) {
        if (now >= s_buzzManualUntilMs) {
            s_buzzManual = false;
            buzzReleasePin();          // restore the pad, then resume auto drive
            Serial.println("buzzpin=auto (manual pin test timed out)");
        }
        return;                         // manual pin test in progress
    }
    if (now - last < 100) return;   // 10 Hz — matches display cadence
    last = now;
    bool on;
    if (s_buzzLoop) {
        on = (now % 1000) < 100;    // continuous 100ms/900ms backup alarm
    } else if (now < s_buzzTestUntilMs) {
        on = true;                   // B T test beep
    } else if (g_cfg.buzzerEnable) {
        bool lowFuel = gasPctFromFilt() <= g_cfg.lowFuelPct;
        bool anyWarn = s_warnLatched != 0 || lowFuel;
        on = anyWarn && ((now % 1000) < 100);
    } else {
        on = false;
    }
    if (g_cfg.pin.buzz != g_cfg.pin.iac)   // never touch the IAC pin
        digitalWrite(g_cfg.pin.buzz, on ? LOW : HIGH);   // INVERTED: LOW = sound
}

static void drawGaugeFrame() {
    s_tft->fillScreen(GC9A01A_BLACK);
    s_tft->drawCircle(120, 120, 119, GC9A01A_NAVY);
    s_tft->drawCircle(120, 120, 118, GC9A01A_DARKGREY);
    s_tft->setTextColor(GC9A01A_CYAN);
    s_tft->setTextSize(2);
    s_tft->setCursor(96, 20);
    s_tft->print("IDLE");
    s_tft->setTextColor(GC9A01A_LIGHTGREY);
    s_tft->setTextSize(1);
    s_tft->setCursor(40, 130); s_tft->print("RPM");
    s_tft->setCursor(108, 130); s_tft->print("TGT");
    s_tft->setCursor(40, 168); s_tft->print("CLT");
    s_tft->setCursor(108, 168); s_tft->print("MODE");
}

static char s_lastDuty[8] = "";
static char s_lastRpm[8] = "";
static char s_lastTgt[8] = "";
static char s_lastClt[8] = "";
static char s_lastMode[8] = "";
static char s_lastStat[24] = "";
static char s_lastWarn[12] = "";

static void drawValue(int16_t x, int16_t y, uint8_t size, uint16_t color,
                      uint16_t clearW, char* last, const char* s) {
    if (strcmp(last, s) == 0) return;
    strcpy(last, s);
    s_tft->fillRect(x, y - 2, clearW, size * 8 + 4, GC9A01A_BLACK);
    s_tft->setCursor(x, y);
    s_tft->setTextSize(size);
    s_tft->setTextColor(color);
    s_tft->print(s);
}

static void updateDisplay() {
    if (!g_cfg.tftEnable || s_tft == nullptr) return;
    /* 24, not 12: the longest string formatted here is "FAN OFF  LINK OK" at
     * 17 chars + NUL = 18. buf[12] truncated it to "FAN OFF  LI", and the
     * centring below then used the truncated length, so the one line that
     * reports both fan state and link state was both wrong and off-centre.
     * s_lastStat is already 24; drawValue needs room for the widest %s/%d too. */
    char buf[24];

    uint8_t duty = (uint8_t)(ledcRead(0) * 100 / 1023);
    snprintf(buf, sizeof buf, "%u%%", duty);
    drawValue(48, 48, 6, GC9A01A_CYAN, 144, s_lastDuty, buf);

    snprintf(buf, sizeof buf, "%u", s_canFresh ? g_rpm : 0);
    drawValue(40, 146, 2, s_canFresh ? GC9A01A_WHITE : GC9A01A_DARKGREY, 64, s_lastRpm, buf);

    snprintf(buf, sizeof buf, "%d", (int)g_cfg.iacTargetRpm);
    drawValue(108, 146, 2, GC9A01A_YELLOW, 100, s_lastTgt, buf);

    bool cltOk = groupSeen(2) && g_clt > 100 && g_clt < 3500;
    snprintf(buf, sizeof buf, cltOk ? "%dF" : "--", g_clt / 10);
    drawValue(40, 184, 2, cltOk ? GC9A01A_YELLOW : GC9A01A_DARKGREY, 64, s_lastClt, buf);

    const char* mode = g_cfg.iacFollow ? "FOLLOW" : (g_cfg.iacAuto ? "AUTO" : "MAN");
    snprintf(buf, sizeof buf, "%s", mode);
    drawValue(108, 184, 2, GC9A01A_GREEN, 100, s_lastMode, buf);

    const char* warn = s_warnLatched ? topWarnName(s_warnLatched) : "";
    if (strcmp(s_lastWarn, warn)) {
        strcpy(s_lastWarn, warn);
        s_tft->fillRect(60, 202, 120, 10, GC9A01A_BLACK);
        if (s_warnLatched) {
            bool blink = ((millis() / 500) & 1) == 0;
            s_tft->setCursor((240 - (int)strlen(warn) * 6) / 2, 202);
            s_tft->setTextSize(1);
            s_tft->setTextColor(blink ? GC9A01A_RED : GC9A01A_DARKGREY);
            s_tft->print(warn);
        }
    }

    uint16_t sc;
    if (!s_canFresh) {
        sc = GC9A01A_RED;
        snprintf(buf, sizeof buf, "LINK LOST");
    } else if (g_cfg.fanOut >= 1 && g_cfg.fanOut <= 7 && digitalRead(g_cfg.pin.out[g_cfg.fanOut - 1])) {
        sc = GC9A01A_GREEN;
        snprintf(buf, sizeof buf, "FAN ON  LINK OK");
    } else {
        sc = GC9A01A_LIGHTGREY;
        snprintf(buf, sizeof buf, "FAN OFF  LINK OK");
    }
    if (strcmp(s_lastStat, buf)) {
        strcpy(s_lastStat, buf);
        s_tft->fillRect(60, 212, 120, 10, GC9A01A_BLACK);
        s_tft->setCursor((240 - (int)strlen(buf) * 6) / 2, 212);
        s_tft->setTextSize(1);
        s_tft->setTextColor(sc);
        s_tft->print(buf);
    }
}

static int16_t s_gasFilt = -1;   // damped reported-mV, -1 = uninitialised
// The HIGHEST/LOWEST percentage observed under the CURRENT table since it last
// moved, and the mV that produced it. These are the auto-cal gates (see
// gasLogUpdate); deliberately NOT the lifetime log extremes.
// The pct is kept because "most extreme" is a property of the pct, not of the
// sample order — recording the LAST sample that happened to cross 95% would let a
// sender bouncing 95/99/97/96 commit 96 and re-linearise the whole tank curve off
// a noise-contaminated endpoint. -1 / 101 are outside [0,100] so the first real
// sample always wins.
static int8_t   s_fullPctSinceCal  = -1;
static int16_t  s_fullMvSinceCal   = -1;
static int8_t   s_emptyPctSinceCal = 101;
static int16_t  s_emptyMvSinceCal  = -1;

/* The ONLY function that reads the ADC and advances the EMA. Called exactly
 * once per loop() iteration, from loop(), before any consumer runs.
 *
 * This used to be the first half of gasPercent(), which was called from six
 * places (0xB0 frame, low-fuel buzzer test, gas log, gas display, and two
 * Serial status prints) each of which also advanced the filter. So the damping
 * a caller got depended on how many OTHER callers happened to run first in the
 * same iteration — gasDamp was a per-call divisor, not a time constant, and the
 * effective smoothing changed with which code path was active (display off,
 * link down, a Serial command mid-print). Separating the sampling from the
 * mapping makes the smoothing rate a property of loop() alone. */
static void gasSampleMv() {
    uint16_t raw = readAnalogMv(3);   // A4 = fuel sender (GPIO35)
    int32_t f;
    if (g_cfg.gasDamp == 0 || s_gasFilt < 0) f = raw;
    else f = ((int32_t)s_gasFilt * g_cfg.gasDamp + raw) / ((int32_t)g_cfg.gasDamp + 1);
    // Cap must sit ABOVE the highest reachable reported-mV (220R/3V3 front end
    // spans ~11.3k-23.5k reported) yet below int16 overflow if the sender
    // unplugs (open node rails toward 3V3 -> ~75k reported). 32k does both.
    // The old 16k cap silently froze every reading below ~half tank.
    s_gasFilt = (int16_t)constrain(f, 0, 32000);
}

/* Pure mapping: an mV reading -> percent, against the current calibration.
 * No ADC read, no side effect. Safe to call any number of times from anywhere.
 * Takes the mV explicitly rather than reading s_gasFilt, so it can also re-score
 * an already-recorded mV against a table that has since changed (see
 * gasLogRescore). */
static int gasPctFromMv(int16_t mv) {
    // Stuck-at-full poison guard (2026-09-20): a SET F pressed while the
    // sender was unplugged recorded the ~60k clamp into the FULL anchor, and
    // every real reading (11.3k-23.5k) is then <= c[0] -> needle pinned at
    // 100% with SET EMPTY powerless (its check comes after the FULL check).
    // Detect off-scale / hard-short / zero-span tables and map against the
    // stock span instead, so the gauge keeps moving until a clean SET F/E.
    uint16_t c[5];
    const uint16_t *src = g_cfg.gasCalMv;
    bool poisoned = (src[0] >= 30000 || src[4] >= 30000 ||   // open-sender clamp
                     src[0] < 1000  || src[4] < 1000  ||     // hard-short reading
                     src[0] == src[4]);                      // zero span
    if (poisoned) src = kGasStockMv;
    for (uint8_t i = 0; i < 5; i++) c[i] = src[i];

    if (c[0] < c[4]) {   // normal slope: low mV = full, high mV = empty
        if (mv <= c[0]) return 100;
        if (mv >= c[4]) return 0;
        for (uint8_t i = 0; i < 4; i++) {
            if (mv <= c[i + 1]) {
                int hiPct = 100 - i * 25;         // % at c[i]
                int loPct = 100 - (i + 1) * 25;   // % at c[i+1]
                if (c[i + 1] == c[i]) return hiPct;
                return loPct + (int32_t)(c[i + 1] - mv) * (hiPct - loPct) / (c[i + 1] - c[i]);
            }
        }
        return 0;
    }

    // Inverted slope: high mV = full, low mV = empty (sender reads high
    // resistance at FULL). c[0] is the FULL high-mV anchor, c[4] the EMPTY
    // low-mV anchor. Without this branch an inverted sender pins at 100%
    // forever — every paired SET F/SET E re-records the same high/low anchors
    // and the FULL-first check above never lets the needle fall.
    if (mv >= c[0]) return 100;
    if (mv <= c[4]) return 0;
    for (uint8_t i = 0; i < 4; i++) {
        uint16_t a = c[i], b = c[i + 1];         // a >= b along the slope
        if (a == b) continue;
        if (mv <= a && mv >= b) {
            int hiPct = 100 - i * 25;            // % at c[i] (FULL end)
            int loPct = 100 - (i + 1) * 25;      // % at c[i+1] (EMPTY end)
            /* Endpoints were swapped pre-2026-09-25: `a` is the FULL (high-mV)
             * anchor so filt == a must yield hiPct, not loPct — the old line
             * returned hiPct at the EMPTY end and read ~25% high per segment. */
            return loPct + (int32_t)(mv - b) * (hiPct - loPct) / (a - b);
        }
    }
    return 0;
}

static inline int gasPctFromFilt() { return gasPctFromMv(s_gasFilt); }

// Min/Max tank-float log: lowest & highest readings ever seen, in BOTH
// damped mv and the resulting %. Lives under its OWN NVS key ("gaslog") so
// adding/changing it can never resize the main "cfg" blob (whose sizeof/
// magic guard would silently wipe the calibrated gasCalMv table). Fully
// automatic — self-seeds from the first plausible reading, records extremes
// forever, never wiped. Empty reference = g_cfg.gasCalMv[4] (SET E), untouched.
static constexpr uint8_t GASLOG_MAGIC = 0x4C;
struct GasLog {
    uint8_t  magic  = GASLOG_MAGIC;
    int8_t   minPct = -1;   // -1 = unseeded
    int8_t   maxPct = -1;
    int16_t  minMv  = -1;
    int16_t  maxMv  = -1;
};
static GasLog s_gasLog;

static void gasLogSave() {
    g_prefs.putBytes("gaslog", &s_gasLog, sizeof(s_gasLog));
}

static void gasLogLoad() {
    size_t len = g_prefs.getBytes("gaslog", &s_gasLog, sizeof(s_gasLog));
    if (len != sizeof(s_gasLog) || s_gasLog.magic != GASLOG_MAGIC) {
        s_gasLog = GasLog{};   // unseeded; first plausible read owns min=max
    }
}

/* Re-derive the logged PERCENTAGES from the logged mV against the CURRENT
 * calibration.
 *
 * minMv/maxMv are calibration-independent — they are physical sender voltages.
 * minPct/maxPct are not: they are what the table in force at the time said.
 * gasAutoCal() and `Q F/E/1/2/3` move that table, which silently invalidated
 * every percentage already in the log, so recompute them from the recorded mV
 * (which are physical and table-independent).
 *
 * These percentages are DISPLAY ONLY. The auto-commit gates used to read them,
 * which meant a log recorded against a stretched table could report maxPct 95
 * that the new table would never produce. That is now handled separately: the
 * gates use s_fullMvSinceCal / s_emptyMvSinceCal, which only record observations
 * made under the CURRENT table, and are cleared whenever it changes.
 *
 * Returns true if anything changed, so the caller can decide to persist. */
static bool gasLogRescore() {
    if (s_gasLog.minPct < 0) return false;   // unseeded; nothing to re-score
    if (s_gasLog.minMv < 0 || s_gasLog.maxMv < 0) return false;
    int loPct = constrain(gasPctFromMv(s_gasLog.maxMv), 0, 100);  // low  mV
    int hiPct = constrain(gasPctFromMv(s_gasLog.minMv), 0, 100);  // high mV
    // mV and % run opposite ways on a normal sender, so don't assume an order.
    if (loPct > hiPct) { int t = loPct; loPct = hiPct; hiPct = t; }
    if (loPct == s_gasLog.minPct && hiPct == s_gasLog.maxPct) return false;
    s_gasLog.minPct = (int8_t)loPct;
    s_gasLog.maxPct = (int8_t)hiPct;
    return true;
}

static void gasLogUpdate() {
    // Both values must come from the SAME sample. Historically gasPercent() was
    // a write function — it advanced the EMA and assigned s_gasFilt as a side
    // effect — so reading `mv = s_gasFilt` BEFORE calling it yielded the previous
    // sample's mV alongside this sample's percent. On a momentary sender dropout
    // (connector bounce, ignition blip) the node railed, pct became 0, and the
    // stale-but-valid mid-tank mv sailed through the plausibility gate below.
    // minPct was then latched to 0 permanently in NVS, which also unlocked
    // gasAutoCal()'s EMPTY-anchor rewrite (gated on minPct <= 5) against a
    // mid-tank mV. 'Q R' was the only way to clear it.
    //
    // Now guaranteed rather than merely ordered: gasLogUpdate() reads the SAME
    // s_gasFilt that gasPctFromFilt() maps, and gasSampleMv() ran once at the
    // top of this loop() iteration, so the two can no longer disagree.
    int pct = gasPctFromFilt();
    int mv  = s_gasFilt;
    // Plausibility gate: a dead/disconnected sender rails to the 32000 clamp
    // (open node ~75k) and reads 0%, a hard short reads ~0 and 100% — neither
    // is a real tank level. Only log inside the sender's live window.
    if (mv < 1000 || mv >= 32000) return;
    if (pct < 0 || pct > 100) return;
    // Track the extremes observed under the CURRENT table, BEFORE the
    // first-boot seeding branch below — that branch returns early, so anything
    // written after it is skipped on exactly the sample that first populates the
    // log. Putting it here also means the seed observation counts as an
    // observation: on a first boot / after `Q R` / after `P WIPE` with the tank
    // already near full, the >=95% reading must arm the FULL gate, or the gate
    // stays disarmed until the tank returns to full a second time.
    if (pct > s_fullPctSinceCal)  { s_fullPctSinceCal  = (int8_t)pct; s_fullMvSinceCal  = (int16_t)mv; }
    if (pct < s_emptyPctSinceCal) { s_emptyPctSinceCal = (int8_t)pct; s_emptyMvSinceCal = (int16_t)mv; }
    if (s_gasLog.minPct < 0) {
        s_gasLog.minPct = s_gasLog.maxPct = (int8_t)pct;
        s_gasLog.minMv  = s_gasLog.maxMv  = (int16_t)mv;
        gasLogSave();
        return;
    }
    bool changed = false;
    if (pct < s_gasLog.minPct) { s_gasLog.minPct = (int8_t)pct; changed = true; }
    if (pct > s_gasLog.maxPct) { s_gasLog.maxPct = (int8_t)pct; changed = true; }
    if (mv  < s_gasLog.minMv)  { s_gasLog.minMv  = (int16_t)mv; changed = true; }
    if (mv  > s_gasLog.maxMv)  { s_gasLog.maxMv  = (int16_t)mv; changed = true; }
    // Track the extremes observed under the CURRENT table. These are the only
    // inputs to the auto-commit gates — deliberately NOT lifetime minPct/maxPct,
    // which gasLogRescore() recomputes from the current table and which
    // therefore describe the CURRENT mapping of a historical voltage rather than
    // a fresh observation.
    if (pct > s_fullPctSinceCal)  { s_fullPctSinceCal  = (int8_t)pct; s_fullMvSinceCal  = (int16_t)mv; }
    if (pct < s_emptyPctSinceCal) { s_emptyPctSinceCal = (int8_t)pct; s_emptyMvSinceCal = (int16_t)mv; }
    if (changed) gasLogSave();

    gasAutoCal();
}

// Auto-calibrate the E/F anchors from the float's REAL extremes recorded in
// the tank log: once the float has actually reached near-full and near-empty,
// commit those measured mV as the FULL/EMPTY anchors and re-linearise the
// mids (same math as Q F/E). No manual steps — just drive the tank to both
// ends once. Slope-aware: with a NORMAL sender (low mV = full) FULL anchors at
// the lowest logged mV; with an INVERTED sender (high mV = full) FULL anchors
// at the highest logged mV — matched to whatever the manual anchors say.
// Off-scale gate MUST match gasPercent poison + Q F/E rejection (>=30000 ||
// <1000): real senders (~11-24k reported) never live above 30k, so a
// floating/unplugged extreme can't be auto-committed as an anchor.
static void gasAutoCal() {
    // Gate on observations made under the CURRENT table, not on the lifetime
    // log extremes. minPct/maxPct are recomputed by gasLogRescore() whenever the
    // table moves, so after `Q E 18000` the log's maxPct could read 100 purely
    // because the NEW table maps an old mid-tank voltage below the new EMPTY
    // anchor — and auto-cal would then immediately overwrite the anchor the
    // operator just typed, from a tank that had not moved. Requiring a fresh
    // <=5% / >=95% observation under the current table fixes that whole class,
    // not just the manual-anchor case.
    if (s_gasLog.minPct < 0) return;              // log not seeded yet
    bool dirty = false, relin = false;
    bool inverted = g_cfg.gasCalMv[0] > g_cfg.gasCalMv[4];
    // Gate on the EXTREME pct seen, and commit that pct's own mV. Testing only
    // the mV would admit a reading that never reached 95%; testing the LAST
    // crossing instead of the highest would commit whichever sample happened to
    // come last and re-linearise the whole tank curve from a jittered endpoint.
    int32_t fMv = (s_fullPctSinceCal  >= 95) ? s_fullMvSinceCal  : -1;
    int32_t eMv = (s_emptyPctSinceCal <= 5)  ? s_emptyMvSinceCal : -1;

    /* Hysteresis on the anchors, in the direction that makes them MORE extreme.
     *
     * The band exists so a jittery sample cannot ratchet an anchor, and it must
     * stay directional - moving the FULL anchor less-full would leave the gauge
     * reading high forever.
     *
     * It used to be an unconditional 50 mV step, and that made the FULL anchor
     * mathematically un-committable out of the box. Stock kGasStockMv[0] is
     * 1009; the validity floor is fMv > 1000. So the improving direction (lower
     * mV) had 9 mV of valid headroom while the guard demanded a 50 mV step: the
     * acceptable window was the EMPTY SET. On a stock install, or after any `Q R`
     * reset, the FULL anchor could never be auto-calibrated - no matter how many
     * full-tank observations accumulated - and it failed silently, because the
     * branch simply never ran and printed nothing. EMPTY had the mirror problem
     * once its anchor approached the 30000 ceiling.
     *
     * So: keep the band where it fits, and fall back to "valid and strictly
     * better" where it does not. The >=95% / <=5% gate above has already thrown
     * away samples that were not convincingly at the end of the scale, so a
     * small step in the right direction is safe. */
    constexpr int32_t GAS_CAL_BAND_MV  = 50;
    /* 900, not 1000, and that is load-bearing.
     *
     * The first version of this fix kept the floor at 1000 and, because the
     * stock FULL anchor is 1009, the 50 mV band needed 959 - which is outside
     * the valid range, so the band was switched OFF for exactly the case the
     * fix exists for and the anchor became un-committable. The obvious repair
     * was to accept "valid and strictly better" when the band did not fit, and
     * that is what 2f086c1 shipped.
     *
     * It does ratchet. A single 1 mV dip satisfies "strictly better", so the
     * anchor walks 1009 -> 1001 in eight commits and then stops at the floor,
     * after which a genuinely full tank reads 99% instead of 100%. Observed on
     * the bench: pts[F,...] = 1001 where stock is 1009. The >=95% gate does not
     * prevent it - that gate is about the tank being full, not about the
     * reading being steady.
     *
     * So the band stays ON, and the floor moves instead. 900 leaves 959 inside
     * the range, so a jitter dip no longer commits and a real 50 mV+ improvement
     * does. The EMPTY side was never affected (band 25064 < 30000), which is
     * why the relaxation was asymmetric and asymmetric in the direction that
     * degrades the reading. With this floor both directions enforce the band.
     *
     * 900 is also consistent with the manual `Q F`/`Q E` rejection, which
     * refuses raw < 1000 as an off-scale short. */
    constexpr int32_t GAS_CAL_VALID_LO = 900;
    constexpr int32_t GAS_CAL_VALID_HI = 30000;

    int32_t fOld = g_cfg.gasCalMv[0];
    if (fMv > GAS_CAL_VALID_LO && fMv < GAS_CAL_VALID_HI && fMv != fOld) {
        int32_t band  = inverted ? fOld + GAS_CAL_BAND_MV : fOld - GAS_CAL_BAND_MV;
        bool better   = inverted ? (fMv > fOld) : (fMv < fOld);
        bool clears   = inverted ? (fMv >= band) : (fMv <= band);
        /* The band must fit inside the valid range, or the FULL anchor silently
         * becomes un-committable again - which is the original bug. Asserted at
         * build time rather than tested at runtime, so raising the floor is a
         * compile failure and not a field report. kGasStockMv[0] is 1009. */
        static_assert(GAS_CAL_VALID_LO < (int32_t)kGasStockMv[0] - GAS_CAL_BAND_MV,
                      "GAS_CAL_VALID_LO leaves no room for the FULL anchor's band; "
                      "the anchor would become un-committable (the 2f086c1 bug)");
        if (better && clears) {
            g_cfg.gasCalMv[0] = (uint16_t)fMv;                       // FULL anchor
            dirty = relin = true;
            Serial.printf("gas auto-cal: F set = %u mv\n", g_cfg.gasCalMv[0]);
        }
    }
    int32_t eOld = g_cfg.gasCalMv[4];
    if (eMv > GAS_CAL_VALID_LO && eMv < GAS_CAL_VALID_HI && eMv != eOld) {
        int32_t band  = inverted ? eOld - GAS_CAL_BAND_MV : eOld + GAS_CAL_BAND_MV;
        bool better   = inverted ? (eMv < eOld) : (eMv > eOld);
        bool clears   = inverted ? (eMv <= band) : (eMv >= band);
        if (better && clears) {
            g_cfg.gasCalMv[4] = (uint16_t)eMv;                       // EMPTY anchor
            dirty = relin = true;
            Serial.printf("gas auto-cal: E set = %u mv\n", g_cfg.gasCalMv[4]);
        }
    }
    if (relin && g_cfg.gasCalMv[0] < g_cfg.gasCalMv[4]) {
        for (uint8_t j = 1; j < 4; j++)
            g_cfg.gasCalMv[j] = g_cfg.gasCalMv[0] +
                (uint16_t)((uint32_t)(g_cfg.gasCalMv[4] - g_cfg.gasCalMv[0]) * j / 4);
        Serial.println("gas auto-cal: mids re-linearised");
    } else if (relin && g_cfg.gasCalMv[0] > g_cfg.gasCalMv[4]) {
        // Inverted slope: mids descend from the FULL (high-mV) anchor.
        for (uint8_t j = 1; j < 4; j++)
            g_cfg.gasCalMv[j] = g_cfg.gasCalMv[4] +
                (uint16_t)((uint32_t)(g_cfg.gasCalMv[0] - g_cfg.gasCalMv[4]) * j / 4);
        Serial.println("gas auto-cal: mids re-linearised (inverted)");
    }
    if (dirty) {
        s_gasFilt = -1;                          // reseed filter after cal change
        // The gates are "observations under the current table", and the table
        // just moved, so whatever was seen under the old one no longer counts.
        s_fullPctSinceCal = -1;  s_fullMvSinceCal = -1;
        s_emptyPctSinceCal = 101; s_emptyMvSinceCal = -1;
        // The percentages in the log are now displayed against a calibration
        // that no longer exists; re-derive them from the recorded mV so the band
        // matches the needle. (They are no longer used as gates.)
        if (gasLogRescore()) gasLogSave();
        saveCfg();
    }
}

// Needle gauge: center pivot (120,128), 240 deg sweep (150..390) across the top,
// leaving a bottom gap for the % / mv text. Ticks are radial lines at 6 deg steps.
static constexpr int   GAS_CX = 120;
static constexpr int   GAS_CY = 128;
static constexpr float GAS_A0  = 150.0f;   // level 0 angle (down-left)
static constexpr float GAS_A1  = 390.0f;   // level 100 angle (down-right)
static constexpr float GAS_RIN  = 92.0f;   // tick inner radius
static constexpr float GAS_ROUT = 108.0f;  // tick outer radius
static constexpr float GAS_NEEDLE_R = 118.0f;
static constexpr float GAS_NEEDLE_W = 6.0f; // needle base half-width at pivot

static float gasAngleDeg(int pct) {
    return GAS_A0 + (GAS_A1 - GAS_A0) * pct / 100.0f;
}

static int gasRound6(float deg) {
    return (int)(deg / 6.0f + 0.5f) * 6;
}

static void drawGasTick(float deg, uint16_t color) {
    float r = deg * 3.14159265f / 180.0f;
    s_gasTft->drawLine(GAS_CX + (int16_t)(GAS_RIN * cosf(r)),
                       GAS_CY + (int16_t)(GAS_RIN * sinf(r)),
                       GAS_CX + (int16_t)(GAS_ROUT * cosf(r)),
                       GAS_CY + (int16_t)(GAS_ROUT * sinf(r)), color);
}

static void drawGasNeedle(float deg, uint16_t color) {
    float a = deg * 3.14159265f / 180.0f;
    float c = cosf(a), s = sinf(a);
    float px = s, py = -c;                     // perpendicular to needle direction
    int16_t bx0 = GAS_CX + (int16_t)(GAS_NEEDLE_W * px);
    int16_t by0 = GAS_CY + (int16_t)(GAS_NEEDLE_W * py);
    int16_t bx1 = GAS_CX - (int16_t)(GAS_NEEDLE_W * px);
    int16_t by1 = GAS_CY - (int16_t)(GAS_NEEDLE_W * py);
    int16_t tx = GAS_CX + (int16_t)(GAS_NEEDLE_R * c);
    int16_t ty = GAS_CY + (int16_t)(GAS_NEEDLE_R * s);
    s_gasTft->fillTriangle(bx0, by0, bx1, by1, tx, ty, color);
}

static char s_lastGasMv[12] = "";
static char s_lastGasPct[8] = "";
static uint16_t s_lastPctColor = 0xFFFF;
// Last-drawn float-history band (LO%/HI% of the tank log), -2 = nothing drawn.
// File scope, not a local of updateGasDisplay(), so drawGasFrame() can clear it:
// the tracker is delta-based and must be invalidated whenever the panel is
// filled, or the band is gone until the lifetime min/max next moves.
static int16_t s_bandMin = -2, s_bandMax = -2;
static char s_lastGasLow[10] = "";
static uint16_t s_lastLowColor = 0xFFFF;
static char s_lastGasLo[8] = "";
static char s_lastGasHi[8] = "";
static char s_lastGasLoMv[8] = "";
static char s_lastGasHiMv[8] = "";
static bool  s_needleDrawn = false;
static float s_needleDeg = 0.0f;
static int   s_gasDisp = -1;
static bool  s_lowWas = false;
static bool  s_blinkPhase = false;

static void drawGasValue(int16_t x, int16_t y, uint8_t size, uint16_t color,
                         uint16_t clearW, char* last, const char* s) {
    if (strcmp(last, s) == 0) return;
    strcpy(last, s);
    s_gasTft->fillRect(x, y - 2, clearW, size * 8 + 4, GC9A01A_BLACK);
    s_gasTft->setCursor(x, y);
    s_gasTft->setTextSize(size);
    s_gasTft->setTextColor(color);
    s_gasTft->print(s);
}

static void drawGasText(int16_t x, int16_t y, uint8_t size, uint16_t clearW,
                        uint16_t color, char* last, uint16_t* lastColor, const char* s) {
    if (strcmp(last, s) == 0 && (lastColor == nullptr || *lastColor == color)) return;
    strcpy(last, s);
    if (lastColor != nullptr) *lastColor = color;
    s_gasTft->fillRect(x, y - 2, clearW, size * 8 + 4, GC9A01A_BLACK);
    s_gasTft->setCursor(x, y);
    s_gasTft->setTextSize(size);
    s_gasTft->setTextColor(color);
    s_gasTft->print(s);
}

static void drawGasMajorTick(float deg) {
    float r = deg * 3.14159265f / 180.0f;
    s_gasTft->drawLine(GAS_CX + (int16_t)(86.0f * cosf(r)),
                       GAS_CY + (int16_t)(86.0f * sinf(r)),
                       GAS_CX + (int16_t)(110.0f * cosf(r)),
                       GAS_CY + (int16_t)(110.0f * sinf(r)), GC9A01A_WHITE);
}

static void drawGasMarks() {
    const int marks[3] = {25, 50, 75};
    const char* labels[3] = {"1/4", "1/2", "3/4"};
    for (int i = 0; i < 3; i++) {
        float a = gasAngleDeg(marks[i]) * 3.14159265f / 180.0f;
        const char* s = labels[i];
        int w = (int)strlen(s) * 6;
        s_gasTft->setTextSize(1);
        s_gasTft->setTextColor(GC9A01A_DARKGREY);
        s_gasTft->setCursor(GAS_CX + (int16_t)(70.0f * cosf(a)) - w / 2,
                            GAS_CY + (int16_t)(70.0f * sinf(a)) - 4);
        s_gasTft->print(s);
    }
}

static void drawGasFrame() {
    s_gasTft->fillScreen(GC9A01A_BLACK);
    s_gasTft->drawCircle(GAS_CX, GAS_CY, 119, GC9A01A_NAVY);
    s_gasTft->drawCircle(GAS_CX, GAS_CY, 118, GC9A01A_DARKGREY);
    for (int a = (int)GAS_A0; a <= (int)GAS_A1; a += 6) drawGasTick((float)a, GC9A01A_DARKGREY);
    for (int p = 0; p <= 100; p += 25) drawGasMajorTick((float)gasAngleDeg(p));
    drawGasMarks();
    s_gasTft->setTextColor(GC9A01A_LIGHTGREY);
    s_gasTft->setTextSize(1);
    s_gasTft->setCursor(10, 200); s_gasTft->print("E");
    s_gasTft->setCursor(218, 200); s_gasTft->print("F");
    s_gasTft->setTextColor(GC9A01A_CYAN);
    s_gasTft->setTextSize(2);
    s_gasTft->setCursor(102, 174);
    s_gasTft->print("GAS");
    s_gasTft->fillCircle(GAS_CX, GAS_CY, 4, GC9A01A_RED);
    s_needleDrawn = false;
    s_gasDisp = -1;
    s_lowWas = false;
    s_blinkPhase = false;
    s_lastGasMv[0] = s_lastGasPct[0] = s_lastGasLow[0] = s_lastGasLo[0] = s_lastGasHi[0] = 0;
    s_lastGasLoMv[0] = s_lastGasHiMv[0] = 0;
    s_lastPctColor = s_lastLowColor = 0xFFFF;
    // The float-history band is static-delta-tracked, so it MUST be invalidated
    // here or it is lost for the life of the frame. It used to be a
    // function-local static inside updateGasDisplay(): that survived the
    // fillScreen() above, so after ANY re-init drawGasFrame() wiped the yellow
    // band off the panel while the delta tracker still believed it was drawn —
    // the band then never came back until the float's lifetime min or max
    // happened to change. Worse, the "erase the previous band" loop would then
    // scribble dark ticks over a region that had just been filled, for a band
    // that no longer existed.
    s_bandMin = s_bandMax = -2;
}

static void updateGasDisplay() {
    if (!g_cfg.tftEnable || s_gasTft == nullptr) return;
    char buf[12];

    int target = gasPctFromFilt();
    bool low = target <= g_cfg.lowFuelPct;
    bool phase = ((millis() / 500) & 1) == 0;

    if (s_gasDisp < 0) s_gasDisp = target;
    else if (s_gasDisp < target) s_gasDisp = s_gasDisp + 2 > target ? target : s_gasDisp + 2;
    else if (s_gasDisp > target) s_gasDisp = s_gasDisp - 2 < target ? target : s_gasDisp - 2;

    int est = (int)((long)s_gasDisp * g_cfg.tankGalX10 * g_cfg.gasMpg / 1000);
    snprintf(buf, sizeof buf, "~%d mi", est);
    drawGasValue(99, 230, 1, GC9A01A_CYAN, 44, s_lastGasMv, buf);

    float deg = gasAngleDeg(s_gasDisp);
    bool blinkChange = low && phase != s_blinkPhase;
    bool lowChange = low != s_lowWas;
    bool moved = !s_needleDrawn || deg != s_needleDeg;

    if (s_needleDrawn && (moved || blinkChange || lowChange)) {
        drawGasNeedle(s_needleDeg, GC9A01A_BLACK);          // erase old needle
        drawGasTick((float)gasRound6(s_needleDeg), GC9A01A_DARKGREY);  // patch the scale
        for (int p = 0; p <= 100; p += 25) drawGasMajorTick((float)gasAngleDeg(p));
        drawGasMarks();
    }
    if (blinkChange || lowChange) {
        uint16_t alarmColor = phase ? GC9A01A_RED : GC9A01A_DARKGREY;
        s_gasTft->drawCircle(GAS_CX, GAS_CY, 118, low ? alarmColor : GC9A01A_DARKGREY);
        s_gasTft->drawCircle(GAS_CX, GAS_CY, 119, low ? alarmColor : GC9A01A_NAVY);
        s_gasTft->setTextSize(1);
        s_gasTft->setTextColor(low ? alarmColor : GC9A01A_LIGHTGREY);
        s_gasTft->setCursor(10, 200); s_gasTft->print("E");
    }
    if (low) {
        uint16_t lowColor = phase ? GC9A01A_RED : GC9A01A_DARKGREY;
        drawGasText(93, 196, 1, 54, lowColor, s_lastGasLow, &s_lastLowColor, "LOW FUEL");
    } else {
        drawGasText(93, 196, 1, 54, GC9A01A_BLACK, s_lastGasLow, &s_lastLowColor, "");
    }
    if (moved || blinkChange || lowChange) {
        uint16_t needleColor = low ? (phase ? GC9A01A_RED : GC9A01A_DARKGREY) : GC9A01A_CYAN;
        drawGasNeedle(deg, needleColor);
    }
    s_gasTft->fillCircle(GAS_CX, GAS_CY, 4, GC9A01A_RED);
    s_needleDrawn = true;
    s_needleDeg = deg;
    s_lowWas = low;
    s_blinkPhase = phase;

    snprintf(buf, sizeof buf, "%d%%", s_gasDisp);
    uint16_t pctColor = low ? (phase ? GC9A01A_RED : GC9A01A_DARKGREY) : GC9A01A_CYAN;
    drawGasText(88, 208, 2, 68, pctColor, s_lastGasPct, &s_lastPctColor, buf);

    // Min/Max tank-float log: a yellow band on the tick ring tracing the
    // sweep the float has covered (from LO% up to HI%). The needle already
    // shows the live level, so the band reads instantly with no text.
    int16_t bMin = s_gasLog.minPct, bMax = s_gasLog.maxPct;
    if (bMin != s_bandMin || bMax != s_bandMax) {
        if (s_bandMin >= 0) {            // erase previous band
            for (int a = s_bandMin; a <= s_bandMax; a += 2)
                drawGasTick(gasAngleDeg(a), GC9A01A_DARKGREY);
        }
        if (bMin >= 0) {                 // draw new band
            for (int a = bMin; a <= bMax; a += 2)
                drawGasTick(gasAngleDeg(a), GC9A01A_YELLOW);
        }
        s_bandMin = bMin;
        s_bandMax = bMax;
    }
}

static void initGasTft() {
    if (!g_cfg.tftEnable) return;
    if (s_gasTft == nullptr) {
        // PINS SWAPPED: gas renderer now drives the screen on the idle-CS/DC
        // (CS17/DC18). Idle renderer took over GAS_CS2/DC21 in initTft().
        s_gasTft = new Adafruit_GC9A01A(g_cfg.pin.tftCs, g_cfg.pin.tftDc, GAS_MOSI, GAS_SCLK, -1);
    }
    s_gasTft->begin();
    s_gasTft->setRotation(1);
    /* No fillScreen(): drawGasFrame() begins with one. See initTft(). */
    drawGasFrame();
}

static void teardownTft() {
    if (s_tft) { delete s_tft; s_tft = nullptr; }
    if (s_gasTft) { delete s_gasTft; s_gasTft = nullptr; }
}

static void initTft() {
    if (g_cfg.tftEnable) {
        if (s_tft == nullptr) {
            // PINS SWAPPED: idle renderer now drives the screen on the gas-CS/DC
            // (CS2/DC21). Gas renderer took over the idle CS17/DC18 in initGasTft().
            s_tft = new Adafruit_GC9A01A(GAS_CS, GAS_DC,
                                         g_cfg.pin.tftMosi, g_cfg.pin.tftSclk, -1);
        }
        s_tft->begin();
        s_tft->setRotation(1);   // content upright when screen mounted pins-right
        /* No fillScreen() here: drawGaugeFrame() starts with one. Two fills back
         * to back over SOFTWARE SPI cost ~180 ms each (Adafruit_SPITFT has no
         * fast path unless connection == TFT_HARD_SPI, so every pixel is 16
         * iterations x 3 digitalWrite() calls). Both displays use -1 for reset,
         * so begin() is ~300 ms each on its own. Dropping one redundant fill
         * saved ~180 ms of a 1.3 s stall. */
        drawGaugeFrame();
        s_lastDuty[0] = s_lastRpm[0] = s_lastTgt[0] = s_lastClt[0] = 0;
        s_lastMode[0] = s_lastStat[0] = s_lastWarn[0] = 0;
    }
}

/* ---------------------------------------------------------------------------
 * Deferred, STEPPED TFT (re)initialisation.
 *
 * Measured cost of a full re-init: begin() is ~300 ms per display (both use
 * RST=-1, so there is no hardware reset to lean on) and the first fillScreen()
 * is ~180 ms (Adafruit_SPITFT has no fast path unless built TFT_HARD_SPI, so
 * every pixel is 16 iterations x 3 digitalWrite()). Two displays ~= 1 s of solid
 * blocking, in ONE uninterrupted stretch.
 *
 * That used to run inside handleCommand(), on the same loop() pass as
 * updateOutputs(). Two consequences, both real:
 *
 *  1. The fan and IAC were frozen for that whole second. Pin changes are a
 *     console operation, but the box is live while you type them.
 *  2. Worse, s_canFresh is only evaluated at the TOP of loop(). During the
 *     stall it is never recomputed, so the 500 ms failsafe could not trip at
 *     all — the actuators sat at their last-written values with the link
 *     already dead underneath them, and no amount of lost CAN would change it.
 *
 * So a re-init is now a small state machine advanced ONE step per loop(),
 * from a point after updateOutputs() has run. The most expensive single step is
 * begin() at ~300 ms, comfortably inside the 500 ms failsafe window, so cooling
 * and the link check are re-evaluated between every step.
 *
 * setup() still calls initTft()/initGasTft() directly: nothing is at risk
 * before the link is up, and a screen that is ready when the boot banner
 * prints is worth more than a few ms. */
enum TftStep : uint8_t {
    TFT_IDLE = 0,
    TFT_STEP_TEARDOWN,
    TFT_STEP_IDLE_ALLOC,
    TFT_STEP_IDLE_BEGIN,
    TFT_STEP_IDLE_FRAME,
    TFT_STEP_GAS_ALLOC,
    TFT_STEP_GAS_BEGIN,
    TFT_STEP_GAS_FRAME,
};
static TftStep s_tftStep = TFT_IDLE;
static bool    s_tftReinitAgain = false;

/* Ask for a re-init. Returns immediately; the work happens in loop().
 *
 * Requests that arrive while a sequence is in flight are LATCHED, not dropped.
 * The original version coalesced by simply ignoring them, which was wrong: the
 * steps read g_cfg.pin.tft* LAZILY, at their own alloc step, not here. So a
 * `P TFTC 13` landing at or after TFT_STEP_GAS_ALLOC changed the pin in NVS,
 * printed "pin map updated", and was then silently discarded — while the gas
 * display went on driving the OLD CS pin. Nothing re-queued it, the sequence ran
 * to TFT_IDLE, and the state looked settled.
 *
 * Reachable over the 0xC0 path: the queue is 6 deep and now drains one command
 * per pass, so a burst of pin commands is consumed across ~1.8 s of sequence and
 * any of them landing past its own alloc step was lost. The 0xD0 ack still said
 * the command succeeded.
 *
 * Latching instead re-runs the whole sequence from teardown on completion, so
 * the final state always matches the config that is actually stored. */
static void tftReinitRequest() {
    if (s_tftStep == TFT_IDLE) s_tftStep = TFT_STEP_TEARDOWN;
    else                        s_tftReinitAgain = true;
}

static void tftIdleAlloc() {
    if (s_tft == nullptr) {
        // PINS SWAPPED: idle renderer drives the gas-CS/DC (CS2/DC21); the gas
        // renderer takes the idle CS17/DC18.
        s_tft = new Adafruit_GC9A01A(GAS_CS, GAS_DC,
                                     g_cfg.pin.tftMosi, g_cfg.pin.tftSclk, -1);
    }
}
static void tftGasAlloc() {
    if (s_gasTft == nullptr) {
        s_gasTft = new Adafruit_GC9A01A(g_cfg.pin.tftCs, g_cfg.pin.tftDc,
                                        GAS_MOSI, GAS_SCLK, -1);
    }
}

/* One step per call. Deliberately NOT called from handleCommand(). */
/* Terminal transition for the re-init state machine. Re-arms from teardown if a
 * request landed while the sequence was in flight, so the final state always
 * matches the config that is actually stored. All paths that end the sequence
 * must go through here rather than assigning TFT_IDLE directly. */
static void tftReinitDone() {
    if (s_tftReinitAgain) {
        s_tftReinitAgain = false;
        s_tftStep = TFT_STEP_TEARDOWN;
        Serial.println("tft re-init: restart (config changed mid-sequence)");
    } else {
        s_tftStep = TFT_IDLE;
    }
}

static void tftReinitStep() {
    /* Re-check tftEnable on EVERY step, not just at teardown. The steps run one
     * per loop() pass, so `P TFT 0` typed during the gas half used to leave the
     * remaining steps initialising a display that had just been switched off —
     * initTft()/initGasTft() both guard on tftEnable, so the state machine has
     * to as well or it quietly diverges from them. */
    if (s_tftStep != TFT_IDLE && s_tftStep != TFT_STEP_TEARDOWN && !g_cfg.tftEnable) {
        teardownTft();
        tftReinitDone();
        Serial.println("tft re-init cancelled (tft disabled mid-sequence)");
        return;
    }
    switch (s_tftStep) {
        case TFT_IDLE: return;                       // nothing pending
        case TFT_STEP_TEARDOWN:
            teardownTft();
            if (g_cfg.tftEnable) s_tftStep = TFT_STEP_IDLE_ALLOC;
            // Must go through tftReinitDone() even on the disabled path: a request
            // latched during the teardown step itself (e.g. `P TFT 0` arriving on
            // the pass after `P TFTC 13`) is still set here, and assigning
            // TFT_IDLE directly stranded it. The next unrelated re-init would then
            // run the full ~1 s sequence twice and print "config changed
            // mid-sequence" when nothing had. Self-healing, but wrong.
            else                 tftReinitDone();
            return;
        case TFT_STEP_IDLE_ALLOC:                    // new(): ~0 ms, no SPI
            tftIdleAlloc();
            s_tftStep = TFT_STEP_IDLE_BEGIN;
            return;
        case TFT_STEP_IDLE_BEGIN:                    // ~300 ms — the worst step
            if (!s_tft) { tftReinitDone(); Serial.println("tft re-init aborted: idle display alloc failed"); return; }
            s_tft->begin();
            s_tft->setRotation(1);
            s_tftStep = TFT_STEP_IDLE_FRAME;
            return;
        case TFT_STEP_IDLE_FRAME:                    // ~180 ms
            drawGaugeFrame();
            s_lastDuty[0] = s_lastRpm[0] = s_lastTgt[0] = s_lastClt[0] = 0;
            s_lastMode[0] = s_lastStat[0] = s_lastWarn[0] = 0;
            s_tftStep = TFT_STEP_GAS_ALLOC;
            return;
        case TFT_STEP_GAS_ALLOC:
            tftGasAlloc();
            s_tftStep = TFT_STEP_GAS_BEGIN;
            return;
        case TFT_STEP_GAS_BEGIN:                     // ~300 ms
            if (!s_gasTft) { tftReinitDone(); Serial.println("tft re-init aborted: gas display alloc failed"); return; }
            s_gasTft->begin();
            s_gasTft->setRotation(1);
            s_tftStep = TFT_STEP_GAS_FRAME;
            return;
        case TFT_STEP_GAS_FRAME:                     // ~180 ms
            drawGasFrame();
            tftReinitDone();
            if (s_tftStep == TFT_IDLE) Serial.println("tft re-init complete");
            return;
    }
}

static void saveCfg() {
    g_prefs.putBytes("cfg", &g_cfg, sizeof(g_cfg));
}

static void loadCfg() {
    // Distinguish the three ways this can fail. They used to collapse into one
    // silent `g_cfg = Cfg{}`, which made a corrupt or truncated NVS blob
    // indistinguishable from a brand-new box: same defaults, same silence, and
    // a calibrated gas table, a pin map and the engine profile simply gone with
    // nothing on the console to say so.
    bool present = s_nvsOk && g_prefs.isKey("cfg");
    size_t len   = s_nvsOk ? g_prefs.getBytes("cfg", &g_cfg, sizeof(g_cfg)) : 0;

    if (len != sizeof(g_cfg) || g_cfg.magic != CFG_MAGIC) {
        bool magicBad = (len == sizeof(g_cfg) && g_cfg.magic != CFG_MAGIC);
        g_cfg = Cfg{};
        saveCfg();
        if (!s_nvsOk) {
            // Must be checked FIRST. Without this the NVS-open failure reported
            // "no stored config — first boot, defaults written", which is the one
            // message that tells the user their settings are safe. It was a lie:
            // NVS never opened, nothing was read and nothing can be written, so
            // every setting from the last session is already gone and will not
            // come back on the next reboot. Two of my own diagnostics then
            // contradicted each other on the console at the same boot.
            Serial.println("cfg: NOT LOADED — NVS is not open, so this is not a first boot.");
            Serial.println("     Stored settings are unavailable this session and will be lost.");
        } else if (!present) {
            Serial.printf("cfg: no stored config — first boot, defaults written (magic %04x)\n", CFG_MAGIC);
        } else if (magicBad) {
            Serial.printf("cfg: STALE CONFIG — stored magic is not %04x, defaults written.\n"
                          "      This is normal after a deliberate magic bump. If you did not expect it,\n"
                          "      the pin map / gas table / engine profile were reset.\n", CFG_MAGIC);
        } else {
            Serial.printf("cfg: CORRUPT — read %u of %u bytes, defaults written and STORED OVER the bad blob.\n"
                          "      The stored config has been overwritten; pin map, gas table and engine\n"
                          "      profile are back to defaults. This is NOT a normal first boot.\n",
                          (unsigned)len, (unsigned)sizeof(g_cfg));
        }
    }
    // Fan temps are now FIXED (command removed) — always authoritative.
    g_cfg.fanOnTemp = 1800;
    g_cfg.fanOffTemp = 1700;
}

static void reportStatus() {
    Serial.printf("link=espnow rx_a0=%lu rx_c0=%lu tx_b0=%lu dropped=%lu can_fresh=%d rpm=%u map=%.1f mat=%.1fF clt=%.1fF tps=%.1f%% batt=%.1fV afr=%.1f buz=%d\n",
                  (unsigned long)s_rxA0Count, (unsigned long)s_rxC0Count, (unsigned long)s_txB0Count,
                  (unsigned long)s_rxDroppedCount,
                  s_canFresh ? 1 : 0, g_rpm,
                  g_map / 10.0f, g_mat / 10.0f, g_clt / 10.0f,
                  g_tps / 10.0f, g_batt / 10.0f, g_afr / 10.0f,
                  g_cfg.buzzerEnable ? 1 : 0);
    Serial.printf("gas=%d%% mv=%d est=%dmi damp=%u warn<=%u%% pts[F,3/4,1/2,1/4,E]=%u,%u,%u,%u,%u\n",
                  gasPctFromFilt(), s_gasFilt,
                  (int)((long)gasPctFromFilt() * g_cfg.tankGalX10 * g_cfg.gasMpg / 1000),
                  g_cfg.gasDamp, g_cfg.lowFuelPct,
                  g_cfg.gasCalMv[0], g_cfg.gasCalMv[1], g_cfg.gasCalMv[2],
                  g_cfg.gasCalMv[3], g_cfg.gasCalMv[4]);
    Serial.printf("fan=%d fanon=%.1fF fanoff=%.1fF fanout=%d", g_cfg.fanOut >= 1 && g_cfg.fanOut <= 7 && digitalRead(g_cfg.pin.out[g_cfg.fanOut - 1]) ? 1 : 0,
                  g_cfg.fanOnTemp / 10.0f, g_cfg.fanOffTemp / 10.0f, g_cfg.fanOut);
    Serial.printf(" iac=%s duty=%d", g_cfg.iacFollow ? "follow" : (g_cfg.iacAuto ? "auto" : "manual"),
                  (uint8_t)(ledcRead(0) * 100 / 1023));
    if (!g_cfg.iacFollow && !g_cfg.iacAuto) Serial.printf(" man=%d", g_cfg.iacManualDuty);
    Serial.printf(" tgt=%d", g_cfg.iacTargetRpm);
    if (g_cfg.iacFollow) Serial.printf(" iacstep=%d", g_iacStep);
    for (uint8_t i = 0; i < 7; i++) Serial.printf(" o%u=%d", i + 1, digitalRead(g_cfg.pin.out[i]) ? 1 : 0);
    Serial.println();
    for (uint8_t i = 0; i < 4; i++) {
        Serial.printf("%s a%u=%.2fV%c(%s)%s%s", s_anForce[i] >= 0 ? "*" : "", i + 1,
                      readAnalogMv(i) / 1000.0f,
                      s_anLow[i] ? 'L' : 'H', tgtName(g_cfg.anOut[i]),
                      g_cfg.anEnable[i] ? "" : " dis",
                      s_anLatch[i] ? " LAT" : "");
    }
    Serial.printf("\nlat=%d%d%d%d\n",
                  s_anLatch[0] ? 1 : 0, s_anLatch[1] ? 1 : 0,
                  s_anLatch[2] ? 1 : 0, s_anLatch[3] ? 1 : 0);
    if (g_cfg.eng.enabled) {
        const char* w = topWarnName(s_warnLatched);
        Serial.printf("eng=on idle[%d-%d] maxrpm=%d clt<=%dF mat<=%dF batt[%d-%d]V map<=%dkPa afr lean>%d rich<%d warnout=%u hold=%ums warn=%s\n",
                      g_cfg.eng.idleRpmMin, g_cfg.eng.idleRpmMax, g_cfg.eng.maxRpm,
                      g_cfg.eng.cltMax / 10, g_cfg.eng.matMax / 10,
                      g_cfg.eng.battMin / 10, g_cfg.eng.battMax / 10,
                      g_cfg.eng.mapMax / 10, g_cfg.eng.afrHigh / 10, g_cfg.eng.afrLow / 10,
                      g_cfg.eng.warnOut, g_cfg.eng.warnHoldMs,
                      s_warnLatched ? w : "none");
    } else {
        Serial.printf("eng=off\n");
    }
}

/* True while a command came in over the unauthenticated ESP-NOW 0xC0 channel
 * rather than the local UART console. Set only around the queue drain in
 * loop(), so the flag cannot leak into a subsequent Serial command. */
static bool s_cmdFromLink = false;

/* ALLOW-LIST of what may arrive over the unauthenticated ESP-NOW 0xC0 channel.
 * Everything else is UART-console only.
 *
 * This was a deny-list twice and was wrong twice, which is the argument for an
 * allow-list: a deny-list can only be as complete as the author's memory of
 * every command in the file, and two review passes each found one it had missed.
 *  - pass 1 caught `P WIPE` (a remote factory reset, worse than the peer-filter
 *    attack the list was written to close).
 *  - pass 2 caught `Y 0`. `Y <n>` picks which output the fan runs on, and
 *    `Y 0` is one frame that removes the fan from the board permanently: setFan()
 *    is `if (fanOut >= 1 && fanOut <= 7) digitalWrite(...)`, so fanOut == 0
 *    writes NOTHING — and the emergency overheat override upstream,
 *    `if (clt >= fanOnTemp) fanOn = true`, still sets fanOn and still calls
 *    setFan(), so every downstream safety path believes cooling is being driven
 *    while no pin is touched. Silent, persistent, survives reboot. Also missed:
 *    `O<n>` (output mode, can latch any of the 7 relays on), `A<n>` (an analog
 *    input can be mapped onto any output, another way to drive the fan relay),
 *    `W` (the whole engine-warning profile, including `W 0` to switch the
 *    OVERHEAT warning off), and `B L` (continuous horn, which the dash never
 *    sends — it only sends `B 0`, `B 1` and `B T`).
 *
 * peerAllowed() accepts ANY peer when nothing is bound, which is the default,
 * so all of the above were reachable by one broadcast frame from anything in
 * radio range, and the 0xD0 ack still said "ok".
 *
 * The allowed set is exactly what can_tx.c emits — F, I, T, Q, B, L — and
 * nothing else in the workspace transmits 0xC0 at all.
 *
 * RESIDUAL, not fixed here: `F 0` (fan off), `I 0` (manual 0% IAC), `B 0`
 * (buzzer mute) and `Q W 90` (low-fuel warning at 90%) are legitimate DASH
 * buttons, so they stay reachable, and each one remotely disables a function the
 * driver relies on. Closing those needs a real fix — an encrypted,
 * authenticated ESP-NOW link (D4 / io-H2, still open) — not a longer list. Do
 * not read this function as making the link safe. */
static bool cmdAllowedFromLink(char key, const String& val) {
    switch (key) {
        // F A|1|0   fan auto / manual on / manual off
        // I F|A|<d> IAC follow MS / closed loop / manual duty
        // T <rpm>   closed-loop idle target
        case 'F':
        case 'I':
        case 'T':
            return true;
        // Q is the only mixed one: the dash sends Q D/W/M/T (damping, low-fuel %,
        // mpg, tank size) and Q alone dumps state. Everything else on this key
        // writes the calibration table — Q R wipes it, Q F/E/1/2/3 records the
        // current A4 reading as an anchor — and must not be reachable from the
        // air. Prefix match on "<letter><space>" rather than an exact-value deny
        // list, so a new Q subcommand added later is refused by default.
        case 'Q':
            return val.length() == 0 ||
                   val.startsWith("D ") || val.startsWith("W ") ||
                   val.startsWith("M ") || val.startsWith("T ");
        // B 0|1|T    buzzer off / on / one test beep — all dash buttons.
        // B L (continuous horn until stopped) is not; the dash has no such
        // button, so refusing it costs nothing.
        case 'B':
            return !val.startsWith("L");
        // L 0        LED bar off
        // L <r> <g> <b>  LED bar colour
        case 'L':
            return true;
        // Everything below is UART-console only:
        //   P *        remap pins, move the fan relay, turn the TFTs off, reset
        //              the pin map, bind a peer filter (which can lock the real
        //              dash out and stop cooling), full factory reset
        //   X 0..3     take the buzzer pin under manual control; X 3 is a
        //              continuous horn with no automatic exit
        //   Y 0..7     which output the fan runs on. Y 0 = NO FAN AT ALL, see above
        //   O 0..6     per-output mode; can latch any of the 7 relays on
        //   A 1..4     analog input enable/threshold/polarity/output-map. Can
        //              drive an OUTPUT from an input, including the fan relay
        //   W          the whole engine-warning profile, incl. W 0 = overheat
        //              warning off
        //   S <rpm>    shift-light output
        //   R, M       informational, but also not needed remotely
        default:
            return false;
    }
}

static void handleCommand(const String& line) {
    String c = line;
    c.trim();
    if (c.length() == 0) return;
    if (c == "?") { reportStatus(); return; }

    char key = c[0];
    String val = c.substring(1);
    val.trim();

    if (s_cmdFromLink && !cmdAllowedFromLink(key, val)) {
        Serial.printf("REFUSED '%s' over ESP-NOW — UART-console only. It rewires actuators,\n"
                      "         silences a warning, or wipes stored settings. The link is not\n"
                      "         authenticated, so this is refused by default rather than trusted.\n",
                      c.c_str());
        return;
    }

    switch (key) {
        case 'F': {
            /* saveCfg() is a ~180-byte g_prefs.putBytes("cfg", ...) - a real
             * flash write. It used to run unconditionally after the if/else,
             * so `F` with a bad value printed a rejection and then still
             * committed the whole config. Repeated by anyone who can reach the
             * console, that is unbounded flash-write amplification for a
             * command that changed nothing.
             *
             * So gate the write on the config genuinely having moved, not on
             * the branch having been taken: compare every field this arm can
             * touch against its previous value, and commit only if one of them
             * actually changed. `F A` deliberately leaves fanManual alone (A is
             * "auto", the manual latch is only meaningful once fanAuto is
             * false), so a redundant `F A` while already in AUTO is a no-op and
             * must not write either. A rejected value must change nothing and
             * must not write - so it breaks out before reaching saveCfg(). */
            bool a0 = g_cfg.fanAuto, m0 = g_cfg.fanManual;
            if (val == "A") { g_cfg.fanAuto = true; }
            else if (val == "1" || val == "0") {
                g_cfg.fanAuto = false;
                g_cfg.fanManual = (val == "1");
            } else { Serial.println("fan A|1|0 only (temps fixed)"); break; }
            if (g_cfg.fanAuto != a0 || g_cfg.fanManual != m0) saveCfg();
            break;
        }
        case 'I': {
            /* Validate the STRING, not the parsed value. String::toInt()
             * returns 0 for anything unparseable and 0 is a LEGAL idle-air
             * duty, so `val.length() > 0` was not a check: `I abc`, `I -5` or
             * any single stray character all reached the manual branch and set
             * iacFollow=false / iacAuto=false / duty=0. updateOutputs() then
             * falls to its final `else` and calls setIac(0), which is the
             * CLOSED stop - the idle-air valve shuts. A bare `I` with no
             * argument did the same, silently, having changed no intent at all.
             * argIsInt() separates "the operator typed 0" from "the operator
             * typed rubbish"; the range check then rejects out-of-band values
             * instead of silently clamping them. A refused argument changes
             * nothing and prints why. */
            bool f0 = g_cfg.iacFollow, a0 = g_cfg.iacAuto;
            uint8_t d0 = g_cfg.iacManualDuty;
            if (val == "F") {
                g_cfg.iacAuto = false;
                g_cfg.iacFollow = true;
            } else if (val == "A") {
                g_cfg.iacFollow = false;
                g_cfg.iacAuto = true;
            } else if (argIsInt(val)) {
                int k = val.toInt();
                if (k < 0 || k > 100) { Serial.println("I duty 0-100"); break; }
                g_cfg.iacFollow = false;
                g_cfg.iacAuto = false;
                g_cfg.iacManualDuty = (uint8_t)k;
            } else {
                Serial.println("I F|A|<duty 0-100>");
                break;
            }
            /* Write only what moved. All three fields are compared, because a
             * mode switch carries the other two as a side effect: `I F` sets
             * iacFollow while clearing iacAuto, so re-sending `I F` while
             * already following must not commit. Same reason H7 rejected
             * input with nothing changed applies to accepted input that was
             * already the requested value: no NVS write for a no-op. */
            if (g_cfg.iacFollow != f0 || g_cfg.iacAuto != a0 || g_cfg.iacManualDuty != d0) saveCfg();
            break;
        }
        case 'T': {
            /* Validate the STRING first, for the same reason as `I`. toFloat()
             * also maps unparseable input to 0, and worse it stops at the first
             * bad character: `T 1000abc` parsed as 1000.0 and was stored as a
             * real target. `T` is on the 0xC0 allow-list, so this is reachable
             * from the air. Optional leading '-', at most one '.', and at least
             * one digit - which rejects "abc", "", "-", "5.5.5", "1000abc" and
             * "nan". Kept local to this arm rather than added beside argIsInt()
             * because that helper is shared with `Y` and is integer-only. */
            auto tArgIsNum = [](const String& s) {
                if (s.length() == 0) return false;
                bool dot = false, digit = false;
                for (unsigned i = 0; i < s.length(); i++) {
                    char c = s[i];
                    if (c == '-' && i == 0 && s.length() > 1) continue;
                    if (c == '.') { if (dot) return false; dot = true; continue; }
                    if (c < '0' || c > '9') return false;
                    digit = true;
                }
                return digit;
            };
            if (!tArgIsNum(val)) { Serial.println("T 500-3000 rpm"); break; }
            float t = val.toFloat();
            // Upper bound as well as the 500 floor. `T` sits on the 0xC0
            // allow-list, and a floor-only check let `T 40000` through: the
            // (int16_t) cast of an out-of-range float is undefined in C++, so
            // whatever the toolchain produced was stored and saved, and
            // CFG_MAGIC is unchanged so it survived every reboot. The target
            // feeds trim = constrain((target - rpm)/20, -5, 8) in
            // updateOutputs(): a saturated 32767 pins the valve +8 above the CLT
            // curve (over-open idle, harder hot restart), a wrapped negative
            // pins it at -5 and starves it.
            if (t < 500 || t > 3000) { Serial.println("T 500-3000 rpm"); break; }
            /* `T` is on the 0xC0 allow-list, so this arm is reachable by any
             * unauthenticated sender on the air. saveCfg() is a full
             * ~180-byte putBytes - one flash write per call - and it used to run
             * unconditionally, including on the reject path above. A remote peer
             * looping `T 40000` therefore forced one NVS commit per queued frame
             * for a command that changed nothing and printed a rejection, and
             * the 0xD0 ack still reported success. NVS wear on a link that is
             * not authenticated is the real cost, not the bytes.
             *
             * Compare the stored value rather than the parsed one: `T 900` when
             * 900 is already the target is a no-op and must not write either. */
            if (g_cfg.iacTargetRpm != (int16_t)t) { g_cfg.iacTargetRpm = (int16_t)t; saveCfg(); }
            break;
        }
        case 'Y': {
            /* Validate the STRING, not the parsed value. toInt() maps every
             * unparseable string to 0, and 0 here means "no fan output" - at
             * which point setFan() writes nothing while the overheat override
             * upstream still sets fanOn = true and reports success. That is a
             * persistent silent cooling kill reachable with one stray
             * character. Rejected input now has NO effect at all: no state
             * change and no NVS write (saveCfg used to run on the reject path
             * too, so a typo was persisted as well as silently applied). */
            if (!argIsInt(val)) { Serial.println("Y <n> where n=0..7 (0 = no fan output)"); break; }
            int k = val.toInt();
            if (k < 0 || k > 7) { Serial.println("Y 0..7 (0 = no fan output)"); break; }
            uint8_t old = g_cfg.fanOut;
            g_cfg.fanOut = (uint8_t)k;
            // Drive the OLD fan output low before re-pointing, or a relay
            // latched ON under the previous fanOut stays stuck until the
            // next failsafe/outputsOff pass.
            if (old >= 1 && old <= 7) setOut(old - 1, false);
            if (k > 0) setFan(false);
            saveCfg();
            Serial.printf("fanOut=%d\n", k);
            break;
        }
        case 'R': {
            Serial.println("resp=off (removed)");
            break;
        }
        case 'S': {
            /* Validate the STRING, not the parsed result. String::toInt()
             * returns 0 for anything unparseable, so `S abc` and a bare `S`
             * both arrived as 0 and a `rpm > 0` test cannot tell a typo from
             * an intended 0. Rejected input now has no effect at all: no state
             * change and no saveCfg(), which used to run on the reject path too.
             *
             * The missing upper bound is the real defect. `S 40000` narrows
             * 40000 into two int16_t fields, and narrowing an out-of-range int
             * is implementation-defined; on this Xtensa build it is a
             * truncating copy of the low 16 bits, so both shiftRpm and
             * outRpm[0] became -25536. outRpm[0] is the field that actually
             * drives output 1, and the consumer is
             *   case OM_RPM: on = groupSeen(0) && g_rpm >= g_cfg.outRpm[i];
             * against a uint32_t g_rpm, so -25536 promotes to 4294941760 and
             * the comparison is never true - output 1 dead, silently, for as
             * long as the value stays in NVS.
             *
             * Range matches the existing `W maxrpm` clamp,
             * constrain(p1.toInt(), 1000, 20000), and the O<n>R check, so all
             * three commands agree on what counts as a plausible rpm. */
            if (!argIsInt(val)) { Serial.println("S <rpm> (e.g. S 7000)"); break; }
            int rpm = val.toInt();
            if (rpm < 1000 || rpm > 20000) { Serial.println("S 1000-20000 rpm"); break; }
            /* outMode[0] is assigned below, so say what output 1 is being taken
             * out of: this command has always reassigned that mode as a side
             * effect and reported nothing. */
            const char* was = "?";
            switch (g_cfg.outMode[0]) {
                case OM_OFF:  was = "off";  break;
                case OM_MAN:  was = "man";  break;
                case OM_TEMP: was = "temp"; break;
                case OM_RPM:  was = "rpm";  break;
            }
            g_cfg.shiftRpm = (int16_t)rpm;
            g_cfg.outMode[0] = OM_RPM;
            g_cfg.outRpm[0] = (int16_t)rpm;
            saveCfg();
            Serial.printf("o1 R=%d rpm (was %s)\n", rpm, was);
            break;
        }
        case 'O': {
            /* These two used to `return` with no message, so `O`, `O9`, `O99` and
             * `O0` were indistinguishable from a lost command - the same defect
             * the `Y` arm had. An output index out of range must say so. */
            if (val.length() < 2) { Serial.println("O<n> <mode>, n=1..7"); break; }
            uint8_t n = (uint8_t)(val[0] - '1');
            if (n > 6) { Serial.println("O<n> <mode>, n=1..7"); break; }
            String mode = val.substring(1);
            mode.trim();
            /* argIsInt()/argIsFloat() at the top of this file already exist
             * because String::toInt() returns 0 for anything it cannot parse,
             * and 0 cannot be told apart from a real 0 by testing the result. */
            /* A rejected mode must not reach saveCfg() - the reject path used
             * to write NVS too, so a typo was persisted as well as applied. */
            bool accept = true;
            if (mode == "0") { g_cfg.outMode[n] = OM_OFF; }
            else if (mode == "1") { g_cfg.outMode[n] = OM_MAN; g_cfg.outManual[n] = true; }
            else if (mode == "A") { g_cfg.outMode[n] = OM_OFF; }
            else if (mode.startsWith("T")) {
                /* Check the STRING, then range-check the float, then cast.
                 * `O6T` with no number is a case of "cannot parse", so toFloat()
                 * hands back 0.0f, 0 * 10 = 0, and the consumer is
                 *   case OM_TEMP: on = cltOk && clt >= g_cfg.outTemp[i];
                 * with cltOk = groupSeen(2) && clt > 100 && clt < 3500. 0 is
                 * satisfied by every valid reading, so one missing argument
                 * latches the relay ON - and CFG_MAGIC is unchanged, so
                 * saveCfg() made it survive every reboot.
                 *
                 * The upper bound is not cosmetic. 4000.0f * 10.0f = 40000 does
                 * not fit int16_t and the (int16_t) cast of an out-of-range
                 * float is undefined in C++, so `O6T 4000` stored whatever the
                 * toolchain produced and `clt >= that` decides the relay. On
                 * this Xtensa build that is trunc.s then sext from bit 15, i.e.
                 * -25536, so it latched ON for the same reason 0 did - but the
                 * result is compiler-dependent (x86-64 saturates to 32767 and
                 * would instead kill the output), which is the whole argument
                 * for refusing the value before it is cast rather than
                 * inspecting what the cast produced.
                 *
                 * Range 20.0-250.0 degF. Floor excludes 0 and negatives, which
                 * are exactly the values a missing argument and a negative
                 * wrap produce. Ceiling is 250.0 degF = 2500 as stored: above
                 * any coolant this engine reaches even under boost, and an
                 * order of magnitude below the int16_t limit, so the wrap is
                 * unreachable rather than merely unlikely. */
                String tArg = mode.substring(1);
                tArg.trim();
                float tF = argIsFloat(tArg) ? tArg.toFloat() : -1.0f;
                if (tF >= 20.0f && tF <= 250.0f) {
                    g_cfg.outMode[n] = OM_TEMP;
                    g_cfg.outTemp[n] = (int16_t)(tF * 10.0f);
                    Serial.printf("o%u T=%.1f degF\n", n + 1, tF);
                } else {
                    accept = false;
                    Serial.printf("O%u T 20.0-250.0 degF (e.g. O%u T 180)\n", n + 1, n + 1);
                }
            } else if (mode.startsWith("R")) {
                /* Same two doors, opposite directions, so one shared check
                 * would be wrong. outRpm[] is int16_t compared against a
                 * uint32_t g_rpm, so the stored value is promoted to unsigned:
                 *   O6R      -> toInt() = 0 -> `g_rpm >= 0` is true for any
                 *                value including a stopped engine -> latched ON.
                 *   O1R 40000 -> 40000 truncates to -25536 -> promotes to
                 *                4294941760 -> `g_rpm >=` is never true -> the
                 *                output is dead, with no message anywhere.
                 * Range 1000-20000 rpm: 1000 excludes 0 and negatives, and both
                 * ends match the existing `W maxrpm` clamp
                 * (constrain(p1.toInt(), 1000, 20000)) so the two commands
                 * cannot disagree about what counts as a plausible rpm. */
                String rArg = mode.substring(1);
                rArg.trim();
                int rK = argIsInt(rArg) ? rArg.toInt() : 0;
                if (rK >= 1000 && rK <= 20000) {
                    g_cfg.outMode[n] = OM_RPM;
                    g_cfg.outRpm[n] = (int16_t)rK;
                    Serial.printf("o%u R=%d rpm\n", n + 1, rK);
                } else {
                    accept = false;
                    Serial.printf("O%u R 1000-20000 rpm (e.g. O%u R 7000)\n", n + 1, n + 1);
                }
            } else {
                /* 0, 1, A, T and R are all handled above, so anything landing
                 * here is an UNDOCUMENTED argument. It used to be interpreted as
                 * manual on/off via `mode.toInt() != 0`, which is the same
                 * always-on class the T and R branches above were fixed for: a
                 * single unrecognised character latched the relay ON and
                 * saveCfg()'d it. `O1 5`, `O1 x`, `O1 2` all meant "on".
                 *
                 * Refused rather than guessed - the documented set is exactly
                 * 0, 1, A, T<degF>, R<rpm>, and a typo must not move a relay. */
                accept = false;
                Serial.printf("O%u 0|1|A|T<degF>|R<rpm> (got \"%s\")\n", n + 1, mode.c_str());
            }
            if (accept) saveCfg();
            break;
        }
        case 'L': {
            if (val == "0") {
                g_cfg.ledOn = false;
                saveCfg();
                ledApply();
                break;
            }
            if (val.length() == 0 || val == "?") {
                Serial.printf("led=%s rgb=%u,%u,%u (cap %u/255) cmds: L <r> <g> <b> | L 0\n",
                              g_cfg.ledOn ? "on" : "off", g_cfg.ledR, g_cfg.ledG, g_cfg.ledB,
                              LED_BRIGHT_MAX);
                break;
            }
            int r = 0, g = 0, b = 0;
            if (sscanf(val.c_str(), "%d %d %d", &r, &g, &b) == 3) {
                g_cfg.ledR = (uint8_t)constrain(r, 0, 255);
                g_cfg.ledG = (uint8_t)constrain(g, 0, 255);
                g_cfg.ledB = (uint8_t)constrain(b, 0, 255);
                g_cfg.ledOn = true;
                saveCfg();
                ledApply();
            }
            break;
        }
        case 'A': {
            if (val.length() < 2) return;
            uint8_t n = (uint8_t)(val[0] - '1');
            if (n > 3) return;
            String m = val.substring(1);
            m.trim();
            /* A rejected sub-form must not reach saveCfg() at the foot of this
             * arm. The O and F arms used to assign anOut[]/anEnable[] and THEN
             * look at the threshold, with no else, so the save below persisted
             * a mapping that was never actually configured. */
            bool accept = true;
            if (m == "0") {
                /* Disable has to drop the MAPPING too, not just the enable flag.
                 * The O and F arms are the only writers of anOut[] anywhere in
                 * the firmware, and neither the H/L arm nor the plain form ever
                 * sets it. So `A1O7 5` + `A1 0` + `A1H 2` left anEnable true,
                 * anOut still 7 and s_anLatch free to drive the fan - and the
                 * `a1 thr=2.0V HIGH(12V)` confirmation the re-enable prints says
                 * nothing about the mapping, so the retained output was
                 * completely silent. The mapping is copied out of tgtName()'s
                 * static buffer before it is cleared so the line can name it. */
                char had[8];
                snprintf(had, sizeof had, "%s", tgtName(g_cfg.anOut[n]));
                g_cfg.anEnable[n] = false;
                g_cfg.anOut[n] = 0;
                s_anForce[n] = -1;      // disabling clears any bench force too
                Serial.printf("a%u disabled, mapping to %s cleared\n", n + 1, had);
            } else if (m == "D1" || m == "D0" || m == "DA") {
                // bench force: D1 latch on, D0 latch off, DA back to auto
                s_anForce[n] = m == "D1" ? 1 : (m == "D0" ? 0 : -1);
                Serial.printf("a%u force=%s\n", n + 1,
                              s_anForce[n] < 0 ? "auto" : (s_anForce[n] ? "ON" : "off"));
            } else if (m.startsWith("O")) {
                /* O<k> <v> = map this input to output k (1..7), latch above v
                 * volts. Both halves are required and both are checked BEFORE
                 * anything is assigned: the mapping used to be written first and
                 * the threshold second, so `A3O7` (no threshold at all - a short,
                 * valid-looking command) enabled the channel and pointed it at
                 * output 7 at whatever anThresh already held, the 2000mV default
                 * from the Cfg struct, with no message on any path. inputForces(7)
                 * then gated an output on that input, and saveCfg() persisted it.
                 *
                 * argIsFloat() first, because toFloat() stops at the first bad
                 * character: "5abc" is 5.0, and an unparseable argument hands back
                 * 0.0 which the range check then has to catch on its own. */
                uint8_t o = (uint8_t)(m[1] - '1');      // m[1] == '\0' on a bare "O" -> out of range
                String tArg = m.substring(2);
                tArg.trim();
                float v = argIsFloat(tArg) ? tArg.toFloat() : -1.0f;
                if (o <= 6 && v >= 0.1f && v <= 15.0f) {
                    /* Polarity: these two forms carry none, so they reset it to
                     * active-HIGH, exactly as the plain form does. They used to
                     * touch neither s_anLow[n] nor saveAnPol(), so `A3 L2` then
                     * `A3 F` left s_anLow[2] == true and updateAnalogLatch()
                     * evaluated the channel with the GND-switched branch
                     * `raw = latched ? (mv < t + 150) : (mv <= t)`. On the A3
                     * high-beam tap, which idles at ~12V through the lamp
                     * filament, mv never approaches t + 150, so the channel
                     * NEVER latched and the output it was mapped to never
                     * activated - with the console showing a successful map.
                     * The stale bit also survived reboot, because saveAnPol()
                     * was never called with the reset. */
                    bool wasLow = s_anLow[n];
                    g_cfg.anOut[n] = o + 1;
                    g_cfg.anEnable[n] = true;
                    g_cfg.anThresh[n] = (uint16_t)(v * 1000.0f);
                    s_anLow[n] = false;
                    saveAnPol();
                    Serial.printf("a%u %s thr=%.1fV %s\n", n + 1, tgtName(o + 1), v,
                                  s_anLow[n] ? "LOW(gnd)" : "HIGH(12V)");
                    if (wasLow) Serial.printf("a%u polarity was LOW(gnd), reset to HIGH(12V): "
                                               "an idling-12V wire will now latch ON, use A%uL<v> for gnd-switched\n", n + 1, n + 1);
                } else {
                    accept = false;
                    Serial.printf("A%u O<k> <v>, k=1..7 v=0.1-15.0V (got \"%s\")\n", n + 1, m.c_str());
                }
            } else if (m.startsWith("F")) {
                /* Same two-part contract as the O arm, same order of checks. F is
                 * just the fan output (7) spelled out. */
                String tArg = m.substring(1);
                tArg.trim();
                float v = argIsFloat(tArg) ? tArg.toFloat() : -1.0f;
                if (v >= 0.1f && v <= 15.0f) {
                    bool wasLow = s_anLow[n];
                    g_cfg.anOut[n] = 7;
                    g_cfg.anEnable[n] = true;
                    g_cfg.anThresh[n] = (uint16_t)(v * 1000.0f);
                    s_anLow[n] = false;   // F carries no polarity: same reset as O
                    saveAnPol();
                    Serial.printf("a%u %s thr=%.1fV %s\n", n + 1, tgtName(7), v,
                                  s_anLow[n] ? "LOW(gnd)" : "HIGH(12V)");
                    if (wasLow) Serial.printf("a%u polarity was LOW(gnd), reset to HIGH(12V): "
                                               "an idling-12V wire will now latch ON, use A%uL<v> for gnd-switched\n", n + 1, n + 1);
                } else {
                    accept = false;
                    Serial.printf("A%u F <v>, v=0.1-15.0V (got \"%s\")\n", n + 1, m.c_str());
                }
            } else if (m.length() >= 2 && (m[0] == 'H' || m[0] == 'L')) {
                // H<v> = active-high threshold, L<v> = active-low (GND-switched).
                // L-mode latches when the wire is PULLED TO GND — only for
                // circuits that idle at ~12V through their lamp filament.
                float v = m.substring(1).toFloat();
                if (v >= 0.1f && v <= 15.0f) {
                    g_cfg.anEnable[n] = true;
                    g_cfg.anThresh[n] = (uint16_t)(v * 1000.0f);
                    s_anLow[n] = (m[0] == 'L');
                    saveAnPol();
                    Serial.printf("a%u thr=%.1fV %s\n", n + 1, v, s_anLow[n] ? "LOW(gnd)" : "HIGH(12V)");
                }
            } else {
                // Plain form: active-high threshold
                float v = m.toFloat();
                if (v >= 0.1f && v <= 15.0f) {
                    g_cfg.anEnable[n] = true;
                    g_cfg.anThresh[n] = (uint16_t)(v * 1000.0f);
                    s_anLow[n] = false;
                    saveAnPol();
                }
            }
            if (accept) saveCfg();
            break;
        }
        case 'M': {
            // M       -> report link mode (ESP-NOW only — CAN removed)
            Serial.println("proto=ms2 link=espnow");
            break;
        }
        case 'Q':
            // Q              -> dump gas calibration + current reading
            // Q R / Q RESET  -> wipe calibration + gas log back to stock
            // Q F / E / 1/2/3 -> record CURRENT A4 reading as FULL/EMPTY/1/4/HALF/3/4
            // Q D <0-15>     -> damping strength (0 = raw)
            // Q W <5-90>     -> low-fuel warning %
            // Q M <mpg>      -> set assumed mpg for est. miles
            // Q T <gal>      -> set tank capacity in gallons (e.g. Q T 13.2)
            if (val == "R" || val == "RESET") {
                memcpy(g_cfg.gasCalMv, kGasStockMv, sizeof(g_cfg.gasCalMv));
                s_gasFilt = -1;
                s_gasLog = GasLog{};
                s_fullPctSinceCal = -1;  s_fullMvSinceCal  = -1;   // table moved
                s_emptyPctSinceCal = 101; s_emptyMvSinceCal = -1;  // old obs don't count
                gasLogSave();
                saveCfg();
                Serial.println("gas cal + log reset to stock (F=1009 E=25014) - now press SET FULL at a full tank, SET EMPTY at empty");
                break;
            }
            if (val == "F" || val == "E" || val == "1" || val == "2" || val == "3") {
                // Table anchors: slot0 = FULL .. slot4 = EMPTY. The sender's
                // slope is read at cal time (see gasPercent) — low mV = full OR
                // high mV = full, both supported.
                uint8_t slot = (val == "F") ? 0 : (val == "E") ? 4 : (uint8_t)(4 - val.toInt());
                uint16_t raw = readAnalogMv(3);
                // Off-scale rejection (2026-09-20 stuck-at-full bug): an open
                // sender reads ~60k (clamped to 60000), a dead short ~0.
                // Recording either into an anchor poisons the table — a huge
                // FULL anchor makes every real reading <= c[0] and the needle
                // pins at 100% with SET EMPTY powerless. Reject + hint instead.
                if (raw >= 30000 || raw < 1000) {
                    Serial.printf("gas point REJECTED: %u mv off-scale (open or shorted sender?) - no cal recorded\n", raw);
                    break;
                }
                g_cfg.gasCalMv[slot] = raw;
                // Re-linearise 1/4..3/4 between the anchors whenever an anchor
                // is (re)set. Signed math keeps it correct for BOTH slopes:
                // c[0]<c[4] rising (low mV = full) or c[0]>c[4] falling
                // (high mV = full). Explicit Q 1/2/3 refinements still win,
                // but do them AFTER F/E or they get re-linearised.
                if (slot == 0 || slot == 4) {
                    int32_t span = (int32_t)g_cfg.gasCalMv[4] - (int32_t)g_cfg.gasCalMv[0];
                    if (span) {
                        for (uint8_t j = 1; j < 4; j++)
                            g_cfg.gasCalMv[j] = (uint16_t)((int32_t)g_cfg.gasCalMv[0] + span * j / 4);
                        Serial.println("gas mids re-linearised; refine with Q 1/2/3 after F/E");
                    }
                }
                s_gasFilt = -1;                     // reseed filter after cal change
                // The auto-cal gates are "seen >=95% / <=5% under the CURRENT
                // table", so they are void the moment the table moves. Without
                // this, a `Q E 18000` could be immediately overwritten by
                // auto-cal from the very reading the operator just rejected.
                s_fullPctSinceCal = -1;  s_fullMvSinceCal  = -1;
                s_emptyPctSinceCal = 101; s_emptyMvSinceCal = -1;
                // The log's percentages now describe a calibration that no longer
                // exists; re-derive them from the recorded mV so the band matches
                // the needle. ('Q R' above needs no rescore — it wipes the log.)
                if (gasLogRescore()) gasLogSave();
                saveCfg();
                Serial.printf("gas point %u = %u mv\n", slot, g_cfg.gasCalMv[slot]);
                break;
            }
            /* All four of these are on the ESP-NOW allow-list
             * (cmdAllowedFromLink permits Q with a "D "/"W "/"M "/"T " prefix),
             * so they are AIR-reachable, and every one of them had the same two
             * defects that 8b0bfbd fixed for F/I/T and 8b93a28 fixed for W:
             *
             *  1. `toInt()` on an unvalidated string. `Q D nan` -> atoi("nan")
             *     is 0 -> 0 is in [0,15] -> gasDamp = 0, and gasSampleMv() does
             *        if (g_cfg.gasDamp == 0 || s_gasFilt < 0) f = raw;
             *     which bypasses the EMA entirely and drives the needle off
             *     unfiltered 8-sample ADC. Same for `Q D abc`. Not a cosmetic
             *     setting: it is the input filter for the fuel gauge.
             *  2. saveCfg() unconditional, so a repeat of the CURRENT value
             *     still commits a full ~180-byte NVS blob. Flooding `Q W 20`
             *     (lowFuelPct defaults to 20) therefore drives one flash erase
             *     per drained queue slot for as long as the flood lasts, on a
             *     link that is not authenticated. That is the same amplification
             *     8b0bfbd's own comment names as "the real cost, not the bytes" -
             *     the rule was just never generalised to the Q family.
             *
             * So: validate the string, and only write NVS when the value moved.
             */
            if (val.startsWith("D ")) {
                String a = val.substring(2); a.trim();
                if (!argIsInt(a)) { Serial.println("damp 0-15"); break; }
                int d = a.toInt();
                if (d < 0 || d > 15) { Serial.println("damp 0-15"); break; }
                if ((uint8_t)d != g_cfg.gasDamp) {
                    g_cfg.gasDamp = (uint8_t)d;
                    s_gasFilt = -1;          // reseed the filter on any change
                    saveCfg();
                }
                Serial.printf("gas damp=%u\n", d);
                break;
            }
            if (val.startsWith("W ")) {
                String a = val.substring(2); a.trim();
                if (!argIsInt(a)) { Serial.println("warn 5-90%"); break; }
                int w = a.toInt();
                if (w < 5 || w > 90) { Serial.println("warn 5-90%"); break; }
                if ((uint8_t)w != g_cfg.lowFuelPct) {
                    g_cfg.lowFuelPct = (uint8_t)w;
                    saveCfg();
                }
                Serial.printf("low-fuel warn=%u%%\n", w);
                break;
            }
            if (val.startsWith("M ")) {
                String a = val.substring(2); a.trim();
                if (!argIsInt(a)) { Serial.println("mpg range 5-99"); break; }
                int m = a.toInt();
                if (m < 5 || m > 99) { Serial.println("mpg range 5-99"); break; }
                if ((uint8_t)m != g_cfg.gasMpg) {
                    g_cfg.gasMpg = (uint8_t)m;
                    saveCfg();
                }
                Serial.printf("est-mpg=%u\n", g_cfg.gasMpg);
                break;
            }
            if (val.startsWith("T ")) {
                /* Range-check the FLOAT before the cast, not the cast result.
                 * `(int)(toFloat() * 10.0f + 0.5f)` is undefined behaviour for
                 * nan/inf - atof accepts both - and the result is
                 * toolchain-dependent (x86-64 cvttss2si gives INT_MIN, Xtensa
                 * trunc.s something else). It happened to land outside [10,500]
                 * and be rejected, so there was no live wrong value, but "it
                 * works by luck on both ISAs" is not a check. Validating first
                 * makes the cast defined by construction. */
                String a = val.substring(2); a.trim();
                if (!argIsFloat(a)) { Serial.println("tank 1.0-50.0 gal"); break; }
                float gal = a.toFloat();
                if (!isfinite(gal) || gal < 1.0f || gal > 50.0f) {
                    Serial.println("tank 1.0-50.0 gal");
                    break;
                }
                int t = (int)(gal * 10.0f + 0.5f);   // now in [10,500]: defined
                if ((uint16_t)t != g_cfg.tankGalX10) {
                    g_cfg.tankGalX10 = (uint16_t)t;
                    saveCfg();
                }
                Serial.printf("tank=%.1f gal\n", g_cfg.tankGalX10 / 10.0f);
                break;
            }
            Serial.printf("gas=%d%% mv=%d est=%dmi damp=%u warn<=%u%% mpg=%u tank=%.1fgal pts[F,3/4,1/2,1/4,E]=%u,%u,%u,%u,%u\n",
                          gasPctFromFilt(), s_gasFilt,
                          (int)((long)gasPctFromFilt() * g_cfg.tankGalX10 * g_cfg.gasMpg / 1000),
                          g_cfg.gasDamp, g_cfg.lowFuelPct, g_cfg.gasMpg, g_cfg.tankGalX10 / 10.0f,
                          g_cfg.gasCalMv[0], g_cfg.gasCalMv[1], g_cfg.gasCalMv[2],
                          g_cfg.gasCalMv[3], g_cfg.gasCalMv[4]);
            if (s_gasLog.minPct < 0)
                Serial.println("gas log: unseeded (learns from first tank read)");
            else
                Serial.printf("gas log: LO %d%% (%dmv) HI %d%% (%dmv)\n",
                              s_gasLog.minPct, s_gasLog.minMv,
                              s_gasLog.maxPct, s_gasLog.maxMv);
            {
                const uint16_t *gc = g_cfg.gasCalMv;
                bool poisoned = (gc[0] >= 30000 || gc[4] >= 30000 || gc[0] < 1000 || gc[4] < 1000 || gc[0] == gc[4]);
                if (poisoned)
                    Serial.println("WARN: anchors off-scale/degenerate -> mapping STOCK span. Send Q R to reset, then SET FULL/SET EMPTY.");
                else if (gc[0] > gc[4])
                    Serial.println("note: inverted sender slope (high mV = FULL) detected");
            }
            break;
        case 'P': {
            // P              -> report map
            // P IAC <pin>    -> set IAC pin
            // P O<n> <pin>   -> set output n pin (1-7)
            // P TFT 0|1      -> display enable
            // P TFTS <pin>   -> set TFT SCLK
            // P TFTM <pin>   -> set TFT MOSI
            // P TFTC <pin>   -> set TFT CS
            // P TFTD <pin>   -> set TFT DC
            // P RESET        -> back to iobox3 defaults
            if (val.length() == 0) {
                Serial.printf("pins iac=%u o1=%u o2=%u o3=%u o4=%u o5=%u o6=%u o7=%u tfts=%u tftm=%u tftc=%u tftd=%u bz=%u\n",
                              g_cfg.pin.iac, g_cfg.pin.out[0], g_cfg.pin.out[1], g_cfg.pin.out[2],
                              g_cfg.pin.out[3], g_cfg.pin.out[4], g_cfg.pin.out[5], g_cfg.pin.out[6],
                              g_cfg.pin.tftSclk, g_cfg.pin.tftMosi, g_cfg.pin.tftCs, g_cfg.pin.tftDc,
                              g_cfg.pin.buzz);
                if (dashMacBound()) {
                    Serial.printf("dashmac=%02x:%02x:%02x:%02x:%02x:%02x\n",
                                  s_dashMac[0], s_dashMac[1], s_dashMac[2],
                                  s_dashMac[3], s_dashMac[4], s_dashMac[5]);
                } else {
                    Serial.println("dashmac=UNBOUND (accept any peer)");
                }
                if (diagMacBound()) {
                    Serial.printf("diagmac=%02x:%02x:%02x:%02x:%02x:%02x\n",
                                  s_diagMac[0], s_diagMac[1], s_diagMac[2],
                                  s_diagMac[3], s_diagMac[4], s_diagMac[5]);
                } else {
                    Serial.println("diagmac=UNBOUND");
                }
                break;
            }
            if (val.startsWith("DASH ")) {
                String m = val.substring(5);
                m.trim();
                if (m.equalsIgnoreCase("CLEAR")) {
                    memset(s_dashMac, 0, sizeof(s_dashMac));
                    saveDashMac();
                    Serial.println("dashmac cleared (accept any peer)");
                    break;
                }
                unsigned int b[6] = {0, 0, 0, 0, 0, 0};
                if (sscanf(m.c_str(), "%x:%x:%x:%x:%x:%x",
                           &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) == 6) {
                    for (uint8_t i = 0; i < 6; i++) s_dashMac[i] = (uint8_t)b[i];
                    saveDashMac();
                    Serial.printf("dashmac bound %02x:%02x:%02x:%02x:%02x:%02x\n",
                                  s_dashMac[0], s_dashMac[1], s_dashMac[2],
                                  s_dashMac[3], s_dashMac[4], s_dashMac[5]);
                } else {
                    Serial.println("P DASH <aa:bb:cc:dd:ee:ff> | P DASH CLEAR");
                }
                break;
            }
            if (val.startsWith("DIAG ")) {
                String m = val.substring(5);
                m.trim();
                if (m.equalsIgnoreCase("CLEAR")) {
                    memset(s_diagMac, 0, sizeof(s_diagMac));
                    saveDiagMac();
                    Serial.println("diagmac cleared");
                    break;
                }
                unsigned int b[6] = {0, 0, 0, 0, 0, 0};
                if (sscanf(m.c_str(), "%x:%x:%x:%x:%x:%x",
                           &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) == 6) {
                    for (uint8_t i = 0; i < 6; i++) s_diagMac[i] = (uint8_t)b[i];
                    saveDiagMac();
                    Serial.printf("diagmac bound %02x:%02x:%02x:%02x:%02x:%02x\n",
                                  s_diagMac[0], s_diagMac[1], s_diagMac[2],
                                  s_diagMac[3], s_diagMac[4], s_diagMac[5]);
                } else {
                    Serial.println("P DIAG <aa:bb:cc:dd:ee:ff> | P DIAG CLEAR");
                }
                break;
            }
            if (val == "RESET") {
                g_cfg.pin = PinMap{};
                saveCfg();
                applyPinConfig();
                tftReinitRequest();
                Serial.println("pin map reset to iobox3 defaults");
                break;
            }
            if (val == "WIPE") {
                // Full factory reset — the ONLY command that clears everything.
                // Previously there was no single recovery path: `P RESET` clears
                // the pin map alone, `Q R` the gas calibration alone, and nothing
                // cleared the peer filters. If a stray `P DASH <mac>` ever landed
                // (now refused over the air, but a stored one can survive a
                // firmware downgrade), the box rejected the real dash and only a
                // hand-edited NVS could recover it.
                /* Preferences::clear() returns whether the namespace was
                 * actually erased, and it was discarded here. s_nvsOk says
                 * nothing about THIS call - it is latched once at boot from
                 * g_prefs.begin() - so a namespace that opened successfully and
                 * then went read-only or full (NVS partition full is a real
                 * state, not a hypothetical) printed "FACTORY RESET ... Reboot
                 * not required" for a reset that neither happened nor persists.
                 * That is the exact class of lie the comments around this block
                 * exist to prevent, and it is why the return was worth reading.
                 *
                 * Prefer reports_harmless=false so the operator is told a reset
                 * did not happen, rather than being handed a silent no-op that
                 * looks like a success. */
                const bool wipeOk = g_prefs.clear();
                g_cfg = Cfg{};
                memcpy(g_cfg.gasCalMv, kGasStockMv, sizeof(g_cfg.gasCalMv));
                saveCfg();
                memset(s_dashMac, 0, sizeof(s_dashMac));
                memset(s_diagMac, 0, sizeof(s_diagMac));
                saveDashMac();
                saveDiagMac();
                s_gasLog = GasLog{};
                gasLogSave();
                for (uint8_t i = 0; i < 4; i++) {
                    s_anForce[i] = -1;          // bench override
                    s_anLow[i]   = false;       // saved polarity — THIS is what
                    s_anLatch[i] = false;       //   saveAnPol() actually persists
                }
                saveAnPol();
                s_dashMacHinted = false;
                // Buzzer overrides are RAM-only, so a wipe that left them behind
                // would keep a piezo screaming with nothing left to stop it:
                // `X 3` holds the pin hard LOW and updateBuzzer() early-returns
                // on s_buzzManual, so applyPinConfig() alone does NOT release it.
                s_buzzManual = false;
                s_buzzManualUntilMs = 0;
                s_buzzLoop   = false;
                s_fullPctSinceCal = -1;  s_fullMvSinceCal  = -1;
                s_emptyPctSinceCal = 101; s_emptyMvSinceCal = -1;
                s_gasFilt = -1;
                applyPinConfig();
                // Same trap on the backlight: g_cfg.ledOn is now false, but the
                // WS2812 bar keeps emitting the pre-wipe colour until the next `L`
                // command or a reboot, because ledApply() is only called from
                // ledInit() and the `L` handler. "Reboot not required" was a lie.
                ledApply();
                tftReinitRequest();
                Serial.println("FACTORY RESET: cfg + gas table + gas log + peer filters + A1-A4 polarity");
                Serial.println("              + LED bar + buzzer overrides, all back to defaults.");
                if (s_nvsOk && wipeOk) {
                    Serial.println("              Reboot not required.");
                } else if (!s_nvsOk) {
                    // True today, false on the next boot: every put above was a
                    // no-op. Do not let the message promise a persistent reset.
                    Serial.println("              *** NVS IS NOT OPEN — NONE OF THIS WILL SURVIVE A REBOOT. ***");
                } else {
                    /* NVS opened at boot, but this erase did not happen: the
                     * namespace is most likely full or read-only. The settings
                     * below are live in RAM, so the box looks reset, and the
                     * stored config survives a reboot intact - the operator is
                     * told a reset that did not occur. */
                    Serial.println("              *** THE ERASE FAILED (NVS full or read-only). ***");
                    Serial.println("              *** Settings changed but will NOT survive a reboot. ***");
                }
                break;
            }
            if (val == "TFT 1" || val == "TFT 0") {
                g_cfg.tftEnable = (val == "TFT 1");
                saveCfg();
                tftReinitRequest();
                Serial.println(g_cfg.tftEnable ? "tft on (re-init queued)" : "tft off (re-init queued)");
                break;
            }
            int sp = val.indexOf(' ');
            if (sp <= 0) { Serial.println("P IAC <pin> | P O<n> <pin> | P TFT 0|1 | P TFTS/TFTM/TFTC/TFTD/BZ <pin> | P DASH/DIAG <mac>|CLEAR | P RESET | P WIPE"); break; }
            String k = val.substring(0, sp);
            int pin = val.substring(sp + 1).toInt();
            k.trim();
            /* Range-check BEFORE the uint8_t narrowing in pinOk(). pinOk()
             * takes a uint8_t, so 'P IAC 275' used to validate GPIO19 and
             * silently store 19 while reporting the input as a bad pin -- the
             * rejection message was unreachable for any aliased value. */
            if (pin < 0 || pin > 39 || !pinOk((uint8_t)pin)) {
                Serial.println("pin rejected: not a usable output on WROOM-32 (0-39, no strapping/flash/input-only/owned pins)");
                break;
            }
            /* Aliasing check. pinOk() proves the pin is legal; this proves it is
             * not already spoken for. Skip the entry being reassigned so
             * 'P O2 <same pin>' stays a no-op instead of failing. */
            int skipIdx = -1;
            if (k == "IAC")        skipIdx = -2;   /* iac: nothing to skip, always check all */
            else if (k.length() == 2 && k[0] == 'O') skipIdx = (int)(uint8_t)(k[1] - '1');
            if (skipIdx == -2) {
                if (!pinAliasFree((uint8_t)pin, -1, PINASSIGN_IAC)) {
                    Serial.println("pin rejected: already used by another output/IAC/buzzer/LED/speed/gas/display pin");
                    break;
                }
            } else if (skipIdx >= 0 && skipIdx <= 6) {
                if (!pinAliasFree((uint8_t)pin, skipIdx, PINASSIGN_NONE)) {
                    Serial.printf("pin rejected: GPIO%u already in use by another mapped pin\n", pin);
                    break;
                }
            } else if (skipIdx > 6) {
                Serial.println("P O<n> <pin>, n=1..7");
                break;
            } else {
                /* The TFT SCLK/MOSI/CS/DC and BZ pins must also not collide
                 * with the actuator pins -- and, since 2026-10-01, each of them
                 * must also not collide with the other three. `assigning` is
                 * what lets a command keep the pin it already owns; naming the
                 * wrong field here fails closed (see pinAliasFree()). */
                uint8_t asg = PINASSIGN_NONE;
                if      (k == "BZ")   asg = PINASSIGN_BZ;
                else if (k == "TFTS") asg = PINASSIGN_TFTS;
                else if (k == "TFTM") asg = PINASSIGN_TFTM;
                else if (k == "TFTC") asg = PINASSIGN_TFTC;
                else if (k == "TFTD") asg = PINASSIGN_TFTD;
                if (!pinAliasFree((uint8_t)pin, -1, asg)) {
                    Serial.println("pin rejected: already used by an output/IAC/buzzer/LED/speed/gas/display pin");
                    break;
                }
            }
            if (k == "IAC") {
                g_cfg.pin.iac = (uint8_t)pin;
            } else if (k.length() == 2 && k[0] == 'O') {
                uint8_t n = (uint8_t)(k[1] - '1');
                if (n <= 6) g_cfg.pin.out[n] = (uint8_t)pin;
                else { Serial.println("P O<n> <pin>, n=1..7"); break; }
            } else if (k == "TFTS") {
                g_cfg.pin.tftSclk = (uint8_t)pin;
            } else if (k == "TFTM") {
                g_cfg.pin.tftMosi = (uint8_t)pin;
            } else if (k == "TFTC") {
                g_cfg.pin.tftCs = (uint8_t)pin;
            } else if (k == "TFTD") {
                g_cfg.pin.tftDc = (uint8_t)pin;
            } else if (k == "BZ") {
                /* Re-pointing the buzzer pin while an `X 0..3` manual test is
                 * active used to orphan the pad the test was driving.
                 *
                 * `X 3` does pinMode(old); digitalWrite(old, LOW) and sets
                 * s_buzzManual. `P BZ <new>` then reassigns g_cfg.pin.buzz and
                 * calls applyPinConfig(), which configures the NEW pad and never
                 * touches the old one. s_buzzManual was still true, so
                 * updateBuzzer() early-returned and never drove anything. `X 9`
                 * and the manual-test timeout both call buzzReleasePin(), which
                 * releases g_cfg.pin.buzz - by then the NEW pin. So the old pad
                 * stayed OUTPUT/LOW, the horn kept sounding, and `X 9` printed
                 * "output restored" while nothing had been restored. Only a power
                 * cycle cleared it.
                 *
                 * That is the exact failure d631cd3 set out to close, reached
                 * through the one command that re-points the pin. Release the OLD
                 * pad and drop the manual flag BEFORE the reassignment, so the
                 * flag is already clear by the time applyPinConfig() runs.
                 *
                 * P WIPE already does the s_buzzManual half of this, with the
                 * comment "applyPinConfig() alone does NOT release it" - the
                 * pin-remap branch was simply never given the same treatment. */
                if (g_cfg.pin.buzz != (uint8_t)pin) {
                    /* buzzReleasePad() re-checks `!= iac` itself, so an
                     * old config that still has pin.buzz == pin.iac is left
                     * alone rather than having its valve gate driven. */
                    buzzReleasePad(g_cfg.pin.buzz);
                    s_buzzManual = false;
                    s_buzzManualUntilMs = 0;
                }
                g_cfg.pin.buzz = (uint8_t)pin;
            } else {
                Serial.println("P IAC <pin> | P O<n> <pin> | P TFT 0|1 | P TFTS/TFTM/TFTC/TFTD/BZ <pin> | P DASH/DIAG <mac>|CLEAR | P RESET | P WIPE");
                break;
            }
            saveCfg();
            applyPinConfig();
            tftReinitRequest();
            Serial.println("pin map updated (tft re-init queued)");
            break;
        }
        case 'W': {
            if (val == "0") { g_cfg.eng.enabled = false; saveCfg(); break; }
            if (val == "1") { g_cfg.eng.enabled = true; saveCfg(); break; }
            int sp = val.indexOf(' ');
            String k = sp > 0 ? val.substring(0, sp) : val;
            String v = sp > 0 ? val.substring(sp + 1) : "";
            v.trim();
            int a = v.indexOf(' ');
            String p1 = a > 0 ? v.substring(0, a) : v;
            String p2 = a > 0 ? v.substring(a + 1) : "";
            p2.trim();

            /* H6 - every numeric argument is validated as a STRING before it is
             * parsed, and no field is written until every argument of the
             * sub-command has passed.
             *
             * String::toFloat() is atof() and String::toInt() is atol(). atof
             * accepts "nan", "inf" and a leading numeric prefix of anything
             * ("212abc" -> 212.0); atol returns 0 for anything that is not a
             * number at all. So a malformed argument used to reach g_cfg as a
             * silently clamped, saveCfg()-persisted threshold instead of a
             * refusal. Every one of these fields is then compared against a
             * LIVE sensor reading in engineWarnFlags(), and a threshold that
             * landed at the wrong end of its range does not just read wrong, it
             * latches. Taking this car's real readings (g_* all in tenths) and
             * running the stored threshold back through engineWarnFlags():
             *
             *   W clt nan      -> 100.0 F. cltOk holds (107-126 F is in range),
             *                     so 107 > 100 => W_OVERHEAT, CONTINUOUSLY:
             *                     buzzer every second, warn relay blinking,
             *                     and saveCfg() makes it survive a power cycle.
             *   W mat nan      ->  50.0 F => 107 >  50 => W_HOTAIR, continuously.
             *   W map nan      ->   0.0 kPa. At the time of this audit this
             *                     check had no sanity gate at all, so 98 > 0 =>
             *                     W_OVERBOOST, continuously. Two later fixes
             *                     closed that: `mapOk` now exists, and the W map
             *                     range floor was raised from 0 to 1000, because
             *                     a well-formed `W map 0` reached it regardless
             *                     and idle MAP is already 30-100 kPa.
             *   W batt nan nan ->   5.0 V, which is UNDER a resting battery, so
             *                     it is an OVER-voltage reading: 126 > 50 =>
             *                     W_HIBATT, continuously.
             *   W afr nan nan  ->   5.0 AFR => 130 > 50 => W_LEAN, continuously.
             *   W maxrpm junk  -> atol's 0 clamped to 1000 rpm => 4200 >= 1000
             *                     => W_OVERREV at cruise.
             *   W idle junk junk -> 300/300 rpm => 900 > 300 with TPS under 20%
             *                     => W_IDLE_HI at cruise, and W_IDLE_LO becomes
             *                     unreachable.
             *   W warnout junk -> 0, and the lamp is gated on warnOut >= 1, so
             *                     the check-engine output silently dies.
             *   W hold junk    -> 0, silently killing the latch hold.
             *
             * So NONE of these nine is safe by accident: five fail through
             * atof's NaN and four through atol's 0, and they were all fixed
             * the same way rather than leaving the integer ones because they
             * looked lower-risk.
             *
             * "inf" is rejected as well, deliberately. (int) of it is undefined
             * behaviour too; on Xtensa it happens to land on the range maximum
             * so it was harmless in practice, but that is luck rather than
             * intent and one rule is easier to keep right than two.
             *
             * The string test cannot be replaced by a test on the parsed
             * result. A result check cannot tell "the operator typed 0" from
             * "the operator typed rubbish", and a NaN result is not
             * distinguishable from a real reading at all. isfinite() on the
             * result is kept as well, because the grammar below still admits an
             * arbitrarily long digit run and atof() of that is +inf. */
            auto argIsNum = [](const String& s) -> bool {
                if (s.length() == 0) return false;
                bool digit = false, dot = false;
                for (unsigned i = 0; i < s.length(); i++) {
                    char c = s[i];
                    if ((c == '-' || c == '+') && i == 0) continue;   /* sign, position 0 only */
                    if (c == '.') {
                        if (dot) return false;
                        dot = true;
                    } else if (c < '0' || c > '9') {
                        return false;   /* letters, "nan", "inf", whitespace */
                    } else {
                        digit = true;
                    }
                }
                return digit;           /* rejects "", "+", "-", "." */
            };
            /* Same arithmetic as the pre-H6 expression (toFloat() * 10.0f, then
             * truncate toward zero, then constrain), but the clamp happens in
             * float BEFORE the int cast. That cast was the undefined step:
             * atof can hand back a finite float far outside any of these
             * ranges, and (int) of that is UB on Xtensa. Once clamped, t is
             * inside [lo,hi] and the cast is defined. */
            auto num10 = [&argIsNum](const String& s, int lo, int hi, int& out) -> bool {
                if (!argIsNum(s)) return false;
                float f = s.toFloat();
                if (!isfinite(f)) return false;
                float t = f * 10.0f;
                if (t < (float)lo) t = (float)lo;
                if (t > (float)hi) t = (float)hi;
                out = (int)t;
                return true;
            };
            /* A refusal must be VISIBLE and INERT. Every reject below breaks
             * out of this case, which skips both the field assignments and the
             * saveCfg() at the bottom, so a refused argument leaves the running
             * profile and the stored profile exactly as they were. */
            auto reject = [](const String& key, const String& arg) {
                Serial.print("W ");
                Serial.print(key);
                Serial.print(": '");
                Serial.print(arg);
                Serial.println("' is not a number - eng profile unchanged");
            };

            if (k == "idle" && a > 0) {
                if (!argIsInt(p1)) { reject(k, p1); break; }
                if (!argIsInt(p2)) { reject(k, p2); break; }
                int mn = constrain(p1.toInt(), 300, 3000);
                int mx = constrain(p2.toInt(), 300, 3000);
                if (mx < mn) { int t = mn; mn = mx; mx = t; }
                g_cfg.eng.idleRpmMin = (int16_t)mn;
                g_cfg.eng.idleRpmMax = (int16_t)mx;
            } else if (k == "maxrpm" && p1.length() > 0) {
                if (!argIsInt(p1)) { reject(k, p1); break; }
                g_cfg.eng.maxRpm = (int16_t)constrain(p1.toInt(), 1000, 20000);
            } else if (k == "clt" && p1.length() > 0) {
                int t;
                if (!num10(p1, 1000, 3000, t)) { reject(k, p1); break; }
                g_cfg.eng.cltMax = (int16_t)t;
            } else if (k == "mat" && p1.length() > 0) {
                int t;
                if (!num10(p1, 500, 2500, t)) { reject(k, p1); break; }
                g_cfg.eng.matMax = (int16_t)t;
            } else if (k == "batt" && a > 0) {
                int mn, mx;
                if (!num10(p1, 50, 200, mn)) { reject(k, p1); break; }
                if (!num10(p2, 50, 200, mx)) { reject(k, p2); break; }
                if (mx < mn) { int t = mn; mn = mx; mx = t; }
                g_cfg.eng.battMin = (int16_t)mn;
                g_cfg.eng.battMax = (int16_t)mx;
            } else if (k == "map" && p1.length() > 0) {
                int t;
                /* Floor 1000 = 100 kPa, and it is not arbitrary. MAP at idle is
                 * already ~30-100 kPa (300-1000 in the x10 units this channel
                 * uses), so ANY threshold below that is exceeded the moment the
                 * engine starts and latches W_OVERBOOST permanently - buzzer
                 * every second, warn relay blinking, threshold persisted.
                 *
                 * `map` was the only one of the nine numeric W sub-commands
                 * whose range floor was 0. clt floors at 1000, mat at 500, afr
                 * at 50, batt at 50, maxrpm at 1000. A zero floor there is not
                 * "permissive", it is a value that cannot mean anything, and it
                 * was reachable with a perfectly well-formed argument - the
                 * malformed-input validation added in 8b93a28 could not catch
                 * it because `0` is a number.
                 *
                 * Ceiling 4000 = 400 kPa, unchanged: well above the 165 kPa
                 * boost target this engine is aiming at.
                 *
                 * Note num10() CLAMPS an out-of-range number to the bound
                 * rather than refusing it, so `W map 0` now lands on 100 kPa
                 * instead of being rejected. That is the same `constrain()`
                 * behaviour the other sub-commands have always had, and it is
                 * what makes this safe: the stored value cannot be 0 whatever
                 * is typed. */
                if (!num10(p1, 1000, 4000, t)) { reject(k, p1); break; }
                g_cfg.eng.mapMax = (int16_t)t;
            } else if (k == "afr" && a > 0) {
                int lo, hi;
                if (!num10(p1, 50, 250, lo)) { reject(k, p1); break; }
                if (!num10(p2, 50, 250, hi)) { reject(k, p2); break; }
                if (hi < lo) { int t = lo; lo = hi; hi = t; }
                g_cfg.eng.afrLow = (int16_t)lo;
                g_cfg.eng.afrHigh = (int16_t)hi;
            } else if (k == "hold" && p1.length() > 0) {
                if (!argIsInt(p1)) { reject(k, p1); break; }
                g_cfg.eng.warnHoldMs = (uint16_t)constrain(p1.toInt(), 0, 60000);
            } else if (k == "warnout" && p1.length() > 0) {
                if (!argIsInt(p1)) { reject(k, p1); break; }
                g_cfg.eng.warnOut = (uint8_t)constrain(p1.toInt(), 0, 7);
            } else if (k == "help") {
                Serial.println("W[0|1] | W idle <min> <max> | W maxrpm <rpm> | W clt <F> | W mat <F> | W batt <min> <max> | W map <kPa> | W afr <min> <max> | W hold <ms> | W warnout <0-7>");
                break;
            } else {
                Serial.println("W[0|1] | W idle <min> <max> | W maxrpm <rpm> | W clt <F> | W mat <F> | W batt <min> <max> | W map <kPa> | W afr <min> <max> | W hold <ms> | W warnout <0-7>");
                break;
            }
            saveCfg();
            Serial.println("eng profile updated");
            break;
        }
        case 'B':
            // B        -> report buzzer state
            // B 0|1    -> disable/enable warning buzzer
            // B T      -> one 100ms test beep now
            if (val == "0") { g_cfg.buzzerEnable = false; s_buzzLoop = false; saveCfg(); Serial.println("buzzer=off"); }
            else if (val == "1") { g_cfg.buzzerEnable = true; s_buzzLoop = false; saveCfg(); Serial.println("buzzer=on"); }
            else if (val == "L") { s_buzzLoop = !s_buzzLoop; Serial.printf("buzzer=loop %s\n", s_buzzLoop ? "on" : "off"); }
            else if (val == "T") { s_buzzTestUntilMs = millis() + 100; Serial.println("beep"); }
            else Serial.printf("buzzer=%s pin=%u (B 0|1 | B L | B T)\n", g_cfg.buzzerEnable ? "on" : "off", g_cfg.pin.buzz);
            break;
        case 'X':
            // buzzer pin manual test: X0=pullup-hiz X1=float X2=drive3v3 X3=gnd(beep) X9=auto
            if (val == "0" || val == "1" || val == "2" || val == "3") {
                /* ONE guard for every sub-command that takes the pin, not one per
                 * line. X 0..3 all set s_buzzManual and then reconfigure the pad,
                 * and on pin.buzz == pin.iac that pad is the IAC MOSFET gate: it is
                 * driven by LEDC at 250Hz, and setIac()'s 0% is the CLOSED stop.
                 *
                 *   X 3 -> pinMode(19,OUTPUT); digitalWrite(19,LOW)   gate railed
                 *          low, valve shut, engine running
                 *   X 2 -> pinMode(19,OUTPUT); digitalWrite(19,HIGH)  gate commanded
                 *          100% continuously against the PWM, valve slammed to the
                 *          full-open stop and the solenoid DC-driven
                 *   X 0/1 -> pinMode(19,INPUT)                       gate driver off
                 *
                 * and in all four cases s_buzzManual suppresses updateBuzzer()
                 * while the restore path in `X 9` is gated OFF by its own
                 * `pin.buzz != pin.iac` guard — so the documented way out could
                 * never undo any of it. Only a power cycle cleared it.
                 *
                 * The check is `==` and sits ABOVE the first mutation, so a refusal
                 * is a true no-op: s_buzzManual is not set and the pad is not
                 * touched, leaving `X 9` still able to clear a manual test set
                 * before this guard existed. A sub-command added below must be
                 * written inside this branch to inherit the guard — it cannot be
                 * added past it. */
                if (g_cfg.pin.buzz == g_cfg.pin.iac) {
                    Serial.printf("X %s REFUSED: buzzer pin GPIO%u is the IAC pin. A manual "
                                  "test would take the idle-air valve off its 250Hz PWM. "
                                  "Give the buzzer its own pin first: P BZ <pin>\n",
                                  val.c_str(), g_cfg.pin.iac);
                    break;
                }
                s_buzzManual = true;                 // pause auto drive; X 9 or the timeout gives it back
                s_buzzManualUntilMs = millis() + BUZZ_MANUAL_MS;
                if      (val == "0") { pinMode(g_cfg.pin.buzz, INPUT_PULLUP); Serial.println("buzzpin=input_pullup"); }
                else if (val == "1") { pinMode(g_cfg.pin.buzz, INPUT);         Serial.println("buzzpin=float"); }
                else if (val == "2") { pinMode(g_cfg.pin.buzz, OUTPUT); digitalWrite(g_cfg.pin.buzz, HIGH); Serial.println("buzzpin=high_3v3"); }
                else                 { pinMode(g_cfg.pin.buzz, OUTPUT); digitalWrite(g_cfg.pin.buzz, LOW);  Serial.println("buzzpin=gnd_beep (X 9 to exit)"); }
            }
            else if (val == "9") {
                s_buzzManual = false;
                s_buzzManualUntilMs = 0;
                // X 9 used to clear the manual flag only. After `X 0` or `X 1` the
                // pin was still INPUT, so updateBuzzer()'s digitalWrite() was a
                // no-op and the buzzer stayed dead until a reboot — the command
                // that is supposed to RESTORE the pin did not restore it. It now
                // calls the same buzzReleasePin() the manual-test timeout uses, so
                // there is exactly one implementation of "give the pad back" and
                // the two paths cannot drift apart again.
                if (g_cfg.pin.buzz != g_cfg.pin.iac) {
                    buzzReleasePin();
                    Serial.println("buzzpin=auto (output restored)");
                } else {
                    /* This used to print "output restored" unconditionally, on
                     * the one configuration where this guard had just refused
                     * to restore anything. `P BZ` cannot create the collision any
                     * more (pinAliasFree rejects it) but a config saved before
                     * that guard existed still loads with pin.buzz == pin.iac, and
                     * on it the message contradicted the code. Report what the
                     * guard actually did instead of claiming a restore. */
                    Serial.printf("buzzpin=auto, but GPIO%u is the IAC pin — not touched, "
                                  "it is on 250Hz PWM\n", g_cfg.pin.iac);
                }
            }
            else Serial.println("X0=pullup-hiz X1=float X2=3v3 X3=gnd(beep) X9=auto");
            break;
        default:
            Serial.println("commands: ? | M | P[IAC <pin>|O<n> <pin>|TFTS/TFTM/TFTC/TFTD <pin>|BZ <pin>|TFT 0|1|RESET|WIPE] | Q[F|E <mv>|M <mpg>|T <gal>] | F[onTempF|A|1|0] | E[offTempF] | I[duty|A|F] | T[targetRpm] | Y[fanOut 1-7|0] | S[shiftRpm] | O<n>[0|1|T<f>|R<rpm>] | A<n>[0|H<v>|L<v>|O<k> <v>|F <v>|<v>] | R | W[0|1|idle|maxrpm|clt|mat|batt|map|afr|hold|warnout|help] | B[0|1|T]");
            break;
    }
}

static bool pinOk(uint8_t p) {
    if (p == 0 || p == 1 || p == 3) return false;          // boot strap / UART0 console
    if (p == 2) return false;                               // GAS_CS (hardcoded, :30)
    if (p >= 6 && p <= 11) return false;                    // flash pins / dead
    if (p == 20 || p == 24) return false;                   // dead
    if (p >= 28 && p <= 31) return false;                   // dead
    /* Input-only GPIOs on the bare ESP32. 37 and 38 were MISSING from this
     * list: they have no output driver either, so 'P IAC 37' validated, RMT/
     * LEDC attached successfully, and the IAC gate was never driven -- a
     * silently dead idle-air valve with "pin map updated" reported. */
    if (p >= 34 && p <= 39) return false;                   // 34,35,36,37,38,39
    /* Pins owned by subsystems with no way to re-point them:
     *   4  = LED_DATA_DEF, WS2812 bar. ledInit() installs RMT on it and there
     *        is NO 'P LED' command, so assigning an output here is a one-way
     *        trap: RMT and setOut() then fight over the pin and the bar can
     *        never be moved again without a reflash.
     *   5  = PIN_SPEED, the ABS/LM393 input configured for PCNT in
     *        speedInit(). Making it an output fights the open-collector
     *        comparator and the speed reading becomes garbage.
     *   15 = GAS_SCLK, 16 = GAS_MOSI, 21 = GAS_DC — the idle display's
     *        software-SPI pins are hardcoded at :28-31. */
    if (p == 4 || p == 5 || p == 15 || p == 16 || p == 21) return false;
    /* GPIO12 (MTDI) is the last strapping pin this function was letting
     * through, so the rejection message above ("no strapping ... pins")
     * contradicted the code. It is sampled at reset to pick the flash supply
     * voltage and must be LOW at boot for a 3.3 V flash; a HIGH there selects
     * 1.8 V and the next reset does not boot. Reachable as a relay target:
     *
     *   P O2 22   frees out[1] = GPIO12
     *   P O3 12   pinAliasFree(12, 2) passes -- out[1] is skip_idx, 12 is not
     *             iac/buzz/ledData/speed/gas/display and not any other out[]
     *             entry -- so O3 was moved onto a strapping pin, and O3 is a
     *             channel that gets driven HIGH by the shift/rpm modes, which
     *             is the level that can stop the next boot.
     *
     * NEW ASSIGNMENTS ONLY, deliberately. loadCfg() does not call this
     * function - it checks blob length and magic and nothing else - so a stored
     * map that already contains GPIO12 loads and runs completely unchanged,
     * and nothing rewrites it. That distinction matters here because GPIO12 IS
     * PIN_O2_DEF: out[1] is 12 in the factory default map, so any rule that
     * rejected a CONFIG containing GPIO12 would reject the default itself and
     * the installed box. This function has exactly one call site, the `P`
     * command's pin-assignment arm, so adding the pin here gates new commands
     * and nothing else. `P O2 12` no longer restores it, but `P RESET`
     * (g_cfg.pin = PinMap{}) still does.
     *
     * Note the limitation this does NOT fix: because 12 is the default out[1],
     * a stored map can still hold it and still drive it HIGH via outMode[1] =
     * OM_MAN. Removing 12 from the factory map is a hardware-visible change to
     * PIN_O2_DEF and is not done here. */
    if (p == 12) return false;
    return p <= 39;
}

/* Aliasing check: pinOk() only proves a pin is legal and free of the
 * hardcoded pins above. It does not stop an output from being re-pointed onto
 * ANOTHER output, onto the LEDC-driven IAC pin, or onto the buzzer pin. Those
 * all write the same output register, so setOut() (digitalWrite) and
 * setIac()/updateBuzzer() (LEDC/direct) end up driving one wire at ~250 Hz
 * from two places — last writer wins, so a load the firmware believes is OFF is
 * energised most of the time. Two commands cause it: 'P O2 13' (13 is out[0]'s
 * default) or 'P IAC 13'.
 *
 * NOTE the buzzer is active-LOW, which makes 'P BZ 13' worse than a fight: it
 * holds out[0] HIGH for 900 ms and LOW for 100 ms, forever — an indicator relay
 * stuck on from a plausible typo.
 *
 * This validates the PROPOSED pin against everything it must not collide with.
 * Returns false if p is already used by iac/buzz/ledData/speed/gas, by any of
 * the four remappable display pins, or by any other out[] entry other than
 * out[skip_idx]. */
/* Which PinMap field the caller is about to overwrite, so pinAliasFree() can
 * tell "this pin is already mine" from "this pin is already someone else's".
 * Deliberately a separate parameter from skip_idx - see pinAliasFree(). */
static bool pinAliasFree(uint8_t p, int skip_idx, uint8_t assigning) {
    /* The iac/buzz checks used to be `p == g_cfg.pin.iac && skip_idx != -1`.
     * That conflated two different ideas behind one sentinel: -1 meant both
     * "no out[] entry is being reassigned" AND "the caller is the IAC itself".
     * Both the P IAC path and the TFT/BZ path pass -1, so those two - the two
     * that must check iac and buzz - skipped the check entirely. One command
     * from factory defaults was enough:
     *
     *   P TFTS 19
     *
     * pinOk(19) passes. 19 is not ledData, not PIN_SPEED, not a GAS_* pin, and
     * not in out[] {13,12,14,27,26,25,33} - so the call returned TRUE and
     * tftSclk became GPIO19, the IAC MOSFET gate. saveCfg() persisted it, and
     * applyPinConfig() then ran ledcAttachPin(19,0) while the GC9A01A clocked
     * that same pin, so the valve was chopped by SPI edges at 10 Hz for as long
     * as a display was attached. P TFTM/TFTC/TFTD 19 were identical, and
     * P IAC 32 (the default buzzer pin) made pin.iac == pin.buzz == 32.
     *
     * skip_idx is now only ever about out[]; `assigning` carries the iac/buzz
     * exemption. A caller may reuse the pin it already owns, but is still
     * rejected if it collides with the other one. */
    if (p == g_cfg.pin.iac  && assigning != PINASSIGN_IAC) return false;
    if (p == g_cfg.pin.buzz && assigning != PINASSIGN_BZ)  return false;
    /* The four REMAPPABLE display pins were never checked at all, in either
     * direction, so two hardware faults were one console command each:
     *
     *   P TFTM 18    tftDc is 18 by default. pinOk(18) passes, 18 is not an
     *                out[], not iac/buzz/ledData/speed and not a GAS_* constant
     *                -- so MOSI and DC became one wire and both panels decode
     *                interleaved SPI edges as garbage.
     *   P TFTC 17 then P O3 17
     *                tftCs is 17 by default. The second command was accepted,
     *                so applyPinConfig() ran pinMode(17, OUTPUT) and setOut(3)
     *                then decided the gas display's chip-select level, while
     *                every CS edge from the gas renderer hit the relay driver.
     *
     * The four hardcoded GAS_* constants below are only HALF the display's pin
     * set, and adding them alone would still be incomplete: the two renderers
     * deliberately CROSS their roles (initTft() builds
     *   Adafruit_GC9A01A(GAS_CS, GAS_DC, tftMosi, tftSclk, -1)
     * and initGasTft() builds
     *   Adafruit_GC9A01A(tftCs, tftDc, GAS_MOSI, GAS_SCLK, -1)),
     * so the idle display also clocks out of tftMosi/tftSclk and the gas
     * display also clocks out of GAS_MOSI/GAS_SCLK. The full live signal set is
     *
     *   idle display : 2, 21, tftMosi, tftSclk
     *   gas  display : tftCs, tftDc, 16, 15
     *
     * i.e. all eight pins, four hardcoded and four remappable, and every one of
     * the eight now has a check against it here.
     *
     * Same shape as the iac/buzz pair: a caller may reuse the pin it already
     * owns ('P TFTC 17' with tftCs already 17 stays an accepted no-op) but is
     * refused for the other three. Each check is gated on its OWN enum value, so
     * a mis-wired or newly added `assigning` value fails CLOSED -- the
     * self-assignment gets refused, which costs a no-op, and never opens a
     * collision. A new display pin added to PinMap later must be added here in
     * the same commit; this is the only place that knows the display's pins. */
    if (p == g_cfg.pin.tftSclk && assigning != PINASSIGN_TFTS) return false;
    if (p == g_cfg.pin.tftMosi && assigning != PINASSIGN_TFTM) return false;
    if (p == g_cfg.pin.tftCs   && assigning != PINASSIGN_TFTC) return false;
    if (p == g_cfg.pin.tftDc   && assigning != PINASSIGN_TFTD) return false;
    if (p == g_cfg.pin.ledData) return false;
    if (p == PIN_SPEED || p == GAS_SCLK || p == GAS_MOSI || p == GAS_CS || p == GAS_DC) return false;
    for (int i = 0; i < 7; i++) {
        if (i == skip_idx) continue;
        if (g_cfg.pin.out[i] == p) return false;
    }
    return true;
}

/* True if the firmware drives this pad for ANY purpose - a relay, the valve,
 * the buzzer, the LED bar, the ABS input, or either display. Used to decide
 * whether a pad that has fallen out of the pin map can safely be released. */
static bool pinInUseAnywhere(uint8_t p) {
    for (int i = 0; i < 7; i++) if (g_cfg.pin.out[i] == p) return true;
    if (p == g_cfg.pin.iac || p == g_cfg.pin.buzz || p == g_cfg.pin.ledData) return true;
    if (p == PIN_SPEED || p == GAS_SCLK || p == GAS_MOSI || p == GAS_CS || p == GAS_DC) return true;
    if (p == g_cfg.pin.tftSclk || p == g_cfg.pin.tftMosi) return true;
    if (p == g_cfg.pin.tftCs   || p == g_cfg.pin.tftDc)   return true;
    return false;
}

static void applyPinConfig() {
    // NOTE: no heartbeat LED — GPIO2 is the gas-gauge TFT CS (GAS_CS).
    // The old PIN_LED=2 blink yanked the display's chip-select every
    // 120ms. Board has zero spare GPIOs, so the status LED is retired.

    /* Release pads this function drove last time that the new map no longer
     * uses. Without this, `P O1 22` leaves the OLD relay pin a latched OUTPUT
     * holding whatever level it last had - a shift-light relay stays energised
     * until `P O1 <old pin>` or a reboot, and nothing else will ever bring it
     * low. outputsOff() below only writes the NEW map, so it cannot help.
     *
     * The P BZ arm already has to do exactly this by hand for the buzzer when a
     * manual pin test is live; this is the general case, and the remembered map
     * is what lets applyPinConfig() see the old assignment at all - it takes no
     * arguments and had no way to know. */
    static uint8_t s_lastOut[7]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    static uint8_t s_lastIac      = 0xFF;
    static bool    s_haveApplied = false;

    if (s_haveApplied) {
        for (uint8_t i = 0; i < 7; i++) {
            uint8_t p = s_lastOut[i];
            if (p != 0xFF && !pinInUseAnywhere(p)) {
                pinMode(p, INPUT);            // nobody drives it now; let it float
                gpio_pullup_dis((gpio_num_t)p);
            }
        }
        if (s_lastIac != 0xFF && s_lastIac != g_cfg.pin.iac && !pinInUseAnywhere(s_lastIac)) {
            ledcDetachPin(s_lastIac);
            pinMode(s_lastIac, INPUT);
        }
    }

    for (uint8_t i = 0; i < 7; i++) pinMode(g_cfg.pin.out[i], OUTPUT);
    ledcDetachPin(g_cfg.pin.iac);
    // Rotary-solenoid IAC (Toyota ISC): spec frequency 250Hz. 30Hz made the
    // rotor buzz and hold position poorly.
    ledcSetup(0, 250, 10);
    ledcAttachPin(g_cfg.pin.iac, 0);
    if (g_cfg.pin.buzz != g_cfg.pin.iac) {
        pinMode(g_cfg.pin.buzz, OUTPUT);
        gpio_pullup_en((gpio_num_t)g_cfg.pin.buzz);   // hold base high through resets (inverted: HIGH=silent)
        digitalWrite(g_cfg.pin.buzz, HIGH);           // silent at boot
    }
    outputsOff();
    setIac(0);

    for (uint8_t i = 0; i < 7; i++) s_lastOut[i] = g_cfg.pin.out[i];
    s_lastIac      = g_cfg.pin.iac;
    s_haveApplied  = true;
}

void setup() {
    Serial.begin(115200);
    delay(200);

    /* ---- Loop watchdog -------------------------------------------------
     *
     * There was NO watchdog of any kind before this. The TWDT was never
     * initialised: arduino-esp32's loopTask() calls esp_task_wdt_reset() but
     * only behind loopTaskWDTEnabled, which app_main() sets to false, and the
     * core never calls esp_task_wdt_init() itself. So a hung loop() was
     * indistinguishable from a running one. The box would stop driving the fan
     * relay, the IAC and the buzzer, stop sending the 10 Hz status frame, stop
     * the ABS speed window and stop the gas log - and from the driver's seat
     * the gauges would simply freeze, with nothing indicating anything was
     * wrong. That is the failure this is here to catch.
     *
     * TIMEOUT = 15 s, and why that number is not marginal. Worst legitimate
     * single pass, itemised from this file's own measured costs:
     *
     *   max( 365, 300 ) + 200 + 300 + 20  ~=  885 ms, call it ~1 s
     *
     *   365 ms  the 10 Hz display block, updateDisplay() + updateGasDisplay()
     *           in one pass. ~180 ms per full-frame-equivalent (the measured
     *           cost of fillScreen() over software SPI, cited at the
     *           tftReinitStep comment), x2 = 360, plus ~5 ms for the two annuli
     *           a low-fuel transition adds: updateGasDisplay() draws r=118 and
     *           r=119 1px circles, ~1490 px between them, at the ~3.1 us/px
     *           implied by 57600 px = 180 ms.
     *   300 ms  tftReinitStep(): begin() is the worst single step. This is a
     *           max(), NOT a sum with the 365 above, and provably so: the
     *           display guard requires s_tftStep == TFT_IDLE, and
     *           tftReinitStep() returns immediately in TFT_IDLE. The two
     *           expensive paths can never share a pass.
     *   200 ms  gasLogUpdate() -> gasLogSave() on a new lifetime extreme: an
     *           NVS commit, i.e. a flash erase. Not measured in this file.
     *   300 ms  handleCommand(), one command per pass: worst is `P WIPE`, whose
     *           g_prefs.clear() erases the whole namespace. No command calls
     *           initTft()/initGasTft() synchronously, so a command does not
     *           also drag in a ~600 ms display block.
     *    20 ms  everything else: ADC latch, decodeOutpc(), updateOutputs(),
     *           the 100 ms espnowSendStatus(), delay(2).
     *
     * 15 s / 1 s = 15x. Even if all three unmeasured items are 4x pessimistic
     * (a ~2.5 s worst pass) it is still 6x. This API takes SECONDS, so 15 is
     * the next step above 10; the sdkconfig default of 5 s would be only 5x
     * over the estimate and 2x over the pessimistic case, which is too tight
     * for a box that must never reset itself in normal use.
     *
     * BOOT SAFETY: armed here, at the top of setup(), before the ~1 s of
     * initTft()/initGasTft(). The countdown starts at esp_task_wdt_add(), and
     * the core only feeds inside loopTask's for(;;), so setup() spends ~1 s of
     * its 15 s budget unfed: worst first pass ~1.9 s against 15 s, ~8x. If
     * setup() ever did exceed 15 s it would reset-loop, and at that point that
     * IS a hang worth recovering from. Honest limit: a hang inside setup() is
     * only caught because it costs more than the whole timeout - loopTask is
     * not fed during setup(), so nothing shorter is detectable there.
     *
     * RECOVERY: panic=true, so a timeout runs the panic handler, which with
     * CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT=y prints the reason and backtrace
     * on the console and REBOOTS the chip (PANIC_PRINT_HALT is not set). It is
     * a software reset, milliseconds - not a power cycle.
     *
     * RESET SAFETY: nothing needs saving first, and nothing safe can be forced
     * first. Every persistent value is written to NVS at the point it changes
     * (saveCfg() on each config change, gasLogSave() on a new extreme), so a
     * reset loses nothing a power cycle would not also lose, and nothing here
     * lives in RTC memory. The TWDT has no callback and a panic handler runs
     * with interrupts off on an arbitrary core, where digitalWrite() is not
     * safe - so there is no pre-reset "park the pins" step, and none is faked
     * here. A reset IS a power cycle from the actuators' point of view: the
     * GPIO matrix resets and the pins float until applyPinConfig() runs, and
     * since setFan() writes HIGH to turn the fan ON, the drivers are
     * active-HIGH, so a floating input de-energises the relay and drops the
     * IAC gate to 0%. That is the state the box powers up in on every power
     * cycle today; the watchdog does not introduce it, it only makes it
     * reachable mid-drive. Bounding that ~1 s float window needs
     * applyPinConfig() (or a panic handler), both outside setup()/loop(), so
     * it is reported rather than faked.
     *
     * SIDE EFFECT: none on this IDF, contrary to what this comment used to
     * claim. It previously said that initialising the TWDT also subscribes the
     * CPU0 idle task via CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0=y, and that a
     * busy-spin in loop() would therefore still be caught by the idle task.
     * That is IDF 5.x behaviour. On IDF 4.4 (what arduino-esp32 2.0.17 ships)
     * esp_task_wdt_init() does NOT subscribe the idle task. Verified against
     * the prebuilt object this firmware links, not from the header:
     *
     *   $ xtensa-esp32-elf-objdump -dr task_wdt.c.obj \
     *       | awk '/^[0-9a-f]+ <.*>:/ {fn=$2} /IdleTaskHandle/ {print fn}' | sort -u
     *     <esp_task_wdt_add>:
     *     <esp_task_wdt_delete>:
     *
     * Only esp_task_wdt_add and esp_task_wdt_delete touch the idle task handle;
     * esp_task_wdt_init has zero references to it. (Arduino's own
     * enableIdleWDT(), which this firmware does not call, is what would
     * subscribe it.)
     *
     * The SAFETY property the old comment claimed is still true, by a different
     * mechanism: a busy-spin in loop() never returns to the top of loopTask(),
     * so it never reaches esp_task_wdt_reset() - and loopTask is itself
     * subscribed to the TWDT by enableLoopWDT() below. Its own 15 s timer is
     * what fires. There is one watchdog on loopTask, not two nets.
     */
    const esp_err_t wdtInit = esp_task_wdt_init(15, /*panic=*/true);
    enableLoopWDT();
    /* Confirm the subscribe actually took. enableLoopWDT() sets its feeding flag
     * only on ESP_OK and reports failure with log_e(), which CORE_DEBUG_LEVEL=0
     * compiles away - so without this check a failed subscribe would leave the
     * box silently watchdog-less, i.e. still exactly the bug being fixed.
     * NULL == the calling task == loopTask. */
    const esp_err_t wdtSub = esp_task_wdt_status(NULL);
    if (wdtSub != ESP_OK) {
        Serial.printf("WDT: LOOP WATCHDOG INACTIVE (init=%d sub=%d) - a hung loop will NOT self-recover.\n",
                      (int)wdtInit, (int)wdtSub);
    }

    // Do not ignore this. A failed NVS mount made every later put/get a no-op,
    // so the box would run on defaults and quietly DISCARD every setting the
    // user made for the rest of the session, with nothing on the console.
    if (!g_prefs.begin(kPrefsName, false)) {
        s_nvsOk = false;
        Serial.println("FATAL: NVS 'iobox' failed to open — settings cannot be saved or loaded.");
        Serial.println("      Running on defaults. All changes this session will be LOST.");
        Serial.println("      Check free flash and that the NVS partition exists.");
    }
    loadCfg();
    gasLogLoad();
    // Re-score the log's percentages against the table that was actually loaded.
    // Normally a no-op (they were scored against this same table when written),
    // but loadCfg() can have just replaced the table wholesale — a magic bump or
    // a corrupt blob falls back to defaults while the gas log survives in its own
    // NVS key. Without this the float-history band is drawn from percentages that
    // describe a calibration that no longer exists, and nothing would correct it
    // until the tank next reached a new lifetime extreme.
    if (s_nvsOk && gasLogRescore()) gasLogSave();
    loadDashMac();
    loadDiagMac();
    loadAnPol();

    // 0xC0 command queue: espnowRecv only enqueues; loop() drains+executes.
    s_cmdQ = xQueueCreate(CMD_Q_SLOTS, CMD_Q_LEN);

    applyPinConfig();
    initTft();
    initGasTft();

    for (uint8_t i = 0; i < 4; i++) {
        pinMode(ADC_PINS[i], INPUT);
        analogSetPinAttenuation(ADC_PINS[i], ADC_11db);
    }

    espnowInit();

    ledInit();

    speedInit();

    Serial.println("MS2/Extra I/O box v3 (iobox3) ready — ESP-NOW link to dash. Type ? for status.");
    Serial.println("boot link=espnow proto=ms2");
}

void loop() {
    s_canFresh = s_anyGroupSeen && (millis() - s_lastFrameMs) < FAILSAFE_MS;
    decodeOutpc();
    // Sample the fuel sender on a FIXED cadence, before any consumer, so every
    // reader below (0xB0 frame, gas log, gas display, low-fuel buzzer test,
    // Serial status) maps the same sample and they can never disagree.
    //
    // Fixed cadence, not once per loop() pass. Two reasons:
    //
    //  - gasDamp has to mean a time constant, and that only holds if the sample
    //    RATE is fixed. Once per pass made it (gasDamp+1) x loop_period, and
    //    this same batch made loop_period wildly variable: a TFT re-init step
    //    runs ~300 ms, and gasLogUpdate can commit to NVS (a flash erase) on a
    //    new extreme. The damping would change by 5x depending on whether the
    //    display was being re-initialised.
    //  - Per pass it also over-sampled. Before the split, the EMA advanced only
    //    when a consumer asked, which on a running car was ~30-40 advances/s.
    //    Once per pass is ~100-160/s, so the on-car default gasDamp=5 — chosen
    //    on 2026-09-21 specifically to tame sender jitter — silently went from
    //    a ~170 ms time constant to ~40-60 ms, i.e. 3-5x LESS smoothing, and the
    //    needle started showing the jitter the setting exists to remove.
    //
    // 20 Hz: (gasDamp+1) x 50 ms = a 300 ms time constant at the on-car default.
    // Slightly slower than the accidental old rate, which is the right direction
    // for a fuel gauge, and now it is a number you can reason about.
    static uint32_t gasLast = 0;
    if (millis() - gasLast >= GAS_SAMPLE_MS) {
        gasLast = millis();
        gasSampleMv();
    }
    updateAnalogLatch();
    updateEngineProfile();
    updateOutputs();
    updateBuzzer();

    uint32_t now = millis();

    static uint32_t dashLast = 0;
    if (now - dashLast >= DASH_TX_MS) {
        uint32_t elapsed = now - dashLast;
        dashLast = now;
        speedTick(elapsed);
        espnowSendStatus();
    }

    static uint32_t tftLast = 0;
    // Skip the 10 Hz redraw while a re-init is in flight. Between alloc and
    // begin() the object exists but the panel is unconfigured, and drawValue()
    // would happily issue drawPixel()s into it — slow at best. updateDisplay()
    // already guards on s_tft == nullptr, which covers the post-teardown window
    // but NOT this one.
    if (g_cfg.tftEnable && now - tftLast >= 100 && s_tftStep == TFT_IDLE) {
        tftLast = now;
        updateDisplay();
        updateGasDisplay();
    }

    // Advance a queued TFT re-init by ONE step. Placed AFTER updateOutputs() so
    // the actuators have already been written with this tick's data, and one
    // step per pass so the worst single block (begin(), ~300 ms) stays well
    // inside the 500 ms failsafe window. See tftReinitStep() for why this was
    // worth splitting at all.
    tftReinitStep();

    static uint32_t gasLogLast = 0;
    if (now - gasLogLast >= 100) {
        gasLogLast = now;
        gasLogUpdate();       // independent of TFT: logs forever, even TFT off
    }

    // Drain queued 0xC0 commands (executed in loop context, not WiFi task).
    //
    // ONE command per pass, not a while-drain of the whole queue. A full drain
    // of 6 slots runs every queued handler back-to-back with no actuator update
    // and no s_canFresh re-evaluation in between, so the worst case was 6x any
    // one command's blocking cost — which is how one pin-map change could stall
    // the loop for seconds. One per pass bounds it to one command, and the
    // queue still empties at the loop rate (~2 ms + work), so nothing backs up.
    if (s_cmdQ) {
        char cmd[CMD_Q_LEN];
        if (xQueueReceive(s_cmdQ, cmd, 0) == pdTRUE) {
            s_cmdFromLink = true;      // UART-only commands must be refused here
            handleCommand(String(cmd));
            s_cmdFromLink = false;
            // Diag reply channel: ack every OTA command; full state after '?'.
            espnowSendAck(cmd);
            if (!strcmp(cmd, "?")) espnowSendSnapshot();
        }
    }

    if (Serial.available()) {
        static String line;
        static bool badLine = false;
        while (Serial.available()) {
            char ch = (char)Serial.read();
            if (ch == '\n' || ch == '\r') {
                // '\r' OR '\n' ends a line: some tools (minicom, screen)
                // send CR-only, which used to be ignored -> a probe got NO
                // reply forever. GPIO3 also floats when no USB host is
                // attached and picks up harness noise as random bytes — those
                // junk lines used to trigger "?)" hints / beep / help spam.
                // Discard lines with non-printables and over-long lines; answer
                // clean ones with a prompt so any probe always gets an ack.
                if (!badLine && line.length()) { handleCommand(line); Serial.print("> "); }
                line = "";
                badLine = false;
            } else {
                // badLine is a latch: only the terminator above clears it, so
                // once a line is flagged, none of its remaining bytes can run.
                if (ch < 32 || ch > 126) badLine = true;   // non-printable = noise
                // Over-long line: empty the buffer AND leave it flagged. Emptying
                // the buffer alone used to clear the flag too, so the 129th byte
                // started a fresh line and the whole SUFFIX after it was handed
                // to handleCommand() on the next \n -- e.g. 128 printable bytes,
                // then "W 0", then \n, ran "W 0" and disarmed the warning system.
                // Discarding the prefix but keeping the suffix is not "dropping
                // the flood", so this flags the line bad and keeps it bad.
                else if (line.length() >= 128) { line = ""; badLine = true; }
                else line += ch;
            }
        }
    }

    delay(2);
}

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Preferences.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>

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

static constexpr uint8_t  CAN_GROUP_COUNT = 9;   // outpc groups (72 bytes) carried in 0xA0
static constexpr uint16_t DASH_TX_MS = 100;
static constexpr uint32_t FAILSAFE_MS = 500;
// A1-A4 input dividers: 100k top (signal->pin) + 4.6k bottom (pin->GND).
// readAnalogMv() reports SOURCE mV: pin_mV * (Rtop+Rbot)/Rbot.
static constexpr float    ADC_R_TOP_OHM = 100000.0f;   // 2026-08-22: front-end rework — all channels 100k series
static constexpr float    ADC_R_BOT_OHM = 4600.0f;
static constexpr float    ADC_DIVIDER   = (ADC_R_TOP_OHM + ADC_R_BOT_OHM) / ADC_R_BOT_OHM;

enum OutMode : uint8_t { OM_OFF = 0, OM_MAN = 1, OM_TEMP = 2, OM_RPM = 3 };

// ESP-NOW link to the dash (replaces the CAN bus). Frame protocol:
//   0xA0 dash -> iobox3: [0xA0][maskLo][maskHi][72B outpc]  = 75B @10Hz
//   0xB0 iobox3 -> dash: [0xB0][anLatch][0][seq][warn][0][0][0] = 8B @10Hz
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
struct Cfg {
    uint16_t magic = CFG_MAGIC;
    PinMap pin;
    uint8_t proto = 0;               // kept for NVS layout compat (always MS2/UART link)
    bool    tftEnable = true;
    int16_t fanOnTemp = 2000;
    int16_t fanOffTemp = 1900;
    bool    fanAuto = true;
    bool    fanManual = false;
    int16_t iacTargetRpm = 900;
    uint8_t iacFailDuty = 0;
    bool    iacAuto = true;
    bool    iacFollow = false;
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
    bool    bootTest = false;    // AE111 install: ACC-fed boots stay quiet
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
    uint8_t  gasDamp = 0;       // EMA smoothing 0(raw)..15 — tames small-signal sender jitter
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
static const char* const kPrefsName = "iobox";

static uint8_t  s_outpc[72];
static bool     s_groupSeen[9] = { false, false, false, false, false, false, false, false, false };
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
    for (uint8_t i = 0; i < 9; i++) s_groupSeen[i] = false;
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
    uint32_t mv = (uint32_t)(analogRead(ADC_PINS[i]) * 3300.0f / 4095.0f * ADC_DIVIDER);
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
        int16_t mv = (int16_t)readAnalogMv(i);
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

static void handleCommand(const String& line);
static bool pinOk(uint8_t p);
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
//   0xB0 iobox3 -> dash: [0xB0][anLatch][0][seq][warn][0][0][0] = 8B @10Hz
//   0xC0 dash -> iobox3: [0xC0][len][cmd...]
// Both devices stay on a fixed channel (1) so no AP association is required.
// ---------------------------------------------------------------------------
static uint8_t  s_anLatchByte = 0;
static uint8_t  s_seq = 0;
static uint16_t s_warnLatched = 0;   // forward decl (real def below)
static int gasPercent();

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
            portEXIT_CRITICAL(&s_outpcMux);
            for (uint8_t i = 0; i < 9; i++) s_groupSeen[i] = (mask & (1u << i)) != 0;
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
    // Extended v4: +[15]=fanMode(0/1/2) +[16]=iacMode(0/1/2) +[17]=buzzerOn +[18]=bootTestOn
    uint8_t f[19] = {0};
    f[0] = FRAME_STATUS;
    uint8_t latch = 0;
    for (uint8_t i = 0; i < 4; i++) if (s_anLatch[i]) latch |= (1u << i);
    f[1] = latch;
    f[3] = ++s_seq;
    f[4] = (uint8_t)(s_warnLatched & 0xFF);
    f[5] = (uint8_t)constrain(gasPercent(), 0, 100);
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
    f[18] = g_cfg.bootTest ? 1 : 0;
    esp_err_t r = esp_now_send(ESP_NOW_BROADCAST, f, sizeof(f));
    s_txB0Count++;
    if (r != ESP_OK) {
        Serial.printf("espnow send B0 FAILED: %s\n", esp_err_to_name(r));
    }
}

// 0xD0 reply channel for the diag module (and any future console peer).
// Sent from loop() context immediately after a queued OTA command executes —
// never inside the ESP-NOW RX callback. Purely additive: A0/B0 formats and
// the outpc receive path are untouched.
//   [D0][len][cmd...]        ack + echo of the command that just ran
//   [D0][0x01][23B snapshot] machine-readable state, sent after a remote '?'
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
    // v3: +[23]=fanMode(0/1/2) +[24]=iacMode(0/1/2) +[25]=bootTestOn +[26]=reserved
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
    f[25] = g_cfg.bootTest ? 1 : 0;
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
    // BE16 pair across two different broadcasts.
    uint8_t local[72];
    portENTER_CRITICAL(&s_outpcMux);
    memcpy(local, s_outpc, sizeof(local));
    portEXIT_CRITICAL(&s_outpcMux);

    if (s_groupSeen[0]) g_rpm  = rdU16(local, 6);
    if (s_groupSeen[2]) {
        g_map  = rdS16(local, 18);
        g_mat  = rdS16(local, 20);
        g_clt  = rdS16(local, 22);
    }
    if (s_groupSeen[3]) {
        g_tps  = rdS16(local, 24);
        g_batt = rdS16(local, 26);
        g_afr  = rdS16(local, 28);
    }
    if (s_groupSeen[6]) g_iacStep = rdS16(local, 54);
}

// ---- CAN-less simulation -------------------------------------------------
// Fills the same s_outpc buffer the CAN RX path writes, so decodeOutpc,
// engine profile, outputs, dash 0x710 and the round display all run as if an
// ECU were broadcasting. G1 enables, G0 disables.
static bool s_simActive = false;

struct SimWp {
    uint32_t t;      // ms from cycle start
    uint16_t rpm;
    int16_t  tps;    // % x10
    int16_t  map;    // kPa x10
    int16_t  clt;    // F x10
    int16_t  mat;    // F x10
    int16_t  batt;   // V x10
    int16_t  afr;    // AFR x10
    uint8_t  iac;    // raw 0-255
};

static const SimWp kSimWp[] = {
    {     0,  800,    0,  400, 1800, 1200, 142, 145,  95 }, // idle warm-up
    {  6000, 4500,  800, 1100, 1850, 1250, 138, 135,  40 }, // accel / cruise
    { 12000, 8500,  950, 3000, 1950, 1450, 132, 115,  10 }, // boost pull -> OVERREV
    { 18000, 7500,  900, 2600, 2050, 1600, 130, 108,  10 }, // hold, MAT rising
    { 24000, 6500,  850, 2800, 2400, 1750, 126,  98,  10 }, // OVERHEAT + RICH
    { 30000, 1000,    0,  600, 2100, 1300, 122, 185,  80 }, // lift-off decel (LEAN gated by TPS)
    { 36000,  900,    0,  500, 2050, 1250, 105, 142,  95 }, // battery sag -> LOWBATT
    { 42000,  600,    0,  450, 1950, 1200, 145, 147, 100 }, // idle hunting low -> IDLELO
    { 48000, 1200,    0,  450, 1900, 1200, 145, 147,  90 }, // idle hunting high -> IDLEHI
    { 60000,  800,    0,  400, 1850, 1150, 142, 145,  95 }, // settle, loop back
};
static constexpr uint32_t SIM_CYCLE_MS = 60000;

// Boot self-test: after a settle delay, run the sim for a few seconds as a
// power-on hardware check (fan channel, display, gauge). CLT is swept from the
// fan off-point past the on-point so the real fan hysteresis turns the fan on
// regardless of what thresholds are configured.
static constexpr uint32_t BOOT_SETTLE_MS = 1000;
static constexpr uint32_t BOOT_TEST_MS  = 3000;
static bool     s_bootTestArmed = false;
static bool     s_bootTestOn    = false;
static uint32_t s_bootTestStartMs = 0;

static void simInject() {
    if (s_bootTestOn) {
        // Boot self-test profile: sweep CLT from the fan off-point to a few
        // degrees past the on-point so the fan hysteresis spins the fan for the
        // tail of the window, whatever the configured setpoints are. Other
        // channels stay calm so nothing else fires.
        uint32_t since = millis() - s_bootTestStartMs;
        uint32_t t = since > BOOT_SETTLE_MS ? since - BOOT_SETTLE_MS : 0;
        if (t > BOOT_TEST_MS) t = BOOT_TEST_MS;
        int32_t cltLo = g_cfg.fanOffTemp;
        int32_t cltHi = g_cfg.fanOnTemp + 80;               // +8.0F
        int32_t clt = cltLo + (cltHi - cltLo) * (int32_t)t / (int32_t)BOOT_TEST_MS;
        uint16_t rpm = 900;

        s_outpc[6] = (uint8_t)(rpm >> 8);  s_outpc[7]  = (uint8_t)(rpm & 0xFF);
        s_outpc[18] = 0x01; s_outpc[19] = 0x90;             // map 40.0 kPa
        s_outpc[20] = 0x04; s_outpc[21] = 0xB0;             // mat 120.0 F
        s_outpc[22] = (uint8_t)(clt >> 8); s_outpc[23] = (uint8_t)(clt & 0xFF);
        s_outpc[24] = 0x00; s_outpc[25] = 0x00;             // tps 0%
        s_outpc[26] = 0x00; s_outpc[27] = 0x8E;             // batt 14.2 V
        s_outpc[28] = 0x00; s_outpc[29] = 0x91;             // afr 14.5
        s_outpc[54] = 0; s_outpc[55] = 95;                  // iacstep idle
        s_groupSeen[0] = s_groupSeen[2] = s_groupSeen[3] = s_groupSeen[6] = true;
        s_anyGroupSeen = true;
        s_lastFrameMs = millis();
        return;
    }

    uint32_t t = millis() % SIM_CYCLE_MS;
    uint8_t n = sizeof(kSimWp) / sizeof(kSimWp[0]);
    uint8_t i = 0;
    for (uint8_t j = 0; j + 1 < n; j++) {
        if (t < kSimWp[j + 1].t) { i = j; break; }
    }
    const SimWp& a = kSimWp[i];
    const SimWp& b = kSimWp[i + 1];
    uint32_t span = b.t - a.t;
    uint32_t dt = t - a.t;

    auto lerp = [&](int32_t av, int32_t bv) -> int32_t {
        return av + (int32_t)((bv - av) * (int32_t)dt) / (int32_t)span;
    };
    uint16_t rpm = (uint16_t)lerp(a.rpm, b.rpm);
    int16_t  tps  = (int16_t)lerp(a.tps,  b.tps);
    int16_t  map  = (int16_t)lerp(a.map,  b.map);
    int16_t  clt  = (int16_t)lerp(a.clt,  b.clt);
    int16_t  mat  = (int16_t)lerp(a.mat,  b.mat);
    int16_t  batt = (int16_t)lerp(a.batt, b.batt);
    int16_t  afr  = (int16_t)lerp(a.afr,  b.afr);
    uint8_t  iac  = (uint8_t)lerp(a.iac,  b.iac);

    s_outpc[6] = (uint8_t)(rpm >> 8);  s_outpc[7]  = (uint8_t)(rpm & 0xFF);
    s_outpc[18] = (uint8_t)(map >> 8); s_outpc[19] = (uint8_t)(map & 0xFF);
    s_outpc[20] = (uint8_t)(mat >> 8); s_outpc[21] = (uint8_t)(mat & 0xFF);
    s_outpc[22] = (uint8_t)(clt >> 8); s_outpc[23] = (uint8_t)(clt & 0xFF);
    s_outpc[24] = (uint8_t)(tps >> 8); s_outpc[25] = (uint8_t)(tps & 0xFF);
    s_outpc[26] = (uint8_t)(batt >> 8); s_outpc[27] = (uint8_t)(batt & 0xFF);
    s_outpc[28] = (uint8_t)(afr >> 8); s_outpc[29] = (uint8_t)(afr & 0xFF);
    s_outpc[54] = 0; s_outpc[55] = iac;   // iacstep is int16 big-endian (0-255)

    s_groupSeen[0] = s_groupSeen[2] = s_groupSeen[3] = s_groupSeen[6] = true;
    s_anyGroupSeen = true;
    s_lastFrameMs = millis();
}

static void simStop() {
    s_simActive = false;
    memset(s_outpc, 0, sizeof(s_outpc));
    for (bool& b : s_groupSeen) b = false;
    s_anyGroupSeen = false;
    g_rpm = g_map = g_mat = g_clt = g_tps = g_batt = g_afr = g_iacStep = 0;
}

// Boot self-test: after a settle delay, run the sim for a few seconds as a
// power-on hardware check (fan channel, display, gauge).
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
    bool cltOk = s_groupSeen[2] && g_clt > 100 && g_clt < 3500;
    bool matOk = s_groupSeen[2] && g_mat > 0 && g_mat < 3000;
    bool onThrottle = s_groupSeen[3] && g_tps >= 50;
    bool afrOk = onThrottle && g_afr >= 100 && g_afr <= 250;

    // Partial-frame hardening: only trust a field if its group was present in
    // the current frame. On a real 0xA0 broadcast all 9 groups arrive together,
    // but if a frame is ever partial, g_* values from a previous frame would be
    // stale here and could misfire a warning / mis-drive an actuator.
    bool rpmOk = s_groupSeen[0];
    bool tpsOk = s_groupSeen[3];
    bool battOk = s_groupSeen[3];

    if (rpmOk && g_rpm >= g_cfg.eng.maxRpm) raw |= W_OVERREV;
    if (rpmOk && g_rpm > 0 && g_rpm < g_cfg.eng.idleRpmMin && tpsOk && g_tps < 200) raw |= W_IDLE_LO;
    if (rpmOk && g_rpm > 0 && g_rpm > g_cfg.eng.idleRpmMax && tpsOk && g_tps < 200) raw |= W_IDLE_HI;
    if (cltOk && g_clt > g_cfg.eng.cltMax) raw |= W_OVERHEAT;
    if (matOk && g_mat > g_cfg.eng.matMax) raw |= W_HOTAIR;
    if (battOk && g_batt > 0 && g_batt < g_cfg.eng.battMin) raw |= W_LOWBATT;
    if (battOk && g_batt > 0 && g_batt > g_cfg.eng.battMax) raw |= W_HIBATT;
    if (s_groupSeen[2] && g_map > g_cfg.eng.mapMax) raw |= W_OVERBOOST;
    if (afrOk && g_afr > g_cfg.eng.afrHigh) raw |= W_LEAN;
    if (afrOk && g_afr < g_cfg.eng.afrLow) raw |= W_RICH;
    return raw;
}

static void updateEngineProfile() {
    s_warnRaw = engineWarnFlags();
    if (s_warnRaw) {
        if (!s_warnLatched) s_warnFirstMs = millis();
        s_warnLatched = s_warnRaw;
    } else if (s_warnLatched && (millis() - s_warnFirstMs) >= g_cfg.eng.warnHoldMs) {
        s_warnLatched = 0;
    }
}

static void updateOutputs() {
    if (!s_canFresh) {
        outputsOff();
        setIac(g_cfg.iacFailDuty);
        return;
    }
    int16_t clt = g_clt;
    bool cltOk = s_groupSeen[2] && clt > 100 && clt < 3500;

    bool fanOn = false;
    if (g_cfg.fanAuto) {
        if (cltOk) {
            static bool fanState = false;
            if (!fanState && clt >= g_cfg.fanOnTemp) fanState = true;
            else if (fanState && clt <= g_cfg.fanOffTemp) fanState = false;
            fanOn = fanState;
        }
    } else {
        fanOn = g_cfg.fanManual;
    }
    fanOn = fanOn || inputForces(7);
    setFan(fanOn);

    for (uint8_t i = 0; i < 7; i++) {
        if ((i + 1) == g_cfg.fanOut) continue;
        bool on = false;
        switch (g_cfg.outMode[i]) {
            case OM_OFF:  on = false; break;
            case OM_MAN:  on = g_cfg.outManual[i]; break;
            case OM_TEMP: on = cltOk && clt >= g_cfg.outTemp[i]; break;
            case OM_RPM:  on = s_groupSeen[0] && g_rpm >= g_cfg.outRpm[i]; break;
        }
        on = on || inputForces(i + 1);
        setOut(i, on);
    }

    if (g_cfg.eng.warnOut >= 1 && g_cfg.eng.warnOut <= 7 &&
        g_cfg.eng.warnOut != g_cfg.fanOut && s_warnLatched) {
        bool blink = ((millis() / 500) & 1) == 0;
        setOut(g_cfg.eng.warnOut - 1, blink);
    }

    uint8_t duty;
    if (g_cfg.iacFollow && s_groupSeen[6]) {
        duty = (uint8_t)constrain((int16_t)(g_iacStep * 100 / 255), 0, 100);
    } else if (g_cfg.iacAuto || (g_cfg.iacFollow && !s_groupSeen[6])) {
        if (!cltOk) {
            duty = interpolateIac(120);
        } else {
            duty = interpolateIac(clt / 10);
            // Trim idle towards the target rpm, but only if the current frame
            // actually carried RPM — otherwise g_rpm is stale and the trim is
            // garbage. Partial-frame hardening (see engineWarnFlags).
            if (s_groupSeen[0]) {
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

static void updateBuzzer() {
    static uint32_t last = 0;
    uint32_t now = millis();
    if (s_buzzManual) return;       // manual pin test in progress
    if (now - last < 100) return;   // 10 Hz — matches display cadence
    last = now;
    bool on;
    if (s_buzzLoop) {
        on = (now % 1000) < 100;    // continuous 100ms/900ms backup alarm
    } else if (now < s_buzzTestUntilMs) {
        on = true;                   // B T test beep
    } else if (g_cfg.buzzerEnable) {
        bool lowFuel = gasPercent() <= g_cfg.lowFuelPct;
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
    char buf[12];

    uint8_t duty = (uint8_t)(ledcRead(0) * 100 / 1023);
    snprintf(buf, sizeof buf, "%u%%", duty);
    drawValue(48, 48, 6, GC9A01A_CYAN, 144, s_lastDuty, buf);

    snprintf(buf, sizeof buf, "%u", s_canFresh ? g_rpm : 0);
    drawValue(40, 146, 2, s_canFresh ? GC9A01A_WHITE : GC9A01A_DARKGREY, 64, s_lastRpm, buf);

    snprintf(buf, sizeof buf, "%d", (int)g_cfg.iacTargetRpm);
    drawValue(108, 146, 2, GC9A01A_YELLOW, 100, s_lastTgt, buf);

    bool cltOk = s_groupSeen[2] && g_clt > 100 && g_clt < 3500;
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

static int gasPercent() {
    uint16_t raw = readAnalogMv(3);   // A4 = fuel sender (GPIO35)
    int32_t f;
    if (g_cfg.gasDamp == 0 || s_gasFilt < 0) f = raw;
    else f = ((int32_t)s_gasFilt * g_cfg.gasDamp + raw) / ((int32_t)g_cfg.gasDamp + 1);
    // Cap must sit ABOVE the highest reachable reported-mV (220R/3V3 front end
    // spans ~11.3k-23.5k reported) yet below int16 overflow if the sender
    // unplugs (open node rails toward 3V3 -> ~75k reported). 32k does both.
    // The old 16k cap silently froze every reading below ~half tank.
    s_gasFilt = (int16_t)constrain(f, 0, 32000);
    const uint16_t *c = g_cfg.gasCalMv;   // c[0]=FULL(low mv) .. c[4]=EMPTY(high mv)
    if (s_gasFilt <= c[0]) return 100;
    if (s_gasFilt >= c[4]) return 0;
    for (uint8_t i = 0; i < 4; i++) {
        if (s_gasFilt <= c[i + 1]) {
            int hiPct = 100 - i * 25;         // % at c[i]
            int loPct = 100 - (i + 1) * 25;   // % at c[i+1]
            if (c[i + 1] == c[i]) return hiPct;
            return loPct + (int32_t)(c[i + 1] - s_gasFilt) * (hiPct - loPct) / (c[i + 1] - c[i]);
        }
    }
    return 0;
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
static char s_lastGasLow[10] = "";
static uint16_t s_lastLowColor = 0xFFFF;
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
    s_lastGasMv[0] = s_lastGasPct[0] = s_lastGasLow[0] = 0;
    s_lastPctColor = s_lastLowColor = 0xFFFF;
}

static void updateGasDisplay() {
    if (!g_cfg.tftEnable || s_gasTft == nullptr) return;
    char buf[12];

    int target = gasPercent();
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
    s_gasTft->fillScreen(GC9A01A_BLACK);
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
        s_tft->fillScreen(GC9A01A_BLACK);
        drawGaugeFrame();
        s_lastDuty[0] = s_lastRpm[0] = s_lastTgt[0] = s_lastClt[0] = 0;
        s_lastMode[0] = s_lastStat[0] = s_lastWarn[0] = 0;
    }
}

static void saveCfg() {
    g_prefs.putBytes("cfg", &g_cfg, sizeof(g_cfg));
}

static void loadCfg() {
    size_t len = g_prefs.getBytes("cfg", &g_cfg, sizeof(g_cfg));
    if (len != sizeof(g_cfg) || g_cfg.magic != CFG_MAGIC) {
        g_cfg = Cfg{};
        saveCfg();
    }
}

static void reportStatus() {
    Serial.printf("link=espnow rx_a0=%lu rx_c0=%lu tx_b0=%lu dropped=%lu sim=%d bt=%d can_fresh=%d rpm=%u map=%.1f mat=%.1fF clt=%.1fF tps=%.1f%% batt=%.1fV afr=%.1f buz=%d\n",
                  (unsigned long)s_rxA0Count, (unsigned long)s_rxC0Count, (unsigned long)s_txB0Count,
                  (unsigned long)s_rxDroppedCount,
                  s_simActive ? 1 : 0,
                  g_cfg.bootTest ? 1 : 0,
                  s_canFresh ? 1 : 0, g_rpm,
                  g_map / 10.0f, g_mat / 10.0f, g_clt / 10.0f,
                  g_tps / 10.0f, g_batt / 10.0f, g_afr / 10.0f,
                  g_cfg.buzzerEnable ? 1 : 0);
    Serial.printf("gas=%d%% mv=%u est=%dmi damp=%u warn<=%u%% pts[F,3/4,1/2,1/4,E]=%u,%u,%u,%u,%u\n",
                  gasPercent(), readAnalogMv(3),
                  (int)((long)gasPercent() * g_cfg.tankGalX10 * g_cfg.gasMpg / 1000),
                  g_cfg.gasDamp, g_cfg.lowFuelPct,
                  g_cfg.gasCalMv[4], g_cfg.gasCalMv[3], g_cfg.gasCalMv[2],
                  g_cfg.gasCalMv[1], g_cfg.gasCalMv[0]);
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

static void handleCommand(const String& line) {
    String c = line;
    c.trim();
    if (c.length() == 0) return;
    if (c == "?") { reportStatus(); return; }

    char key = c[0];
    String val = c.substring(1);
    val.trim();

    switch (key) {
        case 'F':
            if (val == "A") { g_cfg.fanAuto = true; }
            else if (val == "1" || val == "0") {
                g_cfg.fanAuto = false;
                g_cfg.fanManual = (val == "1");
            } else if (val.length() > 0) {
                float t = val.toFloat();
                if (t >= 100.0f && t <= 280.0f) {
                    g_cfg.fanAuto = true;
                    g_cfg.fanOnTemp = (int16_t)(t * 10.0f);
                } else Serial.println("fan-on 100-280F");
            }
            saveCfg();
            break;
        case 'E': {
            float t = val.toFloat();
            if (t >= 90.0f && t <= 270.0f) g_cfg.fanOffTemp = (int16_t)(t * 10.0f);
            else Serial.println("fan-off 90-270F");
            saveCfg();
            break;
        }
        case 'I':
            if (val == "F") {
                g_cfg.iacAuto = false;
                g_cfg.iacFollow = true;
            } else if (val == "A") {
                g_cfg.iacFollow = false;
                g_cfg.iacAuto = true;
            } else if (val.length() > 0) {
                g_cfg.iacFollow = false;
                g_cfg.iacAuto = false;
                g_cfg.iacManualDuty = (uint8_t)constrain((int)val.toInt(), 0, 100);
            }
            saveCfg();
            break;
        case 'T': {
            float t = val.toFloat();
            if (t >= 500) g_cfg.iacTargetRpm = (int16_t)t;
            saveCfg();
            break;
        }
        case 'Y': {
            int k = val.toInt();
            if (k >= 0 && k <= 7) {
                uint8_t old = g_cfg.fanOut;
                g_cfg.fanOut = (uint8_t)k;
                // Drive the OLD fan output low before re-pointing, or a relay
                // latched ON under the previous fanOut stays stuck until the
                // next failsafe/outputsOff pass.
                if (old >= 1 && old <= 7) setOut(old - 1, false);
                if (k > 0) setFan(false);
            }
            saveCfg();
            break;
        }
        case 'R': {
            Serial.println("resp=off (removed)");
            break;
        }
        case 'S': {
            int rpm = val.toInt();
            if (rpm > 0) {
                g_cfg.shiftRpm = rpm;
                g_cfg.outMode[0] = OM_RPM;
                g_cfg.outRpm[0] = rpm;
            }
            saveCfg();
            break;
        }
        case 'O': {
            if (val.length() < 2) return;
            uint8_t n = (uint8_t)(val[0] - '1');
            if (n > 6) return;
            String mode = val.substring(1);
            mode.trim();
            if (mode == "0") { g_cfg.outMode[n] = OM_OFF; }
            else if (mode == "1") { g_cfg.outMode[n] = OM_MAN; g_cfg.outManual[n] = true; }
            else if (mode == "A") { g_cfg.outMode[n] = OM_OFF; }
            else if (mode.startsWith("T")) {
                g_cfg.outMode[n] = OM_TEMP;
                g_cfg.outTemp[n] = (int16_t)(mode.substring(1).toFloat() * 10.0f);
            } else if (mode.startsWith("R")) {
                g_cfg.outMode[n] = OM_RPM;
                g_cfg.outRpm[n] = (int16_t)mode.substring(1).toInt();
            } else {
                g_cfg.outMode[n] = OM_MAN;
                g_cfg.outManual[n] = (mode.toInt() != 0);
            }
            saveCfg();
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
            if (m == "0") {
                g_cfg.anEnable[n] = false;
                s_anForce[n] = -1;      // disabling clears any bench force too
            } else if (m == "D1" || m == "D0" || m == "DA") {
                // bench force: D1 latch on, D0 latch off, DA back to auto
                s_anForce[n] = m == "D1" ? 1 : (m == "D0" ? 0 : -1);
                Serial.printf("a%u force=%s\n", n + 1,
                              s_anForce[n] < 0 ? "auto" : (s_anForce[n] ? "ON" : "off"));
            } else if (m.startsWith("O")) {
                uint8_t o = (uint8_t)(m[1] - '1');
                if (o <= 6) {
                    g_cfg.anOut[n] = o + 1;
                    g_cfg.anEnable[n] = true;
                    float v = m.substring(2).toFloat();
                    if (v >= 0.1f && v <= 15.0f) g_cfg.anThresh[n] = (uint16_t)(v * 1000.0f);
                }
            } else if (m.startsWith("F")) {
                g_cfg.anOut[n] = 7;
                g_cfg.anEnable[n] = true;
                float v = m.substring(1).toFloat();
                if (v >= 0.1f && v <= 15.0f) g_cfg.anThresh[n] = (uint16_t)(v * 1000.0f);
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
            saveCfg();
            break;
        }
        case 'M': {
            // M       -> report link mode (ESP-NOW only — CAN removed)
            Serial.println("proto=ms2 link=espnow");
            break;
        }
        case 'G':
            // G              -> report sim state
            // G 1            -> CAN-less simulation on
            // G 0            -> simulation off, clear synthetic data
            if (val == "1") {
                s_simActive = true;
                simInject();
            } else if (val == "0") {
                simStop();
            }
            Serial.printf("sim=%d\n", s_simActive ? 1 : 0);
            break;
        case 'Z':
            // Z              -> report boot self-test
            // Z 1|0          -> enable/disable 3s fan+sim boot test
            if (val == "1" || val == "0") {
                g_cfg.bootTest = val == "1";
                saveCfg();
                Serial.printf("boottest=%d\n", g_cfg.bootTest ? 1 : 0);
            } else {
                Serial.printf("boottest=%d (Z 1|0 to change)\n", g_cfg.bootTest ? 1 : 0);
            }
            break;
        case 'Q':
            // Q              -> dump gas calibration + current reading
            // Q F / E / 1/2/3 -> record CURRENT A4 reading as FULL/EMPTY/1/4/HALF/3/4
            // Q D <0-15>     -> damping strength (0 = raw)
            // Q W <5-90>     -> low-fuel warning %
            // Q M <mpg>      -> set assumed mpg for est. miles
            // Q T <gal>      -> set tank capacity in gallons (e.g. Q T 13.2)
            if (val == "F" || val == "E" || val == "1" || val == "2" || val == "3") {
                // Table anchors: slot0 = FULL (low mV, 100%) … slot4 = EMPTY (high mV, 0%)
                uint8_t slot = (val == "F") ? 0 : (val == "E") ? 4 : (uint8_t)(4 - val.toInt());
                g_cfg.gasCalMv[slot] = readAnalogMv(3);
                // Recording an anchor invalidates any stale mid points from a
                // previous front end (non-monotonic table makes the piecewise
                // interp return garbage). Re-linearise 1/4..3/4 between the
                // anchors once BOTH are sane (c[0] < c[4]) — i.e. after the
                // second of SET F / SET E. Explicit Q 1/2/3 refinements still
                // win, but do them AFTER F/E or they get re-linearised.
                if (slot == 0 || slot == 4) {
                    const uint16_t *c = g_cfg.gasCalMv;
                    if (c[0] < c[4]) {
                        // mV rises as fuel DROPS: slot j sits j/4 of the span
                        // above FULL (j=1 -> 3/4 tank -> +1/4 span).
                        for (uint8_t j = 1; j < 4; j++)
                            g_cfg.gasCalMv[j] =
                                c[0] + (uint16_t)((uint32_t)(c[4] - c[0]) * j / 4);
                        Serial.println("gas mids re-linearised; refine with Q 1/2/3 after F/E");
                    }
                }
                s_gasFilt = -1;                     // reseed filter after cal change
                saveCfg();
                Serial.printf("gas point %u = %u mv\n", slot, g_cfg.gasCalMv[slot]);
                break;
            }
            if (val.startsWith("D ")) {
                int d = val.substring(2).toInt();
                if (d >= 0 && d <= 15) {
                    g_cfg.gasDamp = (uint8_t)d;
                    s_gasFilt = -1;
                    saveCfg();
                    Serial.printf("gas damp=%u\n", d);
                } else Serial.println("damp 0-15");
                break;
            }
            if (val.startsWith("W ")) {
                int w = val.substring(2).toInt();
                if (w >= 5 && w <= 90) {
                    g_cfg.lowFuelPct = (uint8_t)w;
                    saveCfg();
                    Serial.printf("low-fuel warn=%u%%\n", w);
                } else Serial.println("warn 5-90%");
                break;
            }
            if (val.startsWith("M ")) {
                int m = val.substring(2).toInt();
                if (m >= 5 && m <= 99) {
                    g_cfg.gasMpg = (uint8_t)m;
                    saveCfg();
                    Serial.printf("est-mpg=%u\n", g_cfg.gasMpg);
                } else Serial.println("mpg range 5-99");
                break;
            }
            if (val.startsWith("T ")) {
                int t = (int)(val.substring(2).toFloat() * 10.0f + 0.5f);
                if (t >= 10 && t <= 500) {
                    g_cfg.tankGalX10 = (uint16_t)t;
                    saveCfg();
                    Serial.printf("tank=%.1f gal\n", g_cfg.tankGalX10 / 10.0f);
                } else Serial.println("tank 1.0-50.0 gal");
                break;
            }
            Serial.printf("gas=%d%% mv=%u est=%dmi damp=%u warn<=%u%% mpg=%u tank=%.1fgal pts[F,3/4,1/2,1/4,E]=%u,%u,%u,%u,%u\n",
                          gasPercent(), readAnalogMv(3),
                          (int)((long)gasPercent() * g_cfg.tankGalX10 * g_cfg.gasMpg / 1000),
                          g_cfg.gasDamp, g_cfg.lowFuelPct, g_cfg.gasMpg, g_cfg.tankGalX10 / 10.0f,
                          g_cfg.gasCalMv[4], g_cfg.gasCalMv[3], g_cfg.gasCalMv[2],
                          g_cfg.gasCalMv[1], g_cfg.gasCalMv[0]);
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
                if (g_cfg.tftEnable) {
                    teardownTft();
                    initTft();
                    initGasTft();
                }
                Serial.println("pin map reset to iobox3 defaults");
                break;
            }
            if (val == "TFT 1" || val == "TFT 0") {
                g_cfg.tftEnable = (val == "TFT 1");
                saveCfg();
                teardownTft();
                initTft();
                initGasTft();
                Serial.println(g_cfg.tftEnable ? "tft on" : "tft off");
                break;
            }
            int sp = val.indexOf(' ');
            if (sp <= 0) { Serial.println("P IAC <pin> | P O<n> <pin> | P TFT 0|1 | P TFTS/TFTM/TFTC/TFTD/BZ <pin> | P DASH/DIAG <mac>|CLEAR | P RESET"); break; }
            String k = val.substring(0, sp);
            int pin = val.substring(sp + 1).toInt();
            k.trim();
            if (!pinOk(pin)) { Serial.println("pin rejected: unusable on WROOM-32"); break; }
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
                g_cfg.pin.buzz = (uint8_t)pin;
            } else {
                Serial.println("P IAC <pin> | P O<n> <pin> | P TFT 0|1 | P TFTS/TFTM/TFTC/TFTD/BZ <pin> | P DASH/DIAG <mac>|CLEAR | P RESET");
                break;
            }
            saveCfg();
            applyPinConfig();
            if (g_cfg.tftEnable) {
                teardownTft();
                initTft();
                initGasTft();
            }
            Serial.println("pin map updated");
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
            if (k == "idle" && a > 0) {
                int mn = constrain(p1.toInt(), 300, 3000);
                int mx = constrain(p2.toInt(), 300, 3000);
                if (mx < mn) { int t = mn; mn = mx; mx = t; }
                g_cfg.eng.idleRpmMin = (int16_t)mn;
                g_cfg.eng.idleRpmMax = (int16_t)mx;
            } else if (k == "maxrpm" && p1.length() > 0) {
                g_cfg.eng.maxRpm = (int16_t)constrain(p1.toInt(), 1000, 20000);
            } else if (k == "clt" && p1.length() > 0) {
                g_cfg.eng.cltMax = (int16_t)constrain((int)(p1.toFloat() * 10.0f), 1000, 3000);
            } else if (k == "mat" && p1.length() > 0) {
                g_cfg.eng.matMax = (int16_t)constrain((int)(p1.toFloat() * 10.0f), 500, 2500);
            } else if (k == "batt" && a > 0) {
                int mn = constrain((int)(p1.toFloat() * 10.0f), 50, 200);
                int mx = constrain((int)(p2.toFloat() * 10.0f), 50, 200);
                if (mx < mn) { int t = mn; mn = mx; mx = t; }
                g_cfg.eng.battMin = (int16_t)mn;
                g_cfg.eng.battMax = (int16_t)mx;
            } else if (k == "map" && p1.length() > 0) {
                g_cfg.eng.mapMax = (int16_t)constrain((int)(p1.toFloat() * 10.0f), 0, 4000);
            } else if (k == "afr" && a > 0) {
                int lo = constrain((int)(p1.toFloat() * 10.0f), 50, 250);
                int hi = constrain((int)(p2.toFloat() * 10.0f), 50, 250);
                if (hi < lo) { int t = lo; lo = hi; hi = t; }
                g_cfg.eng.afrLow = (int16_t)lo;
                g_cfg.eng.afrHigh = (int16_t)hi;
            } else if (k == "hold" && p1.length() > 0) {
                g_cfg.eng.warnHoldMs = (uint16_t)constrain(p1.toInt(), 0, 60000);
            } else if (k == "warnout" && p1.length() > 0) {
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
            if (val == "0")      { s_buzzManual = true; pinMode(g_cfg.pin.buzz, INPUT_PULLUP); Serial.println("buzzpin=input_pullup"); }
            else if (val == "1") { s_buzzManual = true; pinMode(g_cfg.pin.buzz, INPUT);         Serial.println("buzzpin=float"); }
            else if (val == "2") { s_buzzManual = true; pinMode(g_cfg.pin.buzz, OUTPUT); digitalWrite(g_cfg.pin.buzz, HIGH); Serial.println("buzzpin=high_3v3"); }
            else if (val == "3") { s_buzzManual = true; pinMode(g_cfg.pin.buzz, OUTPUT); digitalWrite(g_cfg.pin.buzz, LOW);  Serial.println("buzzpin=gnd_beep"); }
            else if (val == "9") { s_buzzManual = false; Serial.println("buzzpin=auto"); }
            else Serial.println("X0=pullup-hiz X1=float X2=3v3 X3=gnd(beep) X9=auto");
            break;
        default:
            Serial.println("commands: ? | M | G[0|1] | Z[0|1] | P[IAC <pin>|O<n> <pin>|TFTS/TFTM/TFTC/TFTD <pin>|BZ <pin>|TFT 0|1|RESET] | Q[F|E <mv>|M <mpg>|T <gal>] | F[onTempF|A|1|0] | E[offTempF] | I[duty|A|F] | T[targetRpm] | Y[fanOut 1-7|0] | S[shiftRpm] | O<n>[0|1|T<f>|R<rpm>] | A<n>[0|H<v>|L<v>|O<k> <v>|F <v>|<v>] | R | W[0|1|idle|maxrpm|clt|mat|batt|map|afr|hold|warnout|help] | B[0|1|T]");
            break;
    }
}

static bool pinOk(uint8_t p) {
    if (p == 0 || p == 1 || p == 3) return false;          // boot strap / UART0 console
    if (p == 2) return false;                               // onboard LED
    if (p >= 6 && p <= 11) return false;                    // flash pins / dead
    if (p == 20 || p == 24) return false;                   // dead
    if (p >= 28 && p <= 31) return false;                   // dead
    if (p == 34 || p == 35 || p == 36 || p == 39) return false; // input-only
    return p <= 39;
}

static void applyPinConfig() {
    // NOTE: no heartbeat LED — GPIO2 is the gas-gauge TFT CS (GAS_CS).
    // The old PIN_LED=2 blink yanked the display's chip-select every
    // 120ms. Board has zero spare GPIOs, so the status LED is retired.
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
}

void setup() {
    Serial.begin(115200);
    delay(200);

    g_prefs.begin(kPrefsName, false);
    loadCfg();
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

    Serial.println("MS2/Extra I/O box v3 (iobox3) ready — ESP-NOW link to dash. Type ? for status.");
    Serial.println("boot link=espnow proto=ms2");

    if (g_cfg.bootTest) {
        s_bootTestArmed = true;
        s_bootTestStartMs = millis();
        Serial.println("boottest: armed (sim 3s after settle)");
    }
}

void loop() {
    // Boot self-test: settle -> run sim for a few seconds -> stop.
    if (s_bootTestArmed) {
        uint32_t since = millis() - s_bootTestStartMs;
        if (!s_bootTestOn && since >= BOOT_SETTLE_MS) {
            s_bootTestOn = true;
            s_simActive = true;
            Serial.println("boottest: sim on");
        } else if (s_bootTestOn && since >= BOOT_SETTLE_MS + BOOT_TEST_MS) {
            s_bootTestOn = false;
            s_bootTestArmed = false;
            simStop();
            Serial.println("boottest: done");
        }
    }

    if (s_simActive) simInject();
    s_canFresh = s_anyGroupSeen && (millis() - s_lastFrameMs) < FAILSAFE_MS;
    decodeOutpc();
    updateAnalogLatch();
    updateEngineProfile();
    updateOutputs();
    updateBuzzer();

    uint32_t now = millis();

    static uint32_t dashLast = 0;
    if (now - dashLast >= DASH_TX_MS) {
        dashLast = now;
        espnowSendStatus();
    }

    static uint32_t tftLast = 0;
    if (g_cfg.tftEnable && now - tftLast >= 100) {
        tftLast = now;
        updateDisplay();
        updateGasDisplay();
    }

    // Drain queued 0xC0 commands (executed in loop context, not WiFi task).
    if (s_cmdQ) {
        char cmd[CMD_Q_LEN];
        while (xQueueReceive(s_cmdQ, cmd, 0) == pdTRUE) {
            handleCommand(String(cmd));
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
            if (ch == '\n') {
                // GPIO3 floats when no USB host is attached and picks up
                // harness noise as random bytes -> junk lines that used to
                // trigger "?)" hints / beep / help spam. Discard them here:
                // zero extra CPU (the bytes are received+parsed regardless),
                // we just skip responding to lines with non-printables.
                if (!badLine && line.length()) handleCommand(line);
                line = "";
                badLine = false;
            }
            else if (ch != '\r') {
                if (ch < 32 || ch > 126) badLine = true;   // non-printable = noise
                else if (line.length() < 128) line += ch;  // cap: flood can't grow heap
            }
        }
    }

    delay(2);
}

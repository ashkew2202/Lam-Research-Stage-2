// Water test for the three-pump flow rig: staged tests plus a full single take.
// Board: ESP32 on the Stage 2 PCB. Serial console at 115200, send each command with Enter.
//
// Wiring assumed (matches the bench setup):
//   - Pump 2 drive pins swapped: RPWM = G18, LPWM = G19 (G19 stays low).
//   - NOT connected and never read: LSL-03 probe (G4), ZSO-01 (G5), all IBT-2 IS lines (G15/G36/G39).
//   - Because there is no empty-tank sensor and no pump current, the flush ends on a timer
//     (tfl, seconds of P-03 running). Measure it with stage 4, then use it in stage 5 and 6.
//
// Stages (type the number):
//   1  P-01 transfer       TK-01 -> SOV-01 -> P-01 -> sump
//   2  P-02 loop + FT-01   sump -> P-02 -> FT-01 -> TK-01, flow and litres live
//   3  Case 1              P-01 50% on level control + P-02 100%, pour water when cued
//   4  Flush rate          P-01 + P-03 for tfr s; then "lit <litres>" sets the flush timer
//   5  Case 2              isolate, open BV-01, flush on the timer, automatic stop
//   6  Full take           Case 1 then Case 2 with no input (button or "6")
//   a  Guided run          stages 1 to 5, pausing between them
// Always: x = stop everything, go = continue / BV-01 is open, st = status.

#include <Arduino.h>
#include <Preferences.h>
#include <esp_arduino_version.h>
#include <esp_timer.h>
#include <soc/gpio_reg.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>

// ------------------------------------------------------------------ pins --
namespace pin {
constexpr uint8_t RPWM[3] = {33, 18, 25};  // pump 2 swapped: G18 is its drive pin
constexpr uint8_t LPWM[3] = {32, 19, 23};
constexpr uint8_t REN[3] = {26, 17, 22};
constexpr uint8_t LEN[3] = {13, 16, 21};
constexpr uint8_t SOV = 27;
constexpr uint8_t FT = 14;
constexpr uint8_t BUTTON = 34;  // LOW = pressed
constexpr uint8_t FLOAT = 35;   // HIGH = level high or wire open
constexpr uint8_t LED = 2;
}  // namespace pin

// This float reads LOW when the water is HIGH. true flips the reading so the logic below stays
// "true = level high". Note: with this setting a broken float wire reads as level OK, so the
// overfill protection is lost if the wire breaks.
constexpr bool FLOAT_INVERTED = true;
static bool floatLevelHigh() {
  const bool pinHigh = digitalRead(pin::FLOAT) == HIGH;
  return FLOAT_INVERTED ? !pinHigh : pinHigh;
}

// ------------------------------------------------------------ parameters --
struct Param {
  const char* key;
  float def, lo, hi;
  const char* help;
  float v;
};
static Param params[] = {
    {"a", 0.010204f, 0.001f, 0.1f, "FT-01 slope, L/min per Hz (Q = a*f + b)", 0},
    {"b", 0.0f, -1.0f, 1.0f, "FT-01 offset, L/min", 0},
    {"fcut", 3.0f, 0.0f, 60.0f, "flow is zero below this frequency, Hz", 0},
    {"d1", 0.50f, 0.0f, 1.0f, "P-01 duty, Case 1", 0},
    {"d2", 1.00f, 0.0f, 1.0f, "P-02 duty", 0},
    {"d1f", 0.50f, 0.0f, 1.0f, "P-01 duty, flush", 0},
    {"d3", 0.45f, 0.0f, 1.0f, "P-03 duty, flush", 0},
    {"t1", 20.0f, 5.0f, 300.0f, "stage 1 run time, s", 0},
    {"t2", 30.0f, 5.0f, 600.0f, "stage 2 run time, s (TK-02 must hold 3 L/min x this + 0.4 L)", 0},
    {"tc1", 180.0f, 20.0f, 3600.0f, "Case 1 run time, s", 0},
    {"tpour", 60.0f, 5.0f, 3600.0f, "pour cue, s after Case 1 starts", 0},
    {"tfr", 20.0f, 5.0f, 120.0f, "stage 4 flush-rate run time, s", 0},
    {"tfl", 90.0f, 10.0f, 1800.0f, "flush timer: seconds of P-03 running that empty TK-03", 0},
    {"tbv", 15.0f, 3.0f, 300.0f, "time to open BV-01 before flush, s", 0},
    {"p1off", 5000.0f, 0.0f, 60000.0f, "P-01 minimum off time after a float trip, ms", 0},
    {"lshon", 500.0f, 0.0f, 5000.0f, "LS-01 high confirm time, ms", 0},
    {"lshoff", 3000.0f, 0.0f, 30000.0f, "LS-01 clear confirm time, ms", 0},
    {"ramp", 1.0f, 0.1f, 20.0f, "pump ramp rate, duty per s", 0},
    {"tkl", 4.2f, 0.5f, 20.0f, "flush tank volume, L", 0},
};
constexpr size_t NPARAM = sizeof(params) / sizeof(params[0]);
static Param* findParam(const char* k) {
  for (auto& p : params)
    if (strcasecmp(p.key, k) == 0) return &p;
  return nullptr;
}
static float P(const char* k) {
  Param* p = findParam(k);
  return p ? p->v : NAN;
}
static Preferences prefs;
static void loadParams() {
  prefs.begin("water", true);
  for (auto& p : params) p.v = prefs.getFloat(p.key, p.def);
  prefs.end();
}
static void saveParams() {
  prefs.begin("water", false);
  for (auto& p : params) prefs.putFloat(p.key, p.v);
  prefs.end();
}

// --------------------------------------------------------------- logging --
static uint32_t nowMs = 0;
static void event(const char* tag, const char* fmt, ...) {
  char msg[200];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(msg, sizeof msg, fmt, ap);
  va_end(ap);
  Serial.printf("E,%lu,%s,%s\n", (unsigned long)millis(), tag, msg);
}
static void say(const char* fmt, ...) {
  char msg[240];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(msg, sizeof msg, fmt, ap);
  va_end(ap);
  Serial.println(msg);
}

// ------------------------------------------------------------------- PWM --
constexpr uint32_t PWM_FREQ = 20000;
constexpr uint8_t PWM_BITS = 10;
constexpr uint32_t PWM_MAX = (1u << PWM_BITS) - 1;
static void pwmAttach(uint8_t p, uint8_t c) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)c;
  ledcAttach(p, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(c, PWM_FREQ, PWM_BITS);
  ledcAttachPin(p, c);
#endif
}
static void pwmWrite(uint8_t p, uint8_t c, uint32_t d) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)c;
  ledcWrite(p, d);
#else
  (void)p;
  ledcWrite(c, d);
#endif
}

// ----------------------------------------------------------------- pumps --
static uint32_t lastStartMs = 0;  // starts are staggered
struct Pump {
  const char* tag;
  uint8_t idx;
  float cmd = 0, out = 0;
  bool en = false;
  uint32_t zeroSinceMs = 0, startMs = 0;
  Pump(const char* t, uint8_t i) : tag(t), idx(i) {}
  void begin() {
    for (uint8_t p : {pin::REN[idx], pin::LEN[idx], pin::LPWM[idx]}) {
      pinMode(p, OUTPUT);
      digitalWrite(p, LOW);
    }
    pwmAttach(pin::RPWM[idx], idx);
    pwmWrite(pin::RPWM[idx], idx, 0);
  }
  void update(float dt) {
    float target = constrain(cmd, 0.0f, 1.0f);
    if (target > out) {
      if (out <= 0.0f) {
        if (nowMs - lastStartMs < 500) target = 0;
        else {
          lastStartMs = nowMs;
          startMs = nowMs;
        }
      }
      out = min(target, out + P("ramp") * dt);
    } else {
      out = target;  // stop immediately, low-side brake
    }
    if (out > 0.0f) {
      if (!en) {
        en = true;
        digitalWrite(pin::REN[idx], HIGH);
        digitalWrite(pin::LEN[idx], HIGH);
      }
      zeroSinceMs = nowMs;
    } else if (en && nowMs - zeroSinceMs > 300) {
      en = false;
      digitalWrite(pin::REN[idx], LOW);
      digitalWrite(pin::LEN[idx], LOW);
    }
    pwmWrite(pin::RPWM[idx], idx, (uint32_t)lroundf(out * PWM_MAX));
  }
  bool running() const { return out > 0.05f; }
  uint32_t runMs() const { return running() ? nowMs - startMs : 0; }
};
static Pump P1{"P-01", 0}, P2{"P-02", 1}, P3{"P-03", 2};

struct Sov {
  bool cmd = false, out = false;
  uint32_t changedMs = 0;
  void update() {
    if (cmd != out) {
      out = cmd;
      changedMs = nowMs;
      digitalWrite(pin::SOV, out ? HIGH : LOW);
    }
  }
  bool openFor(uint32_t ms) const { return out && nowMs - changedMs >= ms; }
} sov;

static void allOff() {
  P1.cmd = P2.cmd = P3.cmd = 0;
  sov.cmd = false;
}
static void hardStop() {
  allOff();
  P1.out = P2.out = P3.out = 0;
  sov.update();
}

// ---------------------------------------------------------------- inputs --
struct Debounce {
  bool raw = false, stable = false;
  uint32_t sinceMs = 0;
  bool update(bool r, uint32_t onMs, uint32_t offMs) {
    if (r != raw) {
      raw = r;
      sinceMs = nowMs;
    }
    if (stable != raw && nowMs - sinceMs >= (raw ? onMs : offMs)) {
      stable = raw;
      return true;
    }
    return false;
  }
};
static Debounce lsh, btn;

// ------------------------------------------------------------ flow meter --
constexpr uint32_t FT_MIN_PERIOD_US = 800;
static portMUX_TYPE ftMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t ftCount = 0, ftLastRise = 0, ftGlitch = 0;

void IRAM_ATTR ftIsr() {
  const uint32_t t = (uint32_t)esp_timer_get_time();
  if (!((REG_READ(GPIO_IN_REG) >> pin::FT) & 1U)) return;
  portENTER_CRITICAL_ISR(&ftMux);
  if (t - ftLastRise >= FT_MIN_PERIOD_US) {
    ftLastRise = t;
    ftCount++;
  } else {
    ftGlitch++;
  }
  portEXIT_CRITICAL_ISR(&ftMux);
}

struct Flow {
  static constexpr uint8_t H = 16;
  uint32_t hc[H] = {0}, ht[H] = {0};
  uint8_t head = 0, n = 0;
  uint32_t lastCount = 0;
  float f = 0, q = 0;
  double vTot = 0, vStage = 0;  // litres
  uint32_t pulsesStage = 0;
  // statistics over the steady part of a stage
  double qSum = 0, qSq = 0;
  uint32_t qN = 0;
  void sample(float dt) {
    uint32_t c, tr;
    portENTER_CRITICAL(&ftMux);
    c = ftCount;
    tr = ftLastRise;
    portEXIT_CRITICAL(&ftMux);
    const uint32_t tNow = (uint32_t)esp_timer_get_time();
    hc[head] = c;
    ht[head] = tr;
    head = (head + 1) % H;
    if (n < H) n++;
    const uint8_t back = n - 1 < 5 ? n - 1 : 5;
    const uint8_t k = (head + H - 1 - back) % H;
    float fNew = 0;
    if (back > 0 && c > hc[k] && tr != ht[k]) fNew = (float)(c - hc[k]) * 1e6f / (float)(tr - ht[k]);
    if ((tNow - tr) > 700000) fNew = 0;  // no pulse for 0.7 s: flow stopped
    f = fNew;
    const bool flowing = f > P("fcut");
    q = flowing ? max(0.0f, P("a") * f + P("b")) : 0.0f;
    const uint32_t dN = c - lastCount;
    lastCount = c;
    double dv = (double)P("a") * dN / 60.0 + (flowing ? (double)P("b") * dt / 60.0 : 0.0);
    if (dv < 0) dv = 0;
    vTot += dv;
    vStage += dv;
    pulsesStage += dN;
  }
  void resetStage() {
    vStage = 0;
    pulsesStage = 0;
    qSum = qSq = 0;
    qN = 0;
  }
  void addStat() {
    qSum += q;
    qSq += (double)q * q;
    qN++;
  }
  float mean() const { return qN ? (float)(qSum / qN) : 0; }
  float cv() const {
    if (qN < 2 || qSum <= 0) return 0;
    const double m = qSum / qN;
    const double var = max(0.0, qSq / qN - m * m);
    return (float)(sqrt(var) / m);
  }
} flow;

// -------------------------------------------------------------- sequence --
enum class St : uint8_t {
  IDLE, S1, S2, C1_START, C1_RUN, C1_HOLD, FR_WAIT, FR_RUN, FR_DONE, C2_ISOLATE, C2_WAIT_BV, C2_FLUSH, C2_HOLD, CAL,
  WAIT_NEXT
};
static const char* stName(St s) {
  switch (s) {
    case St::IDLE: return "IDLE";
    case St::S1: return "S1_P01";
    case St::S2: return "S2_P02_FT01";
    case St::C1_START: return "C1_START";
    case St::C1_RUN: return "C1_RUN";
    case St::C1_HOLD: return "C1_HOLD";
    case St::FR_WAIT: return "FR_WAIT_BV";
    case St::FR_RUN: return "FR_RUN";
    case St::FR_DONE: return "FR_DONE";
    case St::C2_ISOLATE: return "C2_ISOLATE";
    case St::C2_WAIT_BV: return "C2_WAIT_BV";
    case St::C2_FLUSH: return "C2_FLUSH";
    case St::C2_HOLD: return "C2_HOLD";
    case St::CAL: return "CAL";
    case St::WAIT_NEXT: return "WAIT_NEXT";
  }
  return "?";
}
enum class Chain : uint8_t { NONE, GUIDED, FULL };

static St state = St::IDLE;
static Chain chain = Chain::NONE;
static uint8_t guidedNext = 1;  // next stage in a guided run
static uint32_t stateMs = 0, stageStartMs = 0;
static uint32_t p1StopMs = 0, lshEvents = 0, p1OnMs = 0;
static uint32_t flushRunMs = 0;  // P-03 on-time in Case 2
static uint32_t lastTripLatencyMs = 0;
static bool pourCued = false;
static float lastFlushRateLps = 0;

static uint32_t inState() { return nowMs - stateMs; }
static void enter(St s, const char* why) {
  event("STATE", "%s -> %s (%s)", stName(state), stName(s), why);
  state = s;
  stateMs = nowMs;
}
static float stageSeconds() { return (nowMs - stageStartMs) / 1000.0f; }

static void startStage(int n);
static void finish(const char* summary) {
  hardStop();
  say("  %s", summary);
  if (chain == Chain::GUIDED && guidedNext <= 5) {
    say("  Next: stage %d. Prepare water, then type go. Type x to end the guided run.", guidedNext);
    enter(St::WAIT_NEXT, "guided run waiting");
  } else {
    chain = Chain::NONE;
    enter(St::IDLE, "stage complete");
    say("  Done. Type a stage number, a, or help.");
  }
}

static bool floatBlocks(const char* what) {
  if (lsh.stable || lsh.raw) {
    say("  REFUSED (%s): LS-01 reports the sump level high. Lower the level, or check the float position and FLOAT_INVERTED.", what);
    return true;
  }
  return false;
}

static void beginCase1() {
  flow.resetStage();
  lshEvents = 0;
  p1OnMs = 0;
  pourCued = false;
  stageStartMs = nowMs;
  p1StopMs = nowMs - (uint32_t)P("p1off");
  enter(St::C1_START, "Case 1 start");
}
static void beginCase2() {
  flow.resetStage();
  flushRunMs = 0;
  stageStartMs = nowMs;
  enter(St::C2_ISOLATE, "Case 2 start");
}

static void startStage(int n) {
  if (state != St::IDLE && state != St::WAIT_NEXT) {
    say("  Busy (%s). Type x to stop first.", stName(state));
    return;
  }
  hardStop();
  flow.resetStage();
  stageStartMs = nowMs;
  switch (n) {
    case 1:
      if (floatBlocks("stage 1")) return;
      say("STAGE 1: P-01 transfer. TK-01 water, BV-01 shut, SOV-01 opens, %.0f s at %.0f%%.", P("t1"), P("d1") * 100);
      enter(St::S1, "stage 1");
      break;
    case 2:
      say("STAGE 2: P-02 loop + FT-01. BV-03 and BV-02 open, %.0f s at %.0f%%.", P("t2"), P("d2") * 100);
      say("  TK-02 needs at least %.1f L: P-02 moves about 3 L/min and has no dry-run protection. Type x if it sucks air.",
          3.0f * P("t2") / 60.0f + 0.4f);
      enter(St::S2, "stage 2");
      break;
    case 3:
      if (floatBlocks("stage 3")) return;
      say("STAGE 3: Case 1 for %.0f s. POUR the jug into TK-02 at %.0f s, away from the float.", P("tc1"), P("tpour"));
      beginCase1();
      break;
    case 4:
      if (floatBlocks("stage 4")) return;
      say("STAGE 4: flush rate. TK-03 full (%.1f L), BV-01 shut for now. P-01 + P-03 run %.0f s.", P("tkl"), P("tfr"));
      say("  Open BV-01 within %.0f s, or type go when it is open.", P("tbv"));
      enter(St::FR_WAIT, "wait for BV-01");
      break;
    case 5:
      if (floatBlocks("stage 5")) return;
      say("STAGE 5: Case 2. P-02 runs, SOV-01 closes, then open BV-01 within %.0f s. Flush timer %.0f s.", P("tbv"), P("tfl"));
      P2.cmd = P("d2");
      beginCase2();
      break;
    case 6:
      if (floatBlocks("full take")) return;
      say("FULL TAKE: Case 1 (%.0f s, pour at %.0f s) then Case 2 (open BV-01 when the LED blinks fast).", P("tc1"), P("tpour"));
      chain = Chain::FULL;
      beginCase1();
      break;
    default:
      break;
  }
}

// P-01 level control shared by Case 1: LS-01 high stops P-01 and closes SOV-01.
static void p1LevelControl() {
  if (state == St::C1_RUN) {
    if (lsh.stable) {
      P1.cmd = 0;
      sov.cmd = false;
      p1StopMs = nowMs;
      lshEvents++;
      lastTripLatencyMs = nowMs - lsh.sinceMs;
      event("LS01_TRIP", "P-01 stop, SOV-01 close, confirm %lu ms, p2_running=%d", (unsigned long)lastTripLatencyMs, P2.running());
      enter(St::C1_HOLD, "LS-01 high");
      return;
    }
    if (P2.runMs() > 1000 && nowMs - p1StopMs >= (uint32_t)P("p1off")) {
      sov.cmd = true;
      if (sov.openFor(300)) P1.cmd = P("d1");
    }
  } else if (state == St::C1_HOLD) {
    P1.cmd = 0;
    sov.cmd = false;
    if (!lsh.stable && nowMs - p1StopMs >= (uint32_t)P("p1off")) enter(St::C1_RUN, "LS-01 clear: P-01 restart");
  }
}

static void case1Summary() {
  const float t = stageSeconds();
  const float qavg = t > 0 ? flow.vStage * 60.0f / t : 0;
  say("  CASE 1 result: %.0f s, %.3f L returned, mean %.2f L/min (target 3.00 +/-0.10), LS-01 trips %lu, P-01 on %.0f%%",
      t, flow.vStage, qavg, (unsigned long)lshEvents, t > 0 ? 100.0f * p1OnMs / (t * 1000.0f) : 0);
  if (lshEvents) say("  LS-01 trip confirm time %lu ms, P-02 kept running: %s", (unsigned long)lastTripLatencyMs, P2.running() ? "yes" : "NO");
  else say("  No LS-01 trip: either you did not pour, or the pour did not reach the float.");
  event("C1_SUMMARY", "t_s=%.0f v_L=%.3f qavg_lpm=%.3f lsh_events=%lu", t, flow.vStage, qavg, (unsigned long)lshEvents);
}

static void stepSequence() {
  switch (state) {
    case St::IDLE:
    case St::WAIT_NEXT:
      allOff();
      break;

    case St::S1:
      sov.cmd = true;
      if (sov.openFor(300)) P1.cmd = P("d1");
      if (stageSeconds() >= P("t1")) {
        char b[120];
        snprintf(b, sizeof b, "STAGE 1 done: P-01 ran %.0f s. Weigh the gain in TK-02 (1 kg = 1 L); expect water arriving.", P("t1"));
        guidedNext = chain == Chain::GUIDED ? 2 : guidedNext;
        finish(b);
      }
      break;

    case St::S2:
      P2.cmd = P("d2");
      if (P2.runMs() > 4000) flow.addStat();
      if (stageSeconds() >= P("t2")) {
        char b[200];
        snprintf(b, sizeof b, "STAGE 2 done: %.3f L, %lu pulses, steady mean %.2f L/min, variation %.1f%% (target 3.0).", flow.vStage,
                 (unsigned long)flow.pulsesStage, flow.mean(), flow.cv() * 100.0f);
        guidedNext = chain == Chain::GUIDED ? 3 : guidedNext;
        finish(b);
        if (flow.pulsesStage == 0) say("  FT-01 gave no pulses: check water in the line, air in the meter, 5 V and the G14 divider.");
      }
      break;

    case St::C1_START:
      allOff();
      if (inState() >= 600) {
        flow.resetStage();
        P2.cmd = P("d2");
        enter(St::C1_RUN, "P-02 fixed duty");
      }
      break;

    case St::C1_RUN:
    case St::C1_HOLD: {
      P2.cmd = P("d2");
      p1LevelControl();
      if (P1.running()) p1OnMs += 10;
      if (!pourCued && stageSeconds() >= P("tpour")) {
        pourCued = true;
        event("CUE", "POUR the jug into TK-02 now");
        say("  >>> POUR NOW <<<");
      }
      if (state == St::C1_RUN && stageSeconds() >= P("tc1")) {
        case1Summary();
        if (chain == Chain::FULL) {
          beginCase2();
        } else {
          guidedNext = chain == Chain::GUIDED ? 4 : guidedNext;
          finish("STAGE 3 done.");
        }
      }
      break;
    }

    case St::FR_WAIT:
      P1.cmd = P3.cmd = 0;
      sov.cmd = false;
      if (inState() >= (uint32_t)(P("tbv") * 1000)) {
        flow.resetStage();
        stageStartMs = nowMs;
        enter(St::FR_RUN, "BV-01 window elapsed");
      }
      break;

    case St::FR_RUN:
      sov.cmd = false;
      P1.cmd = P("d1f");
      if (P1.runMs() > 500) P3.cmd = P("d3");
      if (stageSeconds() >= P("tfr")) {
        hardStop();
        say("  STAGE 4 run done. Shut BV-01, weigh how much TK-03 lost, then type: lit <litres>  (e.g. lit 0.75)");
        enter(St::FR_DONE, "waiting for litres");
      }
      break;

    case St::FR_DONE:
      allOff();
      break;

    case St::C2_ISOLATE:
      P1.cmd = P3.cmd = 0;
      sov.cmd = false;
      P2.cmd = P("d2");
      if (inState() >= 1000) {
        say("  OPEN BV-01 NOW (LED blinking fast). Window %.0f s. Type go or press the button to skip the wait.", P("tbv"));
        enter(St::C2_WAIT_BV, "process tank isolated, open BV-01");
      }
      break;

    case St::C2_WAIT_BV:
      P1.cmd = P3.cmd = 0;
      sov.cmd = false;
      P2.cmd = P("d2");
      if (inState() >= (uint32_t)(P("tbv") * 1000)) {
        flushRunMs = 0;
        enter(St::C2_FLUSH, "BV-01 window elapsed");
      }
      break;

    case St::C2_FLUSH:
    case St::C2_HOLD: {
      P2.cmd = P("d2");
      if (state == St::C2_HOLD) {
        P1.cmd = P3.cmd = 0;
        if (!lsh.stable && nowMs - p1StopMs >= (uint32_t)P("p1off")) enter(St::C2_FLUSH, "LS-01 clear: flush resumes");
        break;
      }
      if (lsh.stable) {
        P1.cmd = P3.cmd = 0;
        p1StopMs = nowMs;
        lshEvents++;
        enter(St::C2_HOLD, "LS-01 high: P-01 and P-03 stop together");
        break;
      }
      sov.cmd = false;
      P1.cmd = P("d1f");
      if (P1.runMs() > 500) P3.cmd = P("d3");
      if (P3.running()) flushRunMs += 10;
      if (flushRunMs >= (uint32_t)(P("tfl") * 1000)) {
        const float t = stageSeconds();
        char b[200];
        snprintf(b, sizeof b, "CASE 2 done: all pumps off, SOV-01 closed. Flush %.0f s (P-03 on %.0f s), %.3f L returned by P-02.", t,
                 flushRunMs / 1000.0f, flow.vStage);
        event("FLUSH_END", "source=timer p3_on_s=%.1f v_L=%.3f", flushRunMs / 1000.0f, flow.vStage);
        chain = chain == Chain::FULL ? Chain::NONE : chain;
        guidedNext = chain == Chain::GUIDED ? 6 : guidedNext;
        finish(b);
        say("  Check TK-03: if it still holds water, raise tfl; if P-03 ran dry for long, lower tfl.");
      }
      break;
    }

    case St::CAL:
      break;
  }
}

static void applyInterlocks() {
  // I-1: LS-01 high stops P-01 and P-03 and closes SOV-01, always.
  if (lsh.stable) {
    P1.cmd = 0;
    P3.cmd = 0;
    sov.cmd = false;
  }
  // I-2: P-03 only runs while P-01 runs. I-3: SOV-01 stays shut while P-03 runs.
  if (!P1.running() || sov.out) P3.cmd = 0;
  if (P3.cmd > 0 || P3.running()) sov.cmd = false;
}

// -------------------------------------------------------- calibration run --
static uint32_t calCount0 = 0, calT0Us = 0, calT1Us = 0, calRunMs = 0, calId = 0;
static float calDuty = 0;
static bool calStopped = false;
static uint32_t readCount() {
  portENTER_CRITICAL(&ftMux);
  const uint32_t c = ftCount;
  portEXIT_CRITICAL(&ftMux);
  return c;
}
static void calStep() {
  if (state != St::CAL) return;
  if (!calStopped && inState() >= calRunMs) {
    P2.cmd = 0;
    calStopped = true;
    calT1Us = (uint32_t)esp_timer_get_time();
  }
  if (calStopped) {
    uint32_t last;
    portENTER_CRITICAL(&ftMux);
    last = ftLastRise;
    portEXIT_CRITICAL(&ftMux);
    if ((uint32_t)esp_timer_get_time() - last > 1500000) {
      const uint32_t n = readCount() - calCount0;
      const float t = (calT1Us - calT0Us) / 1e6f;
      Serial.printf("C,%lu,%lu,%lu,%.3f,%.3f,%.3f\n", (unsigned long)millis(), (unsigned long)calId, (unsigned long)n, t, n / t, calDuty);
      say("  CAL run %lu: %lu pulses in %.3f s (%.2f Hz). Weigh the water and note it.", (unsigned long)calId, (unsigned long)n, t, n / t);
      chain = Chain::NONE;
      enter(St::IDLE, "calibration run complete");
    }
  }
}

// -------------------------------------------------------------- console --
static void status() {
  say("state=%s chain=%d float=%s sov=%d  P-01 %.0f%%  P-02 %.0f%%  P-03 %.0f%%", stName(state), (int)chain,
      lsh.stable ? "HIGH" : "low", sov.out, P1.out * 100, P2.out * 100, P3.out * 100);
  say("flow %.2f L/min (%.1f mL/s, %.1f Hz), stage %.3f L, total %.3f L, glitches %lu", flow.q, flow.q / 0.06f, flow.f, flow.vStage,
      flow.vTot, (unsigned long)ftGlitch);
}

static void help() {
  say("Stages: 1 P-01 transfer | 2 P-02 + FT-01 | 3 Case 1 | 4 flush rate | 5 Case 2 | 6 full take | a guided 1-5");
  say("Control: x stop all | go continue / BV-01 open | st status | zero clear totaliser | mark <text>");
  say("Params: set <key> <value> | get | save | defaults | lit <L> (after stage 4) | cal <s> <duty%%>");
  say("Safety: no empty-tank sensor and no pump current. Pumps stop only on timers, the float, or x.");
}

static void getAll() {
  for (auto& p : params) say("  %-6s = %-9.4f  %s", p.key, p.v, p.help);
}

static void startCal(float seconds, float dutyPct) {
  if (state != St::IDLE) {
    say("  Busy. Type x first.");
    return;
  }
  hardStop();
  calId++;
  calDuty = constrain(dutyPct, 0.0f, 100.0f) / 100.0f;
  calRunMs = (uint32_t)(constrain(seconds, 5.0f, 300.0f) * 1000);
  calStopped = false;
  calCount0 = readCount();
  calT0Us = (uint32_t)esp_timer_get_time();
  chain = Chain::NONE;
  P2.cmd = calDuty;
  say("  CAL %lu: P-02 at %.0f%% for %.0f s into the weigh bucket.", (unsigned long)calId, calDuty * 100, calRunMs / 1000.0f);
  enter(St::CAL, "calibration run");
}

static void handle(char* line) {
  char* cmd = strtok(line, " \t");
  if (!cmd) return;
  for (char* p = cmd; *p; p++) *p = tolower(*p);
  char* a = strtok(nullptr, " \t");
  char* b = strtok(nullptr, " \t");

  if (!strcmp(cmd, "x") || !strcmp(cmd, "stop")) {
    hardStop();
    chain = Chain::NONE;
    enter(St::IDLE, "operator stop");
    say("  STOPPED. All pumps off, SOV-01 closed.");
  } else if (!strcmp(cmd, "go") || !strcmp(cmd, "g")) {
    if (state == St::WAIT_NEXT) startStage(guidedNext++);
    else if (state == St::FR_WAIT) {
      flow.resetStage();
      stageStartMs = nowMs;
      enter(St::FR_RUN, "operator: BV-01 open");
    } else if (state == St::C2_WAIT_BV) {
      flushRunMs = 0;
      enter(St::C2_FLUSH, "operator: BV-01 open");
    } else say("  Nothing is waiting for go.");
  } else if (!strcmp(cmd, "st")) {
    status();
  } else if (!strcmp(cmd, "help") || !strcmp(cmd, "?") || !strcmp(cmd, "m")) {
    help();
  } else if (!strcmp(cmd, "get")) {
    getAll();
  } else if (!strcmp(cmd, "set") && a && b) {
    Param* p = findParam(a);
    if (!p) say("  unknown key %s", a);
    else {
      p->v = constrain((float)atof(b), p->lo, p->hi);
      say("  %s = %.4f", p->key, p->v);
    }
  } else if (!strcmp(cmd, "save")) {
    saveParams();
    say("  parameters saved");
  } else if (!strcmp(cmd, "defaults")) {
    for (auto& p : params) p.v = p.def;
    say("  defaults loaded (not saved)");
  } else if (!strcmp(cmd, "zero")) {
    flow.vTot = 0;
    flow.resetStage();
    say("  totaliser cleared");
  } else if (!strcmp(cmd, "mark")) {
    event("MARK", "%s", a ? a : "");
  } else if (!strcmp(cmd, "lit") && a) {
    const float lit = (float)atof(a);
    if (state != St::FR_DONE && state != St::IDLE && state != St::WAIT_NEXT) say("  Type lit after stage 4 finishes.");
    else if (lit < 0.1f) say("  litres must be at least 0.1");
    else {
      const float tfl = 0.9f * P("tkl") * P("tfr") / lit;  // 10% margin so the tank is not run dry
      findParam("tfl")->v = constrain(tfl, findParam("tfl")->lo, findParam("tfl")->hi);
      lastFlushRateLps = lit / P("tfr");
      say("  P-03 path moved %.2f L in %.0f s (%.2f L/min). Flush timer tfl = %.0f s (90%% of the tank).", lit, P("tfr"),
          lastFlushRateLps * 60.0f, P("tfl"));
      say("  Type save to keep it. Refill TK-03 to %.1f L before stage 5 or 6.", P("tkl"));
      if (state == St::FR_DONE) {
        guidedNext = chain == Chain::GUIDED ? 5 : guidedNext;
        finish("STAGE 4 done.");
      }
    }
  } else if (!strcmp(cmd, "cal") && a && b) {
    startCal((float)atof(a), (float)atof(b));
  } else if (!strcmp(cmd, "a")) {
    if (state != St::IDLE) say("  Busy. Type x first.");
    else {
      chain = Chain::GUIDED;
      guidedNext = 2;
      startStage(1);
      if (state == St::IDLE) chain = Chain::NONE;
    }
  } else if (strlen(cmd) == 1 && cmd[0] >= '1' && cmd[0] <= '6') {
    chain = Chain::NONE;
    startStage(cmd[0] - '0');
  } else {
    say("  unknown command, type help");
  }
}

static char lineBuf[64];
static uint8_t lineLen = 0;
static void pollSerial() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen) {
        lineBuf[lineLen] = 0;
        lineLen = 0;
        handle(lineBuf);
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

// ----------------------------------------------------------- setup/loop --
void setup() {
  for (int i = 0; i < 3; i++)
    for (uint8_t p : {pin::REN[i], pin::LEN[i], pin::LPWM[i], pin::RPWM[i]}) {
      pinMode(p, OUTPUT);
      digitalWrite(p, LOW);
    }
  pinMode(pin::SOV, OUTPUT);
  digitalWrite(pin::SOV, LOW);
  Serial.begin(115200);
  loadParams();
  P1.begin();
  P2.begin();
  P3.begin();
  pinMode(pin::BUTTON, INPUT);
  pinMode(pin::FLOAT, INPUT);
  pinMode(pin::FT, INPUT);
  pinMode(pin::LED, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(pin::FT), ftIsr, CHANGE);
  delay(1000);
  say("\nWater test ready. FT-01 a=%.5f b=%.3f. Flush timer tfl=%.0f s.", P("a"), P("b"), P("tfl"));
  help();
}

void loop() {
  static uint32_t tick10 = 0, tick100 = 0, tick1000 = 0;
  nowMs = millis();
  pollSerial();

  if (nowMs - tick10 >= 10) {
    const float dt = (nowMs - tick10) / 1000.0f;
    tick10 = nowMs;

    lsh.update(floatLevelHigh(), (uint32_t)P("lshon"), (uint32_t)P("lshoff"));
    const bool pressed = digitalRead(pin::BUTTON) == LOW;
    if (btn.update(pressed, 40, 40) && btn.stable) {
      if (state == St::IDLE) {
        chain = Chain::NONE;
        startStage(6);
      } else if (state == St::C2_WAIT_BV) {
        flushRunMs = 0;
        enter(St::C2_FLUSH, "button: BV-01 open");
      } else {
        hardStop();
        chain = Chain::NONE;
        enter(St::IDLE, "button stop");
        say("  STOPPED by the button.");
      }
    }

    stepSequence();
    calStep();
    applyInterlocks();

    if (state == St::C2_FLUSH || state == St::C2_HOLD || state == St::C2_WAIT_BV || state == St::C2_ISOLATE) flow.addStat();
    P1.update(dt);
    P2.update(dt);
    P3.update(dt);
    sov.update();
  }

  if (nowMs - tick100 >= 100) {
    const float dt = (nowMs - tick100) / 1000.0f;
    tick100 = nowMs;
    flow.sample(dt);
    // LED: fast blink = open BV-01 now, steady = running, slow heartbeat = idle
    bool led;
    if (state == St::C2_WAIT_BV || state == St::FR_WAIT) led = (nowMs / 100) % 2;
    else if (state == St::IDLE || state == St::WAIT_NEXT || state == St::FR_DONE) led = (nowMs / 1000) % 2;
    else led = true;
    digitalWrite(pin::LED, led);
  }

  if (nowMs - tick1000 >= 1000) {
    tick1000 = nowMs;
    if (state != St::IDLE && state != St::WAIT_NEXT && state != St::FR_DONE)
      Serial.printf("T,%lu,%s,%.2f,%.1f,%.3f,%.0f,%.0f,%.0f,%d,%d,%lu\n", (unsigned long)millis(), stName(state), flow.q, flow.q / 0.06f,
                    flow.vStage, P1.out * 100, P2.out * 100, P3.out * 100, sov.out, lsh.stable, (unsigned long)ftGlitch);
  }
  delay(2);
}
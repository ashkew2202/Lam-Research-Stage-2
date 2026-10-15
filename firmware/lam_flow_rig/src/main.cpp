// Flow rig controller, Lam Research Challenge 3.0 stage 2
// Team Catalysts (LRC-26-0146)
// Board: ESP32 devkit (uPesy WROOM, same pins as NodeMCU-32S) on the LAM Stage 2 PCB
// Arduino-ESP32 core 2.0.17, wifi not used

#include <Arduino.h>
#include <Preferences.h>
#include <esp_arduino_version.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <soc/gpio_reg.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>

static const char* const FW_VERSION = "1.2";

// pins, from the PCB schematic
namespace pin {
constexpr uint8_t P1_RPWM = 33, P1_LPWM = 32, P1_REN = 26, P1_LEN = 13;
// P-02: the G19 wire does not reach the driver, so we drive it from G18 and swapped the motor leads
constexpr uint8_t P2_RPWM = 18, P2_LPWM = 19, P2_REN = 17, P2_LEN = 16;
constexpr uint8_t P3_RPWM = 25, P3_LPWM = 23, P3_REN = 22, P3_LEN = 21;
constexpr uint8_t SOV = 27;     // solenoid mosfet, HIGH = valve open
constexpr uint8_t FT = 14;      // flow sensor pulses
constexpr uint8_t BUTTON = 34;  // on/off switch, LOW = pressed
constexpr uint8_t FLOAT = 35;   // float switch (see finv)
// optional extras, not used in our runs
constexpr uint8_t IS1 = 15;     // P-01 current
constexpr uint8_t IS2 = 36;     // P-02 current
constexpr uint8_t IS3 = 39;     // P-03 current
constexpr uint8_t TOUCH = 4;    // touch probe in the flush tank
constexpr uint8_t ZSO = 5;      // BV-01 open contact
constexpr uint8_t LED = 2;      // board LED
}  // namespace pin

struct Param {
  const char* key;
  float def, lo, hi;
  const char* help;
  float v;
};

static Param params[] = {
    {"a", 0.010204f, 0.001f, 0.1f, "FT-01 slope, L/min per Hz (Q = a*f + b)", 0},
    {"b", 0.0f, -1.0f, 1.0f, "FT-01 offset, L/min", 0},
    {"fcut", 3.0f, 0.0f, 60.0f, "low-flow cutoff, Hz (Q = 0 below)", 0},
    {"d1", 0.50f, 0.0f, 1.0f, "P-01 duty, Case 1", 0},
    {"d2", 1.00f, 0.0f, 1.0f, "P-02 duty, Case 1 (spec 1.00)", 0},
    {"d1f", 0.50f, 0.0f, 1.0f, "P-01 duty, flush", 0},
    {"d3", 0.45f, 0.0f, 1.0f, "P-03 duty, flush (keep its flow just below P-01's draw)", 0},
    {"d1fic", 1.00f, 0.0f, 1.0f, "P-01 duty in FIC mode", 0},
    {"tc1", 180.0f, 10.0f, 3600.0f, "Case 1 run time, s", 0},
    {"c1fic", 0.0f, 0.0f, 1.0f, "1 = P-02 on flow control during Case 1", 0},
    {"sp", 3.0f, 0.3f, 6.5f, "FIC set point, L/min", 0},
    {"kc", 0.05f, 0.0f, 1.0f, "FIC gain, duty per L/min", 0},
    {"ti", 1.0f, 0.05f, 30.0f, "FIC integral time, s", 0},
    {"ffq0", -0.5f, -5.0f, 5.0f, "P-02 duty map offset: Q = ffq0 + ffq1*d", 0},
    {"ffq1", 6.0f, 0.5f, 20.0f, "P-02 duty map slope, L/min per unit duty", 0},
    {"ramp", 1.0f, 0.1f, 20.0f, "pump ramp-up rate, duty per s", 0},
    {"lshon", 500.0f, 0.0f, 5000.0f, "LS-01 high confirm time, ms", 0},
    {"lshoff", 3000.0f, 0.0f, 30000.0f, "LS-01 clear confirm time, ms", 0},
    {"p1off", 5000.0f, 0.0f, 60000.0f, "P-01 minimum off time, ms", 0},
    {"bvmode", 0.0f, 0.0f, 1.0f, "0 = timed BV-01 window, 1 = wait for ZSO-01", 0},
    {"tbv", 15.0f, 3.0f, 300.0f, "BV-01 window (mode 0) or alarm delay (mode 1), s", 0},
    {"tchE", 0.0f, 0.0f, 5000.0f, "LSL-03 reading with the probe dry (0 = probe off)", 0},
    {"tchW", 0.0f, 0.0f, 5000.0f, "LSL-03 reading with the probe wet (re-captured each run)", 0},
    {"tchF", 0.5f, 0.1f, 0.95f, "LSL-03 trip point, fraction from wet to dry", 0},
    {"i3dry", 0.0f, 0.0f, 10.0f, "P-03 current below this = pumping air, A (0 = off)", 0},
    {"tfmin", 20.0f, 0.0f, 600.0f, "flush time before touch trip is armed, s", 0},
    {"tfl", 0.0f, 0.0f, 900.0f, "P-03 run time that empties TK-03, s (0 = off; used when LSL-03 is not armed)", 0},
    {"tfmax", 240.0f, 10.0f, 1800.0f, "flush timeout, s", 0},
    {"g1", 0.004255f, 0.0f, 1.0f, "IS1 gain, A per mV", 0},
    {"g2", 0.004255f, 0.0f, 1.0f, "IS2 gain, A per mV", 0},
    {"g3", 0.004255f, 0.0f, 1.0f, "IS3 gain, A per mV", 0},
    {"isok", 0.0f, 0.0f, 1.0f, "1 = pump current sensing is wired", 0},
    {"finv", 1.0f, 0.0f, 1.0f, "1 = LS-01 pin LOW means level high (as built; a broken wire then reads OK)", 0},
    {"mwd", 0.0f, 0.0f, 3600.0f, "MANUAL watchdog: outputs off after this many s without a command (0 = off)", 0},
    {"fdi", 1.0f, 0.0f, 1.0f, "1 = diagnostics on", 0},
    {"trip", 1.0f, 0.0f, 1.0f, "1 = diagnostics may trip P-02", 0},
};
constexpr size_t NPARAM = sizeof(params) / sizeof(params[0]);

static Param* findParam(const char* key) {
  for (auto& p : params)
    if (strcasecmp(p.key, key) == 0) return &p;
  return nullptr;
}
static float P(const char* key) {
  Param* p = findParam(key);
  return p ? p->v : NAN;
}

static Preferences prefs;
static void loadParams() {
  prefs.begin("rig", true);
  for (auto& p : params) p.v = prefs.getFloat(p.key, p.def);
  prefs.end();
}
static void saveParams() {
  prefs.begin("rig", false);
  for (auto& p : params) prefs.putFloat(p.key, p.v);
  prefs.end();
}

static uint32_t nowMs = 0;
static void event(const char* tag, const char* fmt, ...) {
  char msg[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(msg, sizeof msg, fmt, ap);
  va_end(ap);
  Serial.printf("E,%lu,%s,%s\n", (unsigned long)millis(), tag, msg);
}

constexpr uint32_t PWM_FREQ = 20000;  // 20 kHz, the BTS7960 takes up to 25 kHz
constexpr uint8_t PWM_BITS = 10;
constexpr uint32_t PWM_MAX = (1u << PWM_BITS) - 1;

static void pwmAttach(uint8_t pinNo, uint8_t ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcAttach(pinNo, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(ch, PWM_FREQ, PWM_BITS);
  ledcAttachPin(pinNo, ch);
#endif
}
static void pwmWrite(uint8_t pinNo, uint8_t ch, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcWrite(pinNo, duty);
#else
  (void)pinNo;
  ledcWrite(ch, duty);
#endif
}

static uint32_t lastStartMs = 0;  // pumps start one at a time so the 12 V rail does not dip

struct Pump {
  const char* tag;
  uint8_t rpwm, lpwm, ren, len, ch;
  Pump(const char* t, uint8_t r, uint8_t l, uint8_t re, uint8_t le, uint8_t c)
      : tag(t), rpwm(r), lpwm(l), ren(re), len(le), ch(c) {}
  float cmd = 0, out = 0;
  bool en = false;
  uint32_t zeroSinceMs = 0, startMs = 0;

  void begin() {
    for (uint8_t p : {ren, len, lpwm}) {
      pinMode(p, OUTPUT);
      digitalWrite(p, LOW);
    }
    pwmAttach(rpwm, ch);
    pwmWrite(rpwm, ch, 0);
  }

  void update(float dt) {
    float target = constrain(cmd, 0.0f, 1.0f);
    if (target > out) {
      if (out <= 0.0f) {
        if (nowMs - lastStartMs < 500) target = 0;  // 0.5 s gap after the last pump start
        else {
          lastStartMs = nowMs;
          startMs = nowMs;
        }
      }
      out = min(target, out + P("ramp") * dt);
    } else {
      out = target;  // stop right away, the driver brakes the motor
    }
    if (out > 0.0f) {
      if (!en) {
        en = true;
        digitalWrite(ren, HIGH);
        digitalWrite(len, HIGH);
      }
      zeroSinceMs = nowMs;
    } else if (en && nowMs - zeroSinceMs > 300) {
      en = false;
      digitalWrite(ren, LOW);
      digitalWrite(len, LOW);
    }
    pwmWrite(rpwm, ch, (uint32_t)lroundf(out * PWM_MAX));
  }

  bool running() const { return out > 0.05f; }
  uint32_t runMs() const { return running() ? nowMs - startMs : 0; }
};

static Pump P1{"P-01", pin::P1_RPWM, pin::P1_LPWM, pin::P1_REN, pin::P1_LEN, 0};
static Pump P2{"P-02", pin::P2_RPWM, pin::P2_LPWM, pin::P2_REN, pin::P2_LEN, 1};
static Pump P3{"P-03", pin::P3_RPWM, pin::P3_LPWM, pin::P3_REN, pin::P3_LEN, 2};

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
static Debounce lsh, button, zso;

constexpr uint32_t FT_MIN_PERIOD_US = 800;  // pulses closer than 0.8 ms are noise
constexpr uint16_t RING = 512;
static portMUX_TYPE ftMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t ftCount = 0, ftLastRise = 0, ftHighUs = 0, ftGlitch = 0;
static volatile uint32_t ftRing[RING];
static volatile uint16_t ftRingHead = 0;

void IRAM_ATTR ftIsr() {
  const uint32_t t = (uint32_t)esp_timer_get_time();
  const bool high = (REG_READ(GPIO_IN_REG) >> pin::FT) & 1U;
  portENTER_CRITICAL_ISR(&ftMux);
  if (high) {
    if (t - ftLastRise >= FT_MIN_PERIOD_US) {
      ftLastRise = t;
      ftCount++;
      ftRing[ftRingHead] = t;
      ftRingHead = (ftRingHead + 1) & (RING - 1);
    } else {
      ftGlitch++;
    }
  } else {
    ftHighUs = t - ftLastRise;
  }
  portEXIT_CRITICAL_ISR(&ftMux);
}

struct Flow {
  static constexpr uint8_t H = 32;  // 100 ms snapshots
  uint32_t hc[H] = {0}, ht[H] = {0};
  uint8_t head = 0, n = 0;
  uint32_t lastCount = 0;
  float f = 0, q = 0, qShow = 0, dutyPct = 0;
  double vTot = 0, vC1 = 0, vC2 = 0;  // litres
  float qHist[30] = {0};              // last 3 s of flow
  uint8_t qh = 0;

  void sample(float dt) {
    uint32_t c, tr, hi;
    portENTER_CRITICAL(&ftMux);
    c = ftCount;
    tr = ftLastRise;
    hi = ftHighUs;
    portEXIT_CRITICAL(&ftMux);
    const uint32_t tNow = (uint32_t)esp_timer_get_time();

    hc[head] = c;
    ht[head] = tr;
    head = (head + 1) % H;
    if (n < H) n++;

    // frequency from the pulse timestamps over the last 0.5 s
    const uint8_t back = n - 1 < 5 ? n - 1 : 5;
    const uint8_t k = (head + H - 1 - back) % H;
    float fNew = 0;
    if (back > 0 && c > hc[k] && tr != ht[k]) fNew = (float)(c - hc[k]) * 1e6f / (float)(tr - ht[k]);
    if (fNew > 0) {
      const float sinceLast = (float)(tNow - tr);
      if (sinceLast > 1.5e6f / fNew) fNew = min(fNew, 1e6f / sinceLast);  // no new pulse, flow is stopping
    }
    f = fNew;
    if (f > 0 && hi > 0) dutyPct = 100.0f * (float)hi * f / 1e6f;

    const bool flowing = f > P("fcut");
    q = flowing ? max(0.0f, P("a") * f + P("b")) : 0.0f;
    qShow += (q - qShow) * 0.5f;

    const uint32_t dN = c - lastCount;
    lastCount = c;
    double dv = (double)P("a") * dN / 60.0 + (flowing ? (double)P("b") * dt / 60.0 : 0.0);
    if (dv < 0) dv = 0;
    vTot += dv;
    vC1 += inCase1 ? dv : 0;
    vC2 += inCase2 ? dv : 0;

    qHist[qh] = q;
    qh = (qh + 1) % 30;
  }

  float mean3s() const {
    float s = 0;
    for (float x : qHist) s += x;
    return s / 30.0f;
  }
  float cv3s() const {
    const float m = mean3s();
    if (m < 0.05f) return 0;
    float s = 0;
    for (float x : qHist) s += (x - m) * (x - m);
    return sqrtf(s / 30.0f) / m;
  }
  bool inCase1 = false, inCase2 = false;
} flow;

struct CurrentSense {
  uint8_t pinNo;
  const char* gainKey;
  CurrentSense(uint8_t p, const char* k) : pinNo(p), gainKey(k) {}
  float emaMv = 0, offsetMv = 0, amps = 0;
  float hist[20] = {0};  // 2 s at 10 Hz
  uint8_t h = 0;
  void begin() { analogSetPinAttenuation(pinNo, ADC_2_5db); }
  void read() {
    const float mv = (float)analogReadMilliVolts(pinNo);
    emaMv += (mv - emaMv) * 0.1f;
    amps = max(0.0f, (emaMv - offsetMv) * P(gainKey));
  }
  void push() {
    hist[h] = amps;
    h = (h + 1) % 20;
  }
  float cv2s() const {
    float m = 0, s = 0;
    for (float x : hist) m += x;
    m /= 20.0f;
    if (m < 0.05f) return 0;
    for (float x : hist) s += (x - m) * (x - m);
    return sqrtf(s / 20.0f) / m;
  }
  void zero() { offsetMv = emaMv; }
};
static CurrentSense is1{pin::IS1, "g1"}, is2{pin::IS2, "g2"}, is3{pin::IS3, "g3"};

struct Touch {
  uint16_t ring[5] = {0};
  uint8_t i = 0;
  float value = 0;
  void sample() {
    ring[i] = touchRead(pin::TOUCH);
    i = (i + 1) % 5;
    uint16_t s[5];
    memcpy(s, ring, sizeof s);
    for (int a = 1; a < 5; a++)
      for (int b = a; b > 0 && s[b - 1] > s[b]; b--) {
        uint16_t t = s[b];
        s[b] = s[b - 1];
        s[b - 1] = t;
      }
    value = s[2];  // median of 5
  }
} touch;

struct Fic {
  bool on = false;
  float iterm = 0, out = 0;
  void reset() {
    iterm = 0;
    out = 0;
  }
  float update(float pv, float dt) {
    const float sp = P("sp"), kc = P("kc"), ti = P("ti");
    const float ff = constrain((sp - P("ffq0")) / P("ffq1"), 0.0f, 1.0f);
    const float e = sp - pv;
    const float dI = kc * dt / ti * e;
    const float trial = ff + kc * e + iterm + dI;
    if ((trial < 1.0f || e < 0) && (trial > 0.15f || e > 0)) iterm += dI;  // anti-windup
    out = constrain(ff + kc * e + iterm, 0.15f, 1.0f);
    return out;
  }
} fic;

enum class St : uint8_t {
  IDLE, C1_START, C1_RUN, C1_HOLD, C2_ISOLATE, C2_WAIT_BV, C2_FLUSH, C2_HOLD,
  COMPLETE, STOPPED, MANUAL, CAL, FIC, FIC_HOLD
};
static const char* stName(St s) {
  switch (s) {
    case St::IDLE: return "IDLE";
    case St::C1_START: return "C1_START";
    case St::C1_RUN: return "C1_RUN";
    case St::C1_HOLD: return "C1_HOLD_LSH";
    case St::C2_ISOLATE: return "C2_ISOLATE";
    case St::C2_WAIT_BV: return "C2_OPEN_BV01";
    case St::C2_FLUSH: return "C2_FLUSH";
    case St::C2_HOLD: return "C2_HOLD_LSH";
    case St::COMPLETE: return "COMPLETE";
    case St::STOPPED: return "STOPPED";
    case St::MANUAL: return "MANUAL";
    case St::CAL: return "CAL";
    case St::FIC: return "FIC";
    case St::FIC_HOLD: return "FIC_HOLD_LSH";
  }
  return "?";
}

static St state = St::IDLE;
static uint32_t stateMs = 0, c1StartMs = 0, flushStartMs = 0, p1StopMs = 0;
static uint32_t lshEvents = 0, p1OnMs = 0, p3RunMs = 0, lastCmdMs = 0;
static bool ilkBypass = false;  // manual mode only
static uint32_t alarms = 0;
enum : uint32_t {
  A_LSH = 1u << 0, A_P2_DRY = 1u << 1, A_P2_BLOCK = 1u << 2, A_FT_FAIL = 1u << 3,
  A_PUMP_FAIL = 1u << 4, A_BV_TIMEOUT = 1u << 5, A_FLUSH_TIMEOUT = 1u << 6,
  A_BROWNOUT = 1u << 7, A_ILK = 1u << 8, A_TOUCH = 1u << 9, A_FLUSH_SUCTION = 1u << 10
};
static bool p2Trip = false;
static uint32_t p2TripMs = 0;
static uint8_t p2Retries = 0;
static uint8_t flushReprompts = 0;

static uint32_t inState() { return nowMs - stateMs; }
static void enter(St s, const char* why) {
  event("STATE", "%s -> %s (%s)", stName(state), stName(s), why);
  state = s;
  stateMs = nowMs;
}
static void allOff() {
  P1.cmd = P2.cmd = P3.cmd = 0;
  sov.cmd = false;
  fic.on = false;
}

// calibration run
static uint32_t calCount0 = 0, calT0Us = 0, calT1Us = 0, calRunMs = 0, calId = 0;
static float calDuty = 0;
static bool calStopped = false;

static uint32_t ftSnapshotCount() {
  portENTER_CRITICAL(&ftMux);
  const uint32_t c = ftCount;
  portEXIT_CRITICAL(&ftMux);
  return c;
}

// float high: stop P-01 and close SOV-01. restart when the float clears
static void p1LevelControl(float duty, St holdState, St runState) {
  if (state == runState) {
    if (lsh.stable) {
      P1.cmd = 0;
      sov.cmd = false;
      p1StopMs = nowMs;
      lshEvents++;
      enter(holdState, "LS-01 high: P-01 stop, SOV-01 close");
      return;
    }
    const bool p2Ok = P2.runMs() > 1000;
    if (p2Ok && nowMs - p1StopMs >= (uint32_t)P("p1off")) {
      sov.cmd = true;
      if (sov.openFor(300)) P1.cmd = duty;
    }
  } else if (state == holdState) {
    P1.cmd = 0;
    sov.cmd = false;
    if (!lsh.stable && nowMs - p1StopMs >= (uint32_t)P("p1off")) enter(runState, "LS-01 clear: P-01 restart");
  }
}

static void beginSequence() {
  alarms &= A_BROWNOUT;
  p2Trip = false;
  p2Retries = 0;
  lshEvents = 0;
  p1OnMs = 0;
  p3RunMs = 0;
  flushReprompts = 0;
  flow.vC1 = flow.vC2 = 0;
  allOff();
  enter(St::C1_START, "operator start");
}

static void shutdownComplete(const char* source, uint32_t conditionSinceMs) {
  allOff();
  flow.inCase1 = flow.inCase2 = false;
  event("FLUSH_EMPTY", "source=%s detect_latency_ms=%lu flush_s=%.1f vC2_L=%.3f", source,
        (unsigned long)(nowMs - conditionSinceMs), (nowMs - flushStartMs) / 1000.0f, flow.vC2);
  enter(St::COMPLETE, "safe OFF: pumps braked, SOV-01 closed");
}

static uint32_t touchEmptySince = 0, i3DrySince = 0;

// touch probe is used only after TOUCHCAL EMPTY. wet reads lower than dry
static bool probeArmed() { return P("tchE") > 0 && P("tchW") > 0 && P("tchE") > P("tchW"); }
static bool probeDry() { return probeArmed() && touch.value > P("tchW") + P("tchF") * (P("tchE") - P("tchW")); }

static void stepSequence() {
  switch (state) {
    case St::IDLE:
    case St::STOPPED:
    case St::COMPLETE:
      allOff();
      break;

    case St::C1_START:
      allOff();
      if (inState() >= 400 && inState() < 420) {
        is1.zero();
        is2.zero();
        is3.zero();
      }
      if (inState() >= 600) {
        c1StartMs = nowMs;
        flow.inCase1 = true;
        if (P("c1fic") > 0.5f) {
          fic.reset();
          fic.on = true;
        } else {
          P2.cmd = P("d2");
        }
        p1StopMs = nowMs - (uint32_t)P("p1off");
        enter(St::C1_RUN, P("c1fic") > 0.5f ? "P-02 on FIC-01" : "P-02 fixed duty");
      }
      break;

    case St::C1_RUN:
    case St::C1_HOLD:
      if (!fic.on) P2.cmd = P("d2");
      p1LevelControl(P("d1"), St::C1_HOLD, St::C1_RUN);
      if (P1.running()) p1OnMs += 10;
      if (state == St::C1_RUN && nowMs - c1StartMs >= (uint32_t)(P("tc1") * 1000)) {
        const float tS = (nowMs - c1StartMs) / 1000.0f;
        event("C1_SUMMARY", "t_s=%.0f v_L=%.3f qavg_lpm=%.3f lsh_events=%lu p1_on_pct=%.0f", tS, flow.vC1,
              flow.vC1 * 60.0f / tS, (unsigned long)lshEvents, 100.0f * p1OnMs / (tS * 1000.0f));
        flow.inCase1 = false;
        flow.inCase2 = true;
        enter(St::C2_ISOLATE, "Case 1 time complete");
      }
      break;

    case St::C2_ISOLATE:
      P1.cmd = 0;
      P3.cmd = 0;
      sov.cmd = false;
      if (inState() >= 1000) enter(St::C2_WAIT_BV, "process tank isolated, open BV-01");
      break;

    case St::C2_WAIT_BV: {
      P1.cmd = 0;
      P3.cmd = 0;
      sov.cmd = false;
      Param* w = findParam("tchW");
      if (inState() > 500 && touch.value > 0) w->v = touch.value;  // TK-03 is full here
      const bool zsoMode = P("bvmode") > 0.5f;
      const uint32_t limitMs = (uint32_t)(P("tbv") * 1000);
      if (zsoMode) {
        if (zso.stable) enter(St::C2_FLUSH, "ZSO-01 confirms BV-01 open");
        else if (inState() > limitMs && !(alarms & A_BV_TIMEOUT)) {
          alarms |= A_BV_TIMEOUT;
          event("ALARM", "BV-01 not confirmed open after %.0f s", P("tbv"));
        }
      } else if (inState() >= limitMs) {
        enter(St::C2_FLUSH, "BV-01 window elapsed");
      }
      if (state == St::C2_FLUSH) {
        flushStartMs = nowMs;
        touchEmptySince = i3DrySince = 0;
      }
      break;
    }

    case St::C2_FLUSH:
    case St::C2_HOLD: {
      if (state == St::C2_FLUSH) {
        if (lsh.stable) {
          P1.cmd = P3.cmd = 0;
          p1StopMs = nowMs;
          lshEvents++;
          enter(St::C2_HOLD, "LS-01 high: P-01 and P-03 stop together");
          break;
        }
        P1.cmd = P("d1f");
        if (P1.runMs() > 500) P3.cmd = P("d3");
      } else {
        P1.cmd = P3.cmd = 0;
        if (!lsh.stable && nowMs - p1StopMs >= (uint32_t)P("p1off")) enter(St::C2_FLUSH, "LS-01 clear: flush resumes");
        break;
      }

      const float tFlush = (nowMs - flushStartMs) / 1000.0f;
      // probe dry = flush tank empty
      if (probeArmed() && tFlush >= P("tfmin")) {
        if (probeDry()) {
          if (!touchEmptySince) touchEmptySince = nowMs;
          if (nowMs - touchEmptySince >= 300) {
            shutdownComplete("LSL-03 probe dry", touchEmptySince);
            break;
          }
        } else {
          touchEmptySince = 0;
        }
      }
      // P-03 current low = pumping air. if the probe is still wet then BV-01 is shut, the tank is not empty
      if (P("i3dry") > 0 && P3.runMs() > 3000) {
        if (is3.amps < P("i3dry")) {
          if (!i3DrySince) i3DrySince = nowMs;
          if (nowMs - i3DrySince >= 1500) {
            if (probeArmed() && !probeDry()) {
              P1.cmd = P3.cmd = 0;
              alarms |= A_FLUSH_SUCTION;
              if (++flushReprompts > 3) {
                allOff();
                flow.inCase2 = false;
                enter(St::STOPPED, "P-03 cannot draw from a full TK-03 after 3 prompts");
              } else {
                enter(St::C2_WAIT_BV, "P-03 dry but TK-03 has liquid: open BV-01");
              }
              break;
            }
            shutdownComplete("IT-03 dry-run current", i3DrySince);
            break;
          }
        } else {
          i3DrySince = 0;
        }
      }
      // no probe: stop after P-03 has run tfl seconds (time we measured to empty TK-03)
      if (!probeArmed() && P("tfl") > 0 && p3RunMs >= (uint32_t)(P("tfl") * 1000)) {
        shutdownComplete("P-03 run timer (TK-03 volume)", nowMs);
        break;
      }
      if (tFlush >= P("tfmax")) {
        alarms |= A_FLUSH_TIMEOUT;
        shutdownComplete("flush timeout backstop", nowMs);
      }
      break;
    }

    case St::MANUAL:
      break;

    case St::CAL:
      if (!calStopped && inState() >= calRunMs) {
        P2.cmd = 0;
        calStopped = true;
        calT1Us = (uint32_t)esp_timer_get_time();
      }
      if (calStopped) {
        P2.cmd = 0;
        uint32_t last;
        portENTER_CRITICAL(&ftMux);
        last = ftLastRise;
        portEXIT_CRITICAL(&ftMux);
        if ((uint32_t)esp_timer_get_time() - last > 1500000) {  // wait till the rotor stops so the last pulses are counted
          const uint32_t n = ftSnapshotCount() - calCount0;
          const float t = (calT1Us - calT0Us) / 1e6f;
          Serial.printf("C,%lu,%lu,%lu,%.3f,%.3f,%.3f\n", (unsigned long)millis(), (unsigned long)calId,
                        (unsigned long)n, t, n / t, calDuty);
          event("CAL_DONE", "run=%lu pulses=%lu t_s=%.3f f_hz=%.2f duty=%.2f -> weigh the bucket", (unsigned long)calId,
                (unsigned long)n, t, n / t, calDuty);
          enter(St::IDLE, "calibration run complete");
        }
      }
      break;

    case St::FIC:
    case St::FIC_HOLD:
      p1LevelControl(P("d1fic"), St::FIC_HOLD, St::FIC);
      break;
  }
}

enum class Dx : uint8_t {
  NORMAL, PUMP_OFF, RESTRICTION, DISCHARGE_BLOCKED, SUCTION_LOSS, PUMP_FAILURE,
  SENSOR_FAILURE, UNSTABLE_FLOW, SENSOR_INTERMITTENT, SPURIOUS_PULSES, LEARNING, NO_FLOW_UNKNOWN, LOW_FLOW,
  PARTIAL_SUCTION
};
static const char* dxName(Dx d) {
  switch (d) {
    case Dx::NORMAL: return "NORMAL";
    case Dx::PUMP_OFF: return "PUMP_OFF";
    case Dx::RESTRICTION: return "RESTRICTION";
    case Dx::DISCHARGE_BLOCKED: return "DISCHARGE_BLOCKED";
    case Dx::SUCTION_LOSS: return "SUCTION_LOSS";
    case Dx::PUMP_FAILURE: return "PUMP_FAILURE";
    case Dx::SENSOR_FAILURE: return "FT01_SIGNAL_FAILURE";
    case Dx::UNSTABLE_FLOW: return "UNSTABLE_FLOW";
    case Dx::SENSOR_INTERMITTENT: return "FT01_INTERMITTENT";
    case Dx::SPURIOUS_PULSES: return "SPURIOUS_PULSES";
    case Dx::LEARNING: return "LEARNING";
    case Dx::NO_FLOW_UNKNOWN: return "NO_FLOW_CAUSE_UNKNOWN";
    case Dx::LOW_FLOW: return "LOW_FLOW";
    case Dx::PARTIAL_SUCTION: return "PARTIAL_SUCTION";
  }
  return "?";
}

struct Fdi {
  Dx shown = Dx::PUMP_OFF, candidate = Dx::PUMP_OFF;
  uint32_t candSinceMs = 0, shownSinceMs = 0;
  float qRef = 0, iRef = 0, opRef = -1;
  uint32_t settleSinceMs = 0;
  bool learned = false;

  void relearn() {
    learned = false;
    settleSinceMs = nowMs;
  }

  Dx classify() {
    const bool p2On = P2.cmd > 0.05f;
    const float q = flow.mean3s(), i = is2.amps;
    if (!p2On) return flow.f > 20 ? Dx::SPURIOUS_PULSES : Dx::PUMP_OFF;
    if (P2.runMs() < 3000) return Dx::LEARNING;
    // duty or set point changed, learn the normal flow again
    const float op = fic.on ? P("sp") : P2.out;
    if (fabsf(op - opRef) > (fic.on ? 0.05f : 0.02f)) {
      opRef = op;
      relearn();
    }
    if (!learned) {
      if (nowMs - settleSinceMs > 4000 && q > 0.3f) {
        qRef = q;
        iRef = max(i, 0.05f);
        learned = true;
        event("FDI", "baseline learned q=%.2f L/min i2=%.2f A duty=%.2f", qRef, iRef, P2.out);
      }
      return Dx::LEARNING;
    }
    const bool noFlow = flow.f < P("fcut");
    const float qr = q / qRef;
    if (P("isok") < 0.5f) {  // no current sensing, so we cannot say why there is no flow
      if (noFlow) return Dx::NO_FLOW_UNKNOWN;
      if (flow.cv3s() > 0.15f) return Dx::UNSTABLE_FLOW;
      return qr < 0.85f ? Dx::LOW_FLOW : Dx::NORMAL;
    }
    const float ir = i / iRef;
    if (i < 0.10f) return Dx::PUMP_FAILURE;
    if (noFlow) {
      if (ir > 1.15f || is2.cv2s() > 0.35f) return Dx::DISCHARGE_BLOCKED;
      if (ir < 0.80f) return Dx::SUCTION_LOSS;
      return Dx::SENSOR_FAILURE;
    }
    if (flow.cv3s() > 0.15f) return is2.cv2s() > 0.10f ? Dx::UNSTABLE_FLOW : Dx::SENSOR_INTERMITTENT;
    if (fic.on) {  // under flow control a restriction shows up as extra duty
      const float ff = constrain((P("sp") - P("ffq0")) / P("ffq1"), 0.0f, 1.0f);
      if (P2.out > ff + 0.15f && ir > 1.08f) return Dx::RESTRICTION;
    }
    if (qr < 0.85f) return ir > 1.08f ? Dx::RESTRICTION : (ir < 0.85f ? Dx::PARTIAL_SUCTION : Dx::SENSOR_INTERMITTENT);
    return Dx::NORMAL;
  }

  void step() {
    if (P("fdi") < 0.5f) return;
    const Dx d = classify();
    if (d != candidate) {
      candidate = d;
      candSinceMs = nowMs;
    }
    if (candidate != shown && nowMs - candSinceMs >= 1500) {
      shown = candidate;
      shownSinceMs = nowMs;
      event("DIAG", "%s q=%.2f i2=%.2f qref=%.2f iref=%.2f", dxName(shown), flow.mean3s(), is2.amps, qRef, iRef);
    }
    alarms = (shown == Dx::SENSOR_FAILURE || shown == Dx::SENSOR_INTERMITTENT) ? (alarms | A_FT_FAIL) : (alarms & ~A_FT_FAIL);
    alarms = (shown == Dx::PUMP_FAILURE) ? (alarms | A_PUMP_FAIL) : (alarms & ~A_PUMP_FAIL);

    // trip only on real no-flow, not on a noisy reading
    const bool autoMode = state == St::C1_RUN || state == St::C1_HOLD || state == St::C2_FLUSH ||
                          state == St::C2_HOLD || state == St::FIC || state == St::FIC_HOLD;
    if (P("trip") > 0.5f && autoMode && !p2Trip) {
      if (shown == Dx::SUCTION_LOSS && nowMs - shownSinceMs > 5000) {
        p2Trip = true;
        p2TripMs = nowMs;
        alarms |= A_P2_DRY;
        event("TRIP", "P-02 dry run / loss of suction");
      } else if (shown == Dx::NO_FLOW_UNKNOWN && nowMs - shownSinceMs > 5000) {
        // could be a dry sump, a blocked line or a dead sensor. stop P-02 anyway
        p2Trip = true;
        p2TripMs = nowMs;
        alarms |= A_P2_DRY;
        event("TRIP", "P-02 no flow for 5 s (dry sump, blocked line or FT-01 failure)");
      } else if (shown == Dx::DISCHARGE_BLOCKED && nowMs - shownSinceMs > 3000) {
        p2Trip = true;
        p2TripMs = nowMs;
        alarms |= A_P2_BLOCK;
        event("TRIP", "P-02 discharge blocked");
      }
    }
    // dry-run trip in case 1: try again after 10 s, 3 times max. a blocked trip stays until reset
    if (p2Trip && (alarms & A_P2_DRY) && nowMs - p2TripMs > 10000 && p2Retries < 3 &&
        (state == St::C1_RUN || state == St::C1_HOLD || state == St::FIC || state == St::FIC_HOLD)) {
      p2Trip = false;
      p2Retries++;
      alarms &= ~A_P2_DRY;
      relearn();
      event("RESTORE", "P-02 restart attempt %u after dry-run trip", p2Retries);
    }
  }
} fdi;

// last check before the outputs are written. if the sequence asks for something unsafe this overrides it
static void interlocks() {
  static uint32_t lastIlkMs = 0;
  auto violation = [](const char* what) {
    alarms |= A_ILK;
    if (nowMs - lastIlkMs > 2000) {
      lastIlkMs = nowMs;
      event("INTERLOCK", "%s", what);
    }
  };
  if (lsh.stable) {  // I-1 sump full
    if (P1.cmd > 0 || P3.cmd > 0 || sov.cmd) {
      if (state == St::MANUAL) violation("I-1 LS-01 high: P-01/P-03 off, SOV-01 closed");
      P1.cmd = P3.cmd = 0;
      sov.cmd = false;
    }
  }
  if (!(ilkBypass && state == St::MANUAL)) {
    if (P3.cmd > 0 && (sov.cmd || sov.out || !P1.running())) {  // I-2 P-03 only with P-01 running and SOV-01 closed
      violation("I-2 P-03 needs P-01 running and SOV-01 closed");
      P3.cmd = 0;
    }
    if (sov.cmd && P3.out > 0) {  // I-3 keep flush water out of TK-01
      violation("I-3 SOV-01 kept closed while P-03 runs");
      sov.cmd = false;
    }
  }
  if (p2Trip) P2.cmd = 0;  // I-4 P-02 trip
}

static char line[96];
static uint8_t lineLen = 0;

static void printHeader() {
  Serial.println(
      "H,t_ms,state,q_lpm,q_mls,f_hz,v_tot_l,v_c1_l,v_c2_l,d1,d2,d3,i1_a,i2_a,i3_a,lsh,sov,zso,touch,diag,alarms,sp,"
      "ft_duty_pct,glitches,n_ft,p3_run_s");
}

static void printParams() {
  for (auto& p : params) Serial.printf("P,%s,%.6g,%s\n", p.key, p.v, p.help);
}

static void dumpBurst() {
  uint32_t buf[RING];
  uint16_t h;
  portENTER_CRITICAL(&ftMux);
  memcpy(buf, (const void*)ftRing, sizeof buf);
  h = ftRingHead;
  portEXIT_CRITICAL(&ftMux);
  Serial.print("B");
  int printed = 0;
  for (uint16_t k = 1; k < RING; k++) {
    const uint32_t a = buf[(h + k - 1) & (RING - 1)], b = buf[(h + k) & (RING - 1)];
    if (a == 0 || b <= a) continue;
    Serial.printf(",%lu", (unsigned long)(b - a));
    if (++printed % 64 == 0) Serial.print("\nB");
  }
  Serial.println();
  event("BURST", "%d inter-pulse periods (us) dumped", printed);
}

static Pump* pumpByNo(int n) { return n == 1 ? &P1 : n == 2 ? &P2 : n == 3 ? &P3 : nullptr; }

static void handleCommand(char* s) {
  char* cmd = strtok(s, " \t");
  if (!cmd) return;
  char* a1 = strtok(nullptr, " \t");
  char* a2 = strtok(nullptr, " \t");
  const bool idleLike = state == St::IDLE || state == St::STOPPED || state == St::COMPLETE;
  lastCmdMs = nowMs;

  if (!strcasecmp(cmd, "START")) {
    if (idleLike) beginSequence();
  } else if (!strcasecmp(cmd, "STOP")) {
    allOff();
    flow.inCase1 = flow.inCase2 = false;
    enter(St::STOPPED, "operator stop");
  } else if (!strcasecmp(cmd, "RESET")) {
    allOff();
    alarms = 0;
    p2Trip = false;
    ilkBypass = false;
    enter(St::IDLE, "reset");
  } else if (!strcasecmp(cmd, "NEXT")) {  // skip the rest of case 1 (testing)
    if (state == St::C1_RUN) c1StartMs = nowMs - (uint32_t)(P("tc1") * 1000);
  } else if (!strcasecmp(cmd, "MANUAL")) {
    allOff();
    enter(St::MANUAL, "operator");
  } else if (!strcasecmp(cmd, "P") && a1 && a2) {
    Pump* p = pumpByNo(atoi(a1));
    if (state != St::MANUAL) event("CMD", "P needs MANUAL mode");
    else if (p) p->cmd = constrain(atof(a2) / 100.0f, 0.0f, 1.0f);
  } else if (!strcasecmp(cmd, "SOV") && a1) {
    if (state == St::MANUAL) sov.cmd = atoi(a1) != 0;
    else event("CMD", "SOV needs MANUAL mode");
  } else if (!strcasecmp(cmd, "ILK") && a1) {
    ilkBypass = state == St::MANUAL && atoi(a1) == 0;
    event("ILK", ilkBypass ? "I-2/I-3 BYPASSED (manual test only)" : "interlocks normal");
  } else if (!strcasecmp(cmd, "FIC")) {
    if (idleLike || state == St::MANUAL) {
      allOff();
      fic.reset();
      fic.on = true;
      p1StopMs = nowMs - (uint32_t)P("p1off");
      enter(St::FIC, "closed-loop flow control");
    }
  } else if (!strcasecmp(cmd, "CAL") && a1 && a2) {
    if (idleLike) {
      calRunMs = (uint32_t)(atof(a1) * 1000);
      calDuty = constrain(atof(a2) / 100.0f, 0.0f, 1.0f);
      calId++;
      calStopped = false;
      calCount0 = ftSnapshotCount();
      calT0Us = (uint32_t)esp_timer_get_time();
      allOff();
      P2.cmd = calDuty;
      enter(St::CAL, "timed run into the weigh bucket");
    }
  } else if (!strcasecmp(cmd, "SET") && a1 && a2) {
    Param* p = findParam(a1);
    if (p) {
      p->v = constrain((float)atof(a2), p->lo, p->hi);
      event("SET", "%s=%.6g", p->key, p->v);
      if (!strcasecmp(p->key, "sp") || !strcasecmp(p->key, "d2")) fdi.relearn();
    } else {
      event("CMD", "unknown parameter %s", a1);
    }
  } else if (!strcasecmp(cmd, "GET")) {
    printParams();
  } else if (!strcasecmp(cmd, "SAVE")) {
    saveParams();
    event("SAVE", "parameters written to flash");
  } else if (!strcasecmp(cmd, "DEFAULTS")) {
    for (auto& p : params) p.v = p.def;
    event("SET", "defaults restored (not saved)");
  } else if (!strcasecmp(cmd, "ZERO")) {
    flow.vTot = flow.vC1 = flow.vC2 = 0;
    event("ZERO", "totalisers cleared");
  } else if (!strcasecmp(cmd, "IZERO")) {
    is1.zero();
    is2.zero();
    is3.zero();
    event("IZERO", "current offsets %.0f/%.0f/%.0f mV", is1.offsetMv, is2.offsetMv, is3.offsetMv);
  } else if (!strcasecmp(cmd, "TOUCHCAL") && a1) {
    Param* p = findParam(!strcasecmp(a1, "EMPTY") ? "tchE" : "tchW");
    p->v = touch.value;
    event("TOUCHCAL", "%s=%.0f", p->key, p->v);
  } else if (!strcasecmp(cmd, "LEARN")) {
    fdi.relearn();
  } else if (!strcasecmp(cmd, "PING")) {  // keep-alive from the logger
  } else if (!strcasecmp(cmd, "MARK")) {
    event("MARK", "%s", a1 ? a1 : "-");
  } else if (!strcasecmp(cmd, "BURST")) {
    dumpBurst();
  } else if (!strcasecmp(cmd, "PINS")) {
    Serial.printf("I,button=%d float=%d lsh=%d zso=%d ft=%d touch=%u is1=%umV is2=%umV is3=%umV glitches=%lu\n",
                  digitalRead(pin::BUTTON), digitalRead(pin::FLOAT), lsh.stable ? 1 : 0, digitalRead(pin::ZSO), digitalRead(pin::FT),
                  (unsigned)touchRead(pin::TOUCH), (unsigned)analogReadMilliVolts(pin::IS1),
                  (unsigned)analogReadMilliVolts(pin::IS2), (unsigned)analogReadMilliVolts(pin::IS3),
                  (unsigned long)ftGlitch);
  } else if (!strcasecmp(cmd, "HDR")) {
    Serial.printf("I,fw=%s core=%d.%d.%d up_s=%lu state=%s\n", FW_VERSION, ESP_ARDUINO_VERSION_MAJOR,
                  ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH, (unsigned long)(millis() / 1000), stName(state));
    printHeader();
  } else {
    event("CMD", "unknown: %s", cmd);
  }
}

static void pollSerial() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen) {
        line[lineLen] = 0;
        handleCommand(line);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    }
  }
}

static void telemetry() {
  Serial.printf("D,%lu,%s,%.3f,%.1f,%.1f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%.0f,%s,%lu,%.2f,%.0f,%lu,%lu,%.1f\n",
                (unsigned long)millis(), stName(state), flow.qShow, flow.qShow * 1000.0f / 60.0f, flow.f, flow.vTot,
                flow.vC1, flow.vC2, P1.out, P2.out, P3.out, is1.amps, is2.amps, is3.amps, lsh.stable ? 1 : 0,
                sov.out ? 1 : 0, zso.stable ? 1 : 0, touch.value, dxName(fdi.shown), (unsigned long)alarms,
                fic.on ? P("sp") : 0.0f, flow.dutyPct, (unsigned long)ftGlitch, (unsigned long)ftSnapshotCount(),
                p3RunMs / 1000.0f);
}

static void led() {
  const uint32_t t = nowMs;
  bool on;
  switch (state) {
    case St::C2_WAIT_BV: on = (t / 100) % 2; break;                 // fast blink = open BV-01
    case St::C1_HOLD:
    case St::C2_HOLD: on = (t % 1000) < 100 || ((t % 1000) > 200 && (t % 1000) < 300); break;
    case St::COMPLETE: on = (t % 2000) < 1000; break;
    case St::IDLE: on = (t % 1000) < 50; break;
    default: on = true;
  }
  if (alarms & (A_P2_DRY | A_P2_BLOCK | A_FT_FAIL | A_PUMP_FAIL | A_FLUSH_TIMEOUT)) on = (t / 50) % 2;
  digitalWrite(pin::LED, on);
}

void setup() {
  // all driver pins low first so the pumps do not twitch at boot
  for (uint8_t p : {pin::P1_REN, pin::P1_LEN, pin::P1_LPWM, pin::P1_RPWM, pin::P2_REN, pin::P2_LEN, pin::P2_LPWM,
                    pin::P2_RPWM, pin::P3_REN, pin::P3_LEN, pin::P3_LPWM, pin::P3_RPWM, pin::SOV}) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  Serial.begin(115200);
  loadParams();
  P1.begin();
  P2.begin();
  P3.begin();
  pinMode(pin::BUTTON, INPUT);
  pinMode(pin::FLOAT, INPUT);
  pinMode(pin::ZSO, INPUT_PULLUP);
  pinMode(pin::FT, INPUT);
  pinMode(pin::LED, OUTPUT);
  is1.begin();
  is2.begin();
  is3.begin();
  attachInterrupt(digitalPinToInterrupt(pin::FT), ftIsr, CHANGE);
  enableLoopWDT();

  const esp_reset_reason_t r = esp_reset_reason();
  if (r == ESP_RST_BROWNOUT) alarms |= A_BROWNOUT;
  delay(200);
  event("BOOT", "lam_flow_rig fw %s reset_reason=%d core=%d.%d.%d", FW_VERSION, (int)r, ESP_ARDUINO_VERSION_MAJOR,
        ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
  printHeader();
}

void loop() {
  static uint32_t tick10 = 0, tick50 = 0, tick100 = 0;
  nowMs = millis();
  pollSerial();

  if (nowMs - tick10 >= 10) {
    const float dt = (nowMs - tick10) / 1000.0f;
    tick10 = nowMs;

    // inputs
    is1.read();
    is2.read();
    is3.read();
    lsh.update((digitalRead(pin::FLOAT) == HIGH) != (P("finv") > 0.5f), (uint32_t)P("lshon"), (uint32_t)P("lshoff"));
    zso.update(digitalRead(pin::ZSO) == LOW, 300, 300);
    if (button.update(digitalRead(pin::BUTTON) == LOW, 30, 30) && button.stable) {
      const bool idleLike = state == St::IDLE || state == St::STOPPED || state == St::COMPLETE;
      if (state == St::IDLE) beginSequence();
      else if (idleLike) enter(St::IDLE, "operator reset");
      else {
        allOff();
        flow.inCase1 = flow.inCase2 = false;
        enter(St::STOPPED, "operator stop button");
      }
    }
    alarms = lsh.stable ? (alarms | A_LSH) : (alarms & ~A_LSH);

    // logic
    if (fic.on) P2.cmd = fic.update(flow.q, dt);
    stepSequence();
    if (state == St::C2_FLUSH && P3.running()) p3RunMs += 10;
    if (state == St::MANUAL && P("mwd") > 0 && (P1.cmd > 0 || P2.cmd > 0 || P3.cmd > 0 || sov.cmd) &&
        nowMs - lastCmdMs > (uint32_t)(P("mwd") * 1000)) {
      allOff();
      event("WATCHDOG", "MANUAL: no command for %.0f s, outputs off", P("mwd"));
    }
    interlocks();

    // outputs
    P1.update(dt);
    P2.update(dt);
    P3.update(dt);
    sov.update();
    led();
  }
  if (nowMs - tick50 >= 50) {
    tick50 = nowMs;
    touch.sample();
  }
  if (nowMs - tick100 >= 100) {
    const float dt = (nowMs - tick100) / 1000.0f;
    tick100 = nowMs;
    flow.sample(dt);
    is1.push();
    is2.push();
    is3.push();
    fdi.step();
    telemetry();
  }
}

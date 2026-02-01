#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// ============================================================
// Pin Assign
// ============================================================
// TMC2209 (STEP/DIR)
const uint8_t PIN_STEP = 9;   // D9 = OC1A (Timer1 hardware toggle)
const uint8_t PIN_DIR  = 3;
const uint8_t PIN_EN   = 4;   // LOW=enable が多い（逆なら HIGH に変更）

// Grove
const uint8_t PIN_KNOB_SPEED  = A3; // Rotary Angle Sensor（速度）
const uint8_t PIN_SLIDER_BRT  = A0; // Slide Pot（明るさ）
const uint8_t PIN_SLIDER_LED  = A1; // Slide Pot内蔵LED入力（Pin2想定）
const uint8_t PIN_TOUCH       = 7;  // Touch Sensor（方向反転）
const uint8_t PIN_BUZZ        = 6;  // Grove Buzzer
const uint8_t PIN_MODE_BTN    = 8;  // Grove Button（モード切替）

// WS2813 (Grove)
const uint8_t  PIN_LED   = 5;   // WS2813 Data
const uint16_t NUM_LEDS  = 60;  // 60LED
Adafruit_NeoPixel strip(NUM_LEDS, PIN_LED, NEO_GRB + NEO_KHZ800);

// ============================================================
// Step rate range
// ============================================================
const uint32_t MIN_STEP_SPS = 100;
const uint32_t MAX_STEP_SPS = 6000;

// Speed knob update smoothing
const uint16_t UPDATE_MS_SPEED   = 30;
const float    EMA_ALPHA_SPEED   = 0.15f;
const uint16_t CHANGE_TH_SPEED   = 30;

// Brightness slider update smoothing
const uint16_t UPDATE_MS_BRT     = 30;
const float    EMA_ALPHA_BRT     = 0.20f;
const uint8_t  CHANGE_TH_BRT_PCT = 1;

// Touch debounce
const uint16_t TOUCH_DEBOUNCE_MS = 200;

// Mode button debounce
const uint16_t MODEBTN_DEBOUNCE_MS = 50;

// 「ビープ中はLED更新(strip.show)を止める」用
static uint32_t beep_until_ms = 0;

// 明るさ（%）
static uint8_t brightness_pct = 20;  // 20..100

// ============================================================
// (重要) 型定義は関数より前に置く：ToneStep未定義エラー回避
// ============================================================
struct ToneStep { uint16_t freq; uint16_t dur_ms; uint16_t gap_ms; };

struct BeepPlayer {
  const ToneStep* seq = nullptr;
  uint8_t len = 0;
  uint8_t idx = 0;
  bool active = false;
  bool in_gap = false;
  uint32_t next_ms = 0;
};

static BeepPlayer beep;

// ============================================================
// Mode / Pattern (11 modes)
// ============================================================
static uint8_t mode = 0; // 0..10

// LED animation state
static uint32_t next_led_ms = 0;
static uint16_t hue_base    = 0;
static uint16_t chase_pos   = 0;
static int16_t  breathe_v   = 0;
static int16_t  breathe_dv  = 6;
static uint16_t meteor_pos  = 0;
static uint16_t bounce_pos  = 0;
static int8_t   bounce_dir  = 1;
static uint16_t split_pos   = 0;
static uint16_t wipe_pos    = 0;
static uint16_t strobe_cnt  = 0;

// ============================================================
// Utils
// ============================================================
static inline uint8_t clampU8(int v, int lo, int hi) {
  if (v < lo) return (uint8_t)lo;
  if (v > hi) return (uint8_t)hi;
  return (uint8_t)v;
}
static inline uint32_t hsv(uint16_t h, uint8_t s, uint8_t v) {
  uint32_t c = strip.ColorHSV(h, s, v);
  return strip.gamma32(c);
}
static inline void clearStrip() {
  for (uint16_t i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, 0);
}

// ============================================================
// Buzzer sequence player (non-blocking)
// ============================================================
void startBeepSequence(const ToneStep* seq, uint8_t len) {
  beep.seq = seq;
  beep.len = len;
  beep.idx = 0;
  beep.active = (seq && len > 0);
  beep.in_gap = false;
  beep.next_ms = millis();

  uint32_t total = 0;
  for (uint8_t i = 0; i < len; i++) total += (uint32_t)seq[i].dur_ms + (uint32_t)seq[i].gap_ms;
  beep_until_ms = millis() + total + 30;
}

void updateBeepPlayer(uint32_t now) {
  if (!beep.active) return;
  if ((int32_t)(now - beep.next_ms) < 0) return;

  if (!beep.in_gap) {
    ToneStep st = beep.seq[beep.idx];
    if (st.freq > 0) tone(PIN_BUZZ, st.freq);
    else noTone(PIN_BUZZ);
    beep.in_gap = true;
    beep.next_ms = now + st.dur_ms;
  } else {
    noTone(PIN_BUZZ);
    ToneStep st = beep.seq[beep.idx];
    beep.in_gap = false;
    beep.idx++;
    if (beep.idx >= beep.len) {
      beep.active = false;
      beep.seq = nullptr;
      beep.len = 0;
      return;
    }
    beep.next_ms = now + st.gap_ms;
  }
}

void beepTouchClick() {
  tone(PIN_BUZZ, 2200, 60);
  beep_until_ms = millis() + 60 + 20;
}

// 10種類のモード音
const ToneStep BEEP_MODE0[] = { {2200, 60, 20} };
const ToneStep BEEP_MODE1[] = { {880,  70, 40}, {1320, 70, 20} };
const ToneStep BEEP_MODE2[] = { {660,  60, 25}, {990,  60, 25}, {1480, 60, 20} };
const ToneStep BEEP_MODE3[] = { {2000, 40, 15}, {1500, 40, 15}, {1000, 60, 20} };
const ToneStep BEEP_MODE4[] = { {523,  70, 10}, {659,  70, 10}, {784,  90, 20} };
const ToneStep BEEP_MODE5[] = { {1200, 35, 10}, {1200, 35, 10}, {1200, 70, 20} };
const ToneStep BEEP_MODE6[] = { {300,  90, 30}, {600,  90, 30} };
const ToneStep BEEP_MODE7[] = { {1500, 40, 10}, {1800, 40, 10}, {2100, 40, 10}, {2400,40,20} };
const ToneStep BEEP_MODE8[] = { {1960, 60, 20}, {0,   40, 10}, {1960, 60, 20} };
const ToneStep BEEP_MODE9[] = { {1047, 60, 10}, {1319,60, 10}, {1568,60, 10}, {2093,80,20} };
const ToneStep BEEP_MODE10[] = { {700, 50, 10}, {900, 50, 10}, {1100, 50, 10}, {1400, 70, 20} };

void playModeBeep(uint8_t m) {
  switch (m) {
    case 0: startBeepSequence(BEEP_MODE0, sizeof(BEEP_MODE0)/sizeof(BEEP_MODE0[0])); break;
    case 1: startBeepSequence(BEEP_MODE1, sizeof(BEEP_MODE1)/sizeof(BEEP_MODE1[0])); break;
    case 2: startBeepSequence(BEEP_MODE2, sizeof(BEEP_MODE2)/sizeof(BEEP_MODE2[0])); break;
    case 3: startBeepSequence(BEEP_MODE3, sizeof(BEEP_MODE3)/sizeof(BEEP_MODE3[0])); break;
    case 4: startBeepSequence(BEEP_MODE4, sizeof(BEEP_MODE4)/sizeof(BEEP_MODE4[0])); break;
    case 5: startBeepSequence(BEEP_MODE5, sizeof(BEEP_MODE5)/sizeof(BEEP_MODE5[0])); break;
    case 6: startBeepSequence(BEEP_MODE6, sizeof(BEEP_MODE6)/sizeof(BEEP_MODE6[0])); break;
    case 7: startBeepSequence(BEEP_MODE7, sizeof(BEEP_MODE7)/sizeof(BEEP_MODE7[0])); break;
    case 8: startBeepSequence(BEEP_MODE8, sizeof(BEEP_MODE8)/sizeof(BEEP_MODE8[0])); break;
    case 9: startBeepSequence(BEEP_MODE9, sizeof(BEEP_MODE9)/sizeof(BEEP_MODE9[0])); break;
    case 10: startBeepSequence(BEEP_MODE10, sizeof(BEEP_MODE10)/sizeof(BEEP_MODE10[0])); break;
  }
}

// ============================================================
// Timer1: STEP on OC1A (D9) by hardware toggle
// ============================================================
void setStepRate(uint32_t step_sps) {
  if (step_sps < 1) step_sps = 1;
  uint32_t toggle_hz = step_sps * 2UL;

  struct Presc { uint16_t div; uint16_t csbits; };
  const Presc presc_list[] = {
    {1,    (1<<CS10)},
    {8,    (1<<CS11)},
    {64,   (1<<CS11) | (1<<CS10)},
    {256,  (1<<CS12)},
    {1024, (1<<CS12) | (1<<CS10)}
  };

  uint16_t chosen_cs  = presc_list[4].csbits;
  uint16_t chosen_ocr = 65535;

  for (auto &p : presc_list) {
    uint32_t ocr = (F_CPU / (uint32_t)p.div) / toggle_hz;
    if (ocr > 0) ocr -= 1;
    if (ocr <= 65535) { chosen_cs = p.csbits; chosen_ocr = (uint16_t)ocr; break; }
  }

  noInterrupts();
  TCCR1A = 0; TCCR1B = 0; TCNT1 = 0;
  OCR1A  = chosen_ocr;
  TCCR1B |= (1<<WGM12);   // CTC
  TCCR1A |= (1<<COM1A0);  // toggle OC1A
  TCCR1B |= chosen_cs;
  interrupts();
}

// ============================================================
// Brightness from slider + strip
// ============================================================
void applyBrightnessPct(uint8_t pct) {
  brightness_pct = pct;
  uint8_t b = (uint8_t)((uint16_t)brightness_pct * 255 / 100);
  strip.setBrightness(b);
}

void updateBrightnessFromSlider(uint32_t now_ms) {
  static uint32_t last_ms = 0;
  static float ema = 0.0f;
  static bool ema_init = false;
  static uint8_t last_applied_pct = 0;

  if (now_ms - last_ms < UPDATE_MS_BRT) return;
  last_ms = now_ms;

  int raw = analogRead(PIN_SLIDER_BRT);

  if (!ema_init) { ema = (float)raw; ema_init = true; }
  else { ema = ema + EMA_ALPHA_BRT * ((float)raw - ema); }

  int pct = map((long)ema, 0, 1023, 20, 100);
  uint8_t pct_u8 = clampU8(pct, 20, 100);

  int diff = (pct_u8 > last_applied_pct) ? (pct_u8 - last_applied_pct)
                                         : (last_applied_pct - pct_u8);
  if (diff >= CHANGE_TH_BRT_PCT || last_applied_pct == 0) {
    applyBrightnessPct(pct_u8);
    last_applied_pct = pct_u8;
  }
}

// Slide Pot onboard LED (Pin2) pseudo PWM on A1
void runSliderLedPseudoPwm() {
  uint8_t duty = (uint8_t)((uint16_t)brightness_pct * 255 / 100);
  const uint16_t PERIOD_US = 2000;
  static uint32_t t0 = 0;

  uint32_t now = micros();
  uint16_t phase = (uint16_t)((now - t0) % PERIOD_US);
  uint16_t on_us = (uint16_t)((uint32_t)PERIOD_US * duty / 255);

  if (duty == 0) digitalWrite(PIN_SLIDER_LED, LOW);
  else if (duty >= 255) digitalWrite(PIN_SLIDER_LED, HIGH);
  else digitalWrite(PIN_SLIDER_LED, (phase < on_us) ? HIGH : LOW);
}

// ============================================================
// LED Patterns (10)
// ============================================================
void pattern0_rainbowFlow() {
  hue_base += 300;
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    uint16_t h = hue_base + (uint16_t)(i * (65535UL / NUM_LEDS));
    strip.setPixelColor(i, hsv(h, 255, 255));
  }
}

void pattern1_theaterChaseWhite() {
  chase_pos = (chase_pos + 1) % 3;
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    if ((i % 3) == chase_pos) strip.setPixelColor(i, strip.Color(255,255,255));
    else strip.setPixelColor(i, 0);
  }
}

void pattern2_breathingPurple() {
  breathe_v += breathe_dv;
  if (breathe_v >= 255) { breathe_v = 255; breathe_dv = -breathe_dv; }
  if (breathe_v <= 0)   { breathe_v = 0;   breathe_dv = -breathe_dv; }
  uint32_t c = hsv(40000, 255, (uint8_t)breathe_v);
  for (uint16_t i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, c);
}

void pattern3_twinkle() {
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    uint32_t c = strip.getPixelColor(i);
    uint8_t r = (uint8_t)(c >> 16);
    uint8_t g = (uint8_t)(c >> 8);
    uint8_t b = (uint8_t)(c);
    strip.setPixelColor(i, r, g, b);
  }
  for (uint8_t k = 0; k < 4; k++) {
    uint16_t idx = (uint16_t)random(NUM_LEDS);
    uint16_t h = (uint16_t)random(65535);
    strip.setPixelColor(idx, hsv(h, 255, 255));
  }
}

void pattern4_meteor() {
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    uint32_t c = strip.getPixelColor(i);
    uint8_t r = (uint8_t)(c >> 16);
    uint8_t g = (uint8_t)(c >> 8);
    uint8_t b = (uint8_t)(c);
    strip.setPixelColor(i, r, g, b);
  }
  meteor_pos = (meteor_pos + 1) % NUM_LEDS;
  strip.setPixelColor(meteor_pos, strip.Color(255,255,255));
  strip.setPixelColor((meteor_pos + NUM_LEDS - 1) % NUM_LEDS, hsv(8000, 255, 255));
  strip.setPixelColor((meteor_pos + NUM_LEDS - 2) % NUM_LEDS, hsv(8000, 255, 255));
}

void pattern5_redPolice() {
  strobe_cnt++;
  bool leftOn = (strobe_cnt % 2 == 0);
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    bool left = (i < (NUM_LEDS/2));
    if ((left && leftOn) || (!left && !leftOn)) strip.setPixelColor(i, strip.Color(255,0,0));
    else strip.setPixelColor(i, 0);
  }
}

void pattern6_bounceDot() {
  clearStrip();
  strip.setPixelColor(bounce_pos, strip.Color(0,255,80));
  bounce_pos += bounce_dir;
  if (bounce_pos == 0 || bounce_pos == NUM_LEDS-1) bounce_dir = -bounce_dir;
}

void pattern7_splitWaves() {
  split_pos = (split_pos + 1) % (NUM_LEDS/2);
  clearStrip();
  uint16_t h1 = (uint16_t)(hue_base += 500);
  strip.setPixelColor(split_pos, hsv(h1, 255, 255));
  strip.setPixelColor(NUM_LEDS-1-split_pos, hsv((uint16_t)(h1+20000), 255, 255));
  if (split_pos > 0) {
    strip.setPixelColor(split_pos-1, hsv(h1, 255, 255));
    strip.setPixelColor(NUM_LEDS-split_pos, hsv((uint16_t)(h1+20000), 255, 255));
  }
}

void pattern8_colorWipe() {
  wipe_pos++;
  if (wipe_pos >= NUM_LEDS) wipe_pos = 0;
  uint16_t h = (uint16_t)(hue_base += 1200);
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    if (i <= wipe_pos) strip.setPixelColor(i, hsv(h, 255, 255));
    else strip.setPixelColor(i, 0);
  }
}

void pattern9_whiteStrobe() {
  strobe_cnt++;
  bool on = (strobe_cnt % 2 == 0);
  uint32_t c = on ? strip.Color(255,255,255) : 0;
  for (uint16_t i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, c);
}

void pattern10_allWhiteMax() {
  uint32_t c = strip.Color(255,255,255);
  for (uint16_t i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, c);
}

uint16_t ledUpdateIntervalForMode(uint8_t m) {
  switch (m) {
    case 0: return 20;
    case 1: return 90;
    case 2: return 20;
    case 3: return 60;
    case 4: return 35;
    case 5: return 180;
    case 6: return 25;
    case 7: return 40;
    case 8: return 55;
    case 9: return 90;
    case 10: return 50;
    default: return 30;
  }
}

void runLedPatterns(uint32_t now_ms) {
  if ((int32_t)(now_ms - beep_until_ms) < 0) return;

  uint16_t interval = ledUpdateIntervalForMode(mode);
  if (now_ms < next_led_ms) return;
  next_led_ms = now_ms + interval;

  switch (mode) {
    case 0: pattern0_rainbowFlow();        break;
    case 1: pattern1_theaterChaseWhite();  break;
    case 2: pattern2_breathingPurple();    break;
    case 3: pattern3_twinkle();            break;
    case 4: pattern4_meteor();             break;
    case 5: pattern5_redPolice();          break;
    case 6: pattern6_bounceDot();          break;
    case 7: pattern7_splitWaves();         break;
    case 8: pattern8_colorWipe();          break;
    case 9: pattern9_whiteStrobe();        break;
    case 10: pattern10_allWhiteMax();      break;
  }
  strip.show();
}

// ============================================================
// Setup / Loop
// ============================================================
void setup() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);

  pinMode(PIN_TOUCH, INPUT);
  pinMode(PIN_MODE_BTN, INPUT);
  pinMode(PIN_BUZZ, OUTPUT);

  pinMode(PIN_SLIDER_LED, OUTPUT);
  digitalWrite(PIN_SLIDER_LED, LOW);

  digitalWrite(PIN_DIR, HIGH);
  digitalWrite(PIN_EN, LOW);

  strip.begin();
  applyBrightnessPct(brightness_pct);
  strip.show();

  randomSeed((uint32_t)analogRead(A2) ^ (uint32_t)analogRead(A3));

  int raw = analogRead(PIN_KNOB_SPEED);
  uint32_t sps = map(raw, 0, 1023, MIN_STEP_SPS, MAX_STEP_SPS);
  setStepRate(sps);

  playModeBeep(mode);
}

void loop() {
  const uint32_t now = millis();

  updateBeepPlayer(now);

  // Speed update (A3)
  static uint32_t last_ms_speed = 0;
  static float ema_speed = 0.0f;
  static bool ema_speed_init = false;
  static uint32_t last_applied_sps = 0;

  if (now - last_ms_speed >= UPDATE_MS_SPEED) {
    last_ms_speed = now;
    int raw = analogRead(PIN_KNOB_SPEED);
    if (!ema_speed_init) { ema_speed = (float)raw; ema_speed_init = true; }
    else { ema_speed = ema_speed + EMA_ALPHA_SPEED * ((float)raw - ema_speed); }

    uint32_t target_sps = (uint32_t)map((long)ema_speed, 0, 1023,
                                        (long)MIN_STEP_SPS, (long)MAX_STEP_SPS);

    uint32_t diff = (target_sps > last_applied_sps)
                      ? (target_sps - last_applied_sps)
                      : (last_applied_sps - target_sps);

    if (diff >= CHANGE_TH_SPEED || last_applied_sps == 0) {
      setStepRate(target_sps);
      last_applied_sps = target_sps;
    }
  }

  updateBrightnessFromSlider(now);

  // Touch: toggle dir + click
  static bool last_touch = false;
  static uint32_t last_toggle_ms = 0;
  bool touch = (digitalRead(PIN_TOUCH) == HIGH);
  if (touch && !last_touch) {
    if (now - last_toggle_ms >= TOUCH_DEBOUNCE_MS) {
      digitalWrite(PIN_DIR, !digitalRead(PIN_DIR));
      if (!beep.active) beepTouchClick();
      last_toggle_ms = now;
    }
  }
  last_touch = touch;

  // Mode button: next pattern
  static bool last_btn = false;
  static uint32_t last_btn_ms = 0;
  bool btn = (digitalRead(PIN_MODE_BTN) == HIGH);
  if (btn && !last_btn) {
    if (now - last_btn_ms >= MODEBTN_DEBOUNCE_MS) {
      mode = (uint8_t)((mode + 1) % 11);
      playModeBeep(mode);
      next_led_ms = 0;
      last_btn_ms = now;
    }
  }
  last_btn = btn;

  runSliderLedPseudoPwm();
  runLedPatterns(now);
}

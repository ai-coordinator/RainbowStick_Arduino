#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca(0x40);

// ===== チャンネル（あなたの定義）=====
const uint8_t CH_SG90  = 0;  // SG90
const uint8_t CH_G1    = 1;  // Grove Servo（上下）
const uint8_t CH_G2    = 4;  // Grove Servo（左右）
const uint8_t CH_G3    = 8;  // Grove Servo（左右）

// ===== PWMレンジ（あなたの安定値）=====
#define SG90_MIN 110
#define SG90_MAX 500
#define GROVE_MIN 130
#define GROVE_MAX 470

int groveSafe(int a) { return constrain(a, 0, 180); } // ※今のまま

// ===== Joystick（Base Shield A0ポート：A0=X, A1=Y）=====
const uint8_t PIN_X = A0;
const uint8_t PIN_Y = A1;

// ===== Grove Mech Keycap (button only) =====
const uint8_t MECHKEY_BTN_PIN = 5;  // Grove Base Shield D5 (SIG1: Yellow)
const int MECHKEY_ACTIVE = HIGH;

// ===== Mini Fan（D2ポート推奨）=====
const uint8_t FAN_PIN = 2;     // ファン制御（D2）
const int FAN_ON  = HIGH;      // 逆なら LOW/HIGH を入れ替え
const int FAN_OFF = LOW;

// ===== Grove Blue LED Button（D3ポート）=====
// D3ポートに挿すと、例では LED=D3 / Button=D4 を使います
const uint8_t LEDBTN_LED_PIN = 3;   // SIG1: LED制御
const uint8_t LEDBTN_BTN_PIN = 4;   // SIG2: ボタン入力（押すとLOW）

// ===== チューニング（あなたの設定）=====
const int DEADZONE = 35;
const int AMP_X = 180;
const int AMP_Y = 180;
const int LOOP_MS = 20;

int centerX = 512, centerY = 512;

// ===== ファンON/OFFトグル用 =====
bool fanState = false;
int lastBtnReading = HIGH;             // ボタンはデフォルトHIGH、押すとLOW
int stableBtnState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 30; // 20〜60ms
int lastMechReading = LOW;
int stableMechState = LOW;
unsigned long lastMechDebounceTime = 0;


// =====================
// MAX7219 (NO library)  DIN=D11, CLK=D13, CS=D10
// =====================
const uint8_t MAX_DIN = 11;
const uint8_t MAX_CLK = 13;
const uint8_t MAX_CS  = 10;

// ★4-in-1なら4、単体なら1
#define NUM_DEV 4

// モジュール固有の列並び補正（左右が逆に見える場合は 1）
#define FLIP_COLS 0

// 表示の上下左右反転（見た目だけ）
#define FLIP_X 1   // 左右反転（左↔右）
#define FLIP_Y 1   // 上下反転（上↔下）

// MAX7219 registers
const uint8_t REG_NOOP        = 0x00;
const uint8_t REG_DIGIT0      = 0x01;
const uint8_t REG_DECODEMODE  = 0x09;
const uint8_t REG_INTENSITY   = 0x0A;
const uint8_t REG_SCANLIMIT   = 0x0B;
const uint8_t REG_SHUTDOWN    = 0x0C;
const uint8_t REG_DISPLAYTEST = 0x0F;

// ---- パックマン＆ゴースト横スクロール用（表示部分だけ）----
unsigned long lastAnimMs = 0;
const unsigned long ANIM_INTERVAL_MS = 70;

int pacX = 0;
bool pacMouthOpen = true;

int ghostX = 0;
bool ghostFeetAlt = false;

enum DisplayMode { MODE_PACMAN = 0, MODE_HELLO = 1 };
DisplayMode displayMode = MODE_PACMAN;
int helloX = 0;

// ===== スプライト間の間隔と並び (ghost -> fish -> heart -> star -> pac) =====
const int SPRITE_W = 8;
const int SPRITE_GAP = 2;
const int GHOST_TO_PAC = (SPRITE_W * 4) + (SPRITE_GAP * 4); // ghost + fish + heart + star + gaps

// ===== エサの配置（pacXからの相対位置）=====
const int PEL_S0_OFF  = 12;   // 小丸1
const int PEL_S1_OFF  = 18;   // 小丸2
const int PEL_S2_OFF  = 24;   // 小丸3
const int PEL_BIG_OFF = 32;   // 大丸

const int PEL_SMALL_W = 2;    // 2x2
const int PEL_BIG_W   = 4;    // 4x4

// ★重要：スタート時に「一番前の大丸」も画面外にいるようにする
// pacX を -8 からさらに左へ押し込む（方式は変えない）
const int PAC_BASE_OFFSCREEN = 8; // pacスプライト幅
const int PAC_START_X = -(PAC_BASE_OFFSCREEN + PEL_BIG_OFF + PEL_BIG_W); // = -(8+32+4) = -44

// 8x8 パックマン（右向き） open/closed（画像の雰囲気）
const uint8_t PAC_OPEN_R[8] = {
  0b00111100,
  0b01111110,
  0b11111111,
  0b11111000,
  0b11111000,
  0b11111111,
  0b01111110,
  0b00111100
};

const uint8_t PAC_CLOSED[8] = {
  0b00111100,
  0b01111110,
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111,
  0b01111110,
  0b00111100
};

// 8x8 ゴースト（目が抜けた感じ） 2フレーム（足パタ）
const uint8_t GHOST_A[8] = {
  0b00111100,
  0b01111110,
  0b11111111,
  0b11011011,
  0b11011011,
  0b11111111,
  0b10100101,
  0b01011010
};

const uint8_t GHOST_B[8] = {
  0b00111100,
  0b01111110,
  0b11111111,
  0b11011011,
  0b11011011,
  0b11111111,
  0b01011010,
  0b10100101
};

// 8x8 魚
const uint8_t FISH[8] = {
  0b00011000,
  0b00111100,
  0b01111110,
  0b11111111,
  0b11111111,
  0b01111110,
  0b00111100,
  0b00011010
};

// 8x8 ハート
const uint8_t HEART[8] = {
  0b01100110,
  0b11111111,
  0b11111111,
  0b11111111,
  0b01111110,
  0b00111100,
  0b00011000,
  0b00000000
};

// 8x8 星
const uint8_t STAR[8] = {
  0b00011000,
  0b00011000,
  0b11111111,
  0b01111110,
  0b00111100,
  0b01111110,
  0b11111111,
  0b00000000
};

// ===== HELLO (8x8 sprites) =====
const int HELLO_SPR_W = 8;
const int HELLO_SPR_SPACING = 1;
const int HELLO_LEN = 5;
const int HELLO_TEXT_W = (HELLO_SPR_W * HELLO_LEN) + (HELLO_SPR_SPACING * (HELLO_LEN - 1));
const int HELLO_START_X = -HELLO_TEXT_W;
const int HELLO_Y = 0;

const uint8_t HELLO_H[8] = {
  0b00000000,
  0b11000011,
  0b11000011,
  0b11111111,
  0b11000011,
  0b11000011,
  0b11000011,
  0b00000000
};

const uint8_t HELLO_E[8] = {
  0b00000000,
  0b11111111,
  0b11000000,
  0b11111110,
  0b11000000,
  0b11000000,
  0b11111111,
  0b00000000
};

const uint8_t HELLO_L[8] = {
  0b00000000,
  0b11000000,
  0b11000000,
  0b11000000,
  0b11000000,
  0b11000000,
  0b11111111,
  0b00000000
};

const uint8_t HELLO_O[8] = {
  0b00000000,
  0b00111100,
  0b01100110,
  0b11000011,
  0b11000011,
  0b01100110,
  0b00111100,
  0b00000000
};

const uint8_t* const HELLO_SPRS[HELLO_LEN] = {
  HELLO_H, HELLO_E, HELLO_L, HELLO_L, HELLO_O
};

// ---------- MAX7219 low-level ----------

void shiftOutMSB(uint8_t data) {
  for (int i = 7; i >= 0; i--) {
    digitalWrite(MAX_CLK, LOW);
    digitalWrite(MAX_DIN, (data >> i) & 0x01);
    digitalWrite(MAX_CLK, HIGH);
  }
}

void sendAll(uint8_t reg, uint8_t val) {
  digitalWrite(MAX_CS, LOW);
  for (uint8_t dev = 0; dev < NUM_DEV; dev++) {
    shiftOutMSB(reg);
    shiftOutMSB(val);
  }
  digitalWrite(MAX_CS, HIGH);
}

// dev=0 を「Arduinoに近い側」として扱うため、逆順で送る
void sendRowToAllDevices(uint8_t rowReg, const uint8_t *valsPerDev) {
  digitalWrite(MAX_CS, LOW);
  for (int dev = (int)NUM_DEV - 1; dev >= 0; dev--) {
    shiftOutMSB(rowReg);
    shiftOutMSB(valsPerDev[dev]);
  }
  digitalWrite(MAX_CS, HIGH);
}

void fillAllDevices(uint8_t pattern) {
  for (uint8_t row = 0; row < 8; row++) {
    sendAll(REG_DIGIT0 + row, pattern);
  }
}

void initMax7219() {
  pinMode(MAX_DIN, OUTPUT);
  pinMode(MAX_CLK, OUTPUT);
  pinMode(MAX_CS,  OUTPUT);

  digitalWrite(MAX_CS, HIGH);
  digitalWrite(MAX_CLK, LOW);

  sendAll(REG_DISPLAYTEST, 0x00);
  sendAll(REG_DECODEMODE,  0x00);
  sendAll(REG_SCANLIMIT,   0x07);
  sendAll(REG_INTENSITY,   0x06);
  sendAll(REG_SHUTDOWN,    0x01);

  fillAllDevices(0x00);
}

// 1ピクセル点灯（仮想画面W=8*NUM_DEV, y=0..7, x=0..W-1）
void setPixel(uint8_t rows[8][NUM_DEV], int x, int y) {
  int W = 8 * (int)NUM_DEV;

#if FLIP_X
  x = (W - 1) - x;
#endif
#if FLIP_Y
  y = 7 - y;
#endif

  if (y < 0 || y > 7) return;
  if (x < 0 || x >= W) return;

  int dev = x / 8;
  int col = x % 8;

#if FLIP_COLS
  col = 7 - col;
#endif

  rows[y][dev] |= (uint8_t)(1 << col);
}

void blitSprite(uint8_t rows[8][NUM_DEV], int xLeft, const uint8_t spr[8]) {
  for (int sy = 0; sy < 8; sy++) {
    uint8_t bits = spr[sy];
    for (int sx = 0; sx < 8; sx++) {
      bool on = bits & (0x80 >> sx);
      if (!on) continue;
      setPixel(rows, xLeft + sx, sy);
    }
  }
}

uint8_t mirror8(uint8_t v) {
  v = ((v & 0xF0) >> 4) | ((v & 0x0F) << 4);
  v = ((v & 0xCC) >> 2) | ((v & 0x33) << 2);
  v = ((v & 0xAA) >> 1) | ((v & 0x55) << 1);
  return v;
}

// 2x2 / 4x4 pellet
void drawPelletSmall(uint8_t rows[8][NUM_DEV], int x, int y) {
  setPixel(rows, x,     y);
  setPixel(rows, x + 1, y);
  setPixel(rows, x,     y + 1);
  setPixel(rows, x + 1, y + 1);
}

void drawPelletBig(uint8_t rows[8][NUM_DEV], int x, int y) {
  setPixel(rows, x + 1, y + 0);
  setPixel(rows, x + 2, y + 0);

  setPixel(rows, x + 0, y + 1);
  setPixel(rows, x + 1, y + 1);
  setPixel(rows, x + 2, y + 1);
  setPixel(rows, x + 3, y + 1);

  setPixel(rows, x + 0, y + 2);
  setPixel(rows, x + 1, y + 2);
  setPixel(rows, x + 2, y + 2);
  setPixel(rows, x + 3, y + 2);

  setPixel(rows, x + 1, y + 3);
  setPixel(rows, x + 2, y + 3);
}

int helloStartX() {
  return HELLO_START_X;
}

void renderHello(int baseX) {
  uint8_t rows[8][NUM_DEV];
  for (uint8_t r = 0; r < 8; r++) {
    for (uint8_t d = 0; d < NUM_DEV; d++) rows[r][d] = 0x00;
  }

  int x = baseX;
  if (FLIP_X) {
    for (int i = HELLO_LEN - 1; i >= 0; i--) {
      uint8_t tmp[8];
      for (int r = 0; r < 8; r++) tmp[r] = mirror8(HELLO_SPRS[i][r]);
      blitSprite(rows, x, tmp);
      x += HELLO_SPR_W + HELLO_SPR_SPACING;
    }
  } else {
    for (int i = 0; i < HELLO_LEN; i++) {
      blitSprite(rows, x, HELLO_SPRS[i]);
      x += HELLO_SPR_W + HELLO_SPR_SPACING;
    }
  }

  for (uint8_t r = 0; r < 8; r++) {
    sendRowToAllDevices(REG_DIGIT0 + r, rows[r]);
  }
}

void renderScene(int pacLeft, bool mouthOpen, int ghostLeft, bool feetAlt) {
  uint8_t rows[8][NUM_DEV];
  for (uint8_t r = 0; r < 8; r++) {
    for (uint8_t d = 0; d < NUM_DEV; d++) rows[r][d] = 0x00;
  }

  // pellets: pacより前に固定配置（同じ速度、順番も固定）
  const int pelletY = 3;
  drawPelletSmall(rows, pacLeft + PEL_S0_OFF, pelletY);
  drawPelletSmall(rows, pacLeft + PEL_S1_OFF, pelletY);
  drawPelletSmall(rows, pacLeft + PEL_S2_OFF, pelletY);
  drawPelletBig(rows,   pacLeft + PEL_BIG_OFF, pelletY - 1);

  const uint8_t *pac = mouthOpen ? PAC_OPEN_R : PAC_CLOSED;
  const uint8_t *gst = feetAlt ? GHOST_B : GHOST_A;

  int fishLeft = ghostLeft + SPRITE_W + SPRITE_GAP;
  int heartLeft = fishLeft + SPRITE_W + SPRITE_GAP;
  int starLeft = heartLeft + SPRITE_W + SPRITE_GAP;

  blitSprite(rows, ghostLeft, gst);
  blitSprite(rows, fishLeft, FISH);
  blitSprite(rows, heartLeft, HEART);
  blitSprite(rows, starLeft, STAR);
  blitSprite(rows, pacLeft, pac);

  for (uint8_t r = 0; r < 8; r++) {
    sendRowToAllDevices(REG_DIGIT0 + r, rows[r]);
  }
}

// =====================
// Servo helpers（元コードのまま）
// =====================
uint16_t angleToPulse(int angle, int pmin, int pmax) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, pmin, pmax);
}

void setServo(uint8_t ch, int angle, int pmin, int pmax) {
  pca.setPWM(ch, 0, angleToPulse(angle, pmin, pmax));
}

void setSG90(int angle) {
  setServo(CH_SG90, angle, SG90_MIN, SG90_MAX);
}

void setGrove(uint8_t ch, int angle) {
  setServo(ch, groveSafe(angle), GROVE_MIN, GROVE_MAX);
}

int applyDeadzone(int v, int center) {
  int d = v - center;
  if (abs(d) < DEADZONE) return 0;
  return d;
}

float normAxis(int d) {
  float n = (float)d / 350.0f;
  if (n > 1.0f) n = 1.0f;
  if (n < -1.0f) n = -1.0f;
  return n;
}

void calibrateCenter() {
  long sx = 0, sy = 0;
  for (int i = 0; i < 30; i++) {
    sx += analogRead(PIN_X);
    sy += analogRead(PIN_Y);
    delay(5);
  }
  centerX = (int)(sx / 30);
  centerY = (int)(sy / 30);
}

void applyFanState() {
  digitalWrite(FAN_PIN, fanState ? FAN_ON : FAN_OFF);
  digitalWrite(LEDBTN_LED_PIN, fanState ? HIGH : LOW);
}

void setup() {
  Wire.begin();

  // PCA9685
  pca.begin();
  pca.setOscillatorFrequency(25000000);
  pca.setPWMFreq(50);
  delay(10);

  // Joystick center
  calibrateCenter();

  // Fan + LED button
  pinMode(FAN_PIN, OUTPUT);
  pinMode(LEDBTN_LED_PIN, OUTPUT);
  pinMode(LEDBTN_BTN_PIN, INPUT);
  pinMode(MECHKEY_BTN_PIN, INPUT);
  lastMechReading = digitalRead(MECHKEY_BTN_PIN);
  stableMechState = lastMechReading;

  fanState = false;
  applyFanState();

  // MAX7219
  initMax7219();

  // initial servo positions
  setSG90(90);
  setGrove(CH_G1, 90);
  setGrove(CH_G2, 90);
  setGrove(CH_G3, 90);
  delay(300);

  // ★ここがポイント：最初から「エサ列」も画面外から始める
  pacX = PAC_START_X;
  ghostX = pacX - GHOST_TO_PAC;
  helloX = helloStartX();

  pacMouthOpen = true;
  ghostFeetAlt = false;
  lastAnimMs = millis();

  renderScene(pacX, pacMouthOpen, ghostX, ghostFeetAlt);
}

void loop() {
  // ===== 1) LEDボタンでファンON/OFF =====
  int reading = digitalRead(LEDBTN_BTN_PIN);

  if (reading != lastBtnReading) {
    lastDebounceTime = millis();
    lastBtnReading = reading;
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != stableBtnState) {
      stableBtnState = reading;
      if (stableBtnState == LOW) {
        fanState = !fanState;
        applyFanState();
      }
    }
  }


  // ===== 1.5) Mech key -> Display mode toggle =====
  int mechReading = digitalRead(MECHKEY_BTN_PIN);

  if (mechReading != lastMechReading) {
    lastMechDebounceTime = millis();
    lastMechReading = mechReading;
  }

  if ((millis() - lastMechDebounceTime) > debounceDelay) {
    if (mechReading != stableMechState) {
      stableMechState = mechReading;
      if (stableMechState == MECHKEY_ACTIVE) {
        displayMode = (displayMode == MODE_PACMAN) ? MODE_HELLO : MODE_PACMAN;
        pacX = PAC_START_X;
        ghostX = pacX - GHOST_TO_PAC;
        helloX = helloStartX();
        pacMouthOpen = true;
        ghostFeetAlt = false;
        lastAnimMs = millis();
        if (displayMode == MODE_PACMAN) {
          renderScene(pacX, pacMouthOpen, ghostX, ghostFeetAlt);
        } else {
          renderHello(helloX);
        }
      }
    }
  }
  // ===== 2) Joystick -> Servo =====
  int x = analogRead(PIN_X);
  int y = analogRead(PIN_Y);

  int dx = applyDeadzone(x, centerX);
  int dy = applyDeadzone(y, centerY);

  float nx = normAxis(dx);
  float ny = normAxis(dy);

  int angUp_SG90 = 90 + (int)(ny * AMP_Y);
  int angUp_G1   = 90 + (int)(ny * AMP_Y);
  int angLR      = 90 + (int)(nx * AMP_X);

  setSG90(angUp_SG90);
  setGrove(CH_G1, angUp_G1);
  setGrove(CH_G2, angLR);
  setGrove(CH_G3, angLR);

  // ===== 3) MAX7219 アニメーション =====
  unsigned long now = millis();
  if (now - lastAnimMs >= ANIM_INTERVAL_MS) {
    lastAnimMs = now;

    int W = 8 * (int)NUM_DEV;

    if (displayMode == MODE_PACMAN) {
      pacX += 1;
      ghostX += 1;

      pacMouthOpen = !pacMouthOpen;
      ghostFeetAlt = !ghostFeetAlt;

      renderScene(pacX, pacMouthOpen, ghostX, ghostFeetAlt);

      // Reset when ghost leaves screen
      if (ghostX >= W) {
        pacX = PAC_START_X;
        ghostX = pacX - GHOST_TO_PAC;
      }
    } else {
      helloX += 1;
      renderHello(helloX);

      if (helloX >= W) {
        helloX = helloStartX();
      }
    }
  }

  delay(LOOP_MS);
}

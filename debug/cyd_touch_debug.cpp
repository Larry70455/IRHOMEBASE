// ---------------------------------------------------------------------
// Standalone CYD touch/display debug sketch - interactive.
//
//   pio run -e cyd-debug -t upload
//   pio device monitor -e cyd-debug
//
// Everything is adjustable live over serial: single-key commands, no
// re-flashing to try a different combination. Press 'h' for the list.
//
// Serial is quiet by default - it prints on command and on state change,
// not continuously. Per-touch logging is opt-in ('t') and throttled, so
// the monitor stays readable while you work.
//
// Deliberately self-contained: its own display config, its own XPT2046
// driver talking to the chip directly over SPI, no dependency on
// src/main.cpp or LGX_Config_CYD.h, and no reliance on any stored
// calibration - a bad stored calibration being what makes the main
// firmware's own settings screen unreachable in the first place.
//
// When the mapping is right, press 'p' and send me that block.
// ---------------------------------------------------------------------
#include <Arduino.h>   // map()/constrain()/labs() - included explicitly rather than relying on another header pulling it in
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <SPI.h>

// ---- display: 320x240 (confirmed on this hardware - NOT 240x320) ----
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 _panel;
  lgfx::Bus_SPI       _bus;
  lgfx::Light_PWM     _light;
public:
  LGFX() {
    { auto c = _bus.config();
      c.spi_host = VSPI_HOST; c.spi_mode = 0;
      c.freq_write = 20000000; c.freq_read = 16000000;
      c.spi_3wire = true; c.use_lock = true;
      c.dma_channel = SPI_DMA_CH_AUTO;
      c.pin_sclk = 14; c.pin_mosi = 13; c.pin_miso = 12; c.pin_dc = 2;
      _bus.config(c); _panel.setBus(&_bus); }
    { auto c = _panel.config();
      c.pin_cs = 15; c.pin_rst = -1; c.pin_busy = -1;
      c.panel_width = 320; c.panel_height = 240;
      c.offset_x = 0; c.offset_y = 0; c.offset_rotation = 0;
      c.invert = false; c.rgb_order = true;
      _panel.config(c); }
    { auto c = _light.config();
      c.pin_bl = 21; c.invert = false; c.freq = 44100; c.pwm_channel = 7;
      _light.config(c); _panel.setLight(&_light); }
    setPanel(&_panel);
  }
};
LGFX tft;

// ---- XPT2046 touch, driven directly ----
#define T_CLK 25
#define T_MOSI 32
#define T_MISO 39
#define T_CS   33
#define T_IRQ  36

SPIClass touchSPI(HSPI);

// ---- live-adjustable state ----
// Starts at 4, not 0: on this panel 0-3 all render mirrored (text
// backwards). 4-7 are those same four rotations with the mirror bit set,
// and 4 is the one confirmed correct on this hardware. Press 'r'/'R' to
// step through if a different orientation is wanted.
uint8_t  rotation  = 4;
bool     swapXY    = false, invX = false, invY = false;
uint16_t rawXmin   = 300, rawXmax = 3800;
uint16_t rawYmin   = 300, rawYmax = 3800;
bool     verbose   = false;   // per-touch serial logging, off by default
bool     invertDisp = false;
bool     bgrOrder  = true;

unsigned long lastVerboseAt = 0;
const unsigned long VERBOSE_MIN_GAP_MS = 250;  // keeps 't' mode readable rather than a flood

// 0xD0 = read X, 0x90 = read Y (12-bit, differential mode)
uint16_t xptRead(uint8_t cmd) {
  touchSPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(T_CS, LOW);
  touchSPI.transfer(cmd);
  uint8_t hi = touchSPI.transfer(0x00);
  uint8_t lo = touchSPI.transfer(0x00);
  digitalWrite(T_CS, HIGH);
  touchSPI.endTransaction();
  return ((uint16_t)((hi << 8) | lo)) >> 3;   // 12 significant bits
}

bool touchDown() { return digitalRead(T_IRQ) == LOW; }

// Median of several reads - raw values are noisy enough that a single
// sample can be well off, which would skew any range derived from it.
bool readTouch(uint16_t &rx, uint16_t &ry) {
  if (!touchDown()) return false;
  const int N = 9;
  uint16_t xs[N], ys[N];
  for (int i = 0; i < N; i++) {
    if (!touchDown()) return false;
    ys[i] = xptRead(0x90);
    xs[i] = xptRead(0xD0);
    delay(2);
  }
  for (int i = 1; i < N; i++) {            // insertion sort, N is tiny
    uint16_t kx = xs[i], ky = ys[i];
    int j = i - 1;
    while (j >= 0 && xs[j] > kx) { xs[j+1] = xs[j]; j--; }
    xs[j+1] = kx;
    j = i - 1;
    while (j >= 0 && ys[j] > ky) { ys[j+1] = ys[j]; j--; }
    ys[j+1] = ky;
  }
  rx = xs[N/2];
  ry = ys[N/2];
  return rx > 100 && rx < 4000 && ry > 100 && ry < 4000;  // reject rail readings
}

void waitRelease() {
  while (touchDown()) delay(10);
  delay(120);   // settle, so one press can't register twice
}

bool mapTouch(int &sx, int &sy) {
  uint16_t rx, ry;
  if (!readTouch(rx, ry)) return false;
  uint16_t ax = swapXY ? ry : rx;
  uint16_t ay = swapXY ? rx : ry;
  long x = map(ax, rawXmin, rawXmax, 0, tft.width()  - 1);
  long y = map(ay, rawYmin, rawYmax, 0, tft.height() - 1);
  if (invX) x = tft.width()  - 1 - x;
  if (invY) y = tft.height() - 1 - y;
  sx = constrain(x, 0, tft.width()  - 1);
  sy = constrain(y, 0, tft.height() - 1);
  return true;
}

// ---- screen ----
void drawUI() {
  int W = tft.width(), H = tft.height();
  tft.fillScreen(TFT_BLACK);
  tft.drawRect(0, 0, W, H, TFT_DARKGREY);

  // reference marks at the exact corners and centre
  int pts[5][2] = { {0,0}, {W-1,0}, {0,H-1}, {W-1,H-1}, {W/2,H/2} };
  for (auto &p : pts) {
    tft.drawLine(p[0]-12, p[1], p[0]+12, p[1], TFT_DARKGREEN);
    tft.drawLine(p[0], p[1]-12, p[0], p[1]+12, TFT_DARKGREEN);
  }

  tft.setFont(&fonts::FreeSansBold9pt7b);
  tft.setTextDatum(textdatum_t::top_center);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  String line = String(W) + "x" + String(H) + " rot" + String(rotation) +
                (swapXY ? " SWAP" : "") + (invX ? " INVX" : "") + (invY ? " INVY" : "");
  tft.drawString(line, W / 2, 6);
  tft.setFont(&fonts::FreeSans9pt7b);
  tft.drawString("tap = dot under finger", W / 2, 26);
  tft.setTextDatum(textdatum_t::top_left);
}

void printConfig() {
  Serial.println();
  Serial.println("================ CONFIG ================");
  Serial.printf("display        : %d x %d\n", tft.width(), tft.height());
  Serial.printf("rotation       : %u\n", rotation);
  Serial.printf("panel invert   : %s\n", invertDisp ? "true" : "false");
  Serial.printf("rgb_order BGR  : %s\n", bgrOrder ? "true" : "false");
  Serial.printf("axes swapped   : %s\n", swapXY ? "true" : "false");
  Serial.printf("invert X       : %s\n", invX ? "true" : "false");
  Serial.printf("invert Y       : %s\n", invY ? "true" : "false");
  Serial.printf("raw X range    : %u .. %u\n", rawXmin, rawXmax);
  Serial.printf("raw Y range    : %u .. %u\n", rawYmin, rawYmax);
  Serial.println("========================================");
  Serial.println("When taps land correctly, send this block back.");
  Serial.println();
}

void printHelp() {
  Serial.println();
  Serial.println("---------------- KEYS ------------------");
  Serial.println("  h  this help");
  Serial.println("  p  print current config  <-- send me this when it's right");
  Serial.println("  s  toggle axis swap (X<->Y)");
  Serial.println("  x  toggle invert X");
  Serial.println("  y  toggle invert Y");
  Serial.println("  r  next display rotation (0-7; 4-7 are mirrored)");
  Serial.println("  R  previous display rotation");
  Serial.println("  c  auto-calibrate: tap the 4 corner targets");
  Serial.println("  t  toggle per-touch serial logging (default off)");
  Serial.println("  n  toggle panel colour inversion");
  Serial.println("  b  toggle RGB/BGR colour order");
  Serial.println("  w  wipe the screen (clear stray dots)");
  Serial.println("----------------------------------------");
  Serial.println("TEXT BACKWARDS/MIRRORED? That's the DISPLAY, not touch:");
  Serial.println("  press 'r' until it reads correctly. 0-3 are plain");
  Serial.println("  rotations, 4-7 are those same four MIRRORED - only 4-7");
  Serial.println("  can un-mirror the text. ('n' is colour inversion, which");
  Serial.println("  is a different thing and won't fix it.)");
  Serial.println();
  Serial.println("TAPS LANDING WRONG? That's touch: try 's' / 'x' / 'y',");
  Serial.println("  usually one of those three. Use 'c' if taps are in the");
  Serial.println("  right area but drift near the edges.");
  Serial.println();
}

// ---- guided corner capture ----
struct Sample { uint16_t rx, ry; };
Sample corners[4];
const char* cornerName[4] = { "TOP-LEFT", "TOP-RIGHT", "BOTTOM-LEFT", "BOTTOM-RIGHT" };

void drawTarget(int x, int y, const char* label) {
  tft.fillScreen(TFT_BLACK);
  tft.drawRect(0, 0, tft.width(), tft.height(), TFT_DARKGREY);
  const int r = 18;
  tft.drawCircle(x, y, r, TFT_GREEN);
  tft.drawCircle(x, y, r - 6, TFT_GREEN);
  tft.drawLine(x - r - 8, y, x + r + 8, y, TFT_GREEN);
  tft.drawLine(x, y - r - 8, x, y + r + 8, TFT_GREEN);
  tft.setFont(&fonts::FreeSansBold12pt7b);
  tft.setTextDatum(textdatum_t::middle_center);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("TAP", tft.width() / 2, tft.height() / 2 - 14);
  tft.drawString(label, tft.width() / 2, tft.height() / 2 + 14);
  tft.setTextDatum(textdatum_t::top_left);
}

void autoCalibrate() {
  int W = tft.width(), H = tft.height();
  const int inset = 24;
  int px[4] = { inset, W - 1 - inset, inset,         W - 1 - inset };
  int py[4] = { inset, inset,         H - 1 - inset, H - 1 - inset };

  Serial.println("\nCalibrate: tap each target as it appears.");
  for (int i = 0; i < 4; i++) {
    drawTarget(px[i], py[i], cornerName[i]);
    uint16_t rx, ry;
    while (!readTouch(rx, ry)) delay(10);
    corners[i] = { rx, ry };
    Serial.printf("  %-13s screen=(%3d,%3d)  RAW=(%4u,%4u)\n",
                  cornerName[i], px[i], py[i], rx, ry);
    tft.fillCircle(px[i], py[i], 7, TFT_RED);
    delay(200);
    waitRelease();
  }

  // Compare how much each raw axis moves per unit of screen-x versus
  // screen-y: whichever raw axis tracks screen-x is the one that maps to
  // x. That detects a swap from the data instead of guessing. The sign of
  // that movement gives the inversion.
  long dRX_dSX = ((long)corners[1].rx - corners[0].rx) + ((long)corners[3].rx - corners[2].rx);
  long dRX_dSY = ((long)corners[2].rx - corners[0].rx) + ((long)corners[3].rx - corners[1].rx);
  long dRY_dSX = ((long)corners[1].ry - corners[0].ry) + ((long)corners[3].ry - corners[2].ry);
  long dRY_dSY = ((long)corners[2].ry - corners[0].ry) + ((long)corners[3].ry - corners[1].ry);

  swapXY = (labs(dRX_dSY) + labs(dRY_dSX)) > (labs(dRX_dSX) + labs(dRY_dSY));

  if (!swapXY) {
    invX = dRX_dSX < 0;
    invY = dRY_dSY < 0;
    rawXmin = min(min(corners[0].rx, corners[2].rx), min(corners[1].rx, corners[3].rx));
    rawXmax = max(max(corners[0].rx, corners[2].rx), max(corners[1].rx, corners[3].rx));
    rawYmin = min(min(corners[0].ry, corners[1].ry), min(corners[2].ry, corners[3].ry));
    rawYmax = max(max(corners[0].ry, corners[1].ry), max(corners[2].ry, corners[3].ry));
  } else {
    invX = dRY_dSX < 0;
    invY = dRX_dSY < 0;
    rawXmin = min(min(corners[0].ry, corners[2].ry), min(corners[1].ry, corners[3].ry));
    rawXmax = max(max(corners[0].ry, corners[2].ry), max(corners[1].ry, corners[3].ry));
    rawYmin = min(min(corners[0].rx, corners[1].rx), min(corners[2].rx, corners[3].rx));
    rawYmax = max(max(corners[0].rx, corners[1].rx), max(corners[2].rx, corners[3].rx));
  }

  // targets sit `inset` in from each edge, so extrapolate the measured
  // span out to the true 0..W-1 / 0..H-1 edges
  int spanX = W - 1 - 2 * inset, spanY = H - 1 - 2 * inset;
  if (spanX > 0) {
    long perPx = ((long)rawXmax - rawXmin) / spanX;
    rawXmin = (uint16_t)max(0L,    (long)rawXmin - perPx * inset);
    rawXmax = (uint16_t)min(4095L, (long)rawXmax + perPx * inset);
  }
  if (spanY > 0) {
    long perPx = ((long)rawYmax - rawYmin) / spanY;
    rawYmin = (uint16_t)max(0L,    (long)rawYmin - perPx * inset);
    rawYmax = (uint16_t)min(4095L, (long)rawYmax + perPx * inset);
  }

  printConfig();
  drawUI();
}

void applyRotation() {
  tft.setRotation(rotation);
  drawUI();
  Serial.printf("rotation=%u  display=%dx%d\n", rotation, tft.width(), tft.height());
}

void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n' || c == ' ') continue;
    switch (c) {
      case 'h': printHelp(); break;
      case 'p': printConfig(); break;
      case 's': swapXY = !swapXY; Serial.printf("swapXY=%s\n", swapXY?"true":"false"); drawUI(); break;
      case 'x': invX = !invX;     Serial.printf("invX=%s\n",   invX?"true":"false");   drawUI(); break;
      case 'y': invY = !invY;     Serial.printf("invY=%s\n",   invY?"true":"false");   drawUI(); break;
      case 'r': rotation = (rotation + 1) & 7; applyRotation(); break;
      case 'R': rotation = (rotation + 7) & 7; applyRotation(); break;
      case 'c': autoCalibrate(); break;
      case 't': verbose = !verbose; Serial.printf("touch logging %s\n", verbose?"ON":"off"); break;
      case 'w': drawUI(); break;
      case 'n':
        invertDisp = !invertDisp;
        tft.invertDisplay(invertDisp);
        Serial.printf("panel invert=%s\n", invertDisp?"true":"false");
        break;
      case 'b':
        // no runtime setter for colour order - report what to change and
        // where, rather than pretending it took effect
        bgrOrder = !bgrOrder;
        Serial.printf("rgb_order BGR=%s  (compile-time: set cfg.rgb_order in the config to match, then re-flash)\n",
                      bgrOrder?"true":"false");
        break;
      default:
        Serial.printf("unknown key '%c' - press 'h' for help\n", c);
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(600);

  pinMode(T_CS, OUTPUT);
  digitalWrite(T_CS, HIGH);
  pinMode(T_IRQ, INPUT);
  touchSPI.begin(T_CLK, T_MISO, T_MOSI, T_CS);

  tft.init();
  tft.setRotation(rotation);
  tft.setTextWrap(false);
  tft.setBrightness(180);

  Serial.println("\n\n=== CYD touch debug (interactive) ===");
  Serial.printf("display reports %d x %d\n", tft.width(), tft.height());
  printHelp();
  printConfig();
  drawUI();
}

void loop() {
  handleSerial();

  int x, y;
  if (mapTouch(x, y)) {
    tft.fillCircle(x, y, 4, TFT_RED);
    // coordinates on-screen always (not serial spam) so the mapping can be
    // judged by eye without watching the monitor
    tft.fillRect(0, tft.height() - 20, tft.width(), 20, TFT_BLACK);
    tft.setFont(&fonts::FreeSansBold9pt7b);
    tft.setTextDatum(textdatum_t::middle_center);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawString(String(x) + " , " + String(y), tft.width() / 2, tft.height() - 10);
    tft.setTextDatum(textdatum_t::top_left);

    if (verbose && millis() - lastVerboseAt >= VERBOSE_MIN_GAP_MS) {
      lastVerboseAt = millis();
      Serial.printf("touch -> (%3d,%3d)\n", x, y);
    }
    delay(40);
  }
}

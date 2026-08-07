// ---------------------------------------------------------------------
// Standalone CYD touch/display debug sketch.
//
//   pio run -e cyd-debug -t upload && pio device monitor -e cyd-debug
//
// Deliberately self-contained: its own display config, its own touch
// driver, no dependency on src/main.cpp or LGX_Config_CYD.h. If this
// sketch works, the settings it prints are known-good for the hardware,
// independent of anything the main firmware is doing.
//
// It talks to the XPT2046 touch controller directly over SPI rather than
// going through the graphics library's touch layer, so nothing here
// depends on a stored calibration - which is the point, since a bad
// calibration is exactly what stops you reaching the settings screen in
// the main firmware.
//
// What it does:
//   1. Draws a target in each corner in turn and asks you to tap it.
//   2. Records the RAW 12-bit values the touch chip reports for each.
//   3. Prints a summary: the raw ranges, whether the axes are swapped,
//      whether either is inverted, and the exact values to use.
//   4. Drops into a free-draw mode using the mapping it just derived, so
//      you can confirm taps land where you actually press.
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

// Median of several reads - the raw values are noisy enough that a single
// sample can be well off, which would poison the derived mapping.
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
  delay(120);   // settle, avoids one press registering twice
}

// ---- calibration capture ----
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

void captureCorners() {
  int W = tft.width(), H = tft.height();
  const int inset = 24;
  int px[4] = { inset, W - 1 - inset, inset,          W - 1 - inset };
  int py[4] = { inset, inset,         H - 1 - inset,  H - 1 - inset };

  for (int i = 0; i < 4; i++) {
    drawTarget(px[i], py[i], cornerName[i]);
    Serial.printf("\nTap the %s target...\n", cornerName[i]);
    uint16_t rx, ry;
    while (!readTouch(rx, ry)) delay(10);
    corners[i] = { rx, ry };
    Serial.printf("  %-13s screen=(%3d,%3d)  RAW=(%4u,%4u)\n",
                  cornerName[i], px[i], py[i], rx, ry);
    tft.fillCircle(px[i], py[i], 7, TFT_RED);
    delay(250);
    waitRelease();
  }
}

// derived mapping
bool  swapXY = false, invX = false, invY = false;
uint16_t rawXmin, rawXmax, rawYmin, rawYmax;

void analyse() {
  // How much does each raw axis move when the SCREEN x changes vs when
  // the SCREEN y changes? Whichever raw axis tracks screen-x is the one
  // that maps to x - that's what detects a swap, no guessing.
  long dRawX_perScreenX = ((long)corners[1].rx - corners[0].rx) + ((long)corners[3].rx - corners[2].rx);
  long dRawX_perScreenY = ((long)corners[2].rx - corners[0].rx) + ((long)corners[3].rx - corners[1].rx);
  long dRawY_perScreenX = ((long)corners[1].ry - corners[0].ry) + ((long)corners[3].ry - corners[2].ry);
  long dRawY_perScreenY = ((long)corners[2].ry - corners[0].ry) + ((long)corners[3].ry - corners[1].ry);

  swapXY = (labs(dRawX_perScreenY) + labs(dRawY_perScreenX)) >
           (labs(dRawX_perScreenX) + labs(dRawY_perScreenY));

  if (!swapXY) {
    invX = dRawX_perScreenX < 0;
    invY = dRawY_perScreenY < 0;
    rawXmin = min(min(corners[0].rx, corners[2].rx), min(corners[1].rx, corners[3].rx));
    rawXmax = max(max(corners[0].rx, corners[2].rx), max(corners[1].rx, corners[3].rx));
    rawYmin = min(min(corners[0].ry, corners[1].ry), min(corners[2].ry, corners[3].ry));
    rawYmax = max(max(corners[0].ry, corners[1].ry), max(corners[2].ry, corners[3].ry));
  } else {
    invX = dRawY_perScreenX < 0;
    invY = dRawX_perScreenY < 0;
    rawXmin = min(min(corners[0].ry, corners[2].ry), min(corners[1].ry, corners[3].ry));
    rawXmax = max(max(corners[0].ry, corners[2].ry), max(corners[1].ry, corners[3].ry));
    rawYmin = min(min(corners[0].rx, corners[1].rx), min(corners[2].rx, corners[3].rx));
    rawYmax = max(max(corners[0].rx, corners[1].rx), max(corners[2].rx, corners[3].rx));
  }

  Serial.println("\n================ RESULT ================");
  Serial.printf("display          : %d x %d\n", tft.width(), tft.height());
  Serial.printf("axes swapped     : %s\n", swapXY ? "YES (raw X drives screen Y)" : "no");
  Serial.printf("invert X         : %s\n", invX ? "YES" : "no");
  Serial.printf("invert Y         : %s\n", invY ? "YES" : "no");
  Serial.printf("raw X range      : %u .. %u\n", rawXmin, rawXmax);
  Serial.printf("raw Y range      : %u .. %u\n", rawYmin, rawYmax);
  Serial.println("----------------------------------------");
  Serial.println("Paste these 6 lines back and they can be applied to the");
  Serial.println("main firmware's touch config directly.");
  Serial.println("========================================\n");
  Serial.println("Free-draw mode: tap anywhere, a dot should appear exactly");
  Serial.println("under your finger. Coordinates are printed for every touch.\n");
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

void drawFreeDrawScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.drawRect(0, 0, tft.width(), tft.height(), TFT_DARKGREY);
  // reference marks at the exact corners and centre
  int W = tft.width(), H = tft.height();
  int pts[5][2] = { {0,0}, {W-1,0}, {0,H-1}, {W-1,H-1}, {W/2,H/2} };
  for (auto &p : pts) {
    tft.drawLine(p[0]-10, p[1], p[0]+10, p[1], TFT_DARKGREEN);
    tft.drawLine(p[0], p[1]-10, p[0], p[1]+10, TFT_DARKGREEN);
  }
  tft.setFont(&fonts::FreeSansBold9pt7b);
  tft.setTextDatum(textdatum_t::middle_center);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("FREE DRAW - dots follow your finger", W / 2, 14);
  tft.setTextDatum(textdatum_t::top_left);
}

void setup() {
  Serial.begin(115200);
  delay(600);
  Serial.println("\n\n=== CYD touch debug ===");

  pinMode(T_CS, OUTPUT);
  digitalWrite(T_CS, HIGH);
  pinMode(T_IRQ, INPUT);
  touchSPI.begin(T_CLK, T_MISO, T_MOSI, T_CS);

  tft.init();
  tft.setRotation(0);          // deliberately plain - this sketch derives
  tft.setTextWrap(false);      // the mapping itself rather than relying on one
  tft.setBrightness(180);

  Serial.printf("display reports %d x %d\n", tft.width(), tft.height());
  tft.fillScreen(TFT_BLACK);
  tft.setFont(&fonts::FreeSansBold12pt7b);
  tft.setTextDatum(textdatum_t::middle_center);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("CYD TOUCH DEBUG", tft.width() / 2, tft.height() / 2 - 16);
  tft.setFont(&fonts::FreeSans9pt7b);
  tft.drawString(String(tft.width()) + " x " + String(tft.height()),
                 tft.width() / 2, tft.height() / 2 + 14);
  tft.setTextDatum(textdatum_t::top_left);
  delay(1800);

  captureCorners();
  analyse();
  drawFreeDrawScreen();
}

void loop() {
  int x, y;
  if (mapTouch(x, y)) {
    tft.fillCircle(x, y, 4, TFT_RED);
    tft.fillRect(0, tft.height() - 22, tft.width(), 22, TFT_BLACK);
    tft.setFont(&fonts::FreeSansBold9pt7b);
    tft.setTextDatum(textdatum_t::middle_center);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawString(String(x) + " , " + String(y), tft.width() / 2, tft.height() - 11);
    tft.setTextDatum(textdatum_t::top_left);
    Serial.printf("touch -> screen (%3d,%3d)\n", x, y);
    delay(60);
  }
}

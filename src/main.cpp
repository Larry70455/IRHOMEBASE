#define NO_LED_FEEDBACK_CODE
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <IRremote.hpp>
#include <FastLED.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <map>
#include <vector>
#include <algorithm>
#include <utility>
#include <ctype.h>
#include <SPI.h>   // CYD reads the XPT2046 touch controller directly over its own SPI instance

// Board selection is set at the build-system level: platformio.ini's
// [env:cyd] passes -D BOARD_CYD, [env:esp32-s3-devkitm-1] doesn't. Every
// hardware difference between the two boards branches off this one flag
// (or HAS_WS2812 below, which follows from it) rather than a scattered set
// of ad-hoc checks.
#if defined(BOARD_C3KNOB)
  #include "LGX_Config_C3.h"
  #include <lvgl.h>
#elif defined(BOARD_CYD)
  #include "LGX_Config_CYD.h"
#else
  #include "LGX_Config.h"
#endif

#define PROFILE_PATH "/profile.json"

// CYD has no WS2812 strip and no dedicated learn/blast buttons - the
// touchscreen replaces both (see the BOARD_CYD screen-manager section
// further down). IR receive/transmit run directly off GPIO22/GPIO27,
// broken out on CYD's CN1 header and confirmed free/unclaimed by the
// display, touch, RGB LED, speaker, SD card, or light sensor circuitry.
#if defined(BOARD_C3KNOB)
  // ESP32-C3-LCDkit. The board normally shares one IR line on IO4 with a
  // jumper picking RX or TX - which would make learn and blast mutually
  // exclusive. IO8 is given up to carry TX instead so both work at once.
  #define RECV_PIN 4
  #define SEND_PIN 8
  #define HAS_WS2812 0
  // rotary encoder: the entire input device on this board
  #define ENC_A_PIN  10
  #define ENC_B_PIN  6
  #define ENC_SW_PIN 9
#elif defined(BOARD_CYD)
  #define RECV_PIN 22
  #define SEND_PIN 27
  #define HAS_WS2812 0
#else
  #define RECV_PIN 5
  #define SEND_PIN 6
  #define LED_PIN  7
  #define NUM_LEDS 8
  #define HAS_WS2812 1
  #define LEARN_BTN_PIN 19
  #define BLAST_BTN_PIN 20
#endif

// Capability flag rather than another board-name test at every site: both
// the CYD and the C3 knob replace the dedicated learn/blast buttons and
// the 8-LED slot strip with their own UI, so "#ifndef BOARD_CYD" was
// already the wrong question - it silently included S3-only code on any
// future board.
#if defined(BOARD_CYD) || defined(BOARD_C3KNOB)
  #define HAS_PHYSICAL_BUTTONS 0
#else
  #define HAS_PHYSICAL_BUTTONS 1
#endif

// One slot per LED on the original board; on CYD there's no LED strip or
// physical slot-select button, but the slot data/handlers/web tab still
// compile and stay reachable over the web either way (see plan notes) - so
// this stays unconditional rather than following HAS_WS2812.
#define NUM_SLOTS 8

const char* ssid = "Home_2g";
const char* password = "604428LS";
const char* hostname = "IR";

LGFX tft;
#if HAS_WS2812
CRGB leds[NUM_LEDS];
#endif
WebServer server(80);

// ---------- IR code storage ----------
#define MAX_RAW_LEN 400

// Two kinds of code, distinguished by protocol.length(): a raw learned
// signal (protocol == "", the original path - physical learning always
// produces these), or a protocol-encoded one from the IRDB lookup feature
// (protocol/address/command instead of a raw timing buffer). sendCode()
// branches on which one it has.
struct IRCode {
  uint16_t data[MAX_RAW_LEN];
  uint16_t len;
  uint32_t hash;
  String protocol;   // "" = raw code below; else e.g. "NEC"
  uint16_t address;  // protocol-encoded only
  uint16_t command;  // protocol-encoded only
};

std::map<String, IRCode> codeLibrary;  // name -> code (raw or protocol-encoded)

String pendingLearnName = "";  // if set, next captured signal gets saved under this name
bool learning = false;
unsigned long learningStartedAt = 0;
const unsigned long LEARN_TIMEOUT_MS = 30000;  // give up waiting for a signal after 30s
int pendingLearnSlot = -1;     // if >= 0, the in-progress learn (above) is for this physical slot
// If >= 0, the in-progress learn is for a virtual remote button: the
// captured code is saved and assigned to that button in one step, which is
// what makes "lay the remote out, then teach every button" work without
// naming each code by hand first.
int pendingLearnRemote = -1;
int pendingLearnButton = -1;
// Teach-all walks the current remote's un-coded buttons back to back,
// re-arming the next learn as soon as one completes.
bool teachAllActive = false;
String lastStatus = "Booting...";

// ---------- icons ----------
// Small vector-drawn symbols instead of arbitrary bitmaps (no image assets
// to embed/verify) - used by both physical slots and virtual remote
// buttons. Numbers reuse the existing text/font system rather than being
// drawn as shapes.
enum IconId : uint8_t {
  ICON_NONE = 0,
  ICON_UP, ICON_DOWN, ICON_LEFT, ICON_RIGHT,
  ICON_PLUS, ICON_MINUS,
  ICON_CHECK, ICON_X,
  ICON_POWER, ICON_WIFI,
  ICON_0, ICON_1, ICON_2, ICON_3, ICON_4, ICON_5, ICON_6, ICON_7, ICON_8, ICON_9,
  ICON_COUNT
};

// Draws icon `id` centered in the box (x,y,w,h). Every shape is computed
// proportionally from the box size, not fixed pixel offsets, so this works
// regardless of the exact cell size passed in (slots and remote buttons
// use different cell sizes).
void drawIcon(uint8_t id, int x, int y, int w, int h, uint16_t color) {
  int cx = x + w / 2, cy = y + h / 2;
  int s = min(w, h);
  int a = s * 3 / 8;  // half-size of most shapes

  switch (id) {
    case ICON_UP:
      tft.fillTriangle(cx, cy - a, cx - a, cy + a, cx + a, cy + a, color);
      break;
    case ICON_DOWN:
      tft.fillTriangle(cx, cy + a, cx - a, cy - a, cx + a, cy - a, color);
      break;
    case ICON_LEFT:
      tft.fillTriangle(cx - a, cy, cx + a, cy - a, cx + a, cy + a, color);
      break;
    case ICON_RIGHT:
      tft.fillTriangle(cx + a, cy, cx - a, cy - a, cx - a, cy + a, color);
      break;
    case ICON_PLUS: {
      int t = max(2, s / 8);
      tft.fillRect(cx - a, cy - t / 2, a * 2, t, color);
      tft.fillRect(cx - t / 2, cy - a, t, a * 2, color);
      break;
    }
    case ICON_MINUS: {
      int t = max(2, s / 8);
      tft.fillRect(cx - a, cy - t / 2, a * 2, t, color);
      break;
    }
    case ICON_CHECK:
      tft.drawLine(cx - a, cy, cx - a / 3, cy + a, color);
      tft.drawLine(cx - a / 3, cy + a, cx + a, cy - a, color);
      tft.drawLine(cx - a, cy + 1, cx - a / 3, cy + a + 1, color);
      tft.drawLine(cx - a / 3, cy + a + 1, cx + a, cy - a + 1, color);
      break;
    case ICON_X:
      tft.drawLine(cx - a, cy - a, cx + a, cy + a, color);
      tft.drawLine(cx - a, cy + a, cx + a, cy - a, color);
      tft.drawLine(cx - a + 1, cy - a, cx + a + 1, cy + a, color);
      tft.drawLine(cx - a + 1, cy + a, cx + a + 1, cy - a, color);
      break;
    case ICON_POWER:
      tft.drawCircle(cx, cy + a / 4, a, color);
      tft.drawCircle(cx, cy + a / 4, a - 1, color);
      tft.fillRect(cx - 1, cy - a - 2, 3, a, color);
      break;
    case ICON_WIFI: {
      const int bars = 4;
      int barW = max(2, s / 10);
      int gap = max(1, s / 16);
      int total = bars * barW + (bars - 1) * gap;
      int startX = cx - total / 2;
      for (int i = 0; i < bars; i++) {
        int barH = a * (i + 1) / bars;
        tft.fillRect(startX + i * (barW + gap), cy + a - barH, barW, barH, color);
      }
      break;
    }
    default:
      if (id >= ICON_0 && id <= ICON_9) {
        tft.setTextColor(color);
        tft.setTextDatum(textdatum_t::middle_center);
        tft.drawString(String((char)('0' + (id - ICON_0))), cx, cy);
      }
      break;
  }
}

// ---------- physical slots ----------
// 8 virtual buttons, one per LED: button 1 fires whichever slot is
// currently selected, button 2 short-press cycles slots, button 2 held
// learns into the current slot. Each slot just points at a name in
// codeLibrary, so it reuses the same learn/rename/delete machinery the web
// app already has - the app can reassign a slot to any existing code too.
String slotCodeName[NUM_SLOTS];  // "" = unassigned
CRGB slotColor[NUM_SLOTS];       // per-slot indicator color when assigned+unselected
uint8_t slotIcon[NUM_SLOTS];     // IconId; ICON_NONE falls back to the code name as text
int currentSlot = 0;

// ---------- customizable display colors ----------
// Used by both the LEDs and the screen so the two stay visually consistent.
// Defaults match what used to be hardcoded; adjustable from the web app's
// "Display colors" section (persisted below in saveProfile/loadProfile).
// Per-slot/per-remote-button colors (above/below) cover "different colors
// for each button" - these two are the ones that aren't tied to a specific
// button: which one is currently selected, and the transmit animation.
CRGB accentColor = CRGB(0, 90, 100);    // selected slot/button highlight
CRGB waveColor = CRGB(0, 60, 220);      // transmit sweep
CRGB DEFAULT_SLOT_COLOR = CRGB(0, 60, 20);  // a newly-assigned slot starts out this color

String colorToHex(const CRGB &c) {
  char buf[8];
  snprintf(buf, sizeof(buf), "#%02x%02x%02x", c.r, c.g, c.b);
  return String(buf);
}

CRGB parseHexColor(const String &hex, CRGB fallback) {
  String h = hex;
  if (h.startsWith("#")) h = h.substring(1);
  if (h.length() != 6) return fallback;
  for (size_t i = 0; i < 6; i++) {
    if (!isxdigit((unsigned char)h.charAt(i))) return fallback;
  }
  long v = strtol(h.c_str(), nullptr, 16);
  return CRGB((v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF);
}

// picks black or white text for legibility against an arbitrary
// user-chosen background color, since a custom accent color might not be
// as reliably light as the old hardcoded cyan was
uint16_t contrastTextColor(const CRGB &bg) {
  int luminance = (bg.r * 299 + bg.g * 587 + bg.b * 114) / 1000;
  return (luminance > 140) ? TFT_BLACK : TFT_WHITE;
}

// ---------- IR sending ----------
// Sent synchronously, on the same core/task as the receiver (setup()/loop()
// run on core 1) and the web server. IRremote's ESP32 timer-based send/receive
// is not safe to poke from another core concurrently - an earlier version of
// this sent from a separate task pinned to core 0, which manipulated the
// receiver's hardware timer (stop/restartTimer/resume) from a different core
// than it was initialized on. That's the most likely reason blasts were
// silently failing.
volatile bool suppressReceive = false;

// Protocol-encoded codes come from the IRDB lookup feature (see below): it
// gives protocol/address/command rather than a raw timing capture, so
// sending means re-encoding via IRremote's named protocol senders rather
// than replaying a buffer. This is an explicit allow-list, not a "looks
// like it might work" guess - every entry here was checked against
// IRremote's actual public sendXxx() signatures (Arduino-IRremote source)
// and against real sample rows fetched from IRDB during development, so
// the device/subdevice/function -> address/command mapping is grounded in
// something concrete. Still, none of it has been fired at real hardware -
// NEC is the one family independently verified this session (its
// "extended vs classic 8-bit" split falls out naturally from whether the
// combined address exceeds 0xFF, which is also how IrSender.sendNEC()
// itself decides, so there's no guessing there). The rest carries real but
// lower confidence. Deliberately left out: bare "Kaseikyo" (the vendor ID
// baked into the frame differs per manufacturer and IRDB doesn't expose
// which one to use), Samsung's 36/48-bit and "SamsungLG" variants, Sony
// bit-widths other than 12/15/20, and anything not in this list at all
// (Fujitsu, Sharp, Denon, Mitsubishi, Onkyo, ...) - those are reported as
// unsupported rather than sent on a guess.
bool isProtocolSupported(const String &protocolIn) {
  String p = protocolIn;
  p.toUpperCase();
  if (p.indexOf("NEC") >= 0) return true;
  if (p == "SONY" || p == "SONY12" || p == "SONY15" || p == "SONY20") return true;
  if (p == "RC5" || p == "RC-5") return true;
  if (p == "RC6" || p == "RC-6") return true;
  if (p == "JVC") return true;
  if (p == "PANASONIC") return true;
  if (p == "SAMSUNG") return true;
  if (p == "LG") return true;
  return false;
}

// Only called once isProtocolSupported() has already said yes - dispatches
// to the matching IRremote sender. Every non-NEC protocol here uses
// `device` directly as a single address value (confirmed against real IRDB
// samples: e.g. Sony15 rows list device=100 as the SIRC address, not
// something needing NEC's device+subdevice packing).
void sendProtocolEncoded(const IRCode &code) {
  String p = code.protocol;
  p.toUpperCase();
  if (p.indexOf("NEC") >= 0) {
    IrSender.sendNEC(code.address, (uint8_t)code.command, 0);
  } else if (p.startsWith("SONY")) {
    uint8_t bits = 12;
    if (p == "SONY20") bits = 20;
    else if (p == "SONY15") bits = 15;
    IrSender.sendSony(code.address, (uint8_t)code.command, 0, bits);
  } else if (p == "RC5" || p == "RC-5") {
    IrSender.sendRC5((uint8_t)code.address, (uint8_t)code.command, 0, true);
  } else if (p == "RC6" || p == "RC-6") {
    IrSender.sendRC6((uint8_t)code.address, (uint8_t)code.command, 0, true);
  } else if (p == "JVC") {
    IrSender.sendJVC((uint8_t)code.address, (uint8_t)code.command, 0);
  } else if (p == "PANASONIC") {
    IrSender.sendPanasonic(code.address, (uint8_t)code.command, 0);
  } else if (p == "SAMSUNG") {
    IrSender.sendSamsung(code.address, code.command, 0);
  } else if (p == "LG") {
    IrSender.sendLG((uint8_t)code.address, code.command, 0);
  }
}

void sendCode(const IRCode &code) {
  bool isProtocolEncoded = code.protocol.length() > 0;

  if (isProtocolEncoded) {
    if (!isProtocolSupported(code.protocol)) {
      Serial.println("sendCode: protocol '" + code.protocol + "' not supported for sending yet");
      return;
    }
  } else if (code.len == 0) {
    Serial.println("sendCode: skipping empty code (nothing captured)");
    return;
  }

  suppressReceive = true;
  IrReceiver.stop();

  if (isProtocolEncoded) {
    Serial.printf("Sending %s code, address=%u command=%u\n",
                  code.protocol.c_str(), code.address, code.command);
    sendProtocolEncoded(code);
  } else {
    Serial.print("Sending raw code, len=");
    Serial.println(code.len);
    IrSender.sendRaw(code.data, code.len, 38);
  }

  delay(40); // let the IR line settle

#ifdef BOARD_C3KNOB
  // IO8 carries the IR carrier AND the board's onboard LED. IRremote
  // leaves the pin attached to its LEDC channel when it's done, which can
  // leave the LED lit after a transmission. Detach it and drive the pin
  // low so the line - and therefore the LED - actually rests off.
  // IRremote reconfigures the pin on every send (enableIROut ->
  // timerConfigForSend), so taking it back here doesn't break the next one.
  // the detach call was renamed in Arduino core 3.x - same version split
  // this file already handles in initWatchdog()
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcDetach(SEND_PIN);
#else
  ledcDetachPin(SEND_PIN);
#endif
  pinMode(SEND_PIN, OUTPUT);
  digitalWrite(SEND_PIN, LOW);
#endif

  IrReceiver.restartTimer();
  IrReceiver.resume();
  suppressReceive = false;

  Serial.println("Send complete");
}

void sendCodeByName(const String &name) {
  auto it = codeLibrary.find(name);
  if (it != codeLibrary.end()) {
    sendCode(it->second);
  } else {
    Serial.println("sendCodeByName: no code named '" + name + "'");
  }
}

// A single code send still blocks briefly (the raw transmission time plus a
// settle delay) - that part's unavoidable without going back to a separate
// task, which is what broke sending in the first place (see above). Queued
// so the web server/IR receiver/LEDs all still get serviced between sends
// rather than blocking loop() while a request is handled.
std::vector<String> pendingSends;

void queueCodeSend(const String &name) {
  pendingSends.push_back(name);
}

// ---------- screen / LEDs ----------
// Every piece of text below is measured with textWidth() and truncated to
// fit its box before drawing - deliberately not relying on the library's
// own word-wrap, so nothing can ever run off the edge of the panel
// regardless of exact font metrics.
//
// These are seeded with the portrait default but overwritten in setup()
// from tft.width()/tft.height() once the rotation is applied - they are
// NOT constants. Hardcoding them meant that any rotation which swapped the
// axes (or any panel not exactly 240x320) left every layout computing
// against the wrong extent, which is what left part of the screen never
// being drawn to. Deriving them from the panel is correct for both boards
// and for all 8 rotation values.
int SCREEN_W = 240;
int SCREEN_H = 320;
const uint8_t SCREEN_BRIGHTNESS = 180;
bool screenAsleep = false;
unsigned long lastActivityAt = 0;
// 0 = never sleep, which is the default: on a touchscreen-primary build a
// blank screen is just an obstacle - you have to wake it before you can
// use it. Adjustable (and persisted) via /screensleep for anyone who does
// want the backlight to drop after a while.
unsigned long screensaverTimeoutMs = 0;

// Trims s until "s..." fits within maxWidth (assumes the current font is
// already set). Never overflows maxWidth, whatever the font's real metrics.
String truncateToWidth(const String &s, int maxWidth) {
  if (tft.textWidth(s) <= maxWidth) return s;
  String t = s;
  while (t.length() > 1 && tft.textWidth(t + "...") > maxWidth) {
    t.remove(t.length() - 1);
  }
  return t + "...";
}

// Up to 2 lines: splits on the first space (status messages here are all
// short phrases like "Learned: Button 3", not paragraphs), truncating each
// resulting line independently so neither can overflow maxWidth.
void drawStatusText(const String &text, int x, int y, int maxWidth, int lineHeight) {
  if (tft.textWidth(text) <= maxWidth) {
    tft.drawString(text, x, y);
    return;
  }
  int splitAt = text.indexOf(' ');
  if (splitAt == -1) {
    tft.drawString(truncateToWidth(text, maxWidth), x, y);
    return;
  }
  tft.drawString(truncateToWidth(text.substring(0, splitAt), maxWidth), x, y);
  tft.drawString(truncateToWidth(text.substring(splitAt + 1), maxWidth), x, y + lineHeight);
}

void drawWifiIcon(int x, int y) {
  uint16_t color = (WiFi.status() == WL_CONNECTED) ? TFT_GREEN : TFT_RED;
  const int barW = 4, gap = 2;
  const int heights[4] = {4, 8, 12, 16};
  for (int i = 0; i < 4; i++) {
    tft.fillRect(x + i * (barW + gap), y + (16 - heights[i]), barW, heights[i], color);
  }
}

// Same three states as the LEDs (see updateSlotLeds()): filled accentColor
// for the selected slot, that slot's own custom color as an outline for any
// other slot that has a code, plain white outline for a genuinely empty one
// - so you can tell a slot is set (and which is which) without cycling to
// it. Each cell shows the slot's icon if it has one, otherwise the assigned
// code's name (truncated to fit) instead of just a slot number.
void drawSlotGrid(int gridX, int gridY, int cellW, int cellH, int gap) {
  tft.setFont(&fonts::FreeSansBold9pt7b);
  tft.setTextDatum(textdatum_t::middle_center);
  for (int i = 0; i < NUM_SLOTS; i++) {
    int col = i % 4;
    int row = i / 4;
    int cx = gridX + col * (cellW + gap);
    int cy = gridY + row * (cellH + gap);
    bool selected = (i == currentSlot);
    bool assigned = slotCodeName[i].length() > 0;

    uint16_t fillColor, borderColor, textColor;
    if (selected) {
      fillColor = tft.color565(accentColor.r, accentColor.g, accentColor.b);
      borderColor = fillColor;
      textColor = contrastTextColor(accentColor);
    } else if (assigned) {
      fillColor = TFT_BLACK;
      borderColor = tft.color565(slotColor[i].r, slotColor[i].g, slotColor[i].b);
      textColor = TFT_WHITE;
    } else {
      fillColor = TFT_BLACK;
      borderColor = TFT_WHITE;
      textColor = TFT_WHITE;
    }

    tft.fillRoundRect(cx, cy, cellW, cellH, 6, fillColor);
    tft.drawRoundRect(cx, cy, cellW, cellH, 6, borderColor);

    if (assigned && slotIcon[i] != ICON_NONE) {
      drawIcon(slotIcon[i], cx, cy, cellW, cellH, textColor);
      tft.setFont(&fonts::FreeSansBold9pt7b);  // drawIcon's digit case changes font/datum
      tft.setTextDatum(textdatum_t::middle_center);
      continue;
    }
    tft.setTextColor(textColor, fillColor);
    String label = assigned ? slotCodeName[i] : "-";
    tft.drawString(truncateToWidth(label, cellW - 6), cx + cellW / 2, cy + cellH / 2);
  }
  tft.setTextDatum(textdatum_t::top_left);
}

// The three CYD functions below are defined in the CYD screen-manager
// section further down (just above setup()), but called from here and from
// flashLeds()/loop(), all defined earlier in the file - forward declared
// so those call sites compile regardless of definition order.
#ifdef BOARD_CYD
void cydDrawStatusBar(const String &status);
void cydTintBanner(CRGB color, int ms);
void cydCheckBannerTintRevert();
#endif
#ifdef BOARD_C3KNOB
// Same pattern as the CYD wrappers above: the knob UI lives near the
// bottom of the file but these are called from updateScreen()/flashLeds(),
// which are defined here.
void c3NoteStatus(const String &status);
void c3FlashAccent(CRGB color, int ms);
void c3CheckFlashRevert();
#endif

void updateScreen(String status) {
  lastStatus = status;
  lastActivityAt = millis();
  if (screenAsleep) {
    screenAsleep = false;
    tft.setBrightness(SCREEN_BRIGHTNESS);
  }

#if defined(BOARD_C3KNOB)
  // The knob UI is LVGL-managed: this must not draw over it directly.
  // Hand the text to the UI layer, which puts it in its own status label
  // and lets LVGL repaint on its own schedule.
  c3NoteStatus(status);
#elif defined(BOARD_CYD)
  // CYD's screen is a Home/Learn touch GUI, not this board's fixed status
  // layout - only the persistent banner strip (shared across every CYD
  // screen) needs repainting here. A full-screen redraw on every status
  // change would otherwise wipe out whichever screen/tap-zones are showing.
  cydDrawStatusBar(status);
#else
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);

  // header
  uint16_t accent565 = tft.color565(accentColor.r, accentColor.g, accentColor.b);
  tft.fillRect(0, 0, SCREEN_W, 44, accent565);
  tft.setFont(&fonts::FreeSansBold12pt7b);
  tft.setTextDatum(textdatum_t::middle_center);
  tft.setTextColor(contrastTextColor(accentColor), accent565);
  tft.drawString("IR Controller", SCREEN_W / 2, 22);
  tft.setTextDatum(textdatum_t::top_left);

  // status (up to 2 lines)
  tft.setFont(&fonts::FreeSans9pt7b);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  drawStatusText(status, 10, 56, SCREEN_W - 20, 22);

  // 8-slot grid, mirrors the physical LEDs
  drawSlotGrid(12, 110, 48, 44, 8);

  // footer: WiFi status, hostname, code count, current slot
  tft.setFont(&fonts::FreeSans9pt7b);
  drawWifiIcon(10, 224);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  String host = String(hostname) + ".local";
  tft.drawString(truncateToWidth(host, SCREEN_W - 40), 34, 224);

  tft.drawString("Codes: " + String((int)codeLibrary.size()), 10, 250);

  String slotLine = "Slot " + String(currentSlot + 1) + ": " +
                     (slotCodeName[currentSlot].length() ? slotCodeName[currentSlot] : "(empty)");
  tft.drawString(truncateToWidth(slotLine, SCREEN_W - 20), 10, 272);

  tft.endWrite();
#endif
}

// Sets the LEDs immediately and returns without blocking; loop() reverts
// them (to the idle rainbow, or just off while learning) once the requested
// duration has elapsed. Previously this used delay(), which blocked the web
// server/receiver for the flash's full duration on nearly every action.
// CYD has no LEDs - flashLeds() itself branches below so every existing
// call site (learn/blast/assign/lookup/etc, 15+ of them) stays unchanged;
// CYD just tints the on-screen status banner instead of a physical LED,
// with the same non-blocking "set now, revert later" shape (see
// cydTintBanner() in the CYD screen-manager section further down).
#if HAS_WS2812
bool ledFlashActive = false;
unsigned long ledFlashUntil = 0;
#endif

void flashLeds(CRGB color, int ms) {
#if HAS_WS2812
  fill_solid(leds, NUM_LEDS, color);
  FastLED.show();
  ledFlashActive = true;
  ledFlashUntil = millis() + ms;
#elif defined(BOARD_C3KNOB)
  c3FlashAccent(color, ms);
#else
  cydTintBanner(color, ms);
#endif
}

// Visual "signal going out" cue: lights one LED at a time, 1 through 8, in
// place of the old solid flash after a send. This plays after the actual
// (blocking) transmission has already finished - the real send is too fast
// to visualize live - but reads as "there it goes" immediately after. CYD
// has no LED strip to animate this way - its call site (in loop(), where a
// queued send completes) falls back to a plain flashLeds() banner tint
// instead, which already has a CYD equivalent above.
#if HAS_WS2812
bool ledWaveActive = false;
unsigned long ledWaveStartedAt = 0;
const unsigned long LED_WAVE_STEP_MS = 60;

void startTransmitWave() {
  ledFlashActive = false;  // wave takes over the strip immediately
  ledWaveActive = true;
  ledWaveStartedAt = millis();
}
#endif

// ---------- capture helper ----------
IRCode captureCurrent() {
  IRCode code;
  code.len = 0;
  code.protocol = "";  // "" marks this as a raw code, not protocol-encoded (see IRCode)
  code.address = 0;
  code.command = 0;
  for (uint16_t i = 1; i < IrReceiver.decodedIRData.rawlen && code.len < MAX_RAW_LEN; i++) {
    code.data[code.len++] = IrReceiver.irparams.rawbuf[i] * MICROS_PER_TICK;
  }
  // bucketed hash of raw timings - tolerant of small jitter, works for ANY protocol
  // including ones IRremote can't decode by name
  code.hash = 0;
  for (uint16_t i = 0; i < code.len; i++) {
    code.hash = code.hash * 31 + (code.data[i] / 50);
  }
  return code;
}

// ---------- virtual remote ----------
// Separate from the 8 physical slots: any number of buttons, each pointing
// at a code, with its own name/icon/color, arranged in a reorderable grid
// (order in the vector = grid position, row-major by remoteColumns). This
// is the app's home view - build whatever layout you want, not limited to
// what fits on the physical hardware.
struct RemoteButton {
  String name;             // optional - only used in the edit list & MQTT entity naming, never shown on the tile
  String codeName;         // "" = no code assigned yet
  uint8_t icon = ICON_NONE;
  CRGB color = DEFAULT_SLOT_COLOR;
  bool spacer = false;     // true = an invisible layout-only placeholder tile, not a real button
};
// A named remote is a layout plus its buttons. Several can exist (TV,
// soundbar, projector...) and one is "current" - shown on the device
// screen and edited in the web app.
struct Remote {
  String name;
  int columns = 3;
  std::vector<RemoteButton> buttons;
};
std::vector<Remote> remotes;
int currentRemote = 0;

// remotes is never allowed to be empty and currentRemote is always a valid
// index - every path that could break that (load, delete, boot) calls this
// straight after. That keeps the accessors below unconditionally safe
// rather than every use site needing its own bounds check.
void ensureRemoteValid() {
  if (remotes.empty()) {
    Remote r;
    r.name = "Remote 1";
    remotes.push_back(r);
  }
  if (currentRemote < 0 || currentRemote >= (int)remotes.size()) currentRemote = 0;
}

// The whole codebase predates multiple remotes and refers to the single
// remote's data by these two names in ~50 places. Rather than hand-editing
// every one without a compiler to catch a miss, they now resolve to the
// current remote's fields. Macros specifically because they need no call
// syntax at the use sites - and they are safe here: macros are not
// expanded inside string literals, so the "remoteColumns" JSON key and the
// JS in PAGE_HTML are untouched.
#define remoteButtons (remotes[currentRemote].buttons)
#define remoteColumns (remotes[currentRemote].columns)

// Derives a code name for a button being taught, so laying out a remote
// and then teaching it never requires typing a name per button. Uniqueness
// is enforced against codeLibrary; re-teaching a button that already has a
// code reuses that name (see startButtonLearn) so it overwrites in place
// rather than accumulating tv_power_2, tv_power_3, ...
String autoCodeName(int remoteIdx, int btnIdx) {
  Remote &r = remotes[remoteIdx];
  String base = r.buttons[btnIdx].name;
  if (base.length() == 0) base = "btn" + String(btnIdx + 1);
  String raw = r.name + "_" + base;
  String slug;
  for (size_t i = 0; i < raw.length(); i++) {
    char c = raw.charAt(i);
    if (isalnum((unsigned char)c)) slug += (char)tolower(c);
    else if (slug.length() && slug.charAt(slug.length() - 1) != '_') slug += '_';
  }
  while (slug.length() && slug.charAt(slug.length() - 1) == '_') slug.remove(slug.length() - 1);
  if (slug.length() == 0) slug = "code";
  String out = slug;
  int n = 2;
  while (codeLibrary.count(out)) out = slug + "_" + String(n++);
  return out;
}

// Next button with no code yet, skipping spacers. -1 when the remote is
// fully taught - which is how teach-all knows it's finished.
int nextUntaughtButton(int from) {
  for (int i = from; i < (int)remoteButtons.size(); i++) {
    if (!remoteButtons[i].spacer && remoteButtons[i].codeName.length() == 0) return i;
  }
  return -1;
}

bool startButtonLearn(int idx) {
  ensureRemoteValid();
  if (idx < 0 || idx >= (int)remoteButtons.size()) return false;
  if (remoteButtons[idx].spacer) return false;
  String name = remoteButtons[idx].codeName;
  if (name.length() == 0) name = autoCodeName(currentRemote, idx);
  pendingLearnName = name;
  pendingLearnSlot = -1;
  pendingLearnRemote = currentRemote;
  pendingLearnButton = idx;
  learning = true;
  learningStartedAt = millis();
  String label = remoteButtons[idx].name.length() ? remoteButtons[idx].name
                                                  : ("Button " + String(idx + 1));
  updateScreen("Learning: " + label);
  flashLeds(CRGB::Yellow, 150);
  return true;
}

void stopTeachAll(const String &why) {
  teachAllActive = false;
  learning = false;
  pendingLearnName = "";
  pendingLearnRemote = -1;
  pendingLearnButton = -1;
  updateScreen(why);
}

// ---------- persistence (LittleFS + JSON) ----------
// Codes/slots/remote buttons are saved to flash after every mutation and
// reloaded on boot, so the library survives a power cycle or re-flash.
bool saveProfile() {
  JsonDocument doc;

  JsonObject codesObj = doc["codes"].to<JsonObject>();
  for (auto &kv : codeLibrary) {
    JsonObject codeObj = codesObj[kv.first].to<JsonObject>();
    if (kv.second.protocol.length() > 0) {
      codeObj["protocol"] = kv.second.protocol;
      codeObj["address"] = kv.second.address;
      codeObj["command"] = kv.second.command;
    } else {
      codeObj["hash"] = kv.second.hash;
      JsonArray dataArr = codeObj["data"].to<JsonArray>();
      for (uint16_t i = 0; i < kv.second.len; i++) {
        dataArr.add(kv.second.data[i]);
      }
    }
  }

  JsonArray slotsArr = doc["slots"].to<JsonArray>();
  for (int i = 0; i < NUM_SLOTS; i++) {
    JsonObject slotObj = slotsArr.add<JsonObject>();
    slotObj["code"] = slotCodeName[i];
    slotObj["color"] = colorToHex(slotColor[i]);
    slotObj["icon"] = slotIcon[i];
  }
  doc["currentSlot"] = currentSlot;

  JsonObject colorsObj = doc["colors"].to<JsonObject>();
  colorsObj["accent"] = colorToHex(accentColor);
  colorsObj["wave"] = colorToHex(waveColor);

  JsonArray remotesArr = doc["remotes"].to<JsonArray>();
  for (auto &r : remotes) {
    JsonObject rObj = remotesArr.add<JsonObject>();
    rObj["name"] = r.name;
    rObj["columns"] = r.columns;
    JsonArray btnArr = rObj["buttons"].to<JsonArray>();
    for (auto &b : r.buttons) {
      JsonObject bObj = btnArr.add<JsonObject>();
      bObj["name"] = b.name;
      bObj["code"] = b.codeName;
      bObj["icon"] = b.icon;
      bObj["color"] = colorToHex(b.color);
      bObj["spacer"] = b.spacer;
    }
  }
  doc["currentRemote"] = currentRemote;
  doc["screenSleepMs"] = screensaverTimeoutMs;

  // write to a temp file and rename it over the real one rather than
  // writing PROFILE_PATH directly - LittleFS's rename atomically replaces
  // an existing destination, so profile.json is always either fully the
  // old version or fully the new one, never a partial write. Without this,
  // a power loss mid-save (a real risk for a device left plugged in
  // long-term) could corrupt the one file everything is persisted in.
  const char* tmpPath = "/profile.json.tmp";
  File f = LittleFS.open(tmpPath, "w");
  if (!f) {
    Serial.println("saveProfile: failed to open temp file for writing");
    return false;
  }
  size_t written = serializeJson(doc, f);
  f.close();
  if (written == 0) {
    Serial.println("saveProfile: wrote 0 bytes, aborting");
    LittleFS.remove(tmpPath);
    return false;
  }
  if (!LittleFS.rename(tmpPath, PROFILE_PATH)) {
    Serial.println("saveProfile: rename failed");
    return false;
  }
  Serial.printf("saveProfile: wrote %u bytes\n", (unsigned)written);
  return true;
}

bool loadProfile() {
  if (!LittleFS.exists(PROFILE_PATH)) {
    Serial.println("loadProfile: no saved profile yet");
    return false;
  }

  File f = LittleFS.open(PROFILE_PATH, "r");
  if (!f) {
    Serial.println("loadProfile: failed to open " PROFILE_PATH " for reading");
    return false;
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    Serial.print("loadProfile: parse failed: ");
    Serial.println(err.c_str());
    return false;
  }

  codeLibrary.clear();

  JsonObject codesObj = doc["codes"].as<JsonObject>();
  for (JsonPair kv : codesObj) {
    IRCode code;
    JsonObject codeObj = kv.value().as<JsonObject>();
    const char* protocol = codeObj["protocol"].as<const char*>();
    if (protocol) {
      code.protocol = protocol;
      code.address = codeObj["address"] | 0;
      code.command = codeObj["command"] | 0;
      code.len = 0;
      code.hash = 0;
    } else {
      code.protocol = "";
      code.address = 0;
      code.command = 0;
      code.hash = codeObj["hash"].as<uint32_t>();
      JsonArray dataArr = codeObj["data"].as<JsonArray>();
      code.len = 0;
      for (JsonVariant v : dataArr) {
        if (code.len >= MAX_RAW_LEN) break;
        code.data[code.len++] = v.as<uint16_t>();
      }
    }
    codeLibrary[String(kv.key().c_str())] = code;
  }

  for (int i = 0; i < NUM_SLOTS; i++) {
    slotCodeName[i] = "";
    slotColor[i] = DEFAULT_SLOT_COLOR;
    slotIcon[i] = ICON_NONE;
  }
  JsonArray slotsArr = doc["slots"].as<JsonArray>();
  int i = 0;
  for (JsonVariant v : slotsArr) {
    if (i >= NUM_SLOTS) break;
    if (v.is<JsonObject>()) {
      // current format: {code, color, icon} per slot
      JsonObject slotObj = v.as<JsonObject>();
      const char* codeName = slotObj["code"].as<const char*>();
      slotCodeName[i] = codeName ? String(codeName) : "";
      const char* colorHex = slotObj["color"].as<const char*>();
      slotColor[i] = colorHex ? parseHexColor(colorHex, DEFAULT_SLOT_COLOR) : DEFAULT_SLOT_COLOR;
      slotIcon[i] = slotObj["icon"] | (int)ICON_NONE;
    } else {
      // pre-color/icon format: slots were just a plain array of code names
      const char* codeName = v.as<const char*>();
      slotCodeName[i] = codeName ? String(codeName) : "";
    }
    i++;
  }
  currentSlot = doc["currentSlot"] | 0;
  if (currentSlot < 0 || currentSlot >= NUM_SLOTS) currentSlot = 0;

  JsonObject colorsObj = doc["colors"].as<JsonObject>();
  const char* accentHex = colorsObj["accent"].as<const char*>();
  if (accentHex) accentColor = parseHexColor(accentHex, accentColor);
  const char* waveHex = colorsObj["wave"].as<const char*>();
  if (waveHex) waveColor = parseHexColor(waveHex, waveColor);

  // Parses one remote's button array - shared by the current multi-remote
  // format and the single-remote format it replaced.
  auto readButtons = [](JsonArray arr, std::vector<RemoteButton> &out) {
    out.clear();
    for (JsonVariant bv : arr) {
      JsonObject bObj = bv.as<JsonObject>();
      RemoteButton b;
      const char* bName = bObj["name"].as<const char*>();
      b.name = bName ? String(bName) : "";
      const char* bCode = bObj["code"].as<const char*>();
      b.codeName = bCode ? String(bCode) : "";
      b.icon = bObj["icon"] | (int)ICON_NONE;
      const char* bColorHex = bObj["color"].as<const char*>();
      b.color = bColorHex ? parseHexColor(bColorHex, DEFAULT_SLOT_COLOR) : DEFAULT_SLOT_COLOR;
      b.spacer = bObj["spacer"] | false;  // missing (pre-spacer saves) = false
      out.push_back(b);
    }
  };

  remotes.clear();
  if (doc["remotes"].is<JsonArray>()) {
    for (JsonVariant rv : doc["remotes"].as<JsonArray>()) {
      JsonObject rObj = rv.as<JsonObject>();
      Remote r;
      const char* rName = rObj["name"].as<const char*>();
      r.name = rName ? String(rName) : "Remote";
      r.columns = rObj["columns"] | 3;
      if (r.columns < 1) r.columns = 1;
      if (r.columns > 6) r.columns = 6;
      readButtons(rObj["buttons"].as<JsonArray>(), r.buttons);
      remotes.push_back(r);
    }
    currentRemote = doc["currentRemote"] | 0;
  } else {
    // pre-multi-remote profile: one unnamed remote at doc["remote"] with a
    // sibling doc["remoteColumns"]. Migrate rather than discard.
    Remote r;
    r.name = "Remote 1";
    r.columns = doc["remoteColumns"] | 3;
    if (r.columns < 1) r.columns = 1;
    if (r.columns > 6) r.columns = 6;
    readButtons(doc["remote"].as<JsonArray>(), r.buttons);
    remotes.push_back(r);
    currentRemote = 0;
  }
  ensureRemoteValid();
  screensaverTimeoutMs = doc["screenSleepMs"] | 0UL;   // absent (older profile) = never sleep

  Serial.printf("loadProfile: loaded %u codes, %u remotes (current '%s', %u buttons)\n",
                (unsigned)codeLibrary.size(), (unsigned)remotes.size(),
                remotes[currentRemote].name.c_str(),
                (unsigned)remoteButtons.size());
  return true;
}

// saveProfile() does a synchronous flash write - fine for /export (an
// explicit, infrequent action that's already serving a file), but too slow
// to do inline on every learn/delete/rename. Mutation handlers instead mark
// the profile dirty and return immediately; loop() performs the actual write
// a short debounce window later, off the request path, coalescing a burst of
// rapid changes (e.g. several quick deletes) into a single flash write.
bool profileDirty = false;
unsigned long profileDirtyAt = 0;
const unsigned long PROFILE_SAVE_DEBOUNCE_MS = 500;

void markProfileDirty() {
  profileDirty = true;
  profileDirtyAt = millis();
}

// ---------- MQTT / Home Assistant bridge (optional) ----------
// Fill in mqtt_server to enable - left blank, this feature just never
// connects and costs nothing. Publishes one HA "button" entity per virtual
// remote button via MQTT discovery (shows up in Home Assistant
// automatically, no YAML needed) and listens on a single command topic for
// a code name to fire - covers voice control via whatever assistant HA is
// already wired to.
const char* mqtt_server = "";  // e.g. "192.168.1.50" - blank disables MQTT entirely
const uint16_t mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_password = "";

WiFiClient mqttWifiClient;
PubSubClient mqttClient(mqttWifiClient);

String mqttBaseTopic() { return "irhomebase/" + String(hostname); }
String mqttStatusTopic() { return mqttBaseTopic() + "/status"; }
String mqttCmdTopic() { return mqttBaseTopic() + "/cmd"; }

// Two differently-named remote buttons can slugify to the same string (e.g.
// "TV Power" and "TV-Power" both become "tv_power"). Since the slug becomes
// part of the HA discovery topic and unique_id, a collision would make the
// second button's discovery publish silently overwrite the first's entity
// in Home Assistant. Appending a hash of the exact original name makes the
// result unique regardless of how the readable part collides.
String mqttSlug(const String &s) {
  String out;
  out.reserve(s.length());
  for (size_t i = 0; i < s.length(); i++) {
    char c = s.charAt(i);
    if (isalnum((unsigned char)c)) {
      out += (char)tolower(c);
    } else if (out.length() && out.charAt(out.length() - 1) != '_') {
      out += '_';
    }
  }
  while (out.length() && out.charAt(out.length() - 1) == '_') out.remove(out.length() - 1);

  uint32_t h = 0;
  for (size_t i = 0; i < s.length(); i++) h = h * 31 + s.charAt(i);
  out += "_" + String(h, HEX);
  return out;
}

// removes one remote button's HA entity - an empty retained payload on its
// discovery topic tells HA to forget it. Call before a button's code
// changes/is deleted so it doesn't leave a stale duplicate behind. Keyed by
// codeName (see mqttPublishDiscovery) since name is optional and often blank.
void mqttClearDiscovery(const String &codeName) {
  if (!mqttClient.connected()) return;
  String deviceId = "irhomebase_" + String(hostname);
  String topic = "homeassistant/button/" + deviceId + "_" + mqttSlug(codeName) + "/config";
  mqttClient.publish(topic.c_str(), "", true);
}

void mqttPublishDiscovery() {
  if (!mqttClient.connected()) return;
  String deviceId = "irhomebase_" + String(hostname);
  // every remote, not just the current one - Home Assistant should see all
  // buttons regardless of which remote happens to be on screen
  for (auto &r : remotes)
  for (auto &b : r.buttons) {
    // spacers and buttons with no code assigned yet have nothing to expose
    // to HA (name is optional now, so mqttSlug(b.name) alone could also
    // collide across several nameless buttons - key off the code name,
    // which is always unique in codeLibrary, instead)
    if (b.spacer || b.codeName.length() == 0) continue;
    String displayName = b.name.length() ? b.name : b.codeName;

    JsonDocument doc;
    doc["name"] = displayName;
    doc["unique_id"] = deviceId + "_" + mqttSlug(b.codeName);
    doc["command_topic"] = mqttCmdTopic();
    doc["payload_press"] = b.codeName;
    doc["availability_topic"] = mqttStatusTopic();
    JsonObject device = doc["device"].to<JsonObject>();
    device["identifiers"][0] = deviceId;
    device["name"] = "IR Controller (" + String(hostname) + ")";
    device["manufacturer"] = "IRHOMEBASE";

    String payload;
    serializeJson(doc, payload);
    String topic = "homeassistant/button/" + deviceId + "_" + mqttSlug(b.codeName) + "/config";
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.println("MQTT command: " + msg);
  if (codeLibrary.count(msg)) {
    queueCodeSend(msg);
  } else {
    Serial.println("MQTT command: no code named '" + msg + "'");
  }
}

unsigned long lastMqttAttempt = 0;
const unsigned long MQTT_RETRY_MS = 5000;

// Non-blocking: connect attempts are spaced out and mqttClient.loop() just
// services already-open sockets, so this is safe to call every loop().
void handleMqtt() {
  if (strlen(mqtt_server) == 0) return;
  if (WiFi.status() != WL_CONNECTED) return;

  if (!mqttClient.connected()) {
    if (millis() - lastMqttAttempt < MQTT_RETRY_MS) return;
    lastMqttAttempt = millis();
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(mqttCallback);
    // default PubSubClient packet buffer (256 bytes) is too small for an HA
    // discovery payload once the device block is included - publish() fails
    // silently rather than erroring, so this is easy to miss without it
    mqttClient.setBufferSize(512);
    String clientId = "irhomebase-" + String(hostname);
    String statusTopic = mqttStatusTopic();
    bool ok = mqttClient.connect(clientId.c_str(),
                                  strlen(mqtt_user) ? mqtt_user : nullptr,
                                  strlen(mqtt_user) ? mqtt_password : nullptr,
                                  statusTopic.c_str(), 0, true, "offline");
    if (ok) {
      Serial.println("MQTT connected");
      mqttClient.publish(statusTopic.c_str(), "online", true);
      mqttClient.subscribe(mqttCmdTopic().c_str());
      mqttPublishDiscovery();
    } else {
      Serial.printf("MQTT connect failed, rc=%d\n", mqttClient.state());
    }
    return;
  }

  mqttClient.loop();
}

// ---------- web handlers ----------
String htmlEscape(const String &s) {
  String out;
  out.reserve(s.length());
  for (size_t i = 0; i < s.length(); i++) {
    char c = s.charAt(i);
    switch (c) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      case '\'': out += "&#39;"; break;
      default: out += c;
    }
  }
  return out;
}

String jsonEscape(const String &s) {
  String out;
  out.reserve(s.length());
  for (size_t i = 0; i < s.length(); i++) {
    char c = s.charAt(i);
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      default: out += c;
    }
  }
  return out;
}

// mutating endpoints finish either with a redirect (plain form/link navigation)
// or, for the AJAX UI, a bare 204 - the page polls /api/state for the result
// instead of needing a fetched response body
void finishRequest() {
  if (server.hasArg("ajax")) {
    server.send(204);
  } else {
    server.sendHeader("Location", "/");
    server.send(303);
  }
}

// called instead of learning/saving when a name would silently clobber an
// existing entry - caller must resubmit with confirm=1 to proceed
void sendOverwriteConfirm(const String &kind, const String &redirectUrl, const String &name) {
  if (server.hasArg("ajax")) {
    String json = "{\"conflict\":true,\"kind\":\"" + jsonEscape(kind) + "\",\"name\":\"" + jsonEscape(name) + "\"}";
    server.send(409, "application/json", json);
    return;
  }
  // redirectUrl embeds the raw submitted name (e.g. "/learn?name=" + name) -
  // escape the whole thing before dropping it into an href, otherwise a
  // crafted name containing quotes/angle-brackets breaks out of the
  // attribute and injects into the page
  String html = "<html><body style='font-family:sans-serif'>";
  html += "<p>A " + kind + " named '" + htmlEscape(name) + "' already exists.</p>";
  html += "<p><a href='" + htmlEscape(redirectUrl) + "&confirm=1'>Overwrite it</a> | <a href='/'>Cancel</a></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleApiState() {
  // this is rebuilt on every poll (default: once a second) - reserving up
  // front avoids the repeated grow/copy/free cycle that += would otherwise
  // do on nearly every line below, which is a steady source of heap
  // fragmentation on a device that stays powered on for a long time
  String json;
  json.reserve(400 + 40 * (codeLibrary.size() + remoteButtons.size()));
  json += "{";
  json += "\"status\":\"" + jsonEscape(lastStatus) + "\",";
  json += "\"learning\":" + String(learning ? "true" : "false") + ",";
  json += "\"pendingName\":\"" + jsonEscape(pendingLearnName) + "\",";

  json += "\"codes\":[";
  bool first = true;
  for (auto &kv : codeLibrary) {
    if (!first) json += ",";
    first = false;
    json += "\"" + jsonEscape(kv.first) + "\"";
  }
  json += "],";

  json += "\"currentSlot\":" + String(currentSlot) + ",";
  json += "\"slots\":[";
  for (int i = 0; i < NUM_SLOTS; i++) {
    if (i > 0) json += ",";
    json += "{\"code\":\"" + jsonEscape(slotCodeName[i]) + "\",";
    json += "\"color\":\"" + colorToHex(slotColor[i]) + "\",";
    json += "\"icon\":" + String(slotIcon[i]) + "}";
  }
  json += "],";

  json += "\"colors\":{";
  json += "\"accent\":\"" + colorToHex(accentColor) + "\",";
  json += "\"wave\":\"" + colorToHex(waveColor) + "\"";
  json += "},";

  json += "\"screenSleepMin\":" + String((int)(screensaverTimeoutMs / 60000UL)) + ",";
  json += "\"teachAll\":" + String(teachAllActive ? "true" : "false") + ",";
  json += "\"learnButton\":" + String(pendingLearnButton) + ",";

  json += "\"currentRemote\":" + String(currentRemote) + ",";
  json += "\"remotes\":[";
  first = true;
  for (auto &r : remotes) {
    if (!first) json += ",";
    first = false;
    json += "{\"name\":\"" + jsonEscape(r.name) + "\",";
    json += "\"buttons\":" + String((int)r.buttons.size()) + "}";
  }
  json += "],";

  // the selected remote's layout, expanded - the UI renders this one
  json += "\"remoteColumns\":" + String(remoteColumns) + ",";
  json += "\"remote\":[";
  first = true;
  for (auto &b : remoteButtons) {
    if (!first) json += ",";
    first = false;
    json += "{\"name\":\"" + jsonEscape(b.name) + "\",";
    json += "\"code\":\"" + jsonEscape(b.codeName) + "\",";
    json += "\"icon\":" + String(b.icon) + ",";
    json += "\"color\":\"" + colorToHex(b.color) + "\",";
    json += "\"spacer\":" + String(b.spacer ? "true" : "false") + "}";
  }
  json += "]}";

  server.send(200, "application/json", json);
}

// Static page: everything dynamic is fetched via /api/state, so this never
// changes at runtime. Kept in flash (PROGMEM) and streamed directly by
// handleRoot() instead of being copied into a heap String on every request -
// with polling hitting this device constantly, that copy was a steady source
// of heap churn/fragmentation that got worse the longer the device stayed up.
static const char PAGE_HTML[] PROGMEM = R"HTML(<!doctype html>
<html>
<head>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<title>IR Controller</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body { font-family: -apple-system, system-ui, sans-serif; background:#111; color:#eee; margin:0; padding:12px 12px 40px; max-width:480px; margin-left:auto; margin-right:auto; }
  h1 { font-size:1.3rem; margin:4px 0 12px; }
  h2 { font-size:1rem; margin:20px 0 8px; color:#8cf; }
  h3 { font-size:0.9rem; margin:14px 0 6px; color:#8cf; }
  #statusBar { position:sticky; top:8px; background:#1a7a3a; color:#fff; padding:12px; border-radius:10px; margin-bottom:14px; font-weight:600; text-align:center; z-index:5; }
  #statusBar.learning { background:#a06a00; }
  .item { display:flex; align-items:center; gap:6px; background:#1c1c1c; border-radius:10px; padding:10px; margin-bottom:8px; flex-wrap:wrap; }
  .item .name, .item .label { flex:1 1 auto; min-width:70px; font-size:1rem; word-break:break-all; }
  .empty { color:#888; font-size:0.9rem; }
  button { font-size:0.95rem; padding:10px 14px; border-radius:8px; border:none; background:#2a7a4a; color:#fff; }
  button.secondary { background:#3a4a6a; }
  button.danger { background:#a03030; }
  body.busy button { opacity:0.5; pointer-events:none; }
  input[type=text] { font-size:1rem; padding:10px; border-radius:8px; border:1px solid #555; background:#222; color:#eee; width:100%; }
  select { font-size:0.95rem; padding:8px; border-radius:8px; border:1px solid #555; background:#222; color:#eee; flex:1 1 auto; min-width:0; }
  input[type=color] { width:52px; height:38px; padding:2px; border-radius:8px; border:1px solid #555; background:#222; flex:0 0 auto; }
  .item.selected-slot { outline:2px solid #4cf4f4; }
  form.inline { display:flex; flex-direction:column; gap:8px; background:#1c1c1c; border-radius:10px; padding:12px; }
  #tabs { display:flex; gap:6px; overflow-x:auto; margin-bottom:14px; padding-bottom:2px; }
  .tabBtn { flex:0 0 auto; background:#1c1c1c; color:#aaa; padding:8px 12px; font-size:0.9rem; }
  .tabBtn.active { background:#2a7a4a; color:#fff; }
  .tabPanel { display:none; }
  .tabPanel.active { display:block; }
  #remoteGrid { display:grid; gap:8px; margin-bottom:16px; }
  .remoteTile { aspect-ratio:1.3; font-size:0.85rem; font-weight:600; border-radius:12px; display:flex; align-items:center; justify-content:center; text-align:center; padding:4px; overflow:hidden; border:none; }
  .remoteTile svg { width:46%; height:46%; }
  .remoteTile.spacer { background:transparent !important; border:1px dashed #3a3a3a; pointer-events:none; }
  .iconPreview { width:34px; height:34px; border-radius:8px; display:flex; align-items:center; justify-content:center; flex:0 0 auto; font-weight:700; font-size:0.95rem; }
  .iconPreview svg { width:60%; height:60%; }
  .editRow { display:flex; align-items:center; gap:6px; flex-wrap:wrap; }
  .editRow .label { flex:1 1 90px; min-width:70px; }
  .editRow .moveDelBtns { display:flex; gap:6px; margin-left:auto; }
  .dragHandle { touch-action:none; cursor:grab; color:#888; flex:0 0 auto; padding:4px; display:flex; align-items:center; }
  .dragHandle:active { cursor:grabbing; color:#eee; }
  #remoteList .item.dragging { opacity:0.4; }
  #remoteList .item.dragOver { outline:2px dashed #4cf4f4; }
</style>
</head>
<body>
<h1>IR Controller</h1>
<div id='statusBar'>Loading...</div>

<div id='tabs'>
  <button type='button' class='tabBtn active' data-tab='remote'>Remote</button>
  <button type='button' class='tabBtn' data-tab='physical'>Physical</button>
  <button type='button' class='tabBtn' data-tab='lookup'>Lookup</button>
  <button type='button' class='tabBtn' data-tab='codes'>Codes</button>
  <button type='button' class='tabBtn' data-tab='colors'>Colors</button>
  <button type='button' class='tabBtn' data-tab='backup'>Backup</button>
</div>

<div class='tabPanel active' id='tab-remote'>
  <div class='item'>
    <select id='remoteSel' style='flex:1 1 120px'></select>
    <button type='button' class='secondary' id='remoteNewBtn'>New</button>
    <button type='button' class='secondary' id='remoteRenameBtn'>Rename</button>
    <button type='button' class='danger' id='remoteDelBtn'>Delete</button>
  </div>

  <div id='teachBanner'></div>

  <div id='remoteGrid'></div>

  <div class='item'>
    <span class='name'>Columns</span>
    <select id='remoteColumnsSel'>
      <option value='1'>1</option><option value='2'>2</option><option value='3'>3</option>
      <option value='4'>4</option><option value='5'>5</option><option value='6'>6</option>
    </select>
    <button type='button' id='teachAllBtn'>Teach all</button>
  </div>

  <h3>Edit buttons</h3>
  <p class='empty'>"Learn" captures a code straight into that button - no naming needed. "Teach all" walks every button that still needs one.</p>
  <div id='remoteList'></div>

  <h3>Add buttons</h3>
  <form class='inline' id='remoteAddForm'>
    <div class='editRow'>
      <div class='iconPreview' id='remoteAddPreview'></div>
      <select id='remoteAddIcon'></select>
      <input type='color' id='remoteAddColor'>
    </div>
    <select id='remoteAddCode'></select>
    <div class='editRow'>
      <button type='submit'>Add button</button>
      <button type='button' class='secondary' id='remoteAddSpacerBtn'>Add space</button>
    </div>
    <div class='editRow'>
      <span class='name'>Add several blank</span>
      <select id='remoteAddManyCount'>
        <option>2</option><option>4</option><option selected>6</option>
        <option>8</option><option>10</option><option>12</option><option>16</option><option>20</option>
      </select>
      <button type='button' class='secondary' id='remoteAddManyBtn'>Add</button>
    </div>
  </form>
</div>

<div class='tabPanel' id='tab-physical'>
  <h2>Physical buttons</h2>
  <p class='empty'>Button 1 fires the selected slot. Button 2 cycles slots (short press) or learns into the current one (hold).</p>
  <div id='slots'></div>
</div>

<div class='tabPanel' id='tab-lookup'>
  <h2>Look up a remote</h2>
  <p class='empty'>Searches a public IR code database (probonopd/irdb) right from your phone - the device itself only handles the final "try/save" step. Sends NEC, Sony, RC5, RC6, JVC, Panasonic, Samsung, and LG family codes; anything else says so instead of guessing.</p>
  <div class='item'>
    <input type='text' id='lookupManufacturer' placeholder='Manufacturer, e.g. Samsung'>
  </div>
  <div class='item'>
    <input type='text' id='lookupDeviceType' placeholder='Device type' value='TV'>
    <button type='button' onclick='lookupSearch()'>Search</button>
  </div>
  <div id='lookupResults'></div>
  <div id='lookupFunctions'></div>

  <h3>Don't know the manufacturer?</h3>
  <p class='empty'>Blind search: tries devices in the database one at a time so you can watch for a reaction, the same way a universal remote's code-search mode works. Leave device type blank to search everything.</p>
  <div class='item'>
    <input type='text' id='blindDeviceType' placeholder='Device type (optional)'>
    <button type='button' onclick='blindStart()'>Start blind search</button>
    <button type='button' class='secondary' onclick='blindStop()'>Stop</button>
  </div>
  <div id='blindStatus'></div>
  <div id='blindCandidate' class='item'></div>
</div>

<div class='tabPanel' id='tab-codes'>
  <h2>Saved codes</h2>
  <div id='codes'></div>

  <h3>Learn new code</h3>
  <form class='inline' id='learnForm'>
    <input type='text' id='learnName' placeholder='name e.g. tv_power' autocomplete='off'>
    <button type='submit'>Learn</button>
  </form>
</div>

<div class='tabPanel' id='tab-colors'>
  <h2>Display colors</h2>
  <div class='item'>
    <span class='name'>Selected slot / button</span>
    <input type='color' id='colorAccent'>
  </div>
  <div class='item'>
    <span class='name'>Transmit wave</span>
    <input type='color' id='colorWave'>
  </div>
  <p class='empty'>Each physical slot and remote button also has its own color - set those on the Physical/Remote tabs.</p>

  <h3>Screen</h3>
  <div class='item'>
    <span class='name'>Sleep after</span>
    <select id='screenSleepSel'>
      <option value='0'>Never</option>
      <option value='1'>1 min</option>
      <option value='2'>2 min</option>
      <option value='5'>5 min</option>
      <option value='10'>10 min</option>
      <option value='30'>30 min</option>
    </select>
  </div>
  <p class='empty'>Never is the default. If sleep is on, the first tap only wakes the screen - it won't fire whatever button is underneath.</p>
</div>

<div class='tabPanel' id='tab-backup'>
  <h2>Backup</h2>
  <div class='item'>
    <button type='button' onclick="window.location='/export'">Download backup</button>
    <button type='button' class='secondary' onclick="document.getElementById('importFile').click()">Restore from file</button>
    <input type='file' id='importFile' accept='application/json' style='display:none'>
  </div>
</div>

<script>
// Icon ids must match the IconId enum in main.cpp exactly (order = value).
// Drawn as inline SVG (fill/stroke: currentColor) rather than Unicode glyphs -
// glyph font coverage is inconsistent across phones/browsers (the power
// symbol U+23FB in particular doesn't render on a lot of devices), so this
// is what actually shows up reliably instead of a blank box. Digits are
// plain text since digit glyphs render fine everywhere.
const ICONS = [
  {id:0, name:'(none)', svg:null},
  {id:1, name:'Up', svg:'<svg viewBox="0 0 24 24"><polygon points="12,4 4,18 20,18" fill="currentColor"/></svg>'},
  {id:2, name:'Down', svg:'<svg viewBox="0 0 24 24"><polygon points="12,20 4,6 20,6" fill="currentColor"/></svg>'},
  {id:3, name:'Left', svg:'<svg viewBox="0 0 24 24"><polygon points="4,12 18,4 18,20" fill="currentColor"/></svg>'},
  {id:4, name:'Right', svg:'<svg viewBox="0 0 24 24"><polygon points="20,12 6,4 6,20" fill="currentColor"/></svg>'},
  {id:5, name:'Plus', svg:'<svg viewBox="0 0 24 24"><rect x="4" y="10" width="16" height="4" fill="currentColor"/><rect x="10" y="4" width="4" height="16" fill="currentColor"/></svg>'},
  {id:6, name:'Minus', svg:'<svg viewBox="0 0 24 24"><rect x="4" y="10" width="16" height="4" fill="currentColor"/></svg>'},
  {id:7, name:'Check', svg:'<svg viewBox="0 0 24 24"><polyline points="4,13 9,18 20,6" fill="none" stroke="currentColor" stroke-width="3" stroke-linecap="round" stroke-linejoin="round"/></svg>'},
  {id:8, name:'X', svg:'<svg viewBox="0 0 24 24"><line x1="5" y1="5" x2="19" y2="19" stroke="currentColor" stroke-width="3" stroke-linecap="round"/><line x1="19" y1="5" x2="5" y2="19" stroke="currentColor" stroke-width="3" stroke-linecap="round"/></svg>'},
  {id:9, name:'Power', svg:'<svg viewBox="0 0 24 24"><path d="M18.36 6.64a9 9 0 1 1-12.73 0" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round"/><line x1="12" y1="2" x2="12" y2="12" stroke="currentColor" stroke-width="2.2" stroke-linecap="round"/></svg>'},
  {id:10, name:'WiFi', svg:'<svg viewBox="0 0 24 24"><rect x="2" y="15" width="3" height="6" fill="currentColor"/><rect x="8" y="11" width="3" height="10" fill="currentColor"/><rect x="14" y="7" width="3" height="14" fill="currentColor"/><rect x="20" y="3" width="3" height="18" fill="currentColor"/></svg>'},
  {id:11, name:'0', svg:null}, {id:12, name:'1', svg:null}, {id:13, name:'2', svg:null},
  {id:14, name:'3', svg:null}, {id:15, name:'4', svg:null}, {id:16, name:'5', svg:null},
  {id:17, name:'6', svg:null}, {id:18, name:'7', svg:null}, {id:19, name:'8', svg:null},
  {id:20, name:'9', svg:null}
];
function findIcon(id) { return ICONS.find(function (i) { return i.id === id; }); }

// Renders icon `id` into `el` (any block element) tinted `colorHex`. Works
// for both the small edit-list preview swatches and the big remote tiles -
// caller controls size via CSS, this just fills in the right markup/color.
function paintIcon(el, id, colorHex) {
  el.innerHTML = '';
  el.style.color = colorHex;
  const icon = findIcon(id);
  if (icon && icon.svg) {
    el.innerHTML = icon.svg;
  } else if (icon && id >= 11 && id <= 20) {
    el.textContent = icon.name;
  }
}

// Picks black or white so text/icons stay legible against an arbitrary
// user-chosen background color - mirrors contrastTextColor() in main.cpp so
// the web preview matches what actually shows up on the device's screen.
function contrastColor(hex) {
  const h = hex.replace('#', '');
  const r = parseInt(h.substring(0, 2), 16), g = parseInt(h.substring(2, 4), 16), b = parseInt(h.substring(4, 6), 16);
  const luminance = (r * 299 + g * 587 + b * 114) / 1000;
  return luminance > 140 ? '#000' : '#fff';
}

document.querySelectorAll('.tabBtn').forEach(function (btn) {
  btn.addEventListener('click', function () {
    document.querySelectorAll('.tabBtn').forEach(function (b) { b.classList.remove('active'); });
    document.querySelectorAll('.tabPanel').forEach(function (p) { p.classList.remove('active'); });
    btn.classList.add('active');
    document.getElementById('tab-' + btn.dataset.tab).classList.add('active');
  });
});

// Two separate locks, not one shared one:
//  - actionBusy guards user-initiated taps (blast/delete/rename/etc) so a
//    double-tap can't fire the same action twice, and drives the visible
//    "busy" dimming so you get feedback instead of silence.
//  - pollBusy only keeps the background status poll from overlapping itself
//    if one poll takes longer than the interval.
// These used to be the same flag, which meant a tap landing while the
// routine 1s poll was mid-flight got silently dropped (no request even
// sent), and the poll toggled the visible "busy" state every second whether
// or not you were doing anything - buttons dimming on a 1Hz cycle for no
// reason, and single taps going nowhere unless you got lucky with timing.
let actionBusy = false;
let pollBusy = false;
function setActionBusy(value) {
  actionBusy = value;
  document.body.classList.toggle('busy', value);
}

function api(path, params) {
  params.ajax = '1';
  return fetch(path + '?' + new URLSearchParams(params).toString());
}

async function mutate(path, params) {
  if (actionBusy) return;
  setActionBusy(true);
  try {
    const res = await api(path, params);
    if (res.status === 409) {
      const data = await res.json();
      if (confirm("A " + data.kind + " named '" + data.name + "' already exists. Overwrite?")) {
        params.confirm = '1';
        await api(path, params);
      }
    }
  } finally {
    setActionBusy(false);
  }
  refresh();
}

function makeButton(label, cls, onClick) {
  const b = document.createElement('button');
  b.type = 'button';
  b.textContent = label;
  if (cls) b.className = cls;
  b.addEventListener('click', onClick);
  return b;
}

function fillIconSelect(select, selected) {
  select.innerHTML = '';
  ICONS.forEach(function (icon) {
    const opt = document.createElement('option');
    opt.value = String(icon.id);
    opt.textContent = icon.name;
    if (icon.id === selected) opt.selected = true;
    select.appendChild(opt);
  });
}

function fillCodeSelect(select, codes, selected) {
  select.innerHTML = '';
  const noneOpt = document.createElement('option');
  noneOpt.value = '';
  noneOpt.textContent = '(no code)';
  if (!selected) noneOpt.selected = true;
  select.appendChild(noneOpt);
  codes.forEach(function (codeName) {
    const opt = document.createElement('option');
    opt.value = codeName;
    opt.textContent = codeName;
    if (codeName === selected) opt.selected = true;
    select.appendChild(opt);
  });
}

function blastCode(name) { mutate('/blastcode', {name: name}); }
function deleteCode(name) { if (confirm("Delete '" + name + "'?")) mutate('/deletecode', {name: name}); }
function renameCode(oldname) {
  const n = prompt("New name for '" + oldname + "':", oldname);
  if (n) mutate('/renamecode', {oldname: oldname, newname: n});
}

document.getElementById('learnForm').addEventListener('submit', function (e) {
  e.preventDefault();
  const input = document.getElementById('learnName');
  const name = input.value.trim();
  if (name) mutate('/learn', {name: name});
  input.value = '';
});

document.getElementById('importFile').addEventListener('change', async function (e) {
  const file = e.target.files[0];
  e.target.value = '';
  if (!file) return;
  if (!confirm('This replaces every saved code, slot, and remote button with the contents of ' + file.name + '. Continue?')) return;
  const text = await file.text();
  const res = await fetch('/import', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: text });
  if (res.ok) {
    alert('Profile imported.');
    refresh();
  } else {
    alert('Import failed: ' + await res.text());
  }
});

function setColor(key, hex) { mutate('/setcolors', {[key]: hex}); }
document.getElementById('colorAccent').addEventListener('change', function () { setColor('accent', this.value); });
document.getElementById('colorWave').addEventListener('change', function () { setColor('wave', this.value); });
document.getElementById('screenSleepSel').addEventListener('change', function () {
  mutate('/screensleep', {minutes: this.value});
});

document.getElementById('remoteColumnsSel').addEventListener('change', function () {
  mutate('/remote/columns', {n: this.value});
});

document.getElementById('remoteAddForm').addEventListener('submit', function (e) {
  e.preventDefault();
  mutate('/remote/add', {
    code: document.getElementById('remoteAddCode').value,
    icon: document.getElementById('remoteAddIcon').value,
    color: document.getElementById('remoteAddColor').value
  });
});
document.getElementById('remoteAddSpacerBtn').addEventListener('click', function () {
  mutate('/remote/add', {spacer: '1'});
});
document.getElementById('remoteAddManyBtn').addEventListener('click', function () {
  mutate('/remote/addmany', {
    count: document.getElementById('remoteAddManyCount').value,
    icon: document.getElementById('remoteAddIcon').value,
    color: document.getElementById('remoteAddColor').value
  });
});

// ---- multiple remotes ----
document.getElementById('remoteSel').addEventListener('change', function () {
  mutate('/remotes/select', {index: this.value});
});
document.getElementById('remoteNewBtn').addEventListener('click', function () {
  const n = prompt('Name for the new remote:', 'Remote');
  if (n && n.trim()) mutate('/remotes/add', {name: n.trim()});
});
document.getElementById('remoteRenameBtn').addEventListener('click', function () {
  const sel = document.getElementById('remoteSel');
  const cur = sel.options[sel.selectedIndex] ? sel.options[sel.selectedIndex].dataset.rawname : '';
  const n = prompt('Rename this remote:', cur || '');
  if (n && n.trim()) mutate('/remotes/rename', {index: sel.value, name: n.trim()});
});
document.getElementById('remoteDelBtn').addEventListener('click', function () {
  const sel = document.getElementById('remoteSel');
  const cur = sel.options[sel.selectedIndex] ? sel.options[sel.selectedIndex].dataset.rawname : 'this remote';
  if (confirm("Delete '" + cur + "'? Its buttons go too, but the learned codes stay in the library."))
    mutate('/remotes/delete', {index: sel.value});
});

// ---- teach ----
document.getElementById('teachAllBtn').addEventListener('click', function () {
  if (this.dataset.stop === '1') { mutate('/remote/teachstop', {}); return; }
  mutate('/remote/teachall', {});
});
function learnIntoButton(i) { mutate('/remote/learn', {index: String(i)}); }
function updateAddPreview() {
  const preview = document.getElementById('remoteAddPreview');
  const color = document.getElementById('remoteAddColor').value;
  preview.style.background = color;
  paintIcon(preview, parseInt(document.getElementById('remoteAddIcon').value, 10), contrastColor(color));
}
document.getElementById('remoteAddIcon').addEventListener('change', updateAddPreview);
document.getElementById('remoteAddColor').addEventListener('change', updateAddPreview);
document.getElementById('remoteAddColor').addEventListener('input', updateAddPreview);

// Drag-to-reorder for the Remote edit list, built on Pointer Events rather
// than native HTML5 drag-and-drop - the native API essentially doesn't work
// on mobile touch browsers (which is where this app is actually used)
// without extra polyfilling, while Pointer Events (with touch-action:none
// on just the handle, not the whole row) unify mouse/touch/pen and behave
// consistently. Rows still get up/down buttons too as a fallback in case a
// drag gesture doesn't register on some browser.
let dragFromIndex = null;
let dragHoverIndex = null;
function clearDragHighlights() {
  document.querySelectorAll('#remoteList .dragOver').forEach(function (el) { el.classList.remove('dragOver'); });
}
function makeDragHandle(row, index) {
  const handle = document.createElement('div');
  handle.className = 'dragHandle';
  handle.innerHTML = '<svg viewBox="0 0 24 24" width="20" height="20"><circle cx="8" cy="6" r="1.6" fill="currentColor"/><circle cx="16" cy="6" r="1.6" fill="currentColor"/><circle cx="8" cy="12" r="1.6" fill="currentColor"/><circle cx="16" cy="12" r="1.6" fill="currentColor"/><circle cx="8" cy="18" r="1.6" fill="currentColor"/><circle cx="16" cy="18" r="1.6" fill="currentColor"/></svg>';
  handle.addEventListener('pointerdown', function (e) {
    dragFromIndex = index;
    dragHoverIndex = index;
    row.classList.add('dragging');
    handle.setPointerCapture(e.pointerId);
    e.preventDefault();
  });
  handle.addEventListener('pointermove', function (e) {
    if (dragFromIndex === null) return;
    const list = document.getElementById('remoteList');
    let target = null;
    for (const child of list.children) {
      const rect = child.getBoundingClientRect();
      if (e.clientY >= rect.top && e.clientY <= rect.bottom) { target = child; break; }
    }
    clearDragHighlights();
    if (target) {
      dragHoverIndex = parseInt(target.dataset.index, 10);
      target.classList.add('dragOver');
    }
  });
  function endDrag(e) {
    if (dragFromIndex === null) return;
    if (handle.hasPointerCapture(e.pointerId)) handle.releasePointerCapture(e.pointerId);
    row.classList.remove('dragging');
    clearDragHighlights();
    if (dragHoverIndex !== null && dragHoverIndex !== dragFromIndex) {
      mutate('/remote/reorder', {from: String(dragFromIndex), to: String(dragHoverIndex)});
    }
    dragFromIndex = null;
    dragHoverIndex = null;
  }
  handle.addEventListener('pointerup', endDrag);
  handle.addEventListener('pointercancel', endDrag);
  return handle;
}

function renderRemote(state) {
  const colSel = document.getElementById('remoteColumnsSel');
  if (document.activeElement !== colSel) colSel.value = String(state.remoteColumns);

  // remote picker
  const rSel = document.getElementById('remoteSel');
  if (document.activeElement !== rSel) {
    rSel.innerHTML = '';
    (state.remotes || []).forEach(function (r, i) {
      const opt = document.createElement('option');
      opt.value = String(i);
      opt.textContent = r.name + '  (' + r.buttons + ')';
      opt.dataset.rawname = r.name;   // textContent has the count appended; rename/delete need the bare name
      if (i === state.currentRemote) opt.selected = true;
      rSel.appendChild(opt);
    });
  }
  document.getElementById('remoteDelBtn').disabled = (state.remotes || []).length < 2;

  // teach-all banner + button state
  const teachBtn = document.getElementById('teachAllBtn');
  const banner = document.getElementById('teachBanner');
  const untaught = state.remote.filter(function (b) { return !b.spacer && !b.code; }).length;
  if (state.teachAll) {
    teachBtn.textContent = 'Stop';
    teachBtn.dataset.stop = '1';
    teachBtn.className = 'danger';
    banner.innerHTML = "<div class='item' style='background:#5a4a00'>Teaching button " +
      (state.learnButton + 1) + " - point the original remote at the device and press it. " +
      untaught + " left.</div>";
  } else {
    teachBtn.textContent = 'Teach all';
    teachBtn.dataset.stop = '';
    teachBtn.className = '';
    teachBtn.disabled = untaught === 0;
    banner.innerHTML = untaught === 0
      ? ''
      : "<div class='item'><span class='name'>" + untaught + " button(s) still need a code</span></div>";
  }

  const grid = document.getElementById('remoteGrid');
  grid.innerHTML = '';
  grid.style.gridTemplateColumns = 'repeat(' + state.remoteColumns + ', 1fr)';
  if (state.remote.length === 0) {
    grid.innerHTML = "<p class='empty'>No buttons yet - add some below, then hit Teach all.</p>";
  }
  state.remote.forEach(function (b) {
    const tile = document.createElement('button');
    tile.type = 'button';
    tile.className = 'remoteTile' + (b.spacer ? ' spacer' : '');
    if (b.spacer) {
      grid.appendChild(tile);
      return;
    }
    tile.style.background = b.color;
    if (b.icon) {
      paintIcon(tile, b.icon, contrastColor(b.color));
    } else if (b.code) {
      tile.textContent = b.code;
      tile.style.color = contrastColor(b.color);
    }
    tile.addEventListener('click', function () { if (b.code) blastCode(b.code); });
    grid.appendChild(tile);
  });

  const list = document.getElementById('remoteList');
  list.innerHTML = '';
  state.remote.forEach(function (b, i) {
    const row = document.createElement('div');
    row.className = 'item editRow';
    row.dataset.index = String(i);
    row.appendChild(makeDragHandle(row, i));

    if (b.spacer) {
      const label = document.createElement('span');
      label.className = 'label empty';
      label.textContent = 'Empty space';
      row.appendChild(label);
      const btns = document.createElement('div');
      btns.className = 'moveDelBtns';
      btns.appendChild(makeButton('↑', 'secondary', function () { mutate('/remote/move', {index: String(i), dir: 'up'}); }));
      btns.appendChild(makeButton('↓', 'secondary', function () { mutate('/remote/move', {index: String(i), dir: 'down'}); }));
      btns.appendChild(makeButton('Delete', 'danger', function () { mutate('/remote/delete', {index: String(i)}); }));
      row.appendChild(btns);
      list.appendChild(row);
      return;
    }

    const preview = document.createElement('div');
    preview.className = 'iconPreview';
    paintIcon(preview, b.icon, contrastColor(b.color));
    preview.style.background = b.color;
    row.appendChild(preview);

    const iconSel = document.createElement('select');
    fillIconSelect(iconSel, b.icon);
    iconSel.addEventListener('change', function () {
      paintIcon(preview, parseInt(iconSel.value, 10), contrastColor(colorInput.value));
      mutate('/remote/update', {index: String(i), icon: iconSel.value});
    });
    row.appendChild(iconSel);

    const colorInput = document.createElement('input');
    colorInput.type = 'color';
    colorInput.value = b.color;
    colorInput.addEventListener('input', function () {
      preview.style.background = colorInput.value;
      paintIcon(preview, parseInt(iconSel.value, 10), contrastColor(colorInput.value));
    });
    colorInput.addEventListener('change', function () { mutate('/remote/update', {index: String(i), color: colorInput.value}); });
    row.appendChild(colorInput);

    const codeSel = document.createElement('select');
    fillCodeSelect(codeSel, state.codes, b.code);
    codeSel.addEventListener('change', function () { mutate('/remote/update', {index: String(i), code: codeSel.value}); });
    row.appendChild(codeSel);

    // teaching into the button directly is the main path - picking an
    // existing code from the dropdown above is the exception, not the rule
    const learnBtn = makeButton(b.code ? 'Re-learn' : 'Learn',
                                b.code ? 'secondary' : '',
                                function () { learnIntoButton(i); });
    if (state.teachAll && state.learnButton === i) {
      learnBtn.textContent = 'waiting...';
      learnBtn.className = 'danger';
    }
    row.appendChild(learnBtn);

    const btns = document.createElement('div');
    btns.className = 'moveDelBtns';
    btns.appendChild(makeButton('↑', 'secondary', function () { mutate('/remote/move', {index: String(i), dir: 'up'}); }));
    btns.appendChild(makeButton('↓', 'secondary', function () { mutate('/remote/move', {index: String(i), dir: 'down'}); }));
    btns.appendChild(makeButton('Delete', 'danger', function () {
      if (confirm('Delete this button?')) mutate('/remote/delete', {index: String(i)});
    }));
    row.appendChild(btns);

    list.appendChild(row);
  });

  const addCodeSel = document.getElementById('remoteAddCode');
  if (document.activeElement !== addCodeSel) fillCodeSelect(addCodeSel, state.codes, addCodeSel.value);
}

let iconSelectsInited = false;
function renderSlots(state) {
  if (!iconSelectsInited) {
    const addIconSel = document.getElementById('remoteAddIcon');
    fillIconSelect(addIconSel, 0);
    updateAddPreview();
    iconSelectsInited = true;
  }

  const slotsDiv = document.getElementById('slots');
  slotsDiv.innerHTML = '';
  state.slots.forEach(function (slot, i) {
    const row = document.createElement('div');
    row.className = 'item editRow' + (i === state.currentSlot ? ' selected-slot' : '');
    const label = document.createElement('span');
    label.className = 'label';
    label.textContent = 'Button ' + (i + 1);
    row.appendChild(label);

    const preview = document.createElement('div');
    preview.className = 'iconPreview';
    paintIcon(preview, slot.icon, contrastColor(slot.color));
    preview.style.background = slot.color;
    row.appendChild(preview);

    const iconSel = document.createElement('select');
    fillIconSelect(iconSel, slot.icon);
    iconSel.addEventListener('change', function () {
      paintIcon(preview, parseInt(iconSel.value, 10), contrastColor(colorInput.value));
      mutate('/assignslot', {slot: String(i), icon: iconSel.value});
    });
    row.appendChild(iconSel);

    const colorInput = document.createElement('input');
    colorInput.type = 'color';
    colorInput.value = slot.color;
    colorInput.addEventListener('input', function () {
      preview.style.background = colorInput.value;
      paintIcon(preview, parseInt(iconSel.value, 10), contrastColor(colorInput.value));
    });
    colorInput.addEventListener('change', function () { mutate('/assignslot', {slot: String(i), color: colorInput.value}); });
    row.appendChild(colorInput);

    const select = document.createElement('select');
    fillCodeSelect(select, state.codes, slot.code);
    select.addEventListener('change', function () { mutate('/assignslot', {slot: String(i), name: select.value}); });
    row.appendChild(select);

    slotsDiv.appendChild(row);
  });
}

// ---- IRDB lookup: search/parse happens in the browser (has far more RAM
// than the ESP32 for the ~100KB index) against the public jsdelivr mirror.
// The device only ever sees a final protocol/address/command to try/save. ----
const IRDB_BASE = 'https://cdn.jsdelivr.net/gh/probonopd/irdb@master/codes/';
let irdbIndexCache = null;

async function loadIrdbIndex() {
  if (irdbIndexCache) return irdbIndexCache;
  const res = await fetch(IRDB_BASE + 'index');
  const text = await res.text();
  irdbIndexCache = text.split('\n').map(function (l) { return l.trim(); }).filter(function (l) { return l.length > 0; });
  return irdbIndexCache;
}

function irdbPathToUrl(path) {
  return IRDB_BASE + path.split('/').map(encodeURIComponent).join('/');
}

async function lookupSearch() {
  const mfr = document.getElementById('lookupManufacturer').value.trim().toLowerCase();
  const type = document.getElementById('lookupDeviceType').value.trim().toLowerCase();
  const resultsDiv = document.getElementById('lookupResults');
  document.getElementById('lookupFunctions').innerHTML = '';
  if (!mfr) { resultsDiv.innerHTML = "<p class='empty'>Enter a manufacturer.</p>"; return; }
  resultsDiv.innerHTML = "<p class='empty'>Searching...</p>";
  try {
    const index = await loadIrdbIndex();
    const matches = index.filter(function (line) {
      const parts = line.split('/');
      if (parts.length < 2) return false;
      return parts[0].toLowerCase().indexOf(mfr) === 0 &&
             (!type || parts[1].toLowerCase().indexOf(type) >= 0);
    }).slice(0, 40);
    resultsDiv.innerHTML = '';
    if (matches.length === 0) {
      resultsDiv.innerHTML = "<p class='empty'>No matches. Try a shorter manufacturer name or a different/blank device type.</p>";
      return;
    }
    matches.forEach(function (path) {
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.className = 'secondary';
      btn.style.display = 'block';
      btn.style.width = '100%';
      btn.style.marginBottom = '6px';
      btn.style.textAlign = 'left';
      btn.textContent = path.replace(/\.csv$/, '');
      btn.addEventListener('click', function () { lookupLoadDevice(path); });
      resultsDiv.appendChild(btn);
    });
  } catch (e) {
    resultsDiv.innerHTML = "<p class='empty'>Search failed - check your internet connection (this searches a public database directly from your phone, not through the device).</p>";
  }
}

// Fetches one device's CSV and parses it into {funcName,protocol,device,
// subdevice,func} rows - shared by the manufacturer-driven lookup below and
// the blind search further down, so both read a device file the same way.
async function fetchDeviceRows(path) {
  const res = await fetch(irdbPathToUrl(path));
  const text = await res.text();
  const lines = text.split('\n').map(function (l) { return l.trim(); }).filter(function (l) { return l.length > 0; });
  lines.shift(); // header row
  return lines.map(function (line) {
    const parts = line.split(',');
    if (parts.length < 5) return null;
    return {funcName: parts[0], protocol: parts[1], device: parts[2], subdevice: parts[3], func: parts[4]};
  }).filter(function (r) { return r !== null; });
}

function renderDeviceFunctions(rows) {
  const funcsDiv = document.getElementById('lookupFunctions');
  funcsDiv.innerHTML = '';
  if (rows.length === 0) {
    funcsDiv.innerHTML = "<p class='empty'>No functions found in this file.</p>";
    return;
  }
  rows.forEach(function (r) {
    const row = document.createElement('div');
    row.className = 'item';
    const label = document.createElement('span');
    label.className = 'name';
    label.textContent = r.funcName + ' (' + r.protocol + ')';
    row.appendChild(label);
    row.appendChild(makeButton('Try', '', function () { lookupTry(r.protocol, r.device, r.subdevice, r.func); }));
    const nameInput = document.createElement('input');
    nameInput.type = 'text';
    nameInput.placeholder = 'save as...';
    nameInput.value = r.funcName.toLowerCase().replace(/[^a-z0-9]+/g, '_');
    nameInput.style.maxWidth = '110px';
    row.appendChild(nameInput);
    row.appendChild(makeButton('Save', 'secondary', function () {
      if (nameInput.value.trim()) lookupSave(nameInput.value.trim(), r.protocol, r.device, r.subdevice, r.func);
    }));
    funcsDiv.appendChild(row);
  });
}

async function lookupLoadDevice(path) {
  const funcsDiv = document.getElementById('lookupFunctions');
  funcsDiv.innerHTML = "<p class='empty'>Loading...</p>";
  try {
    renderDeviceFunctions(await fetchDeviceRows(path));
  } catch (e) {
    funcsDiv.innerHTML = "<p class='empty'>Failed to load that file.</p>";
  }
}

async function lookupTry(protocol, device, subdevice, func) {
  if (actionBusy) return;
  setActionBusy(true);
  try {
    const res = await fetch('/trylookup?' + new URLSearchParams({protocol: protocol, device: device, subdevice: subdevice, function: func, ajax: '1'}));
    if (!res.ok) alert('Try failed: ' + await res.text());
  } finally {
    setActionBusy(false);
  }
}

async function lookupSave(name, protocol, device, subdevice, func) {
  if (actionBusy) return;
  setActionBusy(true);
  try {
    const params = {name: name, protocol: protocol, device: device, subdevice: subdevice, function: func, ajax: '1'};
    let res = await fetch('/savelookupcode?' + new URLSearchParams(params));
    if (res.status === 409) {
      const data = await res.json();
      if (!confirm("A code named '" + data.name + "' already exists. Overwrite?")) return;
      params.confirm = '1';
      res = await fetch('/savelookupcode?' + new URLSearchParams(params));
    }
    if (!res.ok) { alert('Save failed: ' + await res.text()); return; }
    alert('Saved as ' + name);
  } finally {
    setActionBusy(false);
  }
  refresh();
}

// ---- Blind search: for a device with no manufacturer name to go on (worn
// label, no remote at all, genuinely obscure) - walks IRDB device entries
// one at a time, guesses which row is most likely the power button, and
// lets you Try it until one actually reacts. Same idea as the "code search"
// mode on a universal remote. Device type is optional here too - blank
// searches every device in the index, not just one category.
let blindCandidates = [];
let blindIndex = -1;

function blindPickRow(rows) {
  const guess = rows.find(function (r) { return /power|standby|on\/?off/i.test(r.funcName); });
  return guess || rows[0];
}

async function blindStart() {
  const type = document.getElementById('blindDeviceType').value.trim().toLowerCase();
  const statusDiv = document.getElementById('blindStatus');
  document.getElementById('blindCandidate').innerHTML = '';
  statusDiv.innerHTML = "<p class='empty'>Loading index...</p>";
  try {
    const index = await loadIrdbIndex();
    blindCandidates = index.filter(function (line) {
      if (!type) return true;
      const parts = line.split('/');
      return parts.length >= 2 && parts[1].toLowerCase().indexOf(type) >= 0;
    });
    if (blindCandidates.length === 0) {
      statusDiv.innerHTML = "<p class='empty'>No devices match that type. Try leaving it blank to search everything.</p>";
      return;
    }
    blindIndex = -1;
    blindNext();
  } catch (e) {
    statusDiv.innerHTML = "<p class='empty'>Couldn't load the code index - check your internet connection.</p>";
  }
}

// Always requires an explicit tap to move to the next candidate (Next or
// Skip) rather than auto-advancing on empty/failed entries - auto-recursing
// through failures would turn one offline moment into a tight loop hammering
// every remaining candidate at once.
async function blindNext() {
  blindIndex++;
  const statusDiv = document.getElementById('blindStatus');
  const candDiv = document.getElementById('blindCandidate');
  if (blindIndex >= blindCandidates.length) {
    statusDiv.innerHTML = "<p class='empty'>That's every candidate - none confirmed. Try a different or blank device type.</p>";
    candDiv.innerHTML = '';
    return;
  }
  const path = blindCandidates[blindIndex];
  statusDiv.textContent = 'Candidate ' + (blindIndex + 1) + ' of ' + blindCandidates.length + ': ' + path.replace(/\.csv$/, '');
  candDiv.innerHTML = "<p class='empty'>Loading...</p>";
  let rows;
  try {
    rows = await fetchDeviceRows(path);
  } catch (e) {
    candDiv.innerHTML = "<p class='empty'>Failed to load this one.</p>";
    candDiv.appendChild(makeButton('Skip', 'secondary', blindNext));
    return;
  }
  if (rows.length === 0) {
    candDiv.innerHTML = "<p class='empty'>No functions in this one.</p>";
    candDiv.appendChild(makeButton('Skip', 'secondary', blindNext));
    return;
  }
  const row = blindPickRow(rows);
  candDiv.innerHTML = '';
  candDiv.appendChild(makeButton('Try: ' + row.funcName, '', function () {
    lookupTry(row.protocol, row.device, row.subdevice, row.func);
  }));
  candDiv.appendChild(makeButton('Next', 'secondary', blindNext));
  candDiv.appendChild(makeButton("It worked! Show all buttons", 'secondary', function () {
    statusDiv.innerHTML = '';
    candDiv.innerHTML = '';
    lookupLoadDevice(path);
  }));
}

function blindStop() {
  blindCandidates = [];
  blindIndex = -1;
  document.getElementById('blindStatus').innerHTML = '';
  document.getElementById('blindCandidate').innerHTML = '';
}

function render(state) {
  const bar = document.getElementById('statusBar');
  bar.textContent = state.status + (state.learning ? ' (' + state.pendingName + ')' : '');
  bar.className = state.learning ? 'learning' : '';

  // don't stomp a picker the user currently has open
  [['colorAccent', state.colors.accent], ['colorWave', state.colors.wave]]
    .forEach(function (pair) {
      const input = document.getElementById(pair[0]);
      if (document.activeElement !== input) input.value = pair[1];
    });

  const sleepSel = document.getElementById('screenSleepSel');
  if (document.activeElement !== sleepSel) sleepSel.value = String(state.screenSleepMin);

  renderRemote(state);
  renderSlots(state);

  const codesDiv = document.getElementById('codes');
  codesDiv.innerHTML = '';
  if (state.codes.length === 0) {
    codesDiv.innerHTML = "<p class='empty'>No codes learned yet.</p>";
  }
  state.codes.forEach(function (name) {
    const row = document.createElement('div');
    row.className = 'item';
    const span = document.createElement('span');
    span.className = 'name';
    span.textContent = name;
    row.appendChild(span);
    row.appendChild(makeButton('Blast', '', function () { blastCode(name); }));
    row.appendChild(makeButton('Rename', 'secondary', function () { renameCode(name); }));
    row.appendChild(makeButton('Delete', 'danger', function () { deleteCode(name); }));
    codesDiv.appendChild(row);
  });
}

let lastStateJson = null;

async function refresh() {
  if (pollBusy) return; // previous poll still in flight - don't stack another one
  pollBusy = true;
  try {
    const res = await fetch('/api/state');
    const text = await res.text();
    // skip the DOM rebuild when nothing changed - otherwise every 1s poll
    // tears down and rebuilds dropdowns/inputs, which can interrupt one
    // you have open on some mobile browsers
    if (text !== lastStateJson) {
      lastStateJson = text;
      render(JSON.parse(text));
    }
  } catch (e) {
    // transient - next poll will retry
  } finally {
    pollBusy = false;
  }
}

refresh();
setInterval(refresh, 1000);
</script>
</body>
</html>
)HTML";

void handleRoot() {
  server.send_P(200, "text/html", PAGE_HTML);
}

#ifdef BOARD_CYD
// Both defined in the CYD screen-manager section further down. Wrapped as
// helpers rather than inlining `if (currentCydScreen == CYD_HOME)` at each
// call site, because the CydScreen enum isn't declared until that section
// and these are called from web handlers defined above it.
void cydShowLearnScreen();
void cydRefreshHome();   // redraws Home only if Home is what's showing
void cydLeaveLearn();    // back to Home, but only if the Learn screen is up
#endif

// Also the entry point CYD's on-screen "Learn New" tap uses (see
// cydHandleTouch()) - it sets these same globals directly rather than
// calling this handler, since there's no HTTP request to satisfy there,
// but the effect is identical either way.
void handleLearn() {
  if (server.hasArg("name")) {
    String name = server.arg("name");
    name.trim();
    if (name.length() == 0) {
      server.send(400, "text/plain", "Name can't be empty");
      return;
    }
    bool confirmed = server.hasArg("confirm") && server.arg("confirm") == "1";
    if (codeLibrary.count(name) && !confirmed) {
      sendOverwriteConfirm("code", "/learn?name=" + name, name);
      return;
    }
    pendingLearnName = name;
    pendingLearnSlot = -1;  // this learn came from the web app, not a physical slot
    learning = true;
    learningStartedAt = millis();
    updateScreen("Point remote & press...");
    flashLeds(CRGB::Yellow, 150);
#ifdef BOARD_CYD
    // so a learn started from the web is also reflected on the touchscreen
    // if someone's looking at it, not just via the status banner text
    cydShowLearnScreen();
#endif
  }
  finishRequest();
}

void handleBlastCode() {
  if (server.hasArg("name")) {
    // queued and sent from loop() rather than blocking this request - the
    // status bar reflects the send in real time once loop() drains it
    queueCodeSend(server.arg("name"));
  }
  finishRequest();
}

// /deletecode?name=tv_power
void handleDeleteCode() {
  if (server.hasArg("name")) {
    String name = server.arg("name");
    auto it = codeLibrary.find(name);
    if (it != codeLibrary.end()) {
      codeLibrary.erase(it);
      for (int i = 0; i < NUM_SLOTS; i++) {
        if (slotCodeName[i] == name) slotCodeName[i] = "";
      }
      // every remote - a deleted code can be referenced by buttons on
      // remotes other than the one currently selected
      bool hadRemoteButton = false;
      for (auto &r : remotes)
        for (auto &b : r.buttons) {
          if (b.codeName == name) { b.codeName = ""; hadRemoteButton = true; }
        }
      // MQTT discovery is keyed by codeName - a deleted code's entity would
      // otherwise linger in HA forever since nothing else republishes it
      if (hadRemoteButton) mqttClearDiscovery(name);
      updateScreen("Deleted code: " + name);
      markProfileDirty();
    }
  }
  finishRequest();
}

// /renamecode?oldname=tv_power&newname=tv_on
void handleRenameCode() {
  if (server.hasArg("oldname") && server.hasArg("newname")) {
    String oldName = server.arg("oldname");
    String newName = server.arg("newname");
    newName.trim();
    auto it = codeLibrary.find(oldName);
    if (it != codeLibrary.end() && newName.length() > 0 && !codeLibrary.count(newName)) {
      codeLibrary[newName] = it->second;
      codeLibrary.erase(it);
      for (int i = 0; i < NUM_SLOTS; i++) {
        if (slotCodeName[i] == oldName) slotCodeName[i] = newName;
      }
      bool hadRemoteButton = false;
      for (auto &r : remotes)
        for (auto &b : r.buttons) {
          if (b.codeName == oldName) { b.codeName = newName; hadRemoteButton = true; }
        }
      // discovery is keyed by codeName - move the HA entity from the old
      // topic to the new one instead of leaving a stale duplicate behind
      if (hadRemoteButton) {
        mqttClearDiscovery(oldName);
        mqttPublishDiscovery();
      }
      updateScreen("Renamed: " + oldName + " -> " + newName);
      markProfileDirty();
    }
  }
  finishRequest();
}

// downloads the current profile as a JSON file
void handleExport() {
  if (!saveProfile()) {
    server.send(500, "text/plain", "Failed to prepare profile for export");
    return;
  }
  File f = LittleFS.open(PROFILE_PATH, "r");
  if (!f) {
    server.send(500, "text/plain", "Failed to open profile for export");
    return;
  }
  server.sendHeader("Content-Disposition", "attachment; filename=ir-profile.json");
  server.streamFile(f, "application/json");
  f.close();
}

// replaces the current profile with an uploaded JSON file's contents
void handleImport() {
  if (!server.hasArg("plain") || server.arg("plain").length() == 0) {
    server.send(400, "text/plain", "No profile data in request body");
    return;
  }
  String body = server.arg("plain");

  // parse-validate BEFORE touching the persisted file. Writing straight to
  // PROFILE_PATH and validating after (the old approach) meant a malformed
  // upload would overwrite the working profile with garbage - even though
  // the request got an error response, the live file was already bad, and
  // it would fail to load again on the *next* reboot too, silently wiping
  // everything that used to be there.
  JsonDocument testDoc;
  if (deserializeJson(testDoc, body)) {
    server.send(400, "text/plain", "That file isn't a valid IR Controller profile");
    return;
  }

  const char* tmpPath = "/profile.json.tmp";
  File f = LittleFS.open(tmpPath, "w");
  if (!f) {
    server.send(500, "text/plain", "Failed to open profile for import");
    return;
  }
  f.print(body);
  f.close();
  if (!LittleFS.rename(tmpPath, PROFILE_PATH)) {
    server.send(500, "text/plain", "Failed to save imported profile");
    return;
  }

  if (!loadProfile()) {
    server.send(400, "text/plain", "That file isn't a valid IR Controller profile");
    return;
  }
  updateScreen("Profile imported");
  server.send(200, "text/plain", "OK");
}

// /assignslot?slot=0&name=Button%201 - name="" (or omitted) clears the slot.
// Lets the web app repoint a physical slot at any existing code without
// needing the physical buttons at all.
// /assignslot?slot=N&name=X&color=%23rrggbb&icon=N - name/color/icon are
// each optional and updated independently, so the app can change just the
// color of a slot without touching its code assignment, etc.
void handleAssignSlot() {
  if (!server.hasArg("slot")) {
    server.send(400, "text/plain", "Missing slot");
    return;
  }
  int slot = server.arg("slot").toInt();
  if (slot < 0 || slot >= NUM_SLOTS) {
    server.send(400, "text/plain", "Slot out of range");
    return;
  }
  if (server.hasArg("name")) {
    String name = server.arg("name");
    if (name.length() > 0 && !codeLibrary.count(name)) {
      server.send(400, "text/plain", "No such code");
      return;
    }
    slotCodeName[slot] = name;
  }
  if (server.hasArg("color")) {
    slotColor[slot] = parseHexColor(server.arg("color"), slotColor[slot]);
  }
  if (server.hasArg("icon")) {
    int icon = server.arg("icon").toInt();
    if (icon >= 0 && icon < ICON_COUNT) slotIcon[slot] = icon;
  }
  updateScreen("Slot " + String(slot + 1) + " updated");
  markProfileDirty();
  finishRequest();
}

// /setcolors?accent=%23rrggbb&assigned=%23rrggbb&wave=%23rrggbb - any
// subset of the three; an invalid hex value is silently ignored (keeps the
// previous color) rather than erroring, since this is just cosmetic
void handleSetColors() {
  if (server.hasArg("accent")) accentColor = parseHexColor(server.arg("accent"), accentColor);
  if (server.hasArg("wave")) waveColor = parseHexColor(server.arg("wave"), waveColor);
  updateScreen(lastStatus);  // redraw now so the new colors are visible immediately
  markProfileDirty();
  finishRequest();
}

// /screensleep?minutes=N - 0 means never blank the backlight (the default)
void handleScreenSleep() {
  if (!server.hasArg("minutes")) {
    server.send(400, "text/plain", "Missing minutes");
    return;
  }
  long m = server.arg("minutes").toInt();
  if (m < 0 || m > 240) {
    server.send(400, "text/plain", "minutes must be 0-240 (0 = never)");
    return;
  }
  screensaverTimeoutMs = (unsigned long)m * 60000UL;
  // waking here matters: setting it to "never" while the screen is already
  // blanked would otherwise leave it dark with nothing left to re-trigger
  // the wake path
  if (screenAsleep) {
    screenAsleep = false;
    tft.setBrightness(SCREEN_BRIGHTNESS);
  }
  lastActivityAt = millis();
  markProfileDirty();
  finishRequest();
}

// ---------- virtual remote endpoints ----------
// /remote/add?spacer=1 adds an invisible layout-only placeholder tile - no
// other fields needed or read. Otherwise: name/code/icon/color are all
// optional (a button can be just a symbol + a color, or even blank, added
// with zero typing) since requiring a typed name for every button made
// building a remote tedious for no benefit - the tile only ever shows the
// icon/color, name is solely for the edit list and MQTT entity naming.
void handleRemoteAdd() {
  if (server.hasArg("spacer") && server.arg("spacer") == "1") {
    RemoteButton spacer;
    spacer.spacer = true;
    remoteButtons.push_back(spacer);
    updateScreen("Remote: empty space added");
    markProfileDirty();
    finishRequest();
    return;
  }

  RemoteButton b;
  if (server.hasArg("name")) {
    b.name = server.arg("name");
    b.name.trim();
  }
  b.codeName = server.hasArg("code") ? server.arg("code") : "";
  if (b.codeName.length() > 0 && !codeLibrary.count(b.codeName)) {
    server.send(400, "text/plain", "No such code");
    return;
  }
  if (server.hasArg("icon")) {
    int icon = server.arg("icon").toInt();
    if (icon >= 0 && icon < ICON_COUNT) b.icon = icon;
  }
  if (server.hasArg("color")) {
    b.color = parseHexColor(server.arg("color"), b.color);
  }
  remoteButtons.push_back(b);
  updateScreen("Remote button added" + (b.name.length() ? (": " + b.name) : String("")));
  markProfileDirty();
  mqttPublishDiscovery();
  finishRequest();
}

// /remote/update?index=N&name=X&code=Y&icon=N&color=%23rrggbb - all but
// index optional, updated independently. name may be set to blank (it's
// optional - see handleRemoteAdd).
void handleRemoteUpdate() {
  if (!server.hasArg("index")) {
    server.send(400, "text/plain", "Missing index");
    return;
  }
  int idx = server.arg("index").toInt();
  if (idx < 0 || idx >= (int)remoteButtons.size()) {
    server.send(400, "text/plain", "Index out of range");
    return;
  }
  RemoteButton &b = remoteButtons[idx];
  String oldCodeName = b.codeName;
  if (server.hasArg("name")) {
    String name = server.arg("name");
    name.trim();
    b.name = name;
  }
  if (server.hasArg("code")) {
    String code = server.arg("code");
    if (code.length() > 0 && !codeLibrary.count(code)) {
      server.send(400, "text/plain", "No such code");
      return;
    }
    b.codeName = code;
  }
  if (server.hasArg("icon")) {
    int icon = server.arg("icon").toInt();
    if (icon >= 0 && icon < ICON_COUNT) b.icon = icon;
  }
  if (server.hasArg("color")) {
    b.color = parseHexColor(server.arg("color"), b.color);
  }
  // discovery is keyed by codeName now (see mqttPublishDiscovery) - clear
  // the old topic if the code changed, otherwise a retained entity for the
  // previous code would linger in HA alongside the new one
  if (oldCodeName != b.codeName && oldCodeName.length() > 0) mqttClearDiscovery(oldCodeName);
  updateScreen("Remote button updated" + (b.name.length() ? (": " + b.name) : String("")));
  markProfileDirty();
  mqttPublishDiscovery();
  finishRequest();
}

// /remote/delete?index=N
void handleRemoteDelete() {
  if (!server.hasArg("index")) {
    server.send(400, "text/plain", "Missing index");
    return;
  }
  int idx = server.arg("index").toInt();
  if (idx < 0 || idx >= (int)remoteButtons.size()) {
    server.send(400, "text/plain", "Index out of range");
    return;
  }
  String name = remoteButtons[idx].name;
  String codeName = remoteButtons[idx].codeName;
  remoteButtons.erase(remoteButtons.begin() + idx);
  if (codeName.length() > 0) mqttClearDiscovery(codeName);
  updateScreen("Remote button deleted" + (name.length() ? (": " + name) : String("")));
  markProfileDirty();
  finishRequest();
}

// /remote/move?index=N&dir=up|down - swaps with its neighbor. Kept
// alongside /remote/reorder (below) as a reliable one-tap fallback for the
// drag handle - useful for precise single-step nudges, or if a drag gesture
// doesn't register on a given browser/input method.
void handleRemoteMove() {
  if (!server.hasArg("index") || !server.hasArg("dir")) {
    server.send(400, "text/plain", "Missing index or dir");
    return;
  }
  int idx = server.arg("index").toInt();
  if (idx < 0 || idx >= (int)remoteButtons.size()) {
    server.send(400, "text/plain", "Index out of range");
    return;
  }
  String dir = server.arg("dir");
  int swapWith = (dir == "up") ? idx - 1 : (dir == "down") ? idx + 1 : -1;
  if (swapWith < 0 || swapWith >= (int)remoteButtons.size()) {
    finishRequest();  // already at that end - no-op, not an error
    return;
  }
  std::swap(remoteButtons[idx], remoteButtons[swapWith]);
  markProfileDirty();
  finishRequest();
}

// /remote/reorder?from=N&to=M - moves the button at `from` to sit at
// position `to`, shifting everything between (not a swap - a swap would
// leave the dragged button's old neighbor displaced in the wrong direction,
// which doesn't match what dragging a tile to a new spot visually means).
// Backs the drag-and-drop reorder in the web UI; /remote/move above still
// covers precise single-step nudges.
void handleRemoteReorder() {
  if (!server.hasArg("from") || !server.hasArg("to")) {
    server.send(400, "text/plain", "Missing from or to");
    return;
  }
  int from = server.arg("from").toInt();
  int to = server.arg("to").toInt();
  if (from < 0 || from >= (int)remoteButtons.size() || to < 0 || to >= (int)remoteButtons.size()) {
    server.send(400, "text/plain", "Index out of range");
    return;
  }
  if (from != to) {
    RemoteButton moved = remoteButtons[from];
    remoteButtons.erase(remoteButtons.begin() + from);
    remoteButtons.insert(remoteButtons.begin() + to, moved);
    markProfileDirty();
  }
  finishRequest();
}

// /remote/columns?n=N
void handleRemoteColumns() {
  if (!server.hasArg("n")) {
    server.send(400, "text/plain", "Missing n");
    return;
  }
  int n = server.arg("n").toInt();
  if (n < 1 || n > 6) {
    server.send(400, "text/plain", "Columns must be 1-6");
    return;
  }
  remoteColumns = n;
  markProfileDirty();
  finishRequest();
}

// ---------- multiple remotes ----------
// /remotes/add?name=X - creates an empty remote and switches to it
void handleRemotesAdd() {
  String name = server.hasArg("name") ? server.arg("name") : "";
  name.trim();
  if (name.length() == 0) name = "Remote " + String((int)remotes.size() + 1);
  Remote r;
  r.name = name;
  remotes.push_back(r);
  currentRemote = (int)remotes.size() - 1;   // switch to what was just made
  updateScreen("Remote added: " + name);
  markProfileDirty();
  finishRequest();
}

// /remotes/select?index=N - which remote the device screen shows and the
// web app edits
void handleRemotesSelect() {
  if (!server.hasArg("index")) {
    server.send(400, "text/plain", "Missing index");
    return;
  }
  int idx = server.arg("index").toInt();
  if (idx < 0 || idx >= (int)remotes.size()) {
    server.send(400, "text/plain", "Index out of range");
    return;
  }
  currentRemote = idx;
  updateScreen("Remote: " + remotes[idx].name);
  markProfileDirty();
#ifdef BOARD_CYD
  cydRefreshHome();
#endif
  finishRequest();
}

// /remotes/rename?index=N&name=X
void handleRemotesRename() {
  if (!server.hasArg("index") || !server.hasArg("name")) {
    server.send(400, "text/plain", "Missing index or name");
    return;
  }
  int idx = server.arg("index").toInt();
  if (idx < 0 || idx >= (int)remotes.size()) {
    server.send(400, "text/plain", "Index out of range");
    return;
  }
  String name = server.arg("name");
  name.trim();
  if (name.length() == 0) {
    server.send(400, "text/plain", "Name can't be empty");
    return;
  }
  remotes[idx].name = name;
  markProfileDirty();
  finishRequest();
}

// /remotes/delete?index=N - the remote's buttons go with it, but the codes
// they referenced stay in the library (they may be used elsewhere, and
// losing captured codes to a layout edit would be a nasty surprise)
void handleRemotesDelete() {
  if (!server.hasArg("index")) {
    server.send(400, "text/plain", "Missing index");
    return;
  }
  int idx = server.arg("index").toInt();
  if (idx < 0 || idx >= (int)remotes.size()) {
    server.send(400, "text/plain", "Index out of range");
    return;
  }
  if (remotes.size() <= 1) {
    server.send(400, "text/plain", "Can't delete the only remote");
    return;
  }
  String name = remotes[idx].name;
  for (auto &b : remotes[idx].buttons) {
    if (b.codeName.length()) mqttClearDiscovery(b.codeName);
  }
  remotes.erase(remotes.begin() + idx);
  if (currentRemote >= idx) currentRemote--;
  ensureRemoteValid();
  updateScreen("Remote deleted: " + name);
  markProfileDirty();
  mqttPublishDiscovery();
#ifdef BOARD_CYD
  cydRefreshHome();
#endif
  finishRequest();
}

// /remote/addmany?count=N[&icon=N&color=%23rrggbb] - bulk-create blank
// buttons so a layout can be roughed out in one action instead of tapping
// "add" a dozen times.
void handleRemoteAddMany() {
  if (!server.hasArg("count")) {
    server.send(400, "text/plain", "Missing count");
    return;
  }
  int count = server.arg("count").toInt();
  if (count < 1 || count > 40) {
    server.send(400, "text/plain", "count must be 1-40");
    return;
  }
  int icon = server.hasArg("icon") ? server.arg("icon").toInt() : (int)ICON_NONE;
  if (icon < 0 || icon >= ICON_COUNT) icon = ICON_NONE;
  CRGB color = server.hasArg("color") ? parseHexColor(server.arg("color"), DEFAULT_SLOT_COLOR)
                                      : DEFAULT_SLOT_COLOR;
  for (int i = 0; i < count; i++) {
    RemoteButton b;
    b.icon = (uint8_t)icon;
    b.color = color;
    remoteButtons.push_back(b);
  }
  updateScreen("Added " + String(count) + " buttons");
  markProfileDirty();
  finishRequest();
}

// /remote/learn?index=N - learn a code straight into that button. The code
// is named automatically and assigned on capture, so a laid-out remote can
// be taught without naming anything by hand.
void handleRemoteLearnButton() {
  if (!server.hasArg("index")) {
    server.send(400, "text/plain", "Missing index");
    return;
  }
  int idx = server.arg("index").toInt();
  if (idx < 0 || idx >= (int)remoteButtons.size()) {
    server.send(400, "text/plain", "Index out of range");
    return;
  }
  if (remoteButtons[idx].spacer) {
    server.send(400, "text/plain", "That's a spacer, not a button");
    return;
  }
  teachAllActive = false;   // an explicit single-button learn ends any run
  if (!startButtonLearn(idx)) {
    server.send(400, "text/plain", "Couldn't start learn");
    return;
  }
#ifdef BOARD_CYD
  cydShowLearnScreen();
#endif
  finishRequest();
}

// /remote/teachall - walk every untaught button in the current remote,
// re-arming automatically after each capture (see the learn-complete
// branch in loop()).
void handleRemoteTeachAll() {
  int next = nextUntaughtButton(0);
  if (next < 0) {
    server.send(400, "text/plain", "Every button on this remote already has a code");
    return;
  }
  teachAllActive = true;
  if (!startButtonLearn(next)) {
    teachAllActive = false;
    server.send(400, "text/plain", "Couldn't start teach-all");
    return;
  }
#ifdef BOARD_CYD
  cydShowLearnScreen();
#endif
  finishRequest();
}

// /remote/teachstop
void handleRemoteTeachStop() {
  stopTeachAll("Teach all stopped");
#ifdef BOARD_CYD
  cydLeaveLearn();
#endif
  finishRequest();
}

// ---------- IRDB lookup (search/browse happens client-side in the browser
// against the public jsdelivr/GitHub mirror of probonopd/irdb - the ESP32
// only needs to handle the final "try/save this specific code" step, not
// parse the ~100KB index or search UI itself) ----------
IRCode lookupCodeFromArgs(bool &ok) {
  IRCode code;
  code.len = 0;
  code.hash = 0;
  ok = false;
  if (!server.hasArg("protocol") || !server.hasArg("function")) {
    server.send(400, "text/plain", "Missing protocol or function");
    return code;
  }
  code.protocol = server.arg("protocol");
  if (!isProtocolSupported(code.protocol)) {
    server.send(400, "text/plain", "Protocol '" + code.protocol + "' isn't supported for sending yet - try a different profile/protocol");
    return code;
  }
  int device = server.hasArg("device") ? server.arg("device").toInt() : 0;
  int subdevice = server.hasArg("subdevice") ? server.arg("subdevice").toInt() : -1;
  int function = server.arg("function").toInt();

  String p = code.protocol;
  p.toUpperCase();
  if (p.indexOf("NEC") >= 0) {
    // IRDB gives device+subdevice as separate bytes; for NEC-family that's
    // the standard extended-NEC address layout (low byte, high byte).
    // subdevice -1 means "old" 8-bit NEC with no separate high byte - and
    // that split is also how IrSender.sendNEC() itself decides which form
    // to send, so this isn't a guess either way.
    code.address = (subdevice >= 0) ? (uint16_t)((device & 0xFF) | ((subdevice & 0xFF) << 8))
                                     : (uint16_t)(device & 0xFF);
  } else {
    // every other supported protocol takes a single address value, not a
    // packed device+subdevice pair (confirmed against real IRDB samples)
    code.address = (uint16_t)(device & 0xFFFF);
  }
  code.command = (uint16_t)(function & 0xFFFF);

  ok = true;
  return code;
}

// /trylookup?protocol=NECx2&device=7&subdevice=7&function=2 - fires the
// code immediately, doesn't save it. Point the target device's way and see
// if it reacts before committing to saving it as a named code.
void handleTryLookup() {
  bool ok;
  IRCode code = lookupCodeFromArgs(ok);
  if (!ok) return;  // lookupCodeFromArgs already sent the error response
  sendCode(code);
  finishRequest();
}

// /savelookupcode?name=X&protocol=NECx2&device=7&subdevice=7&function=2 -
// same code as /trylookup, but saved under a name instead of (or as well
// as) fired, once you've confirmed with /trylookup that it actually works.
void handleSaveLookupCode() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "Missing name");
    return;
  }
  String name = server.arg("name");
  name.trim();
  if (name.length() == 0) {
    server.send(400, "text/plain", "Name can't be empty");
    return;
  }
  bool confirmed = server.hasArg("confirm") && server.arg("confirm") == "1";
  if (codeLibrary.count(name) && !confirmed) {
    String redirectUrl = "/savelookupcode?name=" + name +
                          "&protocol=" + server.arg("protocol") +
                          "&device=" + server.arg("device") +
                          "&subdevice=" + server.arg("subdevice") +
                          "&function=" + server.arg("function");
    sendOverwriteConfirm("code", redirectUrl, name);
    return;
  }
  bool ok;
  IRCode code = lookupCodeFromArgs(ok);
  if (!ok) return;
  codeLibrary[name] = code;
  updateScreen("Saved from lookup: " + name);
  markProfileDirty();
  finishRequest();
}

// ---------- physical control: 2 buttons + 8 LEDs as slot indicators ----------
// CYD has neither dedicated buttons nor an LED strip - this entire section
// is specific to the original board (see BOARD_CYD's touch-screen Home/
// Learn screens further down for its equivalent).
#if HAS_PHYSICAL_BUTTONS
// Button 1 fires whichever slot is selected. Button 2 short-press cycles
// slots; held past LONG_PRESS_MS it learns into the current slot instead,
// overwriting whatever was there - no confirmation, since there's no screen
// prompt possible from two buttons alone (the web app's overwrite confirm
// is for the web flow, not this one).
const unsigned long LONG_PRESS_MS = 600;

void startSlotLearn(int slot) {
  String name = slotCodeName[slot];
  if (name.length() == 0) {
    name = "Button " + String(slot + 1);
  }
  pendingLearnName = name;
  pendingLearnSlot = slot;
  learning = true;
  learningStartedAt = millis();
  updateScreen("Learning slot " + String(slot + 1) + ": " + name);
  flashLeds(CRGB::Yellow, 150);
}
#endif

// Idle LED display: each LED mirrors one slot. Three states so you can
// actually tell a slot is set without cycling to it: accentColor for the
// selected slot, that slot's own custom color for any other slot that has a
// code, off for a genuinely empty slot. The blink for a pending learn tracks
// pendingLearnSlot rather than currentSlot deliberately: they're normally
// the same slot, but a physical learn locks in which slot it's for the
// moment it starts (see startSlotLearn()), and if currentSlot were free to
// drift away from that slot in the meantime, the blink would follow the
// wrong LED - showing you were learning into a slot you weren't. Replaces
// the old idle rainbow now that the LEDs mean something. No CYD equivalent
// needed: CYD has no physical slot-select button/LED strip at all (slots
// are web-only on that board - see the plan notes).
#if HAS_WS2812
void updateSlotLeds() {
  for (int i = 0; i < NUM_SLOTS && i < NUM_LEDS; i++) {
    CRGB color;
    if (learning && pendingLearnSlot == i) {
      bool on = (millis() / 300) % 2 == 0;
      color = on ? CRGB(80, 60, 0) : CRGB::Black;
    } else if (i == currentSlot) {
      color = accentColor;
    } else if (slotCodeName[i].length() > 0) {
      color = slotColor[i];
    } else {
      color = CRGB::Black;
    }
    leds[i] = color;
  }
  FastLED.show();
}
#endif

// Debounces a pulled-up button pin: a raw reading only becomes "the truth"
// once it's held steady for DEBOUNCE_MS. Without this, mechanical contact
// bounce on press/release (a few ms of rapid HIGH/LOW chatter) can register
// as several presses and releases for what was physically one press - the
// likely cause of "weird stuff happening" on button 2 in particular, since
// its state machine tracks edges (press/release/hold), not just a level.
// Touch/knob boards debounce their own input instead (see their sections).
#if HAS_PHYSICAL_BUTTONS
const unsigned long DEBOUNCE_MS = 25;

bool debounceButton(int pin, int &lastRaw, unsigned long &lastChangeAt, bool &stable) {
  int raw = digitalRead(pin);
  if (raw != lastRaw) {
    lastRaw = raw;
    lastChangeAt = millis();
  }
  if (millis() - lastChangeAt > DEBOUNCE_MS) {
    stable = (raw == LOW);  // both buttons are INPUT_PULLUP: LOW = pressed
  }
  return stable;
}

int blastBtnLastRaw = HIGH;
unsigned long blastBtnLastChangeAt = 0;
bool blastBtnStable = false;
bool blastBtnWasDown = false;

int learnBtnLastRaw = HIGH;
unsigned long learnBtnLastChangeAt = 0;
bool learnBtnStable = false;
bool learnBtnDown = false;
unsigned long learnBtnPressedAt = 0;
bool learnBtnLongFired = false;

void handlePhysicalButtons() {
  // fires once on the press edge, not once per 300ms while held - the old
  // level+cooldown check would re-fire repeatedly (e.g. a TV power toggle
  // firing 2-3 times) if the button was held even slightly too long
  bool blastIsDown = debounceButton(BLAST_BTN_PIN, blastBtnLastRaw, blastBtnLastChangeAt, blastBtnStable);
  if (blastIsDown && !blastBtnWasDown) {
    blastBtnWasDown = true;
    if (learning) {
      // sendCode() briefly stops the receiver to transmit - firing here
      // could eat the very signal a pending learn is waiting to capture,
      // on top of just being a confusing thing to do mid-learn
      Serial.println("Blast button ignored - learn in progress");
    } else if (slotCodeName[currentSlot].length() > 0) {
      queueCodeSend(slotCodeName[currentSlot]);
    } else {
      updateScreen("Slot " + String(currentSlot + 1) + " is empty");
      flashLeds(CRGB::Orange, 150);
    }
  } else if (!blastIsDown && blastBtnWasDown) {
    blastBtnWasDown = false;
  }

  bool learnBtnIsDown = debounceButton(LEARN_BTN_PIN, learnBtnLastRaw, learnBtnLastChangeAt, learnBtnStable);
  if (learnBtnIsDown && !learnBtnDown) {
    learnBtnDown = true;
    learnBtnPressedAt = millis();
    learnBtnLongFired = false;
  } else if (learnBtnIsDown && learnBtnDown && !learnBtnLongFired &&
             millis() - learnBtnPressedAt >= LONG_PRESS_MS) {
    learnBtnLongFired = true;
    startSlotLearn(currentSlot);
  } else if (!learnBtnIsDown && learnBtnDown) {
    learnBtnDown = false;
    // don't let a second short press change which slot is selected while a
    // physical learn is still waiting on a signal - see updateSlotLeds()
    if (!learnBtnLongFired && pendingLearnSlot < 0) {
      currentSlot = (currentSlot + 1) % NUM_SLOTS;
      updateScreen("Slot " + String(currentSlot + 1) + " selected");
      markProfileDirty();
    }
  }
}
#endif  // HAS_PHYSICAL_BUTTONS (physical control section)

// ---------- WiFi reconnect watchdog ----------
// WiFi.begin() only kicks off a connection attempt - it doesn't block
// waiting for it - so this stays non-blocking, checked once per loop().
// Backs off up to 30s between attempts so a real outage doesn't spam
// reconnect attempts forever.
bool wifiWasConnected = true;
unsigned long lastWifiRetryAt = 0;
unsigned long wifiRetryIntervalMs = 3000;
const unsigned long WIFI_RETRY_MAX_MS = 30000;

void mdnsStart() {
  if (MDNS.begin(hostname)) {
    MDNS.addService("http", "tcp", 80);
  }
}

void handleWifiWatchdog() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiWasConnected) {
      Serial.println("WiFi reconnected");
      updateScreen("WiFi reconnected");
      mdnsStart();  // mDNS doesn't reliably survive a reconnect on its own
      wifiRetryIntervalMs = 3000;
    }
    wifiWasConnected = true;
    return;
  }

  wifiWasConnected = false;
  if (millis() - lastWifiRetryAt < wifiRetryIntervalMs) return;
  lastWifiRetryAt = millis();
  Serial.println("WiFi disconnected, reconnecting...");
  updateScreen("WiFi reconnecting...");
  WiFi.disconnect();
  WiFi.begin(ssid, password);
  wifiRetryIntervalMs = min(wifiRetryIntervalMs * 2, WIFI_RETRY_MAX_MS);
}

// ---------- task watchdog ----------
// Resets the device if loop() ever stops coming back around within
// WDT_TIMEOUT_S - a safety net against any future blocking bug (this one
// has already been hunted down and fixed a few times this session) hanging
// the device silently instead of recovering on its own. The config API
// changed between arduino-esp32 2.x and 3.x (IDF4 vs IDF5), hence the guard -
// this is the one piece of this change set I can't verify without knowing
// exactly which core version the build pulls in, so if it fails to compile
// here specifically, that's why.
//
// 25s, not a tighter number, because setup()'s WiFi connect wait alone can
// take ~10s worst case (see setup()), and LittleFS can take a few seconds
// to format on a first boot / after corruption - both need to fit under
// this with real margin, or the watchdog would fire mid-boot on a slow
// start and reset the device before it ever reaches loop(), which would be
// a boot-crash-loop every time WiFi just happens to be unreachable.
#include <esp_task_wdt.h>
#define WDT_TIMEOUT_S 25

void initWatchdog() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  esp_task_wdt_init(&wdtConfig);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);
}

// ---------- CYD touch GUI: screen manager ----------
// Scope is deliberately narrow (Home + Learn only, per explicit user
// decision) - Physical slots, IRDB Lookup, Colors, and Backup stay
// web-only on this board, reachable through the same WebServer that's
// still fully active here (see setup() below).
//
// No UI framework in use (raw LGFX calls only, same as the other board) -
// each screen does a full redraw into a fixed-size tap-zone list, and
// cydHandleTouch() hit-tests a touch point against whichever list is
// currently active. Reuses the same drawing helpers already written for
// the other board's screen (drawIcon(), contrastTextColor(),
// truncateToWidth()) rather than duplicating them.
#ifdef BOARD_CYD

const int CYD_BANNER_H = 50;  // persistent status strip, shared across every CYD screen

enum CydScreen { CYD_HOME, CYD_LEARN, CYD_SETTINGS, CYD_TOUCHTEST, CYD_REMOTES, CYD_ADVANCED };
CydScreen currentCydScreen = CYD_HOME;

struct TapZone {
  int x, y, w, h;
  int action;  // >=0 on Home = index into remoteButtons; else one of the CYD_ACTION_* codes below
};
const int MAX_TAP_ZONES = 24;
TapZone cydZones[MAX_TAP_ZONES];
int cydZoneCount = 0;

void cydAddZone(int x, int y, int w, int h, int action) {
  if (cydZoneCount >= MAX_TAP_ZONES) return;
  cydZones[cydZoneCount++] = {x, y, w, h, action};
}

const int CYD_ACTION_LEARN_NEW = -2;
const int CYD_ACTION_CANCEL_LEARN = -3;
const int CYD_ACTION_PAGE_PREV = -4;
const int CYD_ACTION_PAGE_NEXT = -5;
const int CYD_ACTION_SETTINGS = -6;
const int CYD_ACTION_BACK_HOME = -7;
const int CYD_ACTION_ROT_NEXT = -8;
const int CYD_ACTION_ROT_PREV = -9;
const int CYD_ACTION_TROT_NEXT = -10;
const int CYD_ACTION_TROT_PREV = -11;
const int CYD_ACTION_TOUCHTEST = -13;
const int CYD_ACTION_REMOTES = -14;      // open the remote picker
const int CYD_ACTION_TEACHALL = -15;
const int CYD_ACTION_ADVANCED = -16;     // touch settings live behind this now
const int CYD_ACTION_BACK_SETTINGS = -17;
// remote picker entries are encoded as -100 - index, keeping them clear of
// the fixed action codes above and of the >=0 button indices used on Home
const int CYD_ACTION_REMOTE_BASE = -100;

int cydHomePage = 0;
const int CYD_COLS_MAX = 4;      // wider remoteColumns settings get clamped here for legibility - the full column count still applies on the web Remote tab
const int CYD_ROWS_PER_PAGE = 3;
const int CYD_MIN_CELL = 40;     // below this a tile is too small to reliably hit - row/col counts get reduced rather than letting tiles collide

// ---------- runtime display settings ----------
// Orientation lives here (not just the compile-time CYD_ROTATION default)
// so it can be changed from the on-device Settings screen and survive a
// reboot - getting this wrong previously meant a full re-flash per attempt,
// which is a terrible way to find the right value. Stored in its own file
// rather than profile.json deliberately: this is per-unit hardware config,
// and it must NOT travel with a profile export/import between boards.
uint8_t cydRotation = CYD_ROTATION;
uint8_t cydTouchRotation = CYD_TOUCH_ROTATION;
#define CYD_DISPLAY_PATH "/cyd_display.txt"

// Stamped with the firmware build. LittleFS survives a firmware upload, so
// without this a value saved while hunting for the right orientation keeps
// overriding the compile-time default on every later flash - which is
// exactly what made a corrected default appear to have no effect. A new
// build discards the saved values and uses the defaults it shipped with;
// anything changed in Settings after that persists normally until the next
// flash.
static const char CYD_BUILD_ID[] = __DATE__ " " __TIME__;

bool cydLoadDisplaySettings() {
  if (!LittleFS.exists(CYD_DISPLAY_PATH)) return false;
  File f = LittleFS.open(CYD_DISPLAY_PATH, "r");
  if (!f) return false;
  String stamp = f.readStringUntil('\n');
  String rot   = f.readStringUntil('\n');
  String fix   = f.readStringUntil('\n');
  f.close();
  stamp.trim();

  if (stamp != CYD_BUILD_ID) {
    Serial.println("Display settings are from an older firmware - using this build's defaults");
    LittleFS.remove(CYD_DISPLAY_PATH);
    return false;
  }
  int r = rot.toInt(), x = fix.toInt();
  if (r < 0 || r > 7 || x < 0 || x > 7) return false;  // corrupt - fall back to defaults
  cydRotation = (uint8_t)r;
  cydTouchRotation = (uint8_t)x;
  Serial.printf("Loaded saved display settings: rotation=%d touchfix=%d\n", r, x);
  return true;
}

void cydSaveDisplaySettings() {
  File f = LittleFS.open(CYD_DISPLAY_PATH, "w");
  if (!f) return;
  f.println(CYD_BUILD_ID);
  f.println(cydRotation);
  f.println(cydTouchRotation);
  f.close();
}

// Applies the current rotation AND re-reads the panel's resulting extent.
// SCREEN_W/H must be refreshed here rather than assumed: rotations 1/3/5/7
// swap the axes, so a stale 240x320 would leave part of the panel never
// drawn to.
void cydApplyRotation() {
  tft.setRotation(cydRotation);
  SCREEN_W = tft.width();
  SCREEN_H = tft.height();
  // Never let a bogus panel report produce a zero/negative canvas - every
  // layout below divides by these, and a 0 would take the whole UI out.
  if (SCREEN_W <= 0) SCREEN_W = 240;
  if (SCREEN_H <= 0) SCREEN_H = 320;
}

// ---------- XPT2046 touch, read directly ----------
// Not via LovyanGFX's touch layer, and with no calibrate-and-store step at
// all. The raw ADC range is a measured property of this hardware (see the
// CYD_RAW_* values in LGX_Config_CYD.h, captured with
// debug/cyd_touch_debug.cpp), so there is nothing to calibrate at runtime -
// which also removes the failure mode where a bad stored calibration made
// the on-device Settings screen unreachable and could only be cleared by
// re-flashing.
#define T_CLK  25
#define T_MOSI 32
#define T_MISO 39
#define T_CS   33
#define T_IRQ  36

SPIClass touchSPI(HSPI);

// 0xD0 = read X, 0x90 = read Y (12-bit, differential mode)
uint16_t cydXptRead(uint8_t cmd) {
  touchSPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(T_CS, LOW);
  touchSPI.transfer(cmd);
  uint8_t hi = touchSPI.transfer(0x00);
  uint8_t lo = touchSPI.transfer(0x00);
  digitalWrite(T_CS, HIGH);
  touchSPI.endTransaction();
  return ((uint16_t)((hi << 8) | lo)) >> 3;   // 12 significant bits
}

bool cydTouchDown() { return digitalRead(T_IRQ) == LOW; }

// Median of several samples - individual reads are noisy enough that one
// sample can be far enough off to land in the wrong button.
bool cydReadRawTouch(uint16_t &rx, uint16_t &ry) {
  if (!cydTouchDown()) return false;
  const int N = 5;
  uint16_t xs[N], ys[N];
  for (int i = 0; i < N; i++) {
    if (!cydTouchDown()) return false;   // released mid-read - discard
    ys[i] = cydXptRead(0x90);
    xs[i] = cydXptRead(0xD0);
  }
  for (int i = 1; i < N; i++) {          // insertion sort, N is tiny
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

// Touch axis correction, applied to the coordinates LGFX hands back rather
// than by reconfiguring the touch driver at runtime. Deliberate choice:
// the library's runtime touch-reconfig API isn't something I could confirm
// the exact shape of, and guessing wrong there is a compile error you'd be
// stuck waiting on. This is plain arithmetic - verifiable by reading it,
// and it covers the same 8 orientations:
//   bit 0 = invert X, bit 1 = invert Y, bit 2 = swap X/Y
// Only needed when the touch layer's physical orientation disagrees with
// the display's; setRotation() already keeps them aligned in the normal
// case, so 0 is correct unless taps land somewhere other than where you
// press.
void cydTransformTouch(int &x, int &y) {
  uint8_t r = cydTouchRotation;
  if (r & 4) { int t = x; x = y; y = t; }
  if (r & 1) x = SCREEN_W - 1 - x;
  if (r & 2) y = SCREEN_H - 1 - y;
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  if (x >= SCREEN_W) x = SCREEN_W - 1;
  if (y >= SCREEN_H) y = SCREEN_H - 1;
}

// Draws just the persistent banner strip - called on every updateScreen(),
// so it stays cheap (partial redraw) rather than touching the rest of
// whichever screen is currently showing underneath it.
void cydDrawStatusBar(const String &status) {
  uint16_t accent565 = tft.color565(accentColor.r, accentColor.g, accentColor.b);
  tft.fillRect(0, 0, SCREEN_W, CYD_BANNER_H, accent565);
  tft.setFont(&fonts::FreeSansBold9pt7b);
  tft.setTextDatum(textdatum_t::middle_center);
  tft.setTextColor(contrastTextColor(accentColor), accent565);
  tft.drawString(truncateToWidth(status, SCREEN_W - 20), SCREEN_W / 2, CYD_BANNER_H / 2);
  tft.setTextDatum(textdatum_t::top_left);
}

// CYD's flashLeds() equivalent: tints the banner instead of an LED, with
// the same non-blocking "set now, revert later" shape as ledFlashActive on
// the other board (see cydCheckBannerTintRevert(), called from loop()).
bool cydBannerTintActive = false;
unsigned long cydBannerTintUntil = 0;

void cydTintBanner(CRGB color, int ms) {
  uint16_t c565 = tft.color565(color.r, color.g, color.b);
  tft.fillRect(0, 0, SCREEN_W, CYD_BANNER_H, c565);
  tft.setFont(&fonts::FreeSansBold9pt7b);
  tft.setTextDatum(textdatum_t::middle_center);
  tft.setTextColor(contrastTextColor(color), c565);
  tft.drawString(truncateToWidth(lastStatus, SCREEN_W - 20), SCREEN_W / 2, CYD_BANNER_H / 2);
  tft.setTextDatum(textdatum_t::top_left);
  cydBannerTintActive = true;
  cydBannerTintUntil = millis() + ms;
}

void cydCheckBannerTintRevert() {
  if (cydBannerTintActive && millis() >= cydBannerTintUntil) {
    cydBannerTintActive = false;
    cydDrawStatusBar(lastStatus);
  }
}

void cydShowScreen(CydScreen s);  // forward-referenced by cydHandleTouch() below

// Home: the virtual Remote grid (remoteButtons/remoteColumns - same data
// the web Remote tab edits), tap a tile to fire it, paged if more tiles
// exist than fit. Spacers render as a real gap (no tile, no tap zone),
// matching how the web Remote builder treats them.
// Shared button painter - every tappable control on every CYD screen goes
// through this, so label centering/truncation/registration can't drift
// between screens or be forgotten on one of them.
void cydDrawButton(int x, int y, int w, int h, const String &label, uint16_t bg, uint16_t fg, int action) {
  tft.fillRoundRect(x, y, w, h, 8, bg);
  tft.setFont(&fonts::FreeSansBold9pt7b);
  tft.setTextDatum(textdatum_t::middle_center);
  tft.setTextColor(fg, bg);
  tft.drawString(truncateToWidth(label, w - 10), x + w / 2, y + h / 2);
  tft.setTextDatum(textdatum_t::top_left);
  cydAddZone(x, y, w, h, action);
}

void cydDrawHome() {
  cydZoneCount = 0;
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  cydDrawStatusBar(lastStatus);

  const int pad = 8, gap = 6, rowH = 44;

  // Lay the fixed rows out from the bottom up first, then give the grid
  // whatever vertical space is actually left. Rows are stacked, never
  // overlaid, so no two controls can collide regardless of screen size -
  // the previous version packed paging and "Learn New" into one row with
  // hand-computed widths, which is what allowed them to overlap.
  int footerY = SCREEN_H - pad - rowH;

  int cols = remoteColumns;
  if (cols < 1) cols = 1;
  if (cols > CYD_COLS_MAX) cols = CYD_COLS_MAX;
  // shrink the column count rather than let tiles get too small to hit
  while (cols > 1 && (SCREEN_W - 2 * pad - (cols - 1) * gap) / cols < CYD_MIN_CELL) cols--;

  int rows = CYD_ROWS_PER_PAGE;
  int perPage = cols * rows;
  int totalPages = max(1, (int)((remoteButtons.size() + perPage - 1) / perPage));

  int pagingY = -1;
  if (totalPages > 1) {
    pagingY = footerY - gap - rowH;
  }
  int gridTop = CYD_BANNER_H + pad;
  int gridBottom = (pagingY > 0 ? pagingY : footerY) - gap;
  int gridH = gridBottom - gridTop;

  // same guard vertically: fewer rows beats unusably short tiles
  while (rows > 1 && (gridH - (rows - 1) * gap) / rows < CYD_MIN_CELL) rows--;
  perPage = cols * rows;
  totalPages = max(1, (int)((remoteButtons.size() + perPage - 1) / perPage));
  if (cydHomePage >= totalPages) cydHomePage = totalPages - 1;
  if (cydHomePage < 0) cydHomePage = 0;

  int cellW = (SCREEN_W - 2 * pad - (cols - 1) * gap) / cols;
  int cellH = (gridH - (rows - 1) * gap) / rows;

  if (remoteButtons.empty()) {
    tft.setFont(&fonts::FreeSans9pt7b);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(textdatum_t::middle_center);
    tft.drawString("No remote buttons yet -", SCREEN_W / 2, gridTop + gridH / 2 - 12);
    tft.drawString("add them in the web app.", SCREEN_W / 2, gridTop + gridH / 2 + 12);
    tft.setTextDatum(textdatum_t::top_left);
  } else if (cellH >= CYD_MIN_CELL) {
    int start = cydHomePage * perPage;
    int end = min((int)remoteButtons.size(), start + perPage);
    for (int i = start; i < end; i++) {
      int slot = i - start;
      int x = pad + (slot % cols) * (cellW + gap);
      int y = gridTop + (slot / cols) * (cellH + gap);
      RemoteButton &b = remoteButtons[i];
      if (b.spacer) continue;  // invisible placeholder - real gap, no tile

      uint16_t bg565 = tft.color565(b.color.r, b.color.g, b.color.b);
      uint16_t textColor = contrastTextColor(b.color);
      tft.fillRoundRect(x, y, cellW, cellH, 8, bg565);
      if (b.icon != ICON_NONE) {
        drawIcon(b.icon, x, y, cellW, cellH, textColor);
      } else {
        tft.setFont(&fonts::FreeSansBold9pt7b);
        tft.setTextDatum(textdatum_t::middle_center);
        tft.setTextColor(textColor, bg565);
        String label = b.name.length() ? b.name : b.codeName;
        tft.drawString(truncateToWidth(label, cellW - 8), x + cellW / 2, y + cellH / 2);
        tft.setTextDatum(textdatum_t::top_left);
      }
      cydAddZone(x, y, cellW, cellH, i);
    }
  }

  // paging row (own row, only when there's more than one page)
  if (pagingY > 0) {
    int navW = (SCREEN_W - 2 * pad - gap) / 2;
    cydDrawButton(pad, pagingY, navW, rowH, "< Prev", TFT_DARKGREY, TFT_WHITE, CYD_ACTION_PAGE_PREV);
    cydDrawButton(pad + navW + gap, pagingY, navW, rowH, "Next >", TFT_DARKGREY, TFT_WHITE, CYD_ACTION_PAGE_NEXT);
  }

  // footer row: Learn New + Settings, split evenly
  uint16_t accent565 = tft.color565(accentColor.r, accentColor.g, accentColor.b);
  int halfW = (SCREEN_W - 2 * pad - gap) / 2;
  cydDrawButton(pad, footerY, halfW, rowH, "Learn New", accent565, contrastTextColor(accentColor), CYD_ACTION_LEARN_NEW);
  cydDrawButton(pad + halfW + gap, footerY, halfW, rowH, "Settings", TFT_NAVY, TFT_WHITE, CYD_ACTION_SETTINGS);

  tft.endWrite();
}

// Settings: on-device display/touch configuration. Exists specifically so
// orientation can be corrected without a re-flash - stepping a value here
// applies it live and saves it, so a wrong guess costs one tap instead of
// a full build/upload cycle.
void cydDrawSettings() {
  cydZoneCount = 0;
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  cydDrawStatusBar("Settings");

  const int pad = 8, gap = 6, rowH = 42;
  int y = CYD_BANNER_H + pad;

  // Everyday things first; the display/touch calibration knobs are behind
  // "Advanced" now that the right values are locked in and shouldn't be
  // one stray tap away.
  cydDrawButton(pad, y, SCREEN_W - 2 * pad, rowH, "Remotes", TFT_NAVY, TFT_WHITE, CYD_ACTION_REMOTES);
  y += rowH + gap;
  cydDrawButton(pad, y, SCREEN_W - 2 * pad, rowH, "Teach all buttons", TFT_DARKGREEN, TFT_WHITE, CYD_ACTION_TEACHALL);
  y += rowH + gap;
  cydDrawButton(pad, y, SCREEN_W - 2 * pad, rowH, "Advanced", TFT_DARKGREY, TFT_WHITE, CYD_ACTION_ADVANCED);

  cydDrawButton(pad, SCREEN_H - pad - rowH, SCREEN_W - 2 * pad, rowH, "Back", TFT_NAVY, TFT_WHITE, CYD_ACTION_BACK_HOME);
  tft.endWrite();
}

// Advanced: display/touch orientation. Separated from Settings because
// these are set-once hardware values - keeping them one level down means a
// misplaced tap can't knock the screen sideways during normal use.
void cydDrawAdvanced() {
  cydZoneCount = 0;
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  cydDrawStatusBar("Advanced");

  const int pad = 8, gap = 6, rowH = 40, stepW = 52;
  int y = CYD_BANNER_H + pad;

  tft.setFont(&fonts::FreeSans9pt7b);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(textdatum_t::top_left);

  tft.drawString(truncateToWidth("Screen rotation", SCREEN_W - 2 * pad), pad, y);
  y += 20;
  cydDrawButton(pad, y, stepW, rowH, "<", TFT_DARKGREY, TFT_WHITE, CYD_ACTION_ROT_PREV);
  tft.setFont(&fonts::FreeSansBold12pt7b);
  tft.setTextDatum(textdatum_t::middle_center);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(String(cydRotation), SCREEN_W / 2, y + rowH / 2);
  tft.setTextDatum(textdatum_t::top_left);
  cydDrawButton(SCREEN_W - pad - stepW, y, stepW, rowH, ">", TFT_DARKGREY, TFT_WHITE, CYD_ACTION_ROT_NEXT);
  y += rowH + gap + 6;

  tft.setFont(&fonts::FreeSans9pt7b);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(truncateToWidth("Touch fix", SCREEN_W - 2 * pad), pad, y);
  y += 20;
  cydDrawButton(pad, y, stepW, rowH, "<", TFT_DARKGREY, TFT_WHITE, CYD_ACTION_TROT_PREV);
  tft.setFont(&fonts::FreeSansBold12pt7b);
  tft.setTextDatum(textdatum_t::middle_center);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(String(cydTouchRotation), SCREEN_W / 2, y + rowH / 2);
  tft.setTextDatum(textdatum_t::top_left);
  cydDrawButton(SCREEN_W - pad - stepW, y, stepW, rowH, ">", TFT_DARKGREY, TFT_WHITE, CYD_ACTION_TROT_NEXT);
  y += rowH + gap + 10;

  int backY = SCREEN_H - pad - rowH;
  if (y + rowH + gap <= backY) {
    cydDrawButton(pad, y, SCREEN_W - 2 * pad, rowH, "Touch test", TFT_PURPLE, TFT_WHITE, CYD_ACTION_TOUCHTEST);
  }
  cydDrawButton(pad, backY, SCREEN_W - 2 * pad, rowH, "Back", TFT_NAVY, TFT_WHITE, CYD_ACTION_BACK_SETTINGS);
  tft.endWrite();
}

// Remote picker: one row per remote, current one highlighted. Paged the
// same way Home is, so a long list stays reachable.
int cydRemotePage = 0;

void cydDrawRemotes() {
  cydZoneCount = 0;
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  cydDrawStatusBar("Remotes");

  const int pad = 8, gap = 6, rowH = 42;
  int y = CYD_BANNER_H + pad;
  int backY = SCREEN_H - pad - rowH;
  int perPage = max(1, (backY - gap - y) / (rowH + gap));
  int total = (int)remotes.size();
  int pages = max(1, (total + perPage - 1) / perPage);
  if (cydRemotePage >= pages) cydRemotePage = pages - 1;
  if (cydRemotePage < 0) cydRemotePage = 0;

  int start = cydRemotePage * perPage;
  int end = min(total, start + perPage);
  for (int i = start; i < end; i++) {
    bool cur = (i == currentRemote);
    uint16_t bg = cur ? tft.color565(accentColor.r, accentColor.g, accentColor.b) : TFT_DARKGREY;
    uint16_t fg = cur ? contrastTextColor(accentColor) : TFT_WHITE;
    String label = remotes[i].name + "  (" + String((int)remotes[i].buttons.size()) + ")";
    cydDrawButton(pad, y, SCREEN_W - 2 * pad, rowH, label, bg, fg, CYD_ACTION_REMOTE_BASE - i);
    y += rowH + gap;
  }

  if (pages > 1) {
    int half = (SCREEN_W - 2 * pad - gap) / 2;
    int navY = backY - rowH - gap;
    cydDrawButton(pad, navY, half, rowH, "< Prev", TFT_DARKGREY, TFT_WHITE, CYD_ACTION_PAGE_PREV);
    cydDrawButton(pad + half + gap, navY, half, rowH, "Next >", TFT_DARKGREY, TFT_WHITE, CYD_ACTION_PAGE_NEXT);
  }
  cydDrawButton(pad, backY, SCREEN_W - 2 * pad, rowH, "Back", TFT_NAVY, TFT_WHITE, CYD_ACTION_BACK_SETTINGS);
  tft.endWrite();
}

// Learn: mirrors the web app's /learn flow exactly (same pendingLearnName/
// pendingLearnSlot/learning globals, same shared loop() IR-decode
// learn-complete branch) - just triggered by a tap instead of a request.
void cydDrawLearn() {
  cydZoneCount = 0;
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  cydDrawStatusBar(lastStatus);

  const int pad = 8, rowH = 44;
  int centerY = CYD_BANNER_H + (SCREEN_H - CYD_BANNER_H - rowH - pad) / 2;

  tft.setFont(&fonts::FreeSansBold12pt7b);
  tft.setTextDatum(textdatum_t::middle_center);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(truncateToWidth("Point remote &", SCREEN_W - 2 * pad), SCREEN_W / 2, centerY - 30);
  tft.drawString(truncateToWidth("press a button", SCREEN_W - 2 * pad), SCREEN_W / 2, centerY);
  tft.setFont(&fonts::FreeSans9pt7b);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(truncateToWidth("Waiting for signal...", SCREEN_W - 2 * pad), SCREEN_W / 2, centerY + 30);
  tft.setTextDatum(textdatum_t::top_left);

  // routed through the shared button painter so it matches every other
  // screen and registers its tap zone the same way
  cydDrawButton(pad, SCREEN_H - pad - rowH, SCREEN_W - 2 * pad, rowH, "Cancel", TFT_MAROON, TFT_WHITE, CYD_ACTION_CANCEL_LEARN);

  tft.endWrite();
}

// Touch test: draws targets at the four extreme corners and the centre at
// known coordinates, then marks wherever you actually touch. Purpose is to
// settle the "is the panel extent right?" question by measurement - if the
// corner targets don't sit at the physical corners of the glass, the
// firmware's idea of the panel size is wrong, and that's visible directly
// rather than deduced. Every touch is also printed to serial.
// Auto-exits so a badly mis-mapped touch layer can't strand you here.
unsigned long cydTouchTestUntil = 0;
const unsigned long CYD_TOUCHTEST_MS = 45000;

void cydDrawTouchTest() {
  cydZoneCount = 0;
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  tft.drawRect(0, 0, SCREEN_W, SCREEN_H, TFT_WHITE);

  // corner + centre targets at exactly-known coordinates
  const int r = 14;
  struct { int x, y; } pts[5] = {
    {0, 0}, {SCREEN_W - 1, 0}, {0, SCREEN_H - 1}, {SCREEN_W - 1, SCREEN_H - 1},
    {SCREEN_W / 2, SCREEN_H / 2}
  };
  for (int i = 0; i < 5; i++) {
    tft.drawCircle(pts[i].x, pts[i].y, r, TFT_GREEN);
    tft.drawLine(pts[i].x - r, pts[i].y, pts[i].x + r, pts[i].y, TFT_GREEN);
    tft.drawLine(pts[i].x, pts[i].y - r, pts[i].x, pts[i].y + r, TFT_GREEN);
  }

  tft.setFont(&fonts::FreeSansBold9pt7b);
  tft.setTextDatum(textdatum_t::middle_center);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(truncateToWidth("TOUCH TEST", SCREEN_W - 20), SCREEN_W / 2, SCREEN_H / 2 - 46);
  tft.drawString(truncateToWidth(String(SCREEN_W) + "x" + String(SCREEN_H) + " rot" + String(cydRotation), SCREEN_W - 20),
                 SCREEN_W / 2, SCREEN_H / 2 - 26);
  tft.drawString(truncateToWidth("tap corners - auto-exits", SCREEN_W - 20), SCREEN_W / 2, SCREEN_H / 2 + 30);
  tft.setTextDatum(textdatum_t::top_left);
  tft.endWrite();

  cydTouchTestUntil = millis() + CYD_TOUCHTEST_MS;
}

// Marks a touch without redrawing the whole screen, so successive taps
// accumulate and you can see the pattern they form.
void cydMarkTouch(int x, int y) {
  tft.fillCircle(x, y, 5, TFT_RED);
  tft.setFont(&fonts::FreeSansBold9pt7b);
  tft.setTextDatum(textdatum_t::middle_center);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  // printed near the centre where it won't be clipped at an edge
  tft.fillRect(0, SCREEN_H / 2 + 44, SCREEN_W, 22, TFT_BLACK);
  tft.drawString(truncateToWidth(String(x) + "," + String(y), SCREEN_W - 20), SCREEN_W / 2, SCREEN_H / 2 + 55);
  tft.setTextDatum(textdatum_t::top_left);
}

void cydShowScreen(CydScreen s) {
  currentCydScreen = s;
  if (s == CYD_HOME) {
    cydHomePage = 0;
    cydDrawHome();
  } else if (s == CYD_SETTINGS) {
    cydDrawSettings();
  } else if (s == CYD_ADVANCED) {
    cydDrawAdvanced();
  } else if (s == CYD_REMOTES) {
    cydRemotePage = 0;
    cydDrawRemotes();
  } else if (s == CYD_TOUCHTEST) {
    cydDrawTouchTest();
  } else {
    cydDrawLearn();
  }
}

// Thin wrapper so handleLearn() (defined earlier in the file, before
// CydScreen exists) can trigger this without needing the enum visible at
// its forward-declaration point.
void cydShowLearnScreen() { cydShowScreen(CYD_LEARN); }
void cydRefreshHome() { if (currentCydScreen == CYD_HOME) cydDrawHome(); }
void cydLeaveLearn() { if (currentCydScreen == CYD_LEARN) cydShowScreen(CYD_HOME); }

// Dispatches a touch point to whichever action zone (if any) it lands in
// on the currently active screen. Reuses the exact same shared triggers
// the web app uses - queueCodeSend() for firing a code, the same
// pendingLearnName/learning/learningStartedAt setup handleLearn() uses to
// start a learn - so there's no forked logic, just a different trigger.
// Immediate visual acknowledgement of a tap. Without this a press that
// queues a send looks identical to a press that missed, since the actual
// send is drained a moment later in loop() - the outline is drawn the
// instant the touch lands, and cydRedrawAt schedules the clean repaint
// (non-blocking, no delay()).
unsigned long cydRedrawAt = 0;

void cydFlashZone(const TapZone &z) {
  tft.drawRect(z.x, z.y, z.w, z.h, TFT_WHITE);
  tft.drawRect(z.x + 1, z.y + 1, z.w - 2, z.h - 2, TFT_WHITE);
  cydRedrawAt = millis() + 180;
}

void cydHandleTouch(int x, int y) {
  // Touch test consumes every touch itself - no zones, nothing to trigger
  // accidentally while a mis-mapped touch layer is being measured.
  if (currentCydScreen == CYD_TOUCHTEST) {
    cydMarkTouch(x, y);
    return;
  }

  for (int i = 0; i < cydZoneCount; i++) {
    TapZone &z = cydZones[i];
    if (x < z.x || x >= z.x + z.w || y < z.y || y >= z.y + z.h) continue;

    int action = z.action;
    cydFlashZone(z);

    if (currentCydScreen == CYD_HOME) {
      if (action == CYD_ACTION_LEARN_NEW) {
        pendingLearnName = "button_code_" + String(millis());
        pendingLearnSlot = -1;
        learning = true;
        learningStartedAt = millis();
        updateScreen("Point remote & press...");
        flashLeds(CRGB::Yellow, 150);
        cydShowScreen(CYD_LEARN);
        cydRedrawAt = 0;  // screen already replaced - nothing stale to repaint
      } else if (action == CYD_ACTION_SETTINGS) {
        cydShowScreen(CYD_SETTINGS);
        cydRedrawAt = 0;
      } else if (action == CYD_ACTION_PAGE_PREV) {
        cydHomePage--;
        cydDrawHome();
        cydRedrawAt = 0;
      } else if (action == CYD_ACTION_PAGE_NEXT) {
        cydHomePage++;
        cydDrawHome();
        cydRedrawAt = 0;
      } else if (action >= 0 && action < (int)remoteButtons.size()) {
        const RemoteButton &b = remoteButtons[action];
        if (b.codeName.length() > 0) {
          queueCodeSend(b.codeName);
        } else {
          updateScreen("No code assigned");
          flashLeds(CRGB::Orange, 150);
        }
      }
    } else if (currentCydScreen == CYD_LEARN) {
      if (action == CYD_ACTION_CANCEL_LEARN) {
        learning = false;
        pendingLearnName = "";
        pendingLearnSlot = -1;
        updateScreen("Learn cancelled");
        cydShowScreen(CYD_HOME);
        cydRedrawAt = 0;
      }
    } else if (currentCydScreen == CYD_SETTINGS) {
      if (action == CYD_ACTION_BACK_HOME) {
        cydShowScreen(CYD_HOME);
        cydRedrawAt = 0;
      } else if (action == CYD_ACTION_REMOTES) {
        cydShowScreen(CYD_REMOTES);
        cydRedrawAt = 0;
      } else if (action == CYD_ACTION_ADVANCED) {
        cydShowScreen(CYD_ADVANCED);
        cydRedrawAt = 0;
      } else if (action == CYD_ACTION_TEACHALL) {
        int next = nextUntaughtButton(0);
        if (next < 0) {
          updateScreen("Nothing left to teach");
          cydDrawSettings();
        } else {
          teachAllActive = true;
          if (startButtonLearn(next)) cydShowScreen(CYD_LEARN);
          else { teachAllActive = false; cydDrawSettings(); }
        }
        cydRedrawAt = 0;
      }
    } else if (currentCydScreen == CYD_ADVANCED) {
      if (action == CYD_ACTION_BACK_SETTINGS) {
        cydShowScreen(CYD_SETTINGS);
        cydRedrawAt = 0;
      } else if (action == CYD_ACTION_ROT_NEXT || action == CYD_ACTION_ROT_PREV) {
        cydRotation = (action == CYD_ACTION_ROT_NEXT) ? (cydRotation + 1) & 7
                                                      : (cydRotation + 7) & 7;
        cydApplyRotation();       // takes effect immediately, including new SCREEN_W/H
        cydSaveDisplaySettings(); // ...and survives the next reboot
        cydDrawAdvanced();
        cydRedrawAt = 0;
      } else if (action == CYD_ACTION_TROT_NEXT || action == CYD_ACTION_TROT_PREV) {
        cydTouchRotation = (action == CYD_ACTION_TROT_NEXT) ? (cydTouchRotation + 1) & 7
                                                            : (cydTouchRotation + 7) & 7;
        cydSaveDisplaySettings();  // no apply step needed - cydTransformTouch() reads the value live
        cydDrawAdvanced();
        cydRedrawAt = 0;
      } else if (action == CYD_ACTION_TOUCHTEST) {
        cydShowScreen(CYD_TOUCHTEST);
        cydRedrawAt = 0;
      }
    } else if (currentCydScreen == CYD_REMOTES) {
      if (action == CYD_ACTION_BACK_SETTINGS) {
        cydShowScreen(CYD_SETTINGS);
        cydRedrawAt = 0;
      } else if (action == CYD_ACTION_PAGE_PREV) {
        cydRemotePage--;
        cydDrawRemotes();
        cydRedrawAt = 0;
      } else if (action == CYD_ACTION_PAGE_NEXT) {
        cydRemotePage++;
        cydDrawRemotes();
        cydRedrawAt = 0;
      } else if (action <= CYD_ACTION_REMOTE_BASE) {
        int idx = CYD_ACTION_REMOTE_BASE - action;
        if (idx >= 0 && idx < (int)remotes.size()) {
          currentRemote = idx;
          markProfileDirty();
          updateScreen("Remote: " + remotes[idx].name);
          cydShowScreen(CYD_HOME);
        }
        cydRedrawAt = 0;
      }
    }
    return;  // stop at the first matching zone
  }
}

// Polled from loop(), non-blocking, debounced the same ~300ms as the other
// board's physical buttons so one physical tap doesn't register twice.
unsigned long lastCydTouchAt = 0;
const unsigned long CYD_TOUCH_DEBOUNCE_MS = 300;

void cydHandleTouchPoll() {
  if (millis() - lastCydTouchAt < CYD_TOUCH_DEBOUNCE_MS) return;
  uint16_t rx, ry;
  if (!cydReadRawTouch(rx, ry)) return;
  lastCydTouchAt = millis();

  // A tap on a blanked screen wakes it and nothing else. Without this the
  // wake-up tap also lands on whatever button happens to be under it and
  // fires that code - you can't see what you're pressing, so it would be
  // firing something at random. (Only reachable when the screensaver has
  // been turned on; it's off by default.)
  if (screenAsleep) {
    screenAsleep = false;
    lastActivityAt = millis();
    tft.setBrightness(SCREEN_BRIGHTNESS);
    return;
  }

  // raw ADC -> screen, using the ranges measured on this hardware
  long x = map(rx, CYD_RAW_X_MIN, CYD_RAW_X_MAX, 0, SCREEN_W - 1);
  long y = map(ry, CYD_RAW_Y_MIN, CYD_RAW_Y_MAX, 0, SCREEN_H - 1);
  int sx = constrain(x, 0, SCREEN_W - 1);
  int sy = constrain(y, 0, SCREEN_H - 1);
  cydTransformTouch(sx, sy);   // applies the swap/invert bits (default: invert Y)

  Serial.printf("TOUCH raw=(%u,%u) -> (%d,%d) screen=%dx%d rot=%u fix=%u scr=%d\n",
                rx, ry, sx, sy, SCREEN_W, SCREEN_H,
                (unsigned)cydRotation, (unsigned)cydTouchRotation, (int)currentCydScreen);

  cydHandleTouch(sx, sy);
}

// /cyddisplay - read and change orientation over HTTP. Exists because the
// on-device Settings screen is unreachable whenever touch is mis-mapped,
// which is exactly when orientation most needs changing. Works from a
// browser regardless of what touch is doing.
//   /cyddisplay                     -> show current values
//   /cyddisplay?rot=5               -> set screen rotation (0-7)
//   /cyddisplay?fix=2               -> set touch fix (bit0 invX, bit1 invY, bit2 swap)
//   /cyddisplay?reset=1             -> discard saved values, back to firmware defaults
void handleCydDisplay() {
  bool changed = false;

  if (server.hasArg("reset")) {
    LittleFS.remove(CYD_DISPLAY_PATH);
    cydRotation = CYD_ROTATION;
    cydTouchRotation = CYD_TOUCH_ROTATION;
    cydApplyRotation();
    changed = true;
  }
  if (server.hasArg("rot")) {
    int r = server.arg("rot").toInt();
    if (r < 0 || r > 7) {
      server.send(400, "text/plain", "rot must be 0-7 (4-7 are the mirrored variants)");
      return;
    }
    cydRotation = (uint8_t)r;
    cydApplyRotation();
    changed = true;
  }
  if (server.hasArg("fix")) {
    int x = server.arg("fix").toInt();
    if (x < 0 || x > 7) {
      server.send(400, "text/plain", "fix must be 0-7 (bit0 invert X, bit1 invert Y, bit2 swap X/Y)");
      return;
    }
    cydTouchRotation = (uint8_t)x;
    changed = true;
  }

  if (changed && !server.hasArg("reset")) cydSaveDisplaySettings();
  if (changed) cydShowScreen(CYD_HOME);

  String out = "rotation = " + String(cydRotation) + "   (0-7, 4-7 mirrored)\n";
  out += "touch fix = " + String(cydTouchRotation) + "   (bit0 invX, bit1 invY, bit2 swapXY)\n";
  out += "screen    = " + String(SCREEN_W) + " x " + String(SCREEN_H) + "\n";
  out += "build     = " + String(CYD_BUILD_ID) + "\n\n";
  out += "/cyddisplay?rot=N   /cyddisplay?fix=N   /cyddisplay?reset=1\n";
  server.send(200, "text/plain", out);
}

#endif  // BOARD_CYD

// ============================================================================
// ESP32-C3-LCDkit: round 240x240 GC9A01 + rotary encoder, LVGL UI
// ============================================================================
// Input is one knob: rotate to move, short press to act, long press to go
// back / open the menu. There is no touch and no keyboard, so every screen
// is a single scrollable list of choices - nothing needs a pointer.
//
// LVGL is used for rendering only. Selection is driven from the encoder
// directly rather than through an LVGL input device and focus groups: the
// group/focus semantics add a lot of behaviour that has to be reasoned
// about blind, and doing it by hand keeps "which item is selected" a plain
// integer this code owns.
#ifdef BOARD_C3KNOB

// --- LVGL plumbing ---
// Partial draw buffer. 240 x 40 x 2 bytes = ~19KB; a full framebuffer
// would be 112KB, which this chip cannot spare alongside WiFi + web server.
#define LV_C3_BUF_LINES 40
static lv_disp_draw_buf_t c3DrawBuf;
static lv_color_t c3Buf[240 * LV_C3_BUF_LINES];
static lv_disp_drv_t c3DispDrv;

static void c3FlushCb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.writePixels((lgfx::rgb565_t *)color_p, w * h);
  tft.endWrite();
  lv_disp_flush_ready(drv);
}

// --- rotary encoder ---
// Quadrature decoded by sampling B on each falling edge of A. Coarse
// compared to a full state-machine decoder, but these detented knobs give
// one clean pulse per click and this cannot mis-count a detent into two.
volatile int c3EncDelta = 0;
int c3LastA = HIGH;
unsigned long c3LastEncAt = 0;

bool c3SwDown = false;
unsigned long c3SwPressedAt = 0;
bool c3SwLongFired = false;
const unsigned long C3_LONG_PRESS_MS = 600;
const unsigned long C3_ENC_MIN_GAP_MS = 5;   // contact-bounce floor

// --- screens ---
enum C3Screen { C3_HOME, C3_MENU, C3_REMOTES, C3_LEARN, C3_SCAN };
C3Screen c3Screen = C3_HOME;
int c3Sel = 0;               // selection index within the current screen
int c3HomeSel = 0;           // Home's selection, preserved while in the menu
int c3ScanIdx = 0;           // which brand the blind scan is sitting on

lv_obj_t *c3ScrRoot = NULL;
lv_obj_t *c3Arc = NULL;
lv_obj_t *c3IconLabel = NULL;
lv_obj_t *c3TitleLabel = NULL;
lv_obj_t *c3SubLabel = NULL;
lv_obj_t *c3StatusLabel = NULL;

String c3Status = "";
unsigned long c3FlashUntil = 0;
lv_color_t c3FlashColor;
bool c3FlashActive = false;

// Baked-in power codes for the blind scan: when you have no idea what a
// device speaks, step through these and press until something reacts.
// Values are real IRDB entries (probonopd/irdb), not invented - and only
// brands whose protocol this firmware can actually transmit are listed.
// Sharp and TCL were dropped for that reason: their IRDB entries use the
// Sharp and RCA-38 protocols, which isProtocolSupported() rejects, so
// including them would just produce buttons that silently do nothing.
struct ScanCode {
  const char *brand;
  const char *protocol;
  int device;
  int subdevice;
  int function;
};
const ScanCode C3_SCAN_CODES[] = {
  { "Samsung",  "NECx2",     7,   7, 2  },
  { "LG",       "NEC1",      1,   1, 28 },
  { "Sony",     "Sony12",    1,  -1, 21 },
  { "Panasonic","Panasonic", 128, 0, 61 },
  { "Philips",  "RC5",       0,  -1, 12 },
  { "Toshiba",  "NEC1",      64, -1, 18 },
};
const int C3_SCAN_COUNT = sizeof(C3_SCAN_CODES) / sizeof(C3_SCAN_CODES[0]);

const char *C3_MENU_ITEMS[] = { "Remotes", "Teach all", "Learn this", "IR scan", "Back" };
const int C3_MENU_COUNT = 5;

// Maps this project's icon ids onto LVGL's built-in symbol glyphs, so the
// knob UI gets real icons without shipping image assets. Digits fall
// through to plain text.
const char *c3IconSymbol(uint8_t id, const char *fallback) {
  switch (id) {
    case ICON_UP:    return LV_SYMBOL_UP;
    case ICON_DOWN:  return LV_SYMBOL_DOWN;
    case ICON_LEFT:  return LV_SYMBOL_LEFT;
    case ICON_RIGHT: return LV_SYMBOL_RIGHT;
    case ICON_PLUS:  return LV_SYMBOL_PLUS;
    case ICON_MINUS: return LV_SYMBOL_MINUS;
    case ICON_CHECK: return LV_SYMBOL_OK;
    case ICON_X:     return LV_SYMBOL_CLOSE;
    case ICON_POWER: return LV_SYMBOL_POWER;
    case ICON_WIFI:  return LV_SYMBOL_WIFI;
    default: break;
  }
  if (id >= ICON_0 && id <= ICON_9) {
    static char digit[2];
    digit[0] = (char)('0' + (id - ICON_0));
    digit[1] = 0;
    return digit;
  }
  return fallback;
}

// Counts only real buttons - spacers exist for layout on the big screens
// and would be dead stops when scrolling with a knob.
int c3RealButtonCount() {
  int n = 0;
  for (auto &b : remoteButtons) if (!b.spacer) n++;
  return n;
}

// nth non-spacer button -> index into remoteButtons, or -1
int c3RealButtonIndex(int nth) {
  int n = 0;
  for (int i = 0; i < (int)remoteButtons.size(); i++) {
    if (remoteButtons[i].spacer) continue;
    if (n == nth) return i;
    n++;
  }
  return -1;
}

void c3Render();

void c3NoteStatus(const String &status) {
  c3Status = status;
  if (c3StatusLabel) lv_label_set_text(c3StatusLabel, c3Status.c_str());
}

// flashLeds() equivalent: briefly recolours the ring instead of an LED,
// reverting non-blocking from loop() exactly like the other boards.
void c3FlashAccent(CRGB color, int ms) {
  if (!c3Arc) return;
  c3FlashColor = lv_color_make(color.r, color.g, color.b);
  lv_obj_set_style_arc_color(c3Arc, c3FlashColor, LV_PART_INDICATOR);
  c3FlashActive = true;
  c3FlashUntil = millis() + ms;
}

void c3CheckFlashRevert() {
  if (c3FlashActive && millis() >= c3FlashUntil) {
    c3FlashActive = false;
    if (c3Arc) {
      lv_obj_set_style_arc_color(c3Arc,
        lv_color_make(accentColor.r, accentColor.g, accentColor.b), LV_PART_INDICATOR);
    }
  }
}

// One screen layout reused by every mode: a progress ring, a big centre
// glyph, a title, a subtitle and a status line. Only the text and the ring
// position change between screens, so there is a single place where the
// round-panel geometry has to be right.
void c3BuildUi() {
  c3ScrRoot = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(c3ScrRoot, lv_color_black(), 0);
  lv_obj_clear_flag(c3ScrRoot, LV_OBJ_FLAG_SCROLLABLE);

  c3Arc = lv_arc_create(c3ScrRoot);
  lv_obj_set_size(c3Arc, 232, 232);
  lv_obj_center(c3Arc);
  lv_arc_set_rotation(c3Arc, 270);
  lv_arc_set_bg_angles(c3Arc, 0, 360);
  lv_arc_set_range(c3Arc, 0, 100);
  lv_arc_set_value(c3Arc, 0);
  lv_obj_remove_style(c3Arc, NULL, LV_PART_KNOB);          // display only, not a control
  lv_obj_clear_flag(c3Arc, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_width(c3Arc, 8, LV_PART_MAIN);
  lv_obj_set_style_arc_width(c3Arc, 8, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(c3Arc, lv_color_hex(0x222222), LV_PART_MAIN);
  lv_obj_set_style_arc_color(c3Arc,
    lv_color_make(accentColor.r, accentColor.g, accentColor.b), LV_PART_INDICATOR);

  c3IconLabel = lv_label_create(c3ScrRoot);
  lv_obj_set_style_text_font(c3IconLabel, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(c3IconLabel, lv_color_white(), 0);
  lv_label_set_text(c3IconLabel, "");
  lv_obj_align(c3IconLabel, LV_ALIGN_CENTER, 0, -28);

  c3TitleLabel = lv_label_create(c3ScrRoot);
  lv_obj_set_style_text_font(c3TitleLabel, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(c3TitleLabel, lv_color_white(), 0);
  lv_label_set_long_mode(c3TitleLabel, LV_LABEL_LONG_DOT);
  lv_obj_set_width(c3TitleLabel, 170);
  lv_obj_set_style_text_align(c3TitleLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_text(c3TitleLabel, "");
  lv_obj_align(c3TitleLabel, LV_ALIGN_CENTER, 0, 24);

  c3SubLabel = lv_label_create(c3ScrRoot);
  lv_obj_set_style_text_font(c3SubLabel, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(c3SubLabel, lv_color_hex(0x9aa0a6), 0);
  lv_label_set_long_mode(c3SubLabel, LV_LABEL_LONG_DOT);
  lv_obj_set_width(c3SubLabel, 170);
  lv_obj_set_style_text_align(c3SubLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_text(c3SubLabel, "");
  lv_obj_align(c3SubLabel, LV_ALIGN_CENTER, 0, 52);

  c3StatusLabel = lv_label_create(c3ScrRoot);
  lv_obj_set_style_text_font(c3StatusLabel, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(c3StatusLabel, lv_color_hex(0x5f6368), 0);
  lv_label_set_long_mode(c3StatusLabel, LV_LABEL_LONG_DOT);
  lv_obj_set_width(c3StatusLabel, 150);
  lv_obj_set_style_text_align(c3StatusLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_text(c3StatusLabel, "");
  lv_obj_align(c3StatusLabel, LV_ALIGN_CENTER, 0, -74);

  lv_scr_load(c3ScrRoot);
}

// Repaints the shared layout for whatever screen is current. Called on
// every encoder move / press rather than diffing - at this size a full
// text refresh is cheap and it keeps the logic obvious.
void c3Render() {
  if (!c3ScrRoot) return;
  char buf[96];

  if (c3Screen == C3_HOME) {
    int total = c3RealButtonCount();
    if (total == 0) {
      lv_label_set_text(c3IconLabel, LV_SYMBOL_WARNING);
      lv_label_set_text(c3TitleLabel, "No buttons");
      lv_label_set_text(c3SubLabel, "Add them in the app");
      lv_arc_set_value(c3Arc, 0);
    } else {
      if (c3Sel >= total) c3Sel = 0;
      if (c3Sel < 0) c3Sel = total - 1;
      int bi = c3RealButtonIndex(c3Sel);
      RemoteButton &b = remoteButtons[bi];
      String label = b.name.length() ? b.name : (b.codeName.length() ? b.codeName : String("(unnamed)"));
      lv_label_set_text(c3IconLabel, c3IconSymbol(b.icon, LV_SYMBOL_PLAY));
      lv_obj_set_style_text_color(c3IconLabel, lv_color_make(b.color.r, b.color.g, b.color.b), 0);
      lv_label_set_text(c3TitleLabel, label.c_str());
      snprintf(buf, sizeof(buf), "%d/%d  %s", c3Sel + 1, total,
               b.codeName.length() ? "ready" : "no code");
      lv_label_set_text(c3SubLabel, buf);
      lv_arc_set_value(c3Arc, (int)((c3Sel + 1) * 100L / total));
    }
    lv_label_set_text(c3StatusLabel, remotes[currentRemote].name.c_str());

  } else if (c3Screen == C3_MENU) {
    if (c3Sel < 0) c3Sel = C3_MENU_COUNT - 1;
    if (c3Sel >= C3_MENU_COUNT) c3Sel = 0;
    lv_label_set_text(c3IconLabel, LV_SYMBOL_LIST);
    lv_obj_set_style_text_color(c3IconLabel, lv_color_white(), 0);
    lv_label_set_text(c3TitleLabel, C3_MENU_ITEMS[c3Sel]);
    snprintf(buf, sizeof(buf), "%d/%d", c3Sel + 1, C3_MENU_COUNT);
    lv_label_set_text(c3SubLabel, buf);
    lv_arc_set_value(c3Arc, (int)((c3Sel + 1) * 100L / C3_MENU_COUNT));
    lv_label_set_text(c3StatusLabel, "Menu");

  } else if (c3Screen == C3_REMOTES) {
    int total = (int)remotes.size();
    if (c3Sel < 0) c3Sel = total - 1;
    if (c3Sel >= total) c3Sel = 0;
    lv_label_set_text(c3IconLabel, LV_SYMBOL_HOME);
    lv_obj_set_style_text_color(c3IconLabel, lv_color_white(), 0);
    lv_label_set_text(c3TitleLabel, remotes[c3Sel].name.c_str());
    snprintf(buf, sizeof(buf), "%d/%d  %d buttons", c3Sel + 1, total,
             (int)remotes[c3Sel].buttons.size());
    lv_label_set_text(c3SubLabel, buf);
    lv_arc_set_value(c3Arc, (int)((c3Sel + 1) * 100L / max(1, total)));
    lv_label_set_text(c3StatusLabel, "Pick remote");

  } else if (c3Screen == C3_LEARN) {
    lv_label_set_text(c3IconLabel, LV_SYMBOL_DOWNLOAD);
    lv_obj_set_style_text_color(c3IconLabel, lv_color_hex(0xffc107), 0);
    lv_label_set_text(c3TitleLabel, "Point & press");
    lv_label_set_text(c3SubLabel, teachAllActive ? "Teaching all" : "Learning one");
    lv_arc_set_value(c3Arc, 100);
    lv_label_set_text(c3StatusLabel, c3Status.c_str());

  } else if (c3Screen == C3_SCAN) {
    if (c3ScanIdx < 0) c3ScanIdx = C3_SCAN_COUNT - 1;
    if (c3ScanIdx >= C3_SCAN_COUNT) c3ScanIdx = 0;
    lv_label_set_text(c3IconLabel, LV_SYMBOL_POWER);
    lv_obj_set_style_text_color(c3IconLabel, lv_color_hex(0xff5252), 0);
    lv_label_set_text(c3TitleLabel, C3_SCAN_CODES[c3ScanIdx].brand);
    snprintf(buf, sizeof(buf), "%d/%d  press to try", c3ScanIdx + 1, C3_SCAN_COUNT);
    lv_label_set_text(c3SubLabel, buf);
    lv_arc_set_value(c3Arc, (int)((c3ScanIdx + 1) * 100L / C3_SCAN_COUNT));
    lv_label_set_text(c3StatusLabel, "IR scan");
  }
}

void c3ShowScreen(C3Screen s, int sel = 0) {
  c3Screen = s;
  c3Sel = sel;
  c3Render();
}

// Fires the currently selected blind-scan brand. Built the same way the
// web lookup path is, so it goes through the one sendCode() that already
// knows which protocols are safe to transmit.
void c3ScanBlast() {
  const ScanCode &sc = C3_SCAN_CODES[c3ScanIdx];
  IRCode code;
  code.len = 0;
  code.hash = 0;
  code.protocol = sc.protocol;
  String p = code.protocol;
  p.toUpperCase();
  if (p.indexOf("NEC") >= 0) {
    code.address = (sc.subdevice >= 0)
      ? (uint16_t)((sc.device & 0xFF) | ((sc.subdevice & 0xFF) << 8))
      : (uint16_t)(sc.device & 0xFF);
  } else {
    code.address = (uint16_t)(sc.device & 0xFFFF);
  }
  code.command = (uint16_t)(sc.function & 0xFFFF);
  sendCode(code);
  updateScreen(String("Tried ") + sc.brand);
}

void c3OnRotate(int delta) {
  if (c3Screen == C3_SCAN) c3ScanIdx += delta;
  else c3Sel += delta;
  c3Render();
}

void c3OnShortPress() {
  if (c3Screen == C3_HOME) {
    int bi = c3RealButtonIndex(c3Sel);
    if (bi < 0) return;
    if (remoteButtons[bi].codeName.length()) {
      queueCodeSend(remoteButtons[bi].codeName);
    } else {
      updateScreen("No code - learn it");
      flashLeds(CRGB::Orange, 200);
    }

  } else if (c3Screen == C3_MENU) {
    switch (c3Sel) {
      case 0: c3ShowScreen(C3_REMOTES, currentRemote); break;
      case 1: {
        int next = nextUntaughtButton(0);
        if (next < 0) { updateScreen("Nothing to teach"); c3ShowScreen(C3_HOME); }
        else { teachAllActive = true; if (startButtonLearn(next)) c3ShowScreen(C3_LEARN); }
        break;
      }
      case 2: {
        // "Learn this" means the button Home was showing - c3Sel now holds
        // the menu position, so use the saved Home index instead
        int bi = c3RealButtonIndex(c3HomeSel);
        if (bi >= 0) { teachAllActive = false; if (startButtonLearn(bi)) c3ShowScreen(C3_LEARN); }
        break;
      }
      case 3: c3ShowScreen(C3_SCAN); break;
      default: c3ShowScreen(C3_HOME); break;
    }

  } else if (c3Screen == C3_REMOTES) {
    currentRemote = c3Sel;
    ensureRemoteValid();
    markProfileDirty();
    updateScreen("Remote: " + remotes[currentRemote].name);
    c3HomeSel = 0;            // different remote, old index means nothing
    c3ShowScreen(C3_HOME);

  } else if (c3Screen == C3_SCAN) {
    c3ScanBlast();

  } else if (c3Screen == C3_LEARN) {
    // pressing during a learn cancels it, matching Cancel on the touch UI
    stopTeachAll("Learn cancelled");
    c3ShowScreen(C3_HOME);
  }
}

void c3OnLongPress() {
  if (c3Screen == C3_HOME) { c3HomeSel = c3Sel; c3ShowScreen(C3_MENU); }
  else if (c3Screen == C3_LEARN) { stopTeachAll("Learn cancelled"); c3ShowScreen(C3_HOME); }
  else c3ShowScreen(C3_HOME);
}

void c3UiInit() {
  lv_init();
  lv_disp_draw_buf_init(&c3DrawBuf, c3Buf, NULL, 240 * LV_C3_BUF_LINES);
  lv_disp_drv_init(&c3DispDrv);
  c3DispDrv.hor_res = 240;
  c3DispDrv.ver_res = 240;
  c3DispDrv.flush_cb = c3FlushCb;
  c3DispDrv.draw_buf = &c3DrawBuf;
  lv_disp_drv_register(&c3DispDrv);

  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);
  c3LastA = digitalRead(ENC_A_PIN);

  c3BuildUi();
  c3ShowScreen(C3_HOME);
}

// Polled from loop(): encoder, button, and LVGL's own timers. Nothing here
// blocks - LVGL gets a millis()-derived tick rather than a hardware timer,
// since the C3's timers are already spoken for by the IR stack.
void c3Task() {
  static unsigned long lastTick = 0;
  unsigned long now = millis();
  if (lastTick == 0) lastTick = now;
  if (now != lastTick) {
    lv_tick_inc(now - lastTick);
    lastTick = now;
  }

  // encoder: act on A's falling edge, direction from B
  int a = digitalRead(ENC_A_PIN);
  if (a != c3LastA) {
    if (a == LOW && now - c3LastEncAt > C3_ENC_MIN_GAP_MS) {
      c3LastEncAt = now;
      // if the screensaver blanked the panel, the first input only wakes
      // it - otherwise you'd be scrolling or firing a code blind. Nothing
      // else on this board could wake it, since the wake path lived in the
      // CYD touch poll.
      if (screenAsleep) {
        screenAsleep = false;
        lastActivityAt = now;
        tft.setBrightness(SCREEN_BRIGHTNESS);
      } else {
        c3OnRotate(digitalRead(ENC_B_PIN) == HIGH ? 1 : -1);
      }
    }
    c3LastA = a;
  }

  // switch: short vs long press, both firing once per physical press
  bool down = (digitalRead(ENC_SW_PIN) == LOW);
  if (down && !c3SwDown) {
    c3SwDown = true;
    c3SwPressedAt = now;
    c3SwLongFired = false;
  } else if (down && c3SwDown && !c3SwLongFired && now - c3SwPressedAt >= C3_LONG_PRESS_MS) {
    c3SwLongFired = true;      // fires while still held, so it feels immediate
    c3OnLongPress();
  } else if (!down && c3SwDown) {
    c3SwDown = false;
    if (screenAsleep) {
      screenAsleep = false;
      lastActivityAt = now;
      tft.setBrightness(SCREEN_BRIGHTNESS);
    } else if (!c3SwLongFired) {
      c3OnShortPress();
    }
  }

  lv_timer_handler();
}

#endif  // BOARD_C3KNOB

// Prints a numbered stage marker. When a board resets during boot the last
// marker printed is the last stage that completed, which turns "it reboots
// and says nothing" into a specific line of code.
static void bootMark(const char *what) {
  static int n = 0;
  Serial.printf("[boot %d] %s\n", ++n, what);
  Serial.flush();
}

void setup() {
  Serial.begin(115200);
#ifdef BOARD_C3KNOB
  // native USB CDC: give the host a moment to enumerate, otherwise the
  // first prints (including any panic during early setup) are lost
  unsigned long usbWait = millis();
  while (!Serial && millis() - usbWait < 2000) delay(10);
#endif
  delay(500);
  Serial.println();
  bootMark("serial up");

  // Armed early on both boards so a hang anywhere in setup() is caught.
  // (CYD used to defer this because the old interactive touch calibration
  // blocked here waiting for taps and tripped the watchdog mid-way. There
  // is no calibration step any more - the touch range is a measured
  // constant - so nothing in setup() blocks on a human.)
  initWatchdog();

  // sensible defaults before loadProfile() has a chance to override them
  // with anything actually saved - otherwise an un-customized slot would
  // default to CRGB(0,0,0) (global arrays zero-init), i.e. invisible/black,
  // rather than a color you can actually see
  for (int i = 0; i < NUM_SLOTS; i++) {
    slotColor[i] = DEFAULT_SLOT_COLOR;
    slotIcon[i] = ICON_NONE;
  }

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed - codes/remote layout will not persist");
  } else {
    loadProfile();
  }
  // covers a failed/absent load too - remotes must never be empty, since
  // the remoteButtons/remoteColumns accessors index into it unconditionally
  bootMark("profile loaded");
  ensureRemoteValid();

#if HAS_PHYSICAL_BUTTONS
  pinMode(LEARN_BTN_PIN, INPUT_PULLUP);
  pinMode(BLAST_BTN_PIN, INPUT_PULLUP);
#endif

  tft.init();
#if defined(BOARD_C3KNOB)
  tft.setRotation(0);            // round 240x240, no rotation needed
  SCREEN_W = tft.width();
  SCREEN_H = tft.height();
#elif defined(BOARD_CYD)
  // 0-7: 0-3 plain rotations, 4-7 the same but mirrored (backward text
  // needs a 4-7 value - see the notes in LGX_Config_CYD.h). A value saved
  // from the on-device Settings screen wins over the compile-time default,
  // so orientation can be fixed without a re-flash.
  cydLoadDisplaySettings();
  cydApplyRotation();      // also refreshes SCREEN_W/SCREEN_H from the panel

  // touch: direct XPT2046 on its own HSPI instance, no calibration step
  pinMode(T_CS, OUTPUT);
  digitalWrite(T_CS, HIGH);
  pinMode(T_IRQ, INPUT);
  touchSPI.begin(T_CLK, T_MISO, T_MOSI, T_CS);
#else
  tft.setRotation(0);
  SCREEN_W = tft.width();
  SCREEN_H = tft.height();
#endif
  bootMark("display init");
  tft.setTextWrap(false);  // all text is measured/truncated manually - never let the library wrap
  tft.setBrightness(SCREEN_BRIGHTNESS);
  tft.fillScreen(TFT_BLACK);


  updateScreen("Booting...");

  bootMark("IR init");
  IrReceiver.begin(RECV_PIN, DISABLE_LED_FEEDBACK);
  IrSender.begin(SEND_PIN);

#if HAS_WS2812
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(50);
#endif

  bootMark("wifi begin");
  WiFi.begin(ssid, password);
  updateScreen("Connecting WiFi...");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    esp_task_wdt_reset();  // this loop alone can run ~10s - feed it or risk a mid-boot reset
    delay(250);
    tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    updateScreen("WiFi connected");
    mdnsStart();
    wifiWasConnected = true;
  } else {
    updateScreen("WiFi FAILED");
    // not fatal - handleWifiWatchdog() in loop() keeps retrying in the
    // background, so the device still comes up (physical buttons/screen/IR
    // all work without WiFi) and joins the network whenever it's available
    wifiWasConnected = false;
  }

  server.on("/", handleRoot);
  server.on("/api/state", handleApiState);
  server.on("/learn", handleLearn);
  server.on("/blastcode", handleBlastCode);
  server.on("/deletecode", handleDeleteCode);
  server.on("/renamecode", handleRenameCode);
  server.on("/export", HTTP_GET, handleExport);
  server.on("/import", HTTP_POST, handleImport);
  server.on("/assignslot", handleAssignSlot);
  server.on("/setcolors", handleSetColors);
  server.on("/screensleep", handleScreenSleep);
  server.on("/remote/add", handleRemoteAdd);
  server.on("/remote/update", handleRemoteUpdate);
  server.on("/remote/delete", handleRemoteDelete);
  server.on("/remote/move", handleRemoteMove);
  server.on("/remote/reorder", handleRemoteReorder);
  server.on("/remote/columns", handleRemoteColumns);
  server.on("/remote/addmany", handleRemoteAddMany);
  server.on("/remote/learn", handleRemoteLearnButton);
  server.on("/remote/teachall", handleRemoteTeachAll);
  server.on("/remote/teachstop", handleRemoteTeachStop);
  server.on("/remotes/add", handleRemotesAdd);
  server.on("/remotes/select", handleRemotesSelect);
  server.on("/remotes/rename", handleRemotesRename);
  server.on("/remotes/delete", handleRemotesDelete);
  server.on("/trylookup", handleTryLookup);
  server.on("/savelookupcode", handleSaveLookupCode);
#ifdef BOARD_CYD
  server.on("/cyddisplay", handleCydDisplay);
#endif
  bootMark("web server up");
  server.begin();

  updateScreen("Ready");
#ifdef BOARD_CYD
  cydShowScreen(CYD_HOME);
#endif
#ifdef BOARD_C3KNOB
  bootMark("lvgl init");
  c3UiInit();
  bootMark("ui ready");
#endif
}

void loop() {
  esp_task_wdt_reset();
  server.handleClient();
  handleWifiWatchdog();
  handleMqtt();

#if HAS_PHYSICAL_BUTTONS
  handlePhysicalButtons();
#elif defined(BOARD_C3KNOB)
  c3Task();          // encoder poll + LVGL tick/timer
#else
  cydHandleTouchPoll();
#endif

  if (!suppressReceive && IrReceiver.decode()) {
    Serial.print("IR event: ");
    IrReceiver.printIRResultShort(&Serial);

    IRCode code = captureCurrent();

    if (code.len == 0) {
      // too short to be a real signal (noise/glitch) - ignore entirely,
      // including staying in "learning" mode if that's what we were doing,
      // rather than saving a useless empty code or eating the learn attempt
      Serial.println("Ignoring empty/noise capture");
    } else if (learning) {
      codeLibrary[pendingLearnName] = code;
      if (pendingLearnSlot >= 0) {
        slotCodeName[pendingLearnSlot] = pendingLearnName;
        pendingLearnSlot = -1;
      }
      // learning straight into a remote button: save and assign in one go
      if (pendingLearnRemote >= 0 && pendingLearnButton >= 0 &&
          pendingLearnRemote < (int)remotes.size() &&
          pendingLearnButton < (int)remotes[pendingLearnRemote].buttons.size()) {
        remotes[pendingLearnRemote].buttons[pendingLearnButton].codeName = pendingLearnName;
      }
      pendingLearnRemote = -1;
      pendingLearnButton = -1;
      updateScreen("Learned: " + pendingLearnName);
      markProfileDirty();
      flashLeds(CRGB::Green, 200);
      learning = false;
      pendingLearnName = "";

      // teach-all: immediately arm the next untaught button, so a whole
      // remote can be taught in one pass without touching the app between
      // presses
      bool advanced = false;
      if (teachAllActive) {
        int next = nextUntaughtButton(0);
        if (next >= 0) {
          advanced = startButtonLearn(next);
        }
        if (!advanced) {
          teachAllActive = false;
          updateScreen("Teach all: done");
        }
      }
#ifdef BOARD_CYD
      if (currentCydScreen == CYD_LEARN && !advanced) cydShowScreen(CYD_HOME);
      else if (advanced) cydDrawLearn();
#endif
#ifdef BOARD_C3KNOB
      // without this the knob UI would sit on "Point & press" forever
      // after a successful capture
      if (c3Screen == C3_LEARN && !advanced) c3ShowScreen(C3_HOME, c3HomeSel);
      else if (advanced) c3Render();
#endif
    } else {
      // not learning - just note that a signal came in
      updateScreen("Signal seen");
      flashLeds(CRGB::Orange, 150);
    }

    IrReceiver.resume();
  }

  // drain one queued send per loop iteration (see queueCodeSend) - the
  // server, receiver, and buttons all still get serviced between sends.
  // Held back entirely while learning: sendCode() briefly stops the receiver
  // to transmit, which could eat the very signal a pending learn (web,
  // physical, or MQTT-triggered) is waiting to capture. The physical blast
  // button already refuses to even queue during a learn (see
  // handlePhysicalButtons()) - this covers every other path that can queue
  // a send (web blast, remote button, MQTT command) so none of them can
  // interfere either. Sends just wait here until learning ends.
  if (!pendingSends.empty() && !learning) {
    String name = pendingSends.front();
    pendingSends.erase(pendingSends.begin());
    sendCodeByName(name);
    updateScreen("Blasted: " + name);
#if HAS_WS2812
    startTransmitWave();
#else
    flashLeds(CRGB::Blue, 150);  // no LED strip to wave - banner tint instead
#endif
  }

  // give up on a learn nobody ever finished (forgot to press the remote,
  // walked away, etc) - otherwise it blocks the queue above forever and the
  // screen/LEDs sit showing "waiting for signal" indefinitely
  if (learning && millis() - learningStartedAt > LEARN_TIMEOUT_MS) {
    Serial.println("Learn timed out waiting for a signal");
    learning = false;
    pendingLearnName = "";
    pendingLearnSlot = -1;
    pendingLearnRemote = -1;
    pendingLearnButton = -1;
    // a timeout ends the whole teach-all run - otherwise walking away
    // mid-way would leave it armed and it would grab the next stray signal
    if (teachAllActive) {
      teachAllActive = false;
      updateScreen("Teach all stopped (timeout)");
    } else {
      updateScreen("Learn timed out");
    }
#ifdef BOARD_CYD
    if (currentCydScreen == CYD_LEARN) cydShowScreen(CYD_HOME);
#endif
#ifdef BOARD_C3KNOB
    if (c3Screen == C3_LEARN) c3ShowScreen(C3_HOME, c3HomeSel);
#endif
  }

#if HAS_WS2812
  if (ledWaveActive) {
    unsigned long elapsed = millis() - ledWaveStartedAt;
    int step = elapsed / LED_WAVE_STEP_MS;
    if (step >= NUM_LEDS) {
      ledWaveActive = false;
    } else {
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      leds[step] = waveColor;
      FastLED.show();
    }
  } else if (ledFlashActive) {
    if (millis() >= ledFlashUntil) {
      ledFlashActive = false;
    }
  } else {
    static unsigned long lastLedUpdate = 0;
    if (millis() - lastLedUpdate > 30) {
      lastLedUpdate = millis();
      updateSlotLeds();
    }
  }
#elif defined(BOARD_C3KNOB)
  c3CheckFlashRevert();   // non-blocking counterpart to c3FlashAccent()
#else
  cydCheckBannerTintRevert();  // non-blocking counterpart to cydTintBanner() - see CYD screen-manager section
  // clears the tap-highlight outline once it's been visible long enough
  if (cydRedrawAt && millis() >= cydRedrawAt) {
    cydRedrawAt = 0;
    if (currentCydScreen == CYD_HOME) cydDrawHome();
    else if (currentCydScreen == CYD_SETTINGS) cydDrawSettings();
  }
  // touch test is self-limiting so a badly mis-mapped touch layer can't
  // leave you stuck on a screen with no reachable way out
  if (currentCydScreen == CYD_TOUCHTEST && cydTouchTestUntil && millis() >= cydTouchTestUntil) {
    cydTouchTestUntil = 0;
    cydShowScreen(CYD_SETTINGS);
  }
#endif

  // flush a dirty profile to flash after it's settled for a bit, off the
  // request path - see markProfileDirty()
  if (profileDirty && millis() - profileDirtyAt >= PROFILE_SAVE_DEBOUNCE_MS) {
    profileDirty = false;
    saveProfile();
  }

  // screensaver: blank the backlight after idle, only if enabled at all.
  // Any updateScreen() call (button press, blast, learn, WiFi status
  // change, ...) counts as activity and wakes it - see updateScreen(). Web
  // polling does NOT count, since /api/state alone doesn't call it.
  if (screensaverTimeoutMs && !screenAsleep &&
      millis() - lastActivityAt > screensaverTimeoutMs) {
    screenAsleep = true;
    tft.setBrightness(0);
  }

  // watch for heap fragmentation/leaks over long uptimes: getFreeHeap()
  // dropping over time points at a leak; getMinFreeHeap() staying near
  // getFreeHeap() while getMaxAllocHeap() falls well below it points at
  // fragmentation (memory is free, just not contiguous)
  static unsigned long lastHeapLog = 0;
  if (millis() - lastHeapLog > 10000) {
    lastHeapLog = millis();
    Serial.printf("[heap] free=%u minFree=%u maxAlloc=%u uptime=%lus\n",
                  ESP.getFreeHeap(), ESP.getMinFreeHeap(), ESP.getMaxAllocHeap(), millis() / 1000);
  }
}

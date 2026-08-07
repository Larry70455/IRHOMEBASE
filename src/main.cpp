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

// Board selection is set at the build-system level: platformio.ini's
// [env:cyd] passes -D BOARD_CYD, [env:esp32-s3-devkitm-1] doesn't. Every
// hardware difference between the two boards branches off this one flag
// (or HAS_WS2812 below, which follows from it) rather than a scattered set
// of ad-hoc checks.
#ifdef BOARD_CYD
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
#ifdef BOARD_CYD
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
// Panel is 240x320 portrait (see LGX_Config.h). Every piece of text below is
// measured with textWidth() and truncated to fit its box before drawing -
// deliberately not relying on the library's own word-wrap, so nothing can
// ever run off the edge of the panel regardless of exact font metrics.
const int SCREEN_W = 240;
const int SCREEN_H = 320;
const uint8_t SCREEN_BRIGHTNESS = 180;
bool screenAsleep = false;
unsigned long lastActivityAt = 0;
const unsigned long SCREENSAVER_TIMEOUT_MS = 120000;  // 2 min idle -> sleep

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

void updateScreen(String status) {
  lastStatus = status;
  lastActivityAt = millis();
  if (screenAsleep) {
    screenAsleep = false;
    tft.setBrightness(SCREEN_BRIGHTNESS);
  }

#ifdef BOARD_CYD
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
std::vector<RemoteButton> remoteButtons;
int remoteColumns = 3;

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

  JsonArray remoteArr = doc["remote"].to<JsonArray>();
  for (auto &b : remoteButtons) {
    JsonObject bObj = remoteArr.add<JsonObject>();
    bObj["name"] = b.name;
    bObj["code"] = b.codeName;
    bObj["icon"] = b.icon;
    bObj["color"] = colorToHex(b.color);
    bObj["spacer"] = b.spacer;
  }
  doc["remoteColumns"] = remoteColumns;

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

  remoteButtons.clear();
  JsonArray remoteArr = doc["remote"].as<JsonArray>();
  for (JsonVariant bv : remoteArr) {
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
    remoteButtons.push_back(b);
  }
  remoteColumns = doc["remoteColumns"] | 3;
  if (remoteColumns < 1) remoteColumns = 1;
  if (remoteColumns > 6) remoteColumns = 6;

  Serial.printf("loadProfile: loaded %u codes, %u remote buttons\n",
                (unsigned)codeLibrary.size(), (unsigned)remoteButtons.size());
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
  for (auto &b : remoteButtons) {
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
  <h2>Your remote</h2>
  <div class='item'>
    <span class='name'>Columns</span>
    <select id='remoteColumnsSel'>
      <option value='1'>1</option><option value='2'>2</option><option value='3'>3</option>
      <option value='4'>4</option><option value='5'>5</option><option value='6'>6</option>
    </select>
  </div>
  <div id='remoteGrid'></div>

  <h3>Edit buttons</h3>
  <div id='remoteList'></div>

  <h3>Add button</h3>
  <p class='empty'>Pick a symbol and color - a code is optional if you just want to lay a button out first and hook it up later.</p>
  <form class='inline' id='remoteAddForm'>
    <div class='editRow'>
      <div class='iconPreview' id='remoteAddPreview'></div>
      <select id='remoteAddIcon'></select>
      <input type='color' id='remoteAddColor'>
    </div>
    <select id='remoteAddCode'></select>
    <div class='editRow'>
      <button type='submit'>Add button</button>
      <button type='button' class='secondary' id='remoteAddSpacerBtn'>Add empty space</button>
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

  const grid = document.getElementById('remoteGrid');
  grid.innerHTML = '';
  grid.style.gridTemplateColumns = 'repeat(' + state.remoteColumns + ', 1fr)';
  if (state.remote.length === 0) {
    grid.innerHTML = "<p class='empty'>No remote buttons yet - add one below.</p>";
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
void cydShowLearnScreen();  // defined in the CYD screen-manager section further down
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
      bool hadRemoteButton = false;
      for (auto &b : remoteButtons) {
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
      for (auto &b : remoteButtons) {
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
#ifndef BOARD_CYD
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
// CYD equivalent: touch polling debounce in loop() (see BOARD_CYD section).
#ifndef BOARD_CYD
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
#endif  // !BOARD_CYD (physical control section)

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

enum CydScreen { CYD_HOME, CYD_LEARN };
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

int cydHomePage = 0;
const int CYD_COLS_MAX = 4;      // wider remoteColumns settings get clamped here for legibility - the full column count still applies on the web Remote tab
const int CYD_ROWS_PER_PAGE = 3;

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
void cydDrawHome() {
  cydZoneCount = 0;
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  cydDrawStatusBar(lastStatus);

  int cols = remoteColumns;
  if (cols < 1) cols = 1;
  if (cols > CYD_COLS_MAX) cols = CYD_COLS_MAX;
  int perPage = cols * CYD_ROWS_PER_PAGE;
  int totalPages = max(1, (int)((remoteButtons.size() + perPage - 1) / perPage));
  if (cydHomePage >= totalPages) cydHomePage = totalPages - 1;
  if (cydHomePage < 0) cydHomePage = 0;

  int gridTop = CYD_BANNER_H + 8;
  int footerH = 54;
  int gridH = SCREEN_H - gridTop - footerH - 8;
  int gap = 6;
  int cellW = (SCREEN_W - 16 - (cols - 1) * gap) / cols;
  int cellH = (gridH - (CYD_ROWS_PER_PAGE - 1) * gap) / CYD_ROWS_PER_PAGE;

  int start = cydHomePage * perPage;
  int end = min((int)remoteButtons.size(), start + perPage);
  for (int i = start; i < end; i++) {
    int slot = i - start;
    int col = slot % cols;
    int row = slot / cols;
    int x = 8 + col * (cellW + gap);
    int y = gridTop + row * (cellH + gap);
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

  if (remoteButtons.size() == 0) {
    tft.setFont(&fonts::FreeSans9pt7b);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(textdatum_t::middle_center);
    tft.drawString("No remote buttons yet -", SCREEN_W / 2, SCREEN_H / 2 - 12);
    tft.drawString("add some from the web app.", SCREEN_W / 2, SCREEN_H / 2 + 12);
    tft.setTextDatum(textdatum_t::top_left);
  }

  // footer: paging (only if needed) + Learn New
  int footerY = SCREEN_H - footerH;
  tft.setFont(&fonts::FreeSansBold9pt7b);
  tft.setTextDatum(textdatum_t::middle_center);
  int learnX = 8;
  int learnW = SCREEN_W - 16;
  if (totalPages > 1) {
    int navW = 60;
    tft.fillRoundRect(8, footerY, navW, footerH - 8, 8, TFT_DARKGREY);
    tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
    tft.drawString("<", 8 + navW / 2, footerY + (footerH - 8) / 2);
    cydAddZone(8, footerY, navW, footerH - 8, CYD_ACTION_PAGE_PREV);

    tft.fillRoundRect(SCREEN_W - 8 - navW, footerY, navW, footerH - 8, 8, TFT_DARKGREY);
    tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
    tft.drawString(">", SCREEN_W - 8 - navW / 2, footerY + (footerH - 8) / 2);
    cydAddZone(SCREEN_W - 8 - navW, footerY, navW, footerH - 8, CYD_ACTION_PAGE_NEXT);

    learnX = 8 + navW + 8;
    learnW = SCREEN_W - 16 - 2 * (navW + 8);
  }
  uint16_t accent565 = tft.color565(accentColor.r, accentColor.g, accentColor.b);
  tft.fillRoundRect(learnX, footerY, learnW, footerH - 8, 8, accent565);
  tft.setTextColor(contrastTextColor(accentColor), accent565);
  tft.drawString("Learn New", learnX + learnW / 2, footerY + (footerH - 8) / 2);
  cydAddZone(learnX, footerY, learnW, footerH - 8, CYD_ACTION_LEARN_NEW);
  tft.setTextDatum(textdatum_t::top_left);

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

  tft.setFont(&fonts::FreeSansBold12pt7b);
  tft.setTextDatum(textdatum_t::middle_center);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Point remote &", SCREEN_W / 2, SCREEN_H / 2 - 40);
  tft.drawString("press a button", SCREEN_W / 2, SCREEN_H / 2 - 10);
  tft.setFont(&fonts::FreeSans9pt7b);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Waiting for signal...", SCREEN_W / 2, SCREEN_H / 2 + 30);
  tft.setTextDatum(textdatum_t::top_left);

  int footerY = SCREEN_H - 54;
  tft.fillRoundRect(8, footerY, SCREEN_W - 16, 46, 8, TFT_MAROON);
  tft.setFont(&fonts::FreeSansBold9pt7b);
  tft.setTextDatum(textdatum_t::middle_center);
  tft.setTextColor(TFT_WHITE, TFT_MAROON);
  tft.drawString("Cancel", SCREEN_W / 2, footerY + 23);
  tft.setTextDatum(textdatum_t::top_left);
  cydAddZone(8, footerY, SCREEN_W - 16, 46, CYD_ACTION_CANCEL_LEARN);

  tft.endWrite();
}

void cydShowScreen(CydScreen s) {
  currentCydScreen = s;
  if (s == CYD_HOME) {
    cydHomePage = 0;
    cydDrawHome();
  } else {
    cydDrawLearn();
  }
}

// Thin wrapper so handleLearn() (defined earlier in the file, before
// CydScreen exists) can trigger this without needing the enum visible at
// its forward-declaration point.
void cydShowLearnScreen() { cydShowScreen(CYD_LEARN); }

// Dispatches a touch point to whichever action zone (if any) it lands in
// on the currently active screen. Reuses the exact same shared triggers
// the web app uses - queueCodeSend() for firing a code, the same
// pendingLearnName/learning/learningStartedAt setup handleLearn() uses to
// start a learn - so there's no forked logic, just a different trigger.
void cydHandleTouch(int x, int y) {
  for (int i = 0; i < cydZoneCount; i++) {
    TapZone &z = cydZones[i];
    if (x < z.x || x >= z.x + z.w || y < z.y || y >= z.y + z.h) continue;

    int action = z.action;
    if (currentCydScreen == CYD_HOME) {
      if (action == CYD_ACTION_LEARN_NEW) {
        pendingLearnName = "button_code_" + String(millis());
        pendingLearnSlot = -1;
        learning = true;
        learningStartedAt = millis();
        updateScreen("Point remote & press...");
        flashLeds(CRGB::Yellow, 150);
        cydShowScreen(CYD_LEARN);
      } else if (action == CYD_ACTION_PAGE_PREV) {
        cydHomePage--;
        cydDrawHome();
      } else if (action == CYD_ACTION_PAGE_NEXT) {
        cydHomePage++;
        cydDrawHome();
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
  int32_t tx, ty;
  if (tft.getTouch(&tx, &ty)) {
    lastCydTouchAt = millis();
    cydHandleTouch((int)tx, (int)ty);
  }
}

// Touch calibration doesn't need to run every boot - save whatever
// calibrateTouch() produces once, and load it back on subsequent boots via
// setTouchCalibrate() (LGFX's "apply already-known calibration" companion
// to calibrateTouch()) instead of re-running the interactive crosshair UI.
#define CYD_CALIB_PATH "/cyd_touch_calib.bin"

bool cydLoadTouchCalib(uint16_t *calibData) {
  if (!LittleFS.exists(CYD_CALIB_PATH)) return false;
  File f = LittleFS.open(CYD_CALIB_PATH, "r");
  if (!f) return false;
  size_t n = f.read((uint8_t*)calibData, sizeof(uint16_t) * 8);
  f.close();
  return n == sizeof(uint16_t) * 8;
}

void cydSaveTouchCalib(const uint16_t *calibData) {
  File f = LittleFS.open(CYD_CALIB_PATH, "w");
  if (!f) return;
  f.write((const uint8_t*)calibData, sizeof(uint16_t) * 8);
  f.close();
}

// /cydrecalibrate - clears the saved calibration and reboots into the
// interactive crosshair UI again. Useful after changing CYD_ROTATION (old
// calibration data doesn't necessarily still line up) or if touch just
// starts feeling off.
void handleCydRecalibrate() {
  LittleFS.remove(CYD_CALIB_PATH);
  server.send(200, "text/plain", "Calibration cleared - rebooting to recalibrate. Point at the crosshairs on the screen after it restarts.");
  delay(300);
  ESP.restart();
}

#endif  // BOARD_CYD

void setup() {
  Serial.begin(115200);
  delay(500);

#ifndef BOARD_CYD
  initWatchdog();  // arm early so a hang anywhere in setup() is also caught
#endif
  // CYD arms it further down instead, after touch calibration - see the
  // comment there for why.

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

#ifndef BOARD_CYD
  pinMode(LEARN_BTN_PIN, INPUT_PULLUP);
  pinMode(BLAST_BTN_PIN, INPUT_PULLUP);
#endif

  tft.init();
#ifdef BOARD_CYD
  tft.setRotation(CYD_ROTATION);  // adjust this constant in LGX_Config_CYD.h if the display looks mirrored/rotated wrong
#else
  tft.setRotation(0);
#endif
  tft.setTextWrap(false);  // all text is measured/truncated manually - never let the library wrap
  tft.setBrightness(SCREEN_BRIGHTNESS);
  tft.fillScreen(TFT_BLACK);

#ifdef BOARD_CYD
  // Touch calibration: load a previously-saved result if there is one:
  // interactive crosshairs (LGFX's built-in helper, not a hand-rolled
  // 2-point tap UI) only run on the very first boot, or after /cydrecalibrate
  // clears the saved file.
  //
  // The watchdog is NOT armed yet at this point (see setup()'s start) -
  // calibrateTouch() blocks waiting for real taps with no way for this code
  // to feed the watchdog while it's waiting (it's one opaque library call,
  // not a loop this code controls), and a first-time calibration taking
  // longer than the watchdog's timeout is completely normal, not a hang.
  // Arming the watchdog before this point was resetting the board mid-
  // calibration. It's armed right after instead, once every remaining step
  // in setup() is either fast or already feeds it explicitly (the WiFi
  // retry loop below does).
  uint16_t cydCalibData[8];
  if (!cydLoadTouchCalib(cydCalibData)) {
    tft.calibrateTouch(cydCalibData, TFT_WHITE, TFT_BLACK, 20);
    cydSaveTouchCalib(cydCalibData);
  } else {
    tft.setTouchCalibrate(cydCalibData);
  }
  initWatchdog();
#endif

  updateScreen("Booting...");

  IrReceiver.begin(RECV_PIN, DISABLE_LED_FEEDBACK);
  IrSender.begin(SEND_PIN);

#if HAS_WS2812
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(50);
#endif

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
  server.on("/remote/add", handleRemoteAdd);
  server.on("/remote/update", handleRemoteUpdate);
  server.on("/remote/delete", handleRemoteDelete);
  server.on("/remote/move", handleRemoteMove);
  server.on("/remote/reorder", handleRemoteReorder);
  server.on("/remote/columns", handleRemoteColumns);
  server.on("/trylookup", handleTryLookup);
  server.on("/savelookupcode", handleSaveLookupCode);
#ifdef BOARD_CYD
  server.on("/cydrecalibrate", handleCydRecalibrate);
#endif
  server.begin();

  updateScreen("Ready");
#ifdef BOARD_CYD
  cydShowScreen(CYD_HOME);
#endif
}

void loop() {
  esp_task_wdt_reset();
  server.handleClient();
  handleWifiWatchdog();
  handleMqtt();

#ifndef BOARD_CYD
  handlePhysicalButtons();
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
      updateScreen("Learned: " + pendingLearnName);
      markProfileDirty();
      flashLeds(CRGB::Green, 200);
      learning = false;
      pendingLearnName = "";
#ifdef BOARD_CYD
      if (currentCydScreen == CYD_LEARN) cydShowScreen(CYD_HOME);
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
    updateScreen("Learn timed out");
    learning = false;
    pendingLearnName = "";
    pendingLearnSlot = -1;
#ifdef BOARD_CYD
    if (currentCydScreen == CYD_LEARN) cydShowScreen(CYD_HOME);
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
#else
  cydCheckBannerTintRevert();  // non-blocking counterpart to cydTintBanner() - see CYD screen-manager section
#endif

  // flush a dirty profile to flash after it's settled for a bit, off the
  // request path - see markProfileDirty()
  if (profileDirty && millis() - profileDirtyAt >= PROFILE_SAVE_DEBOUNCE_MS) {
    profileDirty = false;
    saveProfile();
  }

  // screensaver: blank the backlight after idle. Any updateScreen() call
  // (button press, blast, learn, WiFi status change, ...) counts as
  // activity and wakes it - see updateScreen(). Web polling does NOT count,
  // since /api/state alone doesn't call updateScreen().
  if (!screenAsleep && millis() - lastActivityAt > SCREENSAVER_TIMEOUT_MS) {
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

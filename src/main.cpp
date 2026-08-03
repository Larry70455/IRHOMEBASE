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
#include "LGX_Config.h"

#define PROFILE_PATH "/profile.json"

#define RECV_PIN 5
#define SEND_PIN 6
#define LED_PIN  7
#define NUM_LEDS 8
#define NUM_SLOTS 8  // one per LED - the physical 2-button control scheme

#define LEARN_BTN_PIN 19
#define BLAST_BTN_PIN 20

const char* ssid = "Home_2g";
const char* password = "604428LS";
const char* hostname = "IR";

LGFX tft;
CRGB leds[NUM_LEDS];
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

// Unicode glyph for the web UI - purely cosmetic there (browsers render
// these natively), kept conceptually matched to the on-device vector icon.
const char* iconGlyph(uint8_t id) {
  switch (id) {
    case ICON_UP: return "\xE2\x86\x91";       // ↑
    case ICON_DOWN: return "\xE2\x86\x93";     // ↓
    case ICON_LEFT: return "\xE2\x86\x90";     // ←
    case ICON_RIGHT: return "\xE2\x86\x92";    // →
    case ICON_PLUS: return "+";
    case ICON_MINUS: return "\xE2\x88\x92";    // −
    case ICON_CHECK: return "\xE2\x9C\x93";    // ✓
    case ICON_X: return "\xE2\x9C\x95";        // ✕
    case ICON_POWER: return "\xE2\x8F\xBB";    // ⏻
    case ICON_WIFI: return "\xF0\x9F\x93\xB6"; // 📶
    case ICON_0: return "0"; case ICON_1: return "1"; case ICON_2: return "2";
    case ICON_3: return "3"; case ICON_4: return "4"; case ICON_5: return "5";
    case ICON_6: return "6"; case ICON_7: return "7"; case ICON_8: return "8";
    case ICON_9: return "9";
    default: return "";
  }
}

const char* iconName(uint8_t id) {
  switch (id) {
    case ICON_UP: return "Up"; case ICON_DOWN: return "Down";
    case ICON_LEFT: return "Left"; case ICON_RIGHT: return "Right";
    case ICON_PLUS: return "Plus"; case ICON_MINUS: return "Minus";
    case ICON_CHECK: return "Check"; case ICON_X: return "X";
    case ICON_POWER: return "Power"; case ICON_WIFI: return "WiFi";
    case ICON_0: return "0"; case ICON_1: return "1"; case ICON_2: return "2";
    case ICON_3: return "3"; case ICON_4: return "4"; case ICON_5: return "5";
    case ICON_6: return "6"; case ICON_7: return "7"; case ICON_8: return "8";
    case ICON_9: return "9";
    default: return "(none)";
  }
}

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
// than replaying a buffer. Only the NEC family is attempted with any real
// confidence right now - IRDB covers many other protocols (RC5, RC6, Sony,
// Kaseikyo/Panasonic, ...) whose exact address/command encoding I can't
// verify without hardware to test against, so those are reported as
// unsupported rather than guessed at and possibly sent wrong.
bool isProtocolSupported(const String &protocol) {
  String p = protocol;
  p.toUpperCase();
  return p.indexOf("NEC") >= 0;
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
    IrSender.sendNEC(code.address, code.command, 0);
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

void updateScreen(String status) {
  lastStatus = status;
  lastActivityAt = millis();
  if (screenAsleep) {
    screenAsleep = false;
    tft.setBrightness(SCREEN_BRIGHTNESS);
  }

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
}

// Sets the LEDs immediately and returns without blocking; loop() reverts
// them (to the idle rainbow, or just off while learning) once the requested
// duration has elapsed. Previously this used delay(), which blocked the web
// server/receiver for the flash's full duration on nearly every action.
bool ledFlashActive = false;
unsigned long ledFlashUntil = 0;

void flashLeds(CRGB color, int ms) {
  fill_solid(leds, NUM_LEDS, color);
  FastLED.show();
  ledFlashActive = true;
  ledFlashUntil = millis() + ms;
}

// Visual "signal going out" cue: lights one LED at a time, 1 through 8, in
// place of the old solid flash after a send. This plays after the actual
// (blocking) transmission has already finished - the real send is too fast
// to visualize live - but reads as "there it goes" immediately after.
bool ledWaveActive = false;
unsigned long ledWaveStartedAt = 0;
const unsigned long LED_WAVE_STEP_MS = 60;

void startTransmitWave() {
  ledFlashActive = false;  // wave takes over the strip immediately
  ledWaveActive = true;
  ledWaveStartedAt = millis();
}

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
  String name;
  String codeName;
  uint8_t icon;
  CRGB color;
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
// discovery topic tells HA to forget it. Call before a button is
// renamed/deleted so it doesn't leave a stale duplicate behind.
void mqttClearDiscovery(const String &buttonName) {
  if (!mqttClient.connected()) return;
  String deviceId = "irhomebase_" + String(hostname);
  String topic = "homeassistant/button/" + deviceId + "_" + mqttSlug(buttonName) + "/config";
  mqttClient.publish(topic.c_str(), "", true);
}

void mqttPublishDiscovery() {
  if (!mqttClient.connected()) return;
  String deviceId = "irhomebase_" + String(hostname);
  for (auto &b : remoteButtons) {
    JsonDocument doc;
    doc["name"] = b.name;
    doc["unique_id"] = deviceId + "_" + mqttSlug(b.name);
    doc["command_topic"] = mqttCmdTopic();
    doc["payload_press"] = b.codeName;
    doc["availability_topic"] = mqttStatusTopic();
    JsonObject device = doc["device"].to<JsonObject>();
    device["identifiers"][0] = deviceId;
    device["name"] = "IR Controller (" + String(hostname) + ")";
    device["manufacturer"] = "IRHOMEBASE";

    String payload;
    serializeJson(doc, payload);
    String topic = "homeassistant/button/" + deviceId + "_" + mqttSlug(b.name) + "/config";
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
    json += "\"color\":\"" + colorToHex(b.color) + "\"}";
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
  .item .name { flex:1 1 auto; min-width:70px; font-size:1rem; word-break:break-all; }
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
  .remoteTile { aspect-ratio:1.4; font-size:1.4rem; border-radius:12px; display:flex; align-items:center; justify-content:center; text-align:center; padding:6px; overflow:hidden; }
  .remoteTile .tileLabel { font-size:0.65rem; opacity:0.85; }
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
  <form class='inline' id='remoteAddForm'>
    <input type='text' id='remoteAddName' placeholder='button name' autocomplete='off'>
    <select id='remoteAddCode'></select>
    <select id='remoteAddIcon'></select>
    <input type='color' id='remoteAddColor'>
    <button type='submit'>Add</button>
  </form>
</div>

<div class='tabPanel' id='tab-physical'>
  <h2>Physical buttons</h2>
  <p class='empty'>Button 1 fires the selected slot. Button 2 cycles slots (short press) or learns into the current one (hold).</p>
  <div id='slots'></div>
</div>

<div class='tabPanel' id='tab-lookup'>
  <h2>Look up a remote</h2>
  <p class='empty'>Searches a public IR code database (probonopd/irdb) right from your phone - the device itself only handles the final "try/save" step. Sending only works reliably for NEC-family codes right now; other protocols will say so instead of guessing.</p>
  <div class='item'>
    <input type='text' id='lookupManufacturer' placeholder='Manufacturer, e.g. Samsung'>
  </div>
  <div class='item'>
    <input type='text' id='lookupDeviceType' placeholder='Device type' value='TV'>
    <button type='button' onclick='lookupSearch()'>Search</button>
  </div>
  <div id='lookupResults'></div>
  <div id='lookupFunctions'></div>
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
const ICONS = [
  {id:0, glyph:'', name:'(none)'},
  {id:1, glyph:'↑', name:'Up'},
  {id:2, glyph:'↓', name:'Down'},
  {id:3, glyph:'←', name:'Left'},
  {id:4, glyph:'→', name:'Right'},
  {id:5, glyph:'+', name:'Plus'},
  {id:6, glyph:'−', name:'Minus'},
  {id:7, glyph:'✓', name:'Check'},
  {id:8, glyph:'✕', name:'X'},
  {id:9, glyph:'⏻', name:'Power'},
  {id:10, glyph:'📶', name:'WiFi'},
  {id:11, glyph:'0', name:'0'}, {id:12, glyph:'1', name:'1'}, {id:13, glyph:'2', name:'2'},
  {id:14, glyph:'3', name:'3'}, {id:15, glyph:'4', name:'4'}, {id:16, glyph:'5', name:'5'},
  {id:17, glyph:'6', name:'6'}, {id:18, glyph:'7', name:'7'}, {id:19, glyph:'8', name:'8'},
  {id:20, glyph:'9', name:'9'}
];
function iconGlyph(id) {
  const found = ICONS.find(function (i) { return i.id === id; });
  return found ? found.glyph : '';
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
    opt.textContent = (icon.glyph ? icon.glyph + ' ' : '') + icon.name;
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
  const nameInput = document.getElementById('remoteAddName');
  const name = nameInput.value.trim();
  if (!name) return;
  mutate('/remote/add', {
    name: name,
    code: document.getElementById('remoteAddCode').value,
    icon: document.getElementById('remoteAddIcon').value,
    color: document.getElementById('remoteAddColor').value
  });
  nameInput.value = '';
});

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
    tile.className = 'remoteTile';
    tile.style.background = b.color;
    tile.style.color = '#fff';
    const glyph = iconGlyph(b.icon);
    if (glyph) {
      tile.innerHTML = '';
      const g = document.createElement('div');
      g.textContent = glyph;
      tile.appendChild(g);
      const l = document.createElement('div');
      l.className = 'tileLabel';
      l.textContent = b.name;
      tile.appendChild(l);
    } else {
      tile.textContent = b.name;
    }
    tile.addEventListener('click', function () { if (b.code) blastCode(b.code); });
    grid.appendChild(tile);
  });

  const list = document.getElementById('remoteList');
  list.innerHTML = '';
  state.remote.forEach(function (b, i) {
    const row = document.createElement('div');
    row.className = 'item';

    const nameInput = document.createElement('input');
    nameInput.type = 'text';
    nameInput.value = b.name;
    nameInput.style.flex = '1 1 100px';
    nameInput.addEventListener('change', function () { mutate('/remote/update', {index: String(i), name: nameInput.value}); });
    row.appendChild(nameInput);

    const codeSel = document.createElement('select');
    fillCodeSelect(codeSel, state.codes, b.code);
    codeSel.addEventListener('change', function () { mutate('/remote/update', {index: String(i), code: codeSel.value}); });
    row.appendChild(codeSel);

    const iconSel = document.createElement('select');
    fillIconSelect(iconSel, b.icon);
    iconSel.addEventListener('change', function () { mutate('/remote/update', {index: String(i), icon: iconSel.value}); });
    row.appendChild(iconSel);

    const colorInput = document.createElement('input');
    colorInput.type = 'color';
    colorInput.value = b.color;
    colorInput.addEventListener('change', function () { mutate('/remote/update', {index: String(i), color: colorInput.value}); });
    row.appendChild(colorInput);

    row.appendChild(makeButton('↑', 'secondary', function () { mutate('/remote/move', {index: String(i), dir: 'up'}); }));
    row.appendChild(makeButton('↓', 'secondary', function () { mutate('/remote/move', {index: String(i), dir: 'down'}); }));
    row.appendChild(makeButton('Delete', 'danger', function () {
      if (confirm("Delete '" + b.name + "'?")) mutate('/remote/delete', {index: String(i)});
    }));

    list.appendChild(row);
  });

  const addCodeSel = document.getElementById('remoteAddCode');
  if (document.activeElement !== addCodeSel) fillCodeSelect(addCodeSel, state.codes, addCodeSel.value);
}

let iconSelectsInited = false;
function renderSlots(state) {
  if (!iconSelectsInited) {
    fillIconSelect(document.getElementById('remoteAddIcon'), 0);
    iconSelectsInited = true;
  }

  const slotsDiv = document.getElementById('slots');
  slotsDiv.innerHTML = '';
  state.slots.forEach(function (slot, i) {
    const row = document.createElement('div');
    row.className = 'item' + (i === state.currentSlot ? ' selected-slot' : '');
    const label = document.createElement('span');
    label.className = 'name';
    label.textContent = 'Button ' + (i + 1);
    row.appendChild(label);

    const select = document.createElement('select');
    fillCodeSelect(select, state.codes, slot.code);
    select.addEventListener('change', function () { mutate('/assignslot', {slot: String(i), name: select.value}); });
    row.appendChild(select);

    const iconSel = document.createElement('select');
    fillIconSelect(iconSel, slot.icon);
    iconSel.addEventListener('change', function () { mutate('/assignslot', {slot: String(i), icon: iconSel.value}); });
    row.appendChild(iconSel);

    const colorInput = document.createElement('input');
    colorInput.type = 'color';
    colorInput.value = slot.color;
    colorInput.addEventListener('change', function () { mutate('/assignslot', {slot: String(i), color: colorInput.value}); });
    row.appendChild(colorInput);

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

async function lookupLoadDevice(path) {
  const funcsDiv = document.getElementById('lookupFunctions');
  funcsDiv.innerHTML = "<p class='empty'>Loading...</p>";
  try {
    const res = await fetch(irdbPathToUrl(path));
    const text = await res.text();
    const lines = text.split('\n').map(function (l) { return l.trim(); }).filter(function (l) { return l.length > 0; });
    lines.shift(); // header row
    funcsDiv.innerHTML = '';
    if (lines.length === 0) {
      funcsDiv.innerHTML = "<p class='empty'>No functions found in this file.</p>";
      return;
    }
    lines.forEach(function (line) {
      const parts = line.split(',');
      if (parts.length < 5) return;
      const funcName = parts[0], protocol = parts[1], device = parts[2], subdevice = parts[3], func = parts[4];
      const row = document.createElement('div');
      row.className = 'item';
      const label = document.createElement('span');
      label.className = 'name';
      label.textContent = funcName + ' (' + protocol + ')';
      row.appendChild(label);
      row.appendChild(makeButton('Try', '', function () { lookupTry(protocol, device, subdevice, func); }));
      const nameInput = document.createElement('input');
      nameInput.type = 'text';
      nameInput.placeholder = 'save as...';
      nameInput.value = funcName.toLowerCase().replace(/[^a-z0-9]+/g, '_');
      nameInput.style.maxWidth = '110px';
      row.appendChild(nameInput);
      row.appendChild(makeButton('Save', 'secondary', function () {
        if (nameInput.value.trim()) lookupSave(nameInput.value.trim(), protocol, device, subdevice, func);
      }));
      funcsDiv.appendChild(row);
    });
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
      for (auto &b : remoteButtons) {
        if (b.codeName == name) b.codeName = "";
      }
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
      for (auto &b : remoteButtons) {
        if (b.codeName == oldName) b.codeName = newName;
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
// /remote/add?name=X&code=Y&icon=N&color=%23rrggbb - code/icon/color optional
void handleRemoteAdd() {
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
  RemoteButton b;
  b.name = name;
  b.codeName = server.hasArg("code") ? server.arg("code") : "";
  if (b.codeName.length() > 0 && !codeLibrary.count(b.codeName)) {
    server.send(400, "text/plain", "No such code");
    return;
  }
  int icon = server.hasArg("icon") ? server.arg("icon").toInt() : (int)ICON_NONE;
  b.icon = (icon >= 0 && icon < ICON_COUNT) ? icon : ICON_NONE;
  b.color = server.hasArg("color") ? parseHexColor(server.arg("color"), DEFAULT_SLOT_COLOR) : DEFAULT_SLOT_COLOR;
  remoteButtons.push_back(b);
  updateScreen("Remote button added: " + name);
  markProfileDirty();
  mqttPublishDiscovery();
  finishRequest();
}

// /remote/update?index=N&name=X&code=Y&icon=N&color=%23rrggbb - all but
// index optional, updated independently
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
  String oldName = b.name;
  if (server.hasArg("name")) {
    String name = server.arg("name");
    name.trim();
    if (name.length() == 0) {
      server.send(400, "text/plain", "Name can't be empty");
      return;
    }
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
  if (oldName != b.name) mqttClearDiscovery(oldName);
  updateScreen("Remote button updated: " + b.name);
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
  remoteButtons.erase(remoteButtons.begin() + idx);
  mqttClearDiscovery(name);
  updateScreen("Remote button deleted: " + name);
  markProfileDirty();
  finishRequest();
}

// /remote/move?index=N&dir=up|down - swaps with its neighbor. Up/down
// buttons rather than drag-and-drop: native HTML5 drag-and-drop is
// unreliable on mobile touch browsers, which is where this app is actually
// used, so a fiddly "works on desktop, flaky on your phone" reorder isn't
// worth it for what's ultimately the same outcome.
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
  int device = server.hasArg("device") ? server.arg("device").toInt() : 0;
  int subdevice = server.hasArg("subdevice") ? server.arg("subdevice").toInt() : -1;
  int function = server.arg("function").toInt();
  // IRDB gives device+subdevice as separate bytes; for NEC-family protocols
  // that's the standard extended-NEC address layout (low byte, high byte).
  // subdevice -1 means "old" 8-bit NEC with no separate high byte.
  code.address = (subdevice >= 0) ? (uint16_t)((device & 0xFF) | ((subdevice & 0xFF) << 8))
                                   : (uint16_t)(device & 0xFF);
  code.command = (uint16_t)(function & 0xFFFF);

  if (!isProtocolSupported(code.protocol)) {
    server.send(400, "text/plain", "Protocol '" + code.protocol + "' isn't supported for sending yet - try a different profile/protocol");
    return code;
  }
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

// Idle LED display: each LED mirrors one slot. Three states so you can
// actually tell a slot is set without cycling to it: accentColor for the
// selected slot, that slot's own custom color for any other slot that has a
// code, off for a genuinely empty slot. The blink for a pending learn tracks
// pendingLearnSlot rather than currentSlot deliberately: they're normally
// the same slot, but a physical learn locks in which slot it's for the
// moment it starts (see startSlotLearn()), and if currentSlot were free to
// drift away from that slot in the meantime, the blink would follow the
// wrong LED - showing you were learning into a slot you weren't. Replaces
// the old idle rainbow now that the LEDs mean something.
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

// Debounces a pulled-up button pin: a raw reading only becomes "the truth"
// once it's held steady for DEBOUNCE_MS. Without this, mechanical contact
// bounce on press/release (a few ms of rapid HIGH/LOW chatter) can register
// as several presses and releases for what was physically one press - the
// likely cause of "weird stuff happening" on button 2 in particular, since
// its state machine tracks edges (press/release/hold), not just a level.
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

void setup() {
  Serial.begin(115200);
  delay(500);

  initWatchdog();  // arm early so a hang anywhere in setup() is also caught

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

  pinMode(LEARN_BTN_PIN, INPUT_PULLUP);
  pinMode(BLAST_BTN_PIN, INPUT_PULLUP);

  tft.init();
  tft.setRotation(0);
  tft.setTextWrap(false);  // all text is measured/truncated manually - never let the library wrap
  tft.setBrightness(SCREEN_BRIGHTNESS);
  tft.fillScreen(TFT_BLACK);
  updateScreen("Booting...");

  IrReceiver.begin(RECV_PIN, DISABLE_LED_FEEDBACK);
  IrSender.begin(SEND_PIN);

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(50);

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
  server.on("/remote/columns", handleRemoteColumns);
  server.on("/trylookup", handleTryLookup);
  server.on("/savelookupcode", handleSaveLookupCode);
  server.begin();

  updateScreen("Ready");
}

void loop() {
  esp_task_wdt_reset();
  server.handleClient();
  handleWifiWatchdog();
  handleMqtt();

  handlePhysicalButtons();

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
    startTransmitWave();
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
  }

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

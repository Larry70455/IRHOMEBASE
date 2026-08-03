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

struct IRCode {
  uint16_t data[MAX_RAW_LEN];
  uint16_t len;
  uint32_t hash;
};

std::map<String, IRCode> codeLibrary;              // name -> raw code
std::map<String, std::vector<String>> macros;      // macro name -> list of code names
std::map<uint32_t, String> triggerMap;              // hash of a learned code -> which macro to fire when heard

String pendingLearnName = "";  // if set, next captured signal gets saved under this name
bool learning = false;
unsigned long learningStartedAt = 0;
const unsigned long LEARN_TIMEOUT_MS = 30000;  // give up waiting for a signal after 30s
int pendingLearnSlot = -1;     // if >= 0, the in-progress learn (above) is for this physical slot
String lastStatus = "Booting...";

// ---------- physical slots ----------
// 8 virtual buttons, one per LED: button 1 fires whichever slot is
// currently selected, button 2 short-press cycles slots, button 2 held
// learns into the current slot. Each slot just points at a name in
// codeLibrary, so it reuses the same learn/rename/delete machinery the web
// app already has - the app can reassign a slot to any existing code too.
String slotCodeName[NUM_SLOTS];  // "" = unassigned
int currentSlot = 0;

// ---------- IR sending ----------
// Sent synchronously, on the same core/task as the receiver (setup()/loop()
// run on core 1) and the web server. IRremote's ESP32 timer-based send/receive
// is not safe to poke from another core concurrently - an earlier version of
// this sent from a separate task pinned to core 0, which manipulated the
// receiver's hardware timer (stop/restartTimer/resume) from a different core
// than it was initialized on. That's the most likely reason blasts were
// silently failing.
volatile bool suppressReceive = false;

void sendCode(const IRCode &code) {
  if (code.len == 0) {
    Serial.println("sendCode: skipping empty code (nothing captured)");
    return;
  }

  Serial.print("Sending raw code, len=");
  Serial.println(code.len);

  suppressReceive = true;
  IrReceiver.stop();

  IrSender.sendRaw(code.data, code.len, 38);

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
// task, which is what broke sending in the first place (see above). What we
// CAN avoid is blocking for an entire macro's worth of codes in one unbroken
// call: queued sends are drained one at a time from loop(), so the web
// server, IR receiver, and LEDs all still get serviced between each code in
// a multi-code macro instead of only after the whole thing finishes.
std::vector<String> pendingSends;

void queueCodeSend(const String &name) {
  pendingSends.push_back(name);
}

void queueMacro(const String &macroName) {
  auto it = macros.find(macroName);
  if (it == macros.end()) {
    Serial.println("queueMacro: no macro named '" + macroName + "'");
    return;
  }
  for (auto &codeName : it->second) {
    queueCodeSend(codeName);
  }
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

void drawSlotGrid(int gridX, int gridY, int cellW, int cellH, int gap) {
  tft.setFont(&fonts::FreeSansBold9pt7b);
  tft.setTextDatum(textdatum_t::middle_center);
  for (int i = 0; i < NUM_SLOTS; i++) {
    int col = i % 4;
    int row = i / 4;
    int cx = gridX + col * (cellW + gap);
    int cy = gridY + row * (cellH + gap);
    bool selected = (i == currentSlot);
    if (selected) {
      tft.fillRoundRect(cx, cy, cellW, cellH, 6, TFT_CYAN);
      tft.setTextColor(TFT_BLACK, TFT_CYAN);
    } else {
      tft.fillRoundRect(cx, cy, cellW, cellH, 6, TFT_BLACK);
      tft.drawRoundRect(cx, cy, cellW, cellH, 6, TFT_WHITE);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
    }
    tft.drawString(String(i + 1), cx + cellW / 2, cy + cellH / 2);
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
  tft.fillRect(0, 0, SCREEN_W, 44, TFT_CYAN);
  tft.setFont(&fonts::FreeSansBold12pt7b);
  tft.setTextDatum(textdatum_t::middle_center);
  tft.setTextColor(TFT_BLACK, TFT_CYAN);
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

// ---------- capture helper ----------
IRCode captureCurrent() {
  IRCode code;
  code.len = 0;
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

// ---------- persistence (LittleFS + JSON) ----------
// Codes/macros/triggers are saved to flash after every mutation and reloaded
// on boot, so the library survives a power cycle or re-flash.
bool saveProfile() {
  JsonDocument doc;

  JsonObject codesObj = doc["codes"].to<JsonObject>();
  for (auto &kv : codeLibrary) {
    JsonObject codeObj = codesObj[kv.first].to<JsonObject>();
    codeObj["hash"] = kv.second.hash;
    JsonArray dataArr = codeObj["data"].to<JsonArray>();
    for (uint16_t i = 0; i < kv.second.len; i++) {
      dataArr.add(kv.second.data[i]);
    }
  }

  JsonObject macrosObj = doc["macros"].to<JsonObject>();
  for (auto &kv : macros) {
    JsonArray listArr = macrosObj[kv.first].to<JsonArray>();
    for (auto &codeName : kv.second) {
      listArr.add(codeName);
    }
  }

  JsonObject triggersObj = doc["triggers"].to<JsonObject>();
  for (auto &kv : triggerMap) {
    triggersObj[String(kv.first)] = kv.second;
  }

  JsonArray slotsArr = doc["slots"].to<JsonArray>();
  for (int i = 0; i < NUM_SLOTS; i++) {
    slotsArr.add(slotCodeName[i]);
  }
  doc["currentSlot"] = currentSlot;

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
  macros.clear();
  triggerMap.clear();

  JsonObject codesObj = doc["codes"].as<JsonObject>();
  for (JsonPair kv : codesObj) {
    IRCode code;
    JsonObject codeObj = kv.value().as<JsonObject>();
    code.hash = codeObj["hash"].as<uint32_t>();
    JsonArray dataArr = codeObj["data"].as<JsonArray>();
    code.len = 0;
    for (JsonVariant v : dataArr) {
      if (code.len >= MAX_RAW_LEN) break;
      code.data[code.len++] = v.as<uint16_t>();
    }
    codeLibrary[String(kv.key().c_str())] = code;
  }

  JsonObject macrosObj = doc["macros"].as<JsonObject>();
  for (JsonPair kv : macrosObj) {
    std::vector<String> list;
    JsonArray listArr = kv.value().as<JsonArray>();
    for (JsonVariant v : listArr) {
      list.push_back(String(v.as<const char*>()));
    }
    macros[String(kv.key().c_str())] = list;
  }

  JsonObject triggersObj = doc["triggers"].as<JsonObject>();
  for (JsonPair kv : triggersObj) {
    uint32_t hash = strtoul(kv.key().c_str(), nullptr, 10);
    triggerMap[hash] = String(kv.value().as<const char*>());
  }

  for (int i = 0; i < NUM_SLOTS; i++) {
    slotCodeName[i] = "";
  }
  JsonArray slotsArr = doc["slots"].as<JsonArray>();
  int i = 0;
  for (JsonVariant v : slotsArr) {
    if (i >= NUM_SLOTS) break;
    slotCodeName[i++] = String(v.as<const char*>());
  }
  currentSlot = doc["currentSlot"] | 0;
  if (currentSlot < 0 || currentSlot >= NUM_SLOTS) currentSlot = 0;

  Serial.printf("loadProfile: loaded %u codes, %u macros, %u triggers\n",
                (unsigned)codeLibrary.size(), (unsigned)macros.size(), (unsigned)triggerMap.size());
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
// connects and costs nothing. Publishes one HA "button" entity per macro via
// MQTT discovery (shows up in Home Assistant automatically, no YAML needed)
// and listens on a single command topic for a macro or code name to fire -
// covers the roadmap's "voice control of macros via HA" with whatever
// assistant HA is already wired to.
const char* mqtt_server = "";  // e.g. "192.168.1.50" - blank disables MQTT entirely
const uint16_t mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_password = "";

WiFiClient mqttWifiClient;
PubSubClient mqttClient(mqttWifiClient);

String mqttBaseTopic() { return "irhomebase/" + String(hostname); }
String mqttStatusTopic() { return mqttBaseTopic() + "/status"; }
String mqttCmdTopic() { return mqttBaseTopic() + "/cmd"; }

// Two differently-named macros can slugify to the same string (e.g. "TV
// Power" and "TV-Power" both become "tv_power"). Since the slug becomes
// part of the HA discovery topic and unique_id, a collision would make the
// second macro's discovery publish silently overwrite the first's entity
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

// removes one macro's HA entity - an empty retained payload on its discovery
// topic tells HA to forget it. Call before a macro is renamed/deleted so it
// doesn't leave a stale duplicate behind.
void mqttClearDiscovery(const String &macroName) {
  if (!mqttClient.connected()) return;
  String deviceId = "irhomebase_" + String(hostname);
  String topic = "homeassistant/button/" + deviceId + "_" + mqttSlug(macroName) + "/config";
  mqttClient.publish(topic.c_str(), "", true);
}

void mqttPublishDiscovery() {
  if (!mqttClient.connected()) return;
  String deviceId = "irhomebase_" + String(hostname);
  for (auto &kv : macros) {
    JsonDocument doc;
    doc["name"] = kv.first;
    doc["unique_id"] = deviceId + "_" + mqttSlug(kv.first);
    doc["command_topic"] = mqttCmdTopic();
    doc["payload_press"] = kv.first;
    doc["availability_topic"] = mqttStatusTopic();
    JsonObject device = doc["device"].to<JsonObject>();
    device["identifiers"][0] = deviceId;
    device["name"] = "IR Controller (" + String(hostname) + ")";
    device["manufacturer"] = "IRHOMEBASE";

    String payload;
    serializeJson(doc, payload);
    String topic = "homeassistant/button/" + deviceId + "_" + mqttSlug(kv.first) + "/config";
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.println("MQTT command: " + msg);
  if (macros.count(msg)) {
    queueMacro(msg);
  } else if (codeLibrary.count(msg)) {
    queueCodeSend(msg);
  } else {
    Serial.println("MQTT command: no macro or code named '" + msg + "'");
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
  json.reserve(260 + 24 * (codeLibrary.size() + macros.size()));
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

  json += "\"macros\":[";
  first = true;
  for (auto &kv : macros) {
    if (!first) json += ",";
    first = false;
    json += "\"" + jsonEscape(kv.first) + "\"";
  }
  json += "],";

  json += "\"currentSlot\":" + String(currentSlot) + ",";
  json += "\"slots\":[";
  for (int i = 0; i < NUM_SLOTS; i++) {
    if (i > 0) json += ",";
    json += "\"" + jsonEscape(slotCodeName[i]) + "\"";
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
  #statusBar { position:sticky; top:8px; background:#1a7a3a; color:#fff; padding:12px; border-radius:10px; margin-bottom:14px; font-weight:600; text-align:center; }
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
  .item.selected-slot { outline:2px solid #4cf4f4; }
  form.inline { display:flex; flex-direction:column; gap:8px; background:#1c1c1c; border-radius:10px; padding:12px; }
</style>
</head>
<body>
<h1>IR Controller</h1>
<div id='statusBar'>Loading...</div>

<h2>Physical buttons</h2>
<div id='slots'></div>

<h2>Saved codes</h2>
<div id='codes'></div>

<h2>Learn new code</h2>
<form class='inline' id='learnForm'>
  <input type='text' id='learnName' placeholder='name e.g. tv_power' autocomplete='off'>
  <button type='submit'>Learn</button>
</form>

<h2>Macros</h2>
<div id='macros'></div>

<h2>New macro</h2>
<form class='inline' id='macroForm'>
  <input type='text' id='macroName' placeholder='macro name' autocomplete='off'>
  <input type='text' id='macroCodes' placeholder='code1,code2,code3' autocomplete='off'>
  <button type='submit'>Save macro</button>
</form>

<h2>Backup</h2>
<div class='item'>
  <button type='button' onclick="window.location='/export'">Download backup</button>
  <button type='button' class='secondary' onclick="document.getElementById('importFile').click()">Restore from file</button>
  <input type='file' id='importFile' accept='application/json' style='display:none'>
</div>

<script>
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
  b.textContent = label;
  if (cls) b.className = cls;
  b.addEventListener('click', onClick);
  return b;
}

function blastCode(name) { mutate('/blastcode', {name: name}); }
function deleteCode(name) { if (confirm("Delete '" + name + "'?")) mutate('/deletecode', {name: name}); }
function renameCode(oldname) {
  const n = prompt("New name for '" + oldname + "':", oldname);
  if (n) mutate('/renamecode', {oldname: oldname, newname: n});
}
function fireMacro(name) { mutate('/macro', {name: name}); }
function deleteMacro(name) { if (confirm("Delete '" + name + "'?")) mutate('/deletemacro', {name: name}); }
function renameMacro(oldname) {
  const n = prompt("New name for '" + oldname + "':", oldname);
  if (n) mutate('/renamemacro', {oldname: oldname, newname: n});
}

document.getElementById('learnForm').addEventListener('submit', function (e) {
  e.preventDefault();
  const input = document.getElementById('learnName');
  const name = input.value.trim();
  if (name) mutate('/learn', {name: name});
  input.value = '';
});

document.getElementById('macroForm').addEventListener('submit', function (e) {
  e.preventDefault();
  const nameInput = document.getElementById('macroName');
  const codesInput = document.getElementById('macroCodes');
  const name = nameInput.value.trim();
  const codes = codesInput.value.trim();
  if (name && codes) mutate('/definemacro', {name: name, codes: codes});
  nameInput.value = '';
  codesInput.value = '';
});

document.getElementById('importFile').addEventListener('change', async function (e) {
  const file = e.target.files[0];
  e.target.value = '';
  if (!file) return;
  if (!confirm('This replaces every saved code and macro with the contents of ' + file.name + '. Continue?')) return;
  const text = await file.text();
  const res = await fetch('/import', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: text });
  if (res.ok) {
    alert('Profile imported.');
    refresh();
  } else {
    alert('Import failed: ' + await res.text());
  }
});

function assignSlot(slotIndex, name) { mutate('/assignslot', {slot: String(slotIndex), name: name}); }

function render(state) {
  const bar = document.getElementById('statusBar');
  bar.textContent = state.status + (state.learning ? ' (' + state.pendingName + ')' : '');
  bar.className = state.learning ? 'learning' : '';

  const slotsDiv = document.getElementById('slots');
  slotsDiv.innerHTML = '';
  state.slots.forEach(function (name, i) {
    const row = document.createElement('div');
    row.className = 'item' + (i === state.currentSlot ? ' selected-slot' : '');
    const label = document.createElement('span');
    label.className = 'name';
    label.textContent = 'Button ' + (i + 1);
    row.appendChild(label);
    const select = document.createElement('select');
    const noneOpt = document.createElement('option');
    noneOpt.value = '';
    noneOpt.textContent = '(unassigned)';
    if (!name) noneOpt.selected = true;
    select.appendChild(noneOpt);
    state.codes.forEach(function (codeName) {
      const opt = document.createElement('option');
      opt.value = codeName;
      opt.textContent = codeName;
      if (codeName === name) opt.selected = true;
      select.appendChild(opt);
    });
    select.addEventListener('change', function () { assignSlot(i, select.value); });
    row.appendChild(select);
    slotsDiv.appendChild(row);
  });

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

  const macrosDiv = document.getElementById('macros');
  macrosDiv.innerHTML = '';
  if (state.macros.length === 0) {
    macrosDiv.innerHTML = "<p class='empty'>No macros yet.</p>";
  }
  state.macros.forEach(function (name) {
    const row = document.createElement('div');
    row.className = 'item';
    const span = document.createElement('span');
    span.className = 'name';
    span.textContent = name;
    row.appendChild(span);
    row.appendChild(makeButton('Fire', '', function () { fireMacro(name); }));
    row.appendChild(makeButton('Rename', 'secondary', function () { renameMacro(name); }));
    row.appendChild(makeButton('Delete', 'danger', function () { deleteMacro(name); }));
    macrosDiv.appendChild(row);
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
    // tears down and rebuilds the slot <select> dropdowns, which can
    // interrupt one you have open on some mobile browsers
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

void handleMacro() {
  if (server.hasArg("name")) {
    queueMacro(server.arg("name"));
  }
  finishRequest();
}

// simple endpoint to define a macro from the app, e.g.
// /definemacro?name=power_all&codes=tv_power,fan_power,soundbar_power
void handleDefineMacro() {
  if (server.hasArg("name") && server.hasArg("codes")) {
    String name = server.arg("name");
    name.trim();
    String codesStr = server.arg("codes");
    if (name.length() == 0) {
      server.send(400, "text/plain", "Name can't be empty");
      return;
    }
    bool confirmed = server.hasArg("confirm") && server.arg("confirm") == "1";
    if (macros.count(name) && !confirmed) {
      sendOverwriteConfirm("macro", "/definemacro?name=" + name + "&codes=" + codesStr, name);
      return;
    }
    std::vector<String> list;
    int start = 0;
    while (start < (int)codesStr.length()) {
      int comma = codesStr.indexOf(',', start);
      if (comma == -1) comma = codesStr.length();
      String piece = codesStr.substring(start, comma);
      piece.trim();  // "code1, code2" (space after comma) would otherwise
                      // store " code2", which never matches any real code
      if (piece.length() > 0) list.push_back(piece);
      start = comma + 1;
    }
    if (list.empty()) {
      server.send(400, "text/plain", "Macro needs at least one code");
      return;
    }
    macros[name] = list;
    updateScreen("Macro saved: " + name);
    markProfileDirty();
    mqttPublishDiscovery();
  }
  finishRequest();
}

// link a learned code as a trigger for a macro, e.g.
// /linktrigger?code=tv_power&macro=power_all
void handleLinkTrigger() {
  if (server.hasArg("code") && server.hasArg("macro")) {
    String codeName = server.arg("code");
    String macroName = server.arg("macro");
    auto it = codeLibrary.find(codeName);
    if (it != codeLibrary.end()) {
      triggerMap[it->second.hash] = macroName;
      updateScreen("Trigger linked: " + codeName);
      markProfileDirty();
    }
  }
  finishRequest();
}

// /deletecode?name=tv_power
void handleDeleteCode() {
  if (server.hasArg("name")) {
    String name = server.arg("name");
    auto it = codeLibrary.find(name);
    if (it != codeLibrary.end()) {
      uint32_t hash = it->second.hash;
      codeLibrary.erase(it);
      triggerMap.erase(hash);
      for (auto &kv : macros) {
        auto &list = kv.second;
        list.erase(std::remove(list.begin(), list.end(), name), list.end());
      }
      for (int i = 0; i < NUM_SLOTS; i++) {
        if (slotCodeName[i] == name) slotCodeName[i] = "";
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
      for (auto &kv : macros) {
        for (auto &codeName : kv.second) {
          if (codeName == oldName) codeName = newName;
        }
      }
      for (int i = 0; i < NUM_SLOTS; i++) {
        if (slotCodeName[i] == oldName) slotCodeName[i] = newName;
      }
      updateScreen("Renamed: " + oldName + " -> " + newName);
      markProfileDirty();
    }
  }
  finishRequest();
}

// /deletemacro?name=power_all
void handleDeleteMacro() {
  if (server.hasArg("name")) {
    String name = server.arg("name");
    if (macros.count(name)) {
      mqttClearDiscovery(name);
      macros.erase(name);
      for (auto it = triggerMap.begin(); it != triggerMap.end(); ) {
        if (it->second == name) it = triggerMap.erase(it);
        else ++it;
      }
      updateScreen("Deleted macro: " + name);
      markProfileDirty();
    }
  }
  finishRequest();
}

// /renamemacro?oldname=power_all&newname=movie_time
void handleRenameMacro() {
  if (server.hasArg("oldname") && server.hasArg("newname")) {
    String oldName = server.arg("oldname");
    String newName = server.arg("newname");
    newName.trim();
    auto it = macros.find(oldName);
    if (it != macros.end() && newName.length() > 0 && !macros.count(newName)) {
      mqttClearDiscovery(oldName);
      macros[newName] = it->second;
      macros.erase(it);
      for (auto &kv : triggerMap) {
        if (kv.second == oldName) kv.second = newName;
      }
      updateScreen("Renamed macro: " + oldName + " -> " + newName);
      markProfileDirty();
      mqttPublishDiscovery();
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
  String name = server.hasArg("name") ? server.arg("name") : "";
  if (name.length() > 0 && !codeLibrary.count(name)) {
    server.send(400, "text/plain", "No such code");
    return;
  }
  slotCodeName[slot] = name;
  updateScreen("Slot " + String(slot + 1) + (name.length() ? (" -> " + name) : " cleared"));
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

// Idle LED display: each LED mirrors one slot - bright cyan for the
// selected slot, off for every other slot regardless of whether it's
// assigned. The blink for a pending learn tracks pendingLearnSlot rather
// than currentSlot deliberately: they're normally the same slot, but a
// physical learn locks in which slot it's for the moment it starts (see
// startSlotLearn()), and if currentSlot were free to drift away from that
// slot in the meantime, the blink would follow the wrong LED - showing you
// were learning into a slot you weren't. Replaces the old idle rainbow now
// that the LEDs mean something.
void updateSlotLeds() {
  for (int i = 0; i < NUM_SLOTS && i < NUM_LEDS; i++) {
    CRGB color;
    if (learning && pendingLearnSlot == i) {
      bool on = (millis() / 300) % 2 == 0;
      color = on ? CRGB(80, 60, 0) : CRGB::Black;
    } else if (i == currentSlot) {
      color = CRGB(0, 90, 100);
    } else {
      color = CRGB::Black;  // unselected slots are just off, assigned or not
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

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed - codes/macros will not persist");
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
  server.on("/macro", handleMacro);
  server.on("/definemacro", handleDefineMacro);
  server.on("/linktrigger", handleLinkTrigger);
  server.on("/deletecode", handleDeleteCode);
  server.on("/renamecode", handleRenameCode);
  server.on("/deletemacro", handleDeleteMacro);
  server.on("/renamemacro", handleRenameMacro);
  server.on("/export", HTTP_GET, handleExport);
  server.on("/import", HTTP_POST, handleImport);
  server.on("/assignslot", handleAssignSlot);
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
      auto it = triggerMap.find(code.hash);
      if (it != triggerMap.end()) {
        updateScreen("Trigger matched: " + it->second);
        flashLeds(CRGB::Purple, 150);
        queueMacro(it->second);
      } else {
        updateScreen("Signal seen (no trigger match)");
        flashLeds(CRGB::Orange, 150);
      }
    }

    IrReceiver.resume();
  }

  // drain one queued send per loop iteration (see queueCodeSend/queueMacro) -
  // the server, receiver, and buttons all still get serviced between sends.
  // Held back entirely while learning: sendCode() briefly stops the receiver
  // to transmit, which could eat the very signal a pending learn (web,
  // physical, or MQTT-triggered) is waiting to capture. The physical blast
  // button already refuses to even queue during a learn (see
  // handlePhysicalButtons()) - this covers every other path that can queue
  // a send (web blast/macro, MQTT command, an auto-triggered macro) so none
  // of them can interfere either. Sends just wait here until learning ends.
  if (!pendingSends.empty() && !learning) {
    String name = pendingSends.front();
    pendingSends.erase(pendingSends.begin());
    sendCodeByName(name);
    updateScreen("Blasted: " + name);
    flashLeds(CRGB::Blue, 120);
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

  if (ledFlashActive) {
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

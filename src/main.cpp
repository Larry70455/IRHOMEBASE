#define NO_LED_FEEDBACK_CODE
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <IRremote.hpp>
#include <FastLED.h>
#include <map>
#include <vector>
#include <algorithm>
#include "LGX_Config.h"

#define RECV_PIN 5
#define SEND_PIN 6
#define LED_PIN  7
#define NUM_LEDS 8

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
String lastStatus = "Booting...";

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

void fireMacro(const String &macroName) {
  auto it = macros.find(macroName);
  if (it == macros.end()) {
    Serial.println("fireMacro: no macro named '" + macroName + "'");
    return;
  }
  for (auto &codeName : it->second) {
    sendCodeByName(codeName);
  }
}

// ---------- screen / LEDs ----------
void updateScreen(String status) {
  lastStatus = status;

  tft.fillRect(0, 0, 240, 160, TFT_BLACK);
  tft.setCursor(5, 5);
  tft.setTextSize(2);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.println("IR Controller");

  tft.setCursor(5, 35);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println(status);

  tft.setCursor(5, 60);
  tft.print("Codes saved: ");
  tft.println((int)codeLibrary.size());

  tft.setCursor(5, 85);
  tft.println(learning ? ("LEARNING: " + pendingLearnName) : "MODE: idle/listening");

  tft.setCursor(5, 110);
  tft.print("http://");
  tft.print(hostname);
  tft.println(".local");
}

void flashLeds(CRGB color, int ms) {
  fill_solid(leds, NUM_LEDS, color);
  FastLED.show();
  delay(ms);
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
  String html = "<html><body style='font-family:sans-serif'>";
  html += "<p>A " + kind + " named '" + htmlEscape(name) + "' already exists.</p>";
  html += "<p><a href='" + redirectUrl + "&confirm=1'>Overwrite it</a> | <a href='/'>Cancel</a></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleApiState() {
  // this is rebuilt on every poll (default: once a second) - reserving up
  // front avoids the repeated grow/copy/free cycle that += would otherwise
  // do on nearly every line below, which is a steady source of heap
  // fragmentation on a device that stays powered on for a long time
  String json;
  json.reserve(160 + 24 * (codeLibrary.size() + macros.size()));
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
  input[type=text] { font-size:1rem; padding:10px; border-radius:8px; border:1px solid #555; background:#222; color:#eee; width:100%; }
  form.inline { display:flex; flex-direction:column; gap:8px; background:#1c1c1c; border-radius:10px; padding:12px; }
</style>
</head>
<body>
<h1>IR Controller</h1>
<div id='statusBar'>Loading...</div>

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

<script>
function api(path, params) {
  params.ajax = '1';
  return fetch(path + '?' + new URLSearchParams(params).toString());
}

async function mutate(path, params) {
  const res = await api(path, params);
  if (res.status === 409) {
    const data = await res.json();
    if (confirm("A " + data.kind + " named '" + data.name + "' already exists. Overwrite?")) {
      params.confirm = '1';
      await api(path, params);
    }
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

function render(state) {
  const bar = document.getElementById('statusBar');
  bar.textContent = state.status + (state.learning ? ' (' + state.pendingName + ')' : '');
  bar.className = state.learning ? 'learning' : '';

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

async function refresh() {
  try {
    const res = await fetch('/api/state');
    const state = await res.json();
    render(state);
  } catch (e) { /* transient - next poll will retry */ }
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
    bool confirmed = server.hasArg("confirm") && server.arg("confirm") == "1";
    if (codeLibrary.count(name) && !confirmed) {
      sendOverwriteConfirm("code", "/learn?name=" + name, name);
      return;
    }
    pendingLearnName = name;
    learning = true;
    updateScreen("Point remote & press...");
    flashLeds(CRGB::Yellow, 150);
  }
  finishRequest();
}

void handleBlastCode() {
  if (server.hasArg("name")) {
    sendCodeByName(server.arg("name"));
    updateScreen("Blasted: " + server.arg("name"));
    flashLeds(CRGB::Blue, 120);
  }
  finishRequest();
}

void handleMacro() {
  if (server.hasArg("name")) {
    fireMacro(server.arg("name"));
    updateScreen("Macro fired: " + server.arg("name"));
    flashLeds(CRGB::Blue, 120);
  }
  finishRequest();
}

// simple endpoint to define a macro from the app, e.g.
// /definemacro?name=power_all&codes=tv_power,fan_power,soundbar_power
void handleDefineMacro() {
  if (server.hasArg("name") && server.hasArg("codes")) {
    String name = server.arg("name");
    String codesStr = server.arg("codes");
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
      list.push_back(codesStr.substring(start, comma));
      start = comma + 1;
    }
    macros[name] = list;
    updateScreen("Macro saved: " + name);
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
      updateScreen("Deleted code: " + name);
    }
  }
  finishRequest();
}

// /renamecode?oldname=tv_power&newname=tv_on
void handleRenameCode() {
  if (server.hasArg("oldname") && server.hasArg("newname")) {
    String oldName = server.arg("oldname");
    String newName = server.arg("newname");
    auto it = codeLibrary.find(oldName);
    if (it != codeLibrary.end() && newName.length() > 0 && !codeLibrary.count(newName)) {
      codeLibrary[newName] = it->second;
      codeLibrary.erase(it);
      for (auto &kv : macros) {
        for (auto &codeName : kv.second) {
          if (codeName == oldName) codeName = newName;
        }
      }
      updateScreen("Renamed: " + oldName + " -> " + newName);
    }
  }
  finishRequest();
}

// /deletemacro?name=power_all
void handleDeleteMacro() {
  if (server.hasArg("name")) {
    String name = server.arg("name");
    if (macros.erase(name)) {
      for (auto it = triggerMap.begin(); it != triggerMap.end(); ) {
        if (it->second == name) it = triggerMap.erase(it);
        else ++it;
      }
      updateScreen("Deleted macro: " + name);
    }
  }
  finishRequest();
}

// /renamemacro?oldname=power_all&newname=movie_time
void handleRenameMacro() {
  if (server.hasArg("oldname") && server.hasArg("newname")) {
    String oldName = server.arg("oldname");
    String newName = server.arg("newname");
    auto it = macros.find(oldName);
    if (it != macros.end() && newName.length() > 0 && !macros.count(newName)) {
      macros[newName] = it->second;
      macros.erase(it);
      for (auto &kv : triggerMap) {
        if (kv.second == oldName) kv.second = newName;
      }
      updateScreen("Renamed macro: " + oldName + " -> " + newName);
    }
  }
  finishRequest();
}

unsigned long lastLearnBtn = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LEARN_BTN_PIN, INPUT_PULLUP);
  pinMode(BLAST_BTN_PIN, INPUT_PULLUP);

  tft.init();
  tft.setRotation(0);
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
    delay(250);
    tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    updateScreen("WiFi connected");
    if (MDNS.begin(hostname)) {
      MDNS.addService("http", "tcp", 80);
    }
  } else {
    updateScreen("WiFi FAILED");
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
  server.begin();

  updateScreen("Ready");
}

void loop() {
  server.handleClient();

  if (digitalRead(LEARN_BTN_PIN) == LOW && millis() - lastLearnBtn > 300) {
    lastLearnBtn = millis();
    pendingLearnName = "button_code_" + String(millis());
    learning = true;
    updateScreen("Point remote & press...");
    flashLeds(CRGB::Yellow, 150);
  }

  if (!suppressReceive && IrReceiver.decode()) {
    Serial.print("IR event: ");
    IrReceiver.printIRResultShort(&Serial);

    IRCode code = captureCurrent();

    if (learning) {
      codeLibrary[pendingLearnName] = code;
      updateScreen("Learned: " + pendingLearnName);
      flashLeds(CRGB::Green, 200);
      learning = false;
      pendingLearnName = "";
    } else {
      auto it = triggerMap.find(code.hash);
      if (it != triggerMap.end()) {
        updateScreen("Trigger matched: " + it->second);
        flashLeds(CRGB::Purple, 150);
        fireMacro(it->second);
      } else {
        updateScreen("Signal seen (no trigger match)");
        flashLeds(CRGB::Orange, 150);
      }
    }

    IrReceiver.resume();
  }

  static unsigned long lastLedUpdate = 0;
  if (!learning && millis() - lastLedUpdate > 30) {
    lastLedUpdate = millis();
    static uint8_t hue = 0;
    fill_rainbow(leds, NUM_LEDS, hue++, 7);
    FastLED.show();
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

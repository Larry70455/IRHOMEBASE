#define NO_LED_FEEDBACK_CODE
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <IRremote.hpp>
#include <FastLED.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <map>
#include <vector>
#include <algorithm>
#include "LGX_Config.h"

#define PROFILE_PATH "/profile.json"

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

  File f = LittleFS.open(PROFILE_PATH, "w");
  if (!f) {
    Serial.println("saveProfile: failed to open " PROFILE_PATH " for writing");
    return false;
  }
  size_t written = serializeJson(doc, f);
  f.close();
  Serial.printf("saveProfile: wrote %u bytes\n", (unsigned)written);
  return written > 0;
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
  body.busy button { opacity:0.5; pointer-events:none; }
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
  if (pollBusy) return; // previous poll still in flight - don't stack another one
  pollBusy = true;
  try {
    const res = await fetch('/api/state');
    const state = await res.json();
    render(state);
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
    markProfileDirty();
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
      markProfileDirty();
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
    auto it = macros.find(oldName);
    if (it != macros.end() && newName.length() > 0 && !macros.count(newName)) {
      macros[newName] = it->second;
      macros.erase(it);
      for (auto &kv : triggerMap) {
        if (kv.second == oldName) kv.second = newName;
      }
      updateScreen("Renamed macro: " + oldName + " -> " + newName);
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

  File f = LittleFS.open(PROFILE_PATH, "w");
  if (!f) {
    server.send(500, "text/plain", "Failed to open profile for import");
    return;
  }
  f.print(server.arg("plain"));
  f.close();

  if (!loadProfile()) {
    server.send(400, "text/plain", "That file isn't a valid IR Controller profile");
    return;
  }
  updateScreen("Profile imported");
  server.send(200, "text/plain", "OK");
}

unsigned long lastLearnBtn = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed - codes/macros will not persist");
  } else {
    loadProfile();
  }

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
  server.on("/export", HTTP_GET, handleExport);
  server.on("/import", HTTP_POST, handleImport);
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
  // the server, receiver, and buttons all still get serviced between sends
  if (!pendingSends.empty()) {
    String name = pendingSends.front();
    pendingSends.erase(pendingSends.begin());
    sendCodeByName(name);
    updateScreen("Blasted: " + name);
    flashLeds(CRGB::Blue, 120);
  }

  if (ledFlashActive) {
    if (millis() >= ledFlashUntil) {
      ledFlashActive = false;
    }
  } else {
    static unsigned long lastLedUpdate = 0;
    if (!learning && millis() - lastLedUpdate > 30) {
      lastLedUpdate = millis();
      static uint8_t hue = 0;
      fill_rainbow(leds, NUM_LEDS, hue++, 7);
      FastLED.show();
    }
  }

  // flush a dirty profile to flash after it's settled for a bit, off the
  // request path - see markProfileDirty()
  if (profileDirty && millis() - profileDirtyAt >= PROFILE_SAVE_DEBOUNCE_MS) {
    profileDirty = false;
    saveProfile();
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

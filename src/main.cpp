#define NO_LED_FEEDBACK_CODE
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <IRremote.hpp>
#include <FastLED.h>
#include <map>
#include <vector>
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

// ---------- async send queue ----------
struct SendJob {
  IRCode code;
};

QueueHandle_t sendQueue;
TaskHandle_t senderTaskHandle;
volatile bool suppressReceive = false;

void senderTask(void* param) {
  SendJob job;
  for (;;) {
    if (xQueueReceive(sendQueue, &job, portMAX_DELAY) == pdTRUE) {
      suppressReceive = true;
      IrReceiver.stop();

      IrSender.sendRaw(job.code.data, job.code.len, 38);

      delay(40); // let the IR line settle
      IrReceiver.restartTimer();
      IrReceiver.resume();
      suppressReceive = false;
    }
  }
}

void queueSend(const IRCode &code) {
  SendJob job;
  job.code = code;
  xQueueSend(sendQueue, &job, portMAX_DELAY);
}

void queueSendByName(const String &name) {
  auto it = codeLibrary.find(name);
  if (it != codeLibrary.end()) {
    queueSend(it->second);
  }
}

void fireMacro(const String &macroName) {
  auto it = macros.find(macroName);
  if (it == macros.end()) return;
  for (auto &codeName : it->second) {
    queueSendByName(codeName);
  }
}

// ---------- screen / LEDs ----------
void updateScreen(String status) {
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
  auto *raw = IrReceiver.decodedIRData.rawDataPtr;
  code.len = 0;
  for (uint16_t i = 1; i < raw->rawlen && code.len < MAX_RAW_LEN; i++) {
    code.data[code.len++] = raw->rawbuf[i] * MICROS_PER_TICK;
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
void handleRoot() {
  String html = "<html><body style='font-family:sans-serif'>";
  html += "<h2>IR Controller</h2>";
  html += "<p>" + String(learning ? ("Learning: " + pendingLearnName) : "Idle/listening") + "</p>";

  html += "<h3>Saved codes</h3><ul>";
  for (auto &kv : codeLibrary) {
    html += "<li>" + kv.first + " <a href='/blastcode?name=" + kv.first + "'>[blast]</a></li>";
  }
  html += "</ul>";

  html += "<h3>Learn new code</h3>";
  html += "<form action='/learn' method='get'>Name: <input name='name'> <input type='submit' value='Learn'></form>";

  html += "<h3>Macros</h3><ul>";
  for (auto &kv : macros) {
    html += "<li>" + kv.first + " <a href='/macro?name=" + kv.first + "'>[fire]</a></li>";
  }
  html += "</ul>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleLearn() {
  if (server.hasArg("name")) {
    pendingLearnName = server.arg("name");
    learning = true;
    updateScreen("Point remote & press...");
    flashLeds(CRGB::Yellow, 150);
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleBlastCode() {
  if (server.hasArg("name")) {
    queueSendByName(server.arg("name"));
    updateScreen("Blasted: " + server.arg("name"));
    flashLeds(CRGB::Blue, 120);
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleMacro() {
  if (server.hasArg("name")) {
    fireMacro(server.arg("name"));
    updateScreen("Macro fired: " + server.arg("name"));
    flashLeds(CRGB::Blue, 120);
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

// simple endpoint to define a macro from the app, e.g.
// /definemacro?name=power_all&codes=tv_power,fan_power,soundbar_power
void handleDefineMacro() {
  if (server.hasArg("name") && server.hasArg("codes")) {
    String name = server.arg("name");
    String codesStr = server.arg("codes");
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
  server.sendHeader("Location", "/");
  server.send(303);
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
  server.sendHeader("Location", "/");
  server.send(303);
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
  server.on("/learn", handleLearn);
  server.on("/blastcode", handleBlastCode);
  server.on("/macro", handleMacro);
  server.on("/definemacro", handleDefineMacro);
  server.on("/linktrigger", handleLinkTrigger);
  server.begin();

  // async sender task, pinned to core 0, so core 1's loop() (receiver + web server) never blocks
  sendQueue = xQueueCreate(10, sizeof(SendJob));
  xTaskCreatePinnedToCore(senderTask, "IRSender", 4096, NULL, 1, &senderTaskHandle, 0);

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
}
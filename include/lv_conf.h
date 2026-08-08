// Minimal LVGL v8 configuration for the ESP32-C3-LCDkit build.
//
// Only the settings that differ from LVGL's defaults are listed:
// lv_conf_internal.h fills in everything else, so this stays short and the
// intent of each line is visible rather than buried in a 700-line template.
//
// Reached via -D LV_CONF_INCLUDE_SIMPLE and -I include (see platformio.ini).
#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

// GC9A01 is RGB565. LV_COLOR_16_SWAP stays 0 - LovyanGFX is handed a
// typed rgb565_t* in the flush callback and does any byte ordering itself,
// so pre-swapping here would double up and produce wrong colours.
#define LV_COLOR_DEPTH 16
#define LV_COLOR_16_SWAP 0

// The C3 has 400KB SRAM total and is also running WiFi, a web server and
// the IR stack. A partial draw buffer (see LV_C3_BUF_LINES in main.cpp)
// keeps LVGL's footprint small; a full 240x240 framebuffer would be 112KB
// on its own.
#define LV_MEM_CUSTOM 0
#define LV_MEM_SIZE (24U * 1024U)

#define LV_DISP_DEF_REFR_PERIOD 20
#define LV_INDEV_DEF_READ_PERIOD 20

// Ticks come from millis() in loop() - there is no spare hardware timer
// being dedicated to LVGL here.
#define LV_TICK_CUSTOM 0

#define LV_USE_PERF_MONITOR 0
#define LV_USE_MEM_MONITOR 0
#define LV_USE_LOG 0

// Fonts actually referenced by the C3 UI. Each one costs flash, so this is
// deliberately a short list rather than "enable everything".
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_MONTSERRAT_28 1
#define LV_FONT_MONTSERRAT_48 1
#define LV_FONT_DEFAULT &lv_font_montserrat_14

// Built-in symbol glyphs - these are what give the knob UI real icons
// (power, arrows, wifi, ok/close) without shipping image assets.
#define LV_USE_ARC 1
#define LV_USE_LABEL 1
#define LV_USE_BTN 1
#define LV_USE_LIST 1
#define LV_USE_ANIMIMG 0
#define LV_USE_CANVAS 0

// Widgets the UI never creates - off to save flash.
#define LV_USE_CHART 0
#define LV_USE_CALENDAR 0
#define LV_USE_KEYBOARD 0
#define LV_USE_TABLE 0
#define LV_USE_TEXTAREA 0
#define LV_USE_SPINBOX 0
#define LV_USE_METER 0

#endif  // LV_CONF_H

// LGFX config for the CYD ("Cheap Yellow Display", ESP32-2432S028R):
// ILI9341 TFT + XPT2046 resistive touch, each on its own SPI bus (they do
// not share physical bus lines on this board, unlike some similar-looking
// boards - touch runs on entirely separate pins from the panel). Pin
// numbers are confirmed against the community pinout reference
// (witnessmenow/ESP32-Cheap-Yellow-Display); the bus-sharing flags below
// are the one part of this file unverified against real hardware, since
// there's no CYD available to test against in this environment.
//
// That same reference project's own TFT_eSPI config uses ILI9341_2_DRIVER,
// not the plain ILI9341 driver - a known alternate init sequence some
// cheap ILI9341 clone panels need. I pulled TFT_eSPI's actual init tables
// for both drivers to see the real difference rather than guess: the "_2"
// variant sends a fixed MADCTL of 0x08 (bit3 = BGR color order, every
// other bit - row/column mirror, row/column swap - left at 0) instead of
// computing MADCTL per-rotation the way the standard driver (and
// LovyanGFX's stock Panel_ILI9341) does. Two concrete, evidence-based
// consequences:
//  1. rgb_order is BGR (true), not RGB - set below. This part is no
//     longer a guess.
//  2. That fixed 0x08 leaves MADCTL's MX/MY/MV mirror+swap bits at 0,
//     which does NOT match what LovyanGFX's stock Panel_ILI9341 assumes it
//     wrote. When the library's idea of the mirror bits disagrees with the
//     panel's actual state, two things follow together: the image comes out
//     mirrored, AND the CASET/RASET address window the library sets for a
//     fillScreen() maps to a shifted region on the panel - leaving a strip
//     that never gets written, in a fixed place, no matter the rotation.
//     Mirrored text and the unfilled strip are one bug, not two.
//
// THE FIX - CYD_ROTATION accepts 0-7, not 0-3. LovyanGFX's
// Panel_LCD::setRotation() does `r &= 7`, and bit 2 is the mirror/flip bit:
// 0-3 are the four plain rotations, 4-7 are those same four MIRRORED. A
// mirrored panel can never be corrected by 0-3 (a rotation cannot undo a
// reflection) - it just spins the mirrored image, which is exactly the
// "it rotates but text is still backwards" symptom. Values 4-7 are the
// ones that fix it. Try 4, then 5, 6, 7.
//
// CYD_ROTATION only drives tft.setRotation() at runtime (see setup() in
// main.cpp) - the panel's own offset_rotation below stays fixed at 0. They
// compose (offset_rotation is a baked-in pre-rotation, setRotation() is
// added on top), so driving both off the same constant would have doubled
// up and only reached half the possible states.
#define CYD_ROTATION 4
// Touch: same 0-7 convention. Leave this alone until the DISPLAY is
// correct - only then does a mismatch here mean anything.
#define CYD_TOUCH_ROTATION 0

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 _panel_instance;
  lgfx::Bus_SPI _bus_instance;
  lgfx::Light_PWM _light_instance;
  lgfx::Touch_XPT2046 _touch_instance;

public:
  LGFX(void) {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = VSPI_HOST;
      cfg.spi_mode = 0;
      // Backward text and a screen that's never fully filled, persisting
      // across every rotation value, doesn't fit a pure orientation/MADCTL
      // problem (rotation can't produce a mirror on its own, and a real
      // MV/transpose issue would move the unfilled region as rotation
      // changes, not leave it fixed) - dropped/corrupted bytes on a long
      // fillScreen() transfer would produce exactly this instead: same
      // symptom regardless of orientation. Backed off from 40MHz as a
      // cheap, safe thing to rule out - this can only help or do nothing,
      // never make the corruption worse.
      cfg.freq_write = 20000000;
      cfg.freq_read  = 16000000;
      cfg.spi_3wire  = true;
      cfg.use_lock   = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = 14;
      cfg.pin_mosi = 13;
      cfg.pin_miso = 12;
      cfg.pin_dc   = 2;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs           = 15;
      cfg.pin_rst          = -1;   // tied to EN on this board, no dedicated reset pin
      cfg.pin_busy         = -1;
      cfg.panel_width      = 240;
      cfg.panel_height     = 320;
      cfg.offset_x         = 0;
      cfg.offset_y         = 0;
      cfg.offset_rotation  = 0;  // fixed - CYD_ROTATION drives tft.setRotation() at runtime instead, see the comment above
      cfg.invert           = false;  // ILI9341 - unlike the other board's ST7789, no invert needed. Flip this too if colors look inverted once orientation is fixed.
      cfg.rgb_order        = true;   // this board's clone panel init sets the BGR bit - see the comment above
      _panel_instance.config(cfg);
    }

    {
      auto cfg = _light_instance.config();
      cfg.pin_bl = 21;
      cfg.invert = false;
      cfg.freq   = 44100;
      cfg.pwm_channel = 7;
      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);
    }

    {
      // Separate SPI bus from the panel above - genuinely different pins,
      // not a shared bus with a second chip-select, so bus_shared is false
      // and this gets its own spi_host.
      auto cfg = _touch_instance.config();
      cfg.x_min = 0;
      cfg.x_max = 4095;
      cfg.y_min = 0;
      cfg.y_max = 4095;
      cfg.pin_int  = 36;
      cfg.pin_sclk = 25;
      cfg.pin_mosi = 32;
      cfg.pin_miso = 39;
      cfg.pin_cs   = 33;
      cfg.spi_host = HSPI_HOST;
      cfg.freq = 1000000;
      cfg.bus_shared = false;
      cfg.offset_rotation = CYD_TOUCH_ROTATION;
      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);
    }

    setPanel(&_panel_instance);
  }
};

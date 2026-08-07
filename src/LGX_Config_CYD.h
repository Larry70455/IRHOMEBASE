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
// Both of these are only the power-on DEFAULTS now - the on-device
// Settings screen changes them live and saves the result, so neither
// normally needs editing here.
#define CYD_ROTATION 4
// Touch axis fix, applied in software by cydTransformTouch() in main.cpp
// (bit0 = invert X, bit1 = invert Y, bit2 = swap X/Y). The panel's own
// touch offset_rotation below is left at 0 so this is the single place
// that controls touch orientation - two independent corrections applied to
// the same coordinates would just fight each other. Only needed if taps
// land somewhere other than where you actually press.
#define CYD_TOUCH_ROTATION 0

// Two CYD hardware variants exist with the SAME pins but DIFFERENT display
// controllers - the reference project ships two separate display configs
// for exactly this reason:
//   * one micro-USB port only  -> ILI9341   (leave the line below commented)
//   * micro-USB AND USB-C      -> ST7789    (uncomment the line below)
// Only relevant if the display doesn't init at all; the panel-size fix
// below is what resolved the partial-update problem on this unit.
// #define CYD_PANEL_ST7789

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

class LGFX : public lgfx::LGFX_Device {
#ifdef CYD_PANEL_ST7789
  lgfx::Panel_ST7789 _panel_instance;
#else
  lgfx::Panel_ILI9341 _panel_instance;
#endif
  lgfx::Bus_SPI _bus_instance;
  lgfx::Light_PWM _light_instance;
  lgfx::Touch_XPT2046 _touch_instance;

public:
  LGFX(void) {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = VSPI_HOST;
      cfg.spi_mode = 0;
      // 20MHz rather than 40MHz. This was originally tried while chasing
      // the partial-update bug (that turned out to be the panel size, see
      // below) and made no difference to it - kept anyway because the
      // lower rate is the safer default on these clone boards.
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
      // THIS is what made part of the panel never update and the layout
      // not line up: this board's panel is 320x240, not the 240x320 that
      // an ILI9341 datasheet leads you to assume. Confirmed on hardware -
      // swapping these two numbers made everything line up. Do not "fix"
      // this back to 240x320.
      cfg.panel_width      = 320;
      cfg.panel_height     = 240;
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
      cfg.offset_rotation = 0;  // fixed - touch orientation is handled by cydTransformTouch() in main.cpp, see above
      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);
    }

    setPanel(&_panel_instance);
  }
};

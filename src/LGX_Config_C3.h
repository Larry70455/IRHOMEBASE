// LGFX config for the ESP32-C3-LCDkit: 1.28" round GC9A01, 240x240, SPI.
//
// Pins are from Espressif's own user guide for the board:
//   CS IO7, CLK IO1, MOSI IO0, D/C IO2, backlight IO5
// There is no MISO on this panel and no touch controller - the rotary
// encoder (IO10/IO6, switch IO9) is the entire input device.
//
// Note the display is ROUND. Anything drawn into the corners of the
// 240x240 square is simply not visible, so the UI built on top of this is
// laid out radially rather than as a rectangular grid.
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_GC9A01 _panel_instance;
  lgfx::Bus_SPI      _bus_instance;
  lgfx::Light_PWM    _light_instance;

public:
  LGFX(void) {
    {
      auto cfg = _bus_instance.config();
      // The C3 has a single general-purpose SPI peripheral available to
      // sketches; SPI2_HOST is it (SPI0/1 are tied up with flash).
      cfg.spi_host   = SPI2_HOST;
      cfg.spi_mode   = 0;
      cfg.freq_write = 40000000;
      cfg.freq_read  = 16000000;
      cfg.spi_3wire  = true;
      cfg.use_lock   = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = 1;
      cfg.pin_mosi = 0;
      cfg.pin_miso = -1;   // panel is write-only on this board
      cfg.pin_dc   = 2;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs          = 7;
      cfg.pin_rst         = -1;   // tied to the module reset, no dedicated GPIO
      cfg.pin_busy        = -1;
      cfg.panel_width     = 240;
      cfg.panel_height    = 240;
      cfg.offset_x        = 0;
      cfg.offset_y        = 0;
      cfg.offset_rotation = 0;
      cfg.invert          = true;   // GC9A01 panels are normally inverted
      cfg.rgb_order       = false;
      _panel_instance.config(cfg);
    }

    {
      auto cfg = _light_instance.config();
      cfg.pin_bl = 5;
      cfg.invert = false;
      cfg.freq   = 44100;
      cfg.pwm_channel = 0;   // C3 has fewer LEDC channels than the S3
      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);
    }

    setPanel(&_panel_instance);
  }
};

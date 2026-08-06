// LGFX config for the CYD ("Cheap Yellow Display", ESP32-2432S028R):
// ILI9341 TFT + XPT2046 resistive touch, each on its own SPI bus (they do
// not share physical bus lines on this board, unlike some similar-looking
// boards - touch runs on entirely separate pins from the panel). Pin
// numbers are confirmed against the community pinout reference
// (witnessmenow/ESP32-Cheap-Yellow-Display); the bus-sharing flags below
// are the one part of this file unverified against real hardware, since
// there's no CYD available to test against in this environment.
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
      cfg.freq_write = 40000000;
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
      cfg.offset_rotation  = 0;
      cfg.invert           = false;  // ILI9341 - unlike the other board's ST7789, no invert needed
      cfg.rgb_order        = false;
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
      cfg.offset_rotation = 0;
      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);
    }

    setPanel(&_panel_instance);
  }
};

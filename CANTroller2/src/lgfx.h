#pragma once
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
// Custom settings for ESP32 LovyanGFX library - all chinese commentary has been translated
// The original setting is the same as the original setting, and the LGFX_Device is a derivative of the original creation.

// Soren notes from Espressif site:  The SPI Master driver allows multiple Devices to be connected on a same SPI bus (sharing 
// a single ESP32 SPI peripheral). As long as each Device is accessed by only one task, the driver is thread-safe. However, if
// multiple tasks try to access the same SPI Device, the driver is not thread-safe. In this case, it is recommended to either:
// 1. Refactor your application so that each SPI peripheral is only accessed by a single task at a time. You can use 
// spi_bus_config_t::isr_cpu_id to register the SPI ISR to the same core as SPI peripheral-related tasks to ensure thread safety.
// 2. Add a mutex lock around the shared Device using xSemaphoreCreateMutex.

// You can change the class name from "LGFX" to another name.
// When used together with AUTODETECT, "LGFX" is already in use, so change it to a name other than LGFX.
// Also, if you use multiple panels at the same time, give each one a different name.
// If you change the class name, you must also change the constructor name to the same name.
// You can choose the name as you like, but in case the number of settings increases,
// For example, if you configure ILI9341 with SPI connection using ESP32 DevKit-C, by using a name
// like "LGFX_DevKitC_SPI_ILI9341" with a matching filename and class name, it'll avoid confusion.
class LGFX : public lgfx::LGFX_Device {
    lgfx::Panel_ILI9341     _panel_instance;     // Prepare an instance that matches the type of panel you want to connect.
    lgfx::Bus_SPI           _bus_instance;       // SPI bus instance // Prepare an instance that matches the type of bus that connects the panel.
    // lgfx::Bus_I2C        _bus_instance;       // I2C bus instance
    // lgfx::Light_PWM      _light_instance;     // Prepare an instance if backlight control is possible. (Delete if unnecessary)
    lgfx::Touch_FT5x06      _cap_touch_instance; // FT5206, FT5306, FT5406, FT6206, FT6236, FT6336, FT6436
    lgfx::Touch_XPT2046     _res_touch_instance;
  public:
    LGFX(void) {}  // Create a constructor and configure various settings here.  If you change the class name, please specify the same name for the constructor.
    void init() {
        {  // Configure bus control settings.
            auto cfg = _bus_instance.config(); // Get the structure for bus settings.
            cfg.spi_host = SPI2_HOST;          // VSPI_HOST is deprecated, use SPI2_HOST.  HSPI_HOST = SPI1_HOST or SPI3_HOST (?)
            cfg.spi_mode = 0;                  // Set SPI communication mode (0 ~ 3)
            cfg.freq_write = 40000000;         // SPI clock when transmitting (maximum 80MHz, rounded to 80MHz divided by an integer)
            cfg.freq_read  = 16000000;         // SPI clock when receiving
            cfg.spi_3wire  = true;             // Set true if receiving is done using the MOSI pin.
            cfg.use_lock   = true;             // Set true to use transaction locking
            cfg.dma_channel = SPI_DMA_CH_AUTO; // Set the DMA channel (0=No DMA / 1=1ch / 2=ch / SPI_DMA_CH_AUTO=automatic setting)  // Due to the ESP-IDF version upgrade, SPI_DMA_CH_AUTO is recommended for the DMA channel. 1ch or 2ch is not recommended.
            cfg.pin_sclk = spi_sclk_pin;        // SCLK pin number
            cfg.pin_mosi = spi_mosi_pin;        // MOSI pin number
            cfg.pin_miso = spi_miso_pin;        // MISO pin number
            cfg.pin_dc   = tft_dc_pin;                  // Set SPI D/C pin number (-1 = disable)
            // // I2C bus settings
            // cfg.i2c_port    = 0;       // Select I2C port to use (0 or 1)
            // cfg.freq_write  = 400000;  // Clock when transmitting
            // cfg.freq_read   = 400000;  // Clock when receiving
            // cfg.pin_sda     = 8;       // SDA pin number
            // cfg.pin_scl     = 9;       // SCL pin number
            // cfg.i2c_addr    = 0x38;    // I2C device address
            _bus_instance.config(cfg);         // Reflects the setting value on the bus.
            _panel_instance.setBus(&_bus_instance); // Place the bus on the panel.
        }
        {  // Configure display panel control settings.
            auto cfg = _panel_instance.config();    // Gets the structure for display panel settings.
            cfg.pin_cs   = tft_cs_pin;        //   CS pin number
            // cfg.pin_cs           =    10;  // CS pin number   (-1 = disable)
            cfg.pin_rst      = tft_rst_pin;  // RST pin number  (-1 = disable)
            cfg.pin_busy         =    -1;  // BUSY pin number (-1 = disable)
            cfg.panel_width      =   240;  // Actual displayable width, default = 240
            cfg.panel_height     =   320;  // Actual display height, default = 320
            cfg.offset_rotation  =     8;  // Offset of value in rotation direction 0~7 (4~7 are upside down), default = 1
            cfg.rgb_order        = false;  // Set to true if the red and blue colors of the panel are swapped, default = false
            cfg.offset_x         =     0;  // Panel X direction offset amount
            cfg.offset_y         =     0;  // Panel Y direction offset amount
            cfg.dummy_read_pixel =     8;  // Number of dummy read bits before pixel readout
            cfg.dummy_read_bits  =     1;  // Number of bits for dummy read before reading data other than pixels
            cfg.readable         =  true;  // Set to true if data reading is possible            
            cfg.invert      = (captouch);  // Set to true if the brightness and darkness of the panel is reversed.
            cfg.dlen_16bit       = false;  // Set to true for panels that transmit data length in 16-bit units using 16-bit parallel or SPI.
            cfg.bus_shared       =  true;  // Set to true when sharing the bus with the SD card (control the bus using drawJpgFile, etc.)
            // Please set the following only if the display is misaligned with a variable pixel number driver such as ST7735 or ILI9163.
            // cfg.memory_width  =   240;  // Maximum width supported by driver IC
            // cfg.memory_height =   320;  // Maximum height supported by driver IC
            _panel_instance.config(cfg);
        }
        if (captouch) {  // Configure touch screen control settings. (Delete if unnecessary)
            auto cfg = _cap_touch_instance.config();
            cfg.x_min      = 0;       // Minimum X value obtained from touch screen (raw value)
            cfg.x_max      = 239;     // Maximum X value obtained from touch screen (raw value)
            cfg.y_min      = 0;       // Minimum Y value obtained from touch screen (raw value)
            cfg.y_max      = 319;     // Maximum Y value obtained from touch screen (raw value)
            // cfg.pin_int    = touch_irq_pin;      // INT pin number
            cfg.offset_rotation = 2;  // Adjustment when the display and touch direction do not match. Set as a value from 0 to 7
            cfg.bus_shared = true;   // Set true if using the same bus as the screen
            cfg.i2c_port   = 0;       // Select I2C to use (0 or 1)
            cfg.i2c_addr   = 0x38;    // I2C device address number
            cfg.pin_sda    = i2c_sda_pin;       // SDA pin number
            cfg.pin_scl    = i2c_scl_pin;       // SCL pin number
            cfg.freq       = 400000;  // Set I2C clock
            _cap_touch_instance.config(cfg);
            _panel_instance.setTouch(&_cap_touch_instance);  // Place the touch screen on the panel
        }
        else {  // resistive touch instead
            auto cfg = _res_touch_instance.config();
            cfg.x_min      = 0;       // Minimum X value obtained from touch screen (raw value)
            cfg.x_max      = 239;     // Maximum X value obtained from touch screen (raw value)
            cfg.y_min      = 0;       // Minimum Y value obtained from touch screen (raw value)
            cfg.y_max      = 319;     // Maximum Y value obtained from touch screen (raw value)
            cfg.pin_int  = touch_irq_pin;      // INT pin number
            cfg.offset_rotation = 2;  // Adjustment when the display and touch direction do not match. Set as a value from 0 to 7
            cfg.bus_shared = true;    // Set true if using the same bus as the screen
            cfg.spi_host = SPI2_HOST; // VSPI_HOST (doesn't recognize?) Select the SPI to use (HSPI_HOST or VSPI_HOST)
            cfg.freq = 1000000;       // Set SPI clock
            cfg.pin_sclk = spi_sclk_pin;        // SCLK pin number
            cfg.pin_mosi = spi_mosi_pin;        // MOSI pin number
            cfg.pin_miso = spi_miso_pin;        // MISO pin number
            cfg.pin_cs   = touch_cs_pin;        //   CS pin number
            // cfg.pin_sclk = 12;        // SCLK pin number
            // cfg.pin_mosi = 11;        // MOSI pin number
            // cfg.pin_miso = 13;        // MISO pin number
            // cfg.pin_cs   = 47;        //   CS pin number
            _res_touch_instance.config(cfg);
            _panel_instance.setTouch(&_res_touch_instance);  // Place the touch screen on the panel.
        }
        // {  // Configure backlight control settings. (Delete if unnecessary)
        //     auto cfg = _light_instance.config();    // Gets the structure for backlight settings.
        //     cfg.pin_bl = 32;            // Backlight pin number
        //     cfg.invert = false;         // true to invert the backlight brightness
        //     cfg.freq   = 44100;         // Backlight PWM frequency
        //     cfg.pwm_channel = 7;        // PWM channel number to use
        //     _light_instance.config(cfg);
        //     _panel_instance.setLight(&_light_instance);  // Set the backlight on the panel.
        // }
        setPanel(&_panel_instance);        // Set the panel to be used.
    }
};
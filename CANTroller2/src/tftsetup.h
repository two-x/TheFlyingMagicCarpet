#pragma once
// This is a setup file from the TFT_eSPI library, with edits for our setup.
// Whenever the library is first installed or updated, to make it include this you need
// to edit the following file (within the library package):
// <project-dir>/.pio/libdeps/esp32-s3-devkitc-1-n8/TFT_eSPI/User_Setup_Select.h
//
// Make this edit:
// #include <User_Setup.h>                // Replace this line 
// #include <../../../../src/tftsetup.h>  // with this one

// User defined information reported by "Read_User_Setup" test & diagnostics example
#define USER_SETUP_INFO "FlyingCarpet"

// Define to disable all #warnings in library (can be put in User_Setup_Select.h)
//#define DISABLE_ALL_LIBRARY_WARNINGS

// Section 1. Call up the right driver file - only one - and any options for it
#define ILI9341_DRIVER       // Generic driver for common displays

// Some displays support SPI reads via the MISO pin, other displays have a single SDA
// #define TFT_SDA_READ      // This option is for ESP32 ONLY, tested with ST7789 and GC9A01 display only

// Section 2. Define the pins that are used to interface with the display here
// If a backlight control signal is available then define the TFT_BL pin in Section 2
// below. The backlight will be turned ON when tft.begin() is called, but the library
// needs to know if the LEDs are ON with the pin HIGH or LOW. If the LEDs are to be
// driven with a PWM signal or turned OFF/ON then this must be handled by user code
// #define TFT_BL   32            // LED back-light control pin
// #define TFT_BACKLIGHT_ON HIGH  // Level to turn ON back-light (HIGH or LOW)

// ###### EDIT THE PIN NUMBERS IN THE LINES FOLLOWING TO SUIT YOUR ESP32 SETUP   ######
// For ESP32 Dev board (only tested with ILI9341 display). The hardware SPI can be mapped to any pins
#define TFT_MISO 13
#define TFT_MOSI 11
#define TFT_SCLK 12
#define TFT_CS   10  // Chip select control pin
#define TFT_DC    3  // Data Command control pin
// #define TFT_RST   4  // Reset pin (could connect to RST pin)
#define TFT_RST  -1  // Set TFT_RST to -1 if display RESET is connected to ESP32 board RST
#define TOUCH_CS 39     // Chip select pin (T_CS) of touch screen
//#define TFT_WR 22    // Write strobe for modified Raspberry Pi TFT only

// The ESP32 and TFT the pins used for testing are:
//#define TFT_CS   33  // Chip select control pin (library pulls permanently low
//#define TFT_DC   15  // Data Command control pin - must use a pin in the range 0-31
//#define TFT_RST  32  // Reset pin, toggles on startup
//#define TFT_WR    4  // Write strobe control pin - must use a pin in the range 0-31
//#define TFT_RD    2  // Read strobe control pin
//#define TFT_D0   12  // Must use pins in the range 0-31 for the data bus
//#define TFT_D1   13  // so a single register write sets/clears all bits.
//#define TFT_D2   26  // Pins can be randomly assigned, this does not affect
//#define TFT_D3   25  // TFT screen update performance.
//#define TFT_D4   17
//#define TFT_D5   16
//#define TFT_D6   27
//#define TFT_D7   14

// Section 3. Define the fonts that are to be used here
// Comment out the #defines below with // to stop that font being loaded
// The ESP8366 and ESP32 have plenty of memory so commenting out fonts is not
// normally necessary. If all fonts are loaded the extra FLASH space required is
// about 17Kbytes. To save FLASH space only enable the fonts you need!

#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2  // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
#define LOAD_FONT4  // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters
#define LOAD_FONT6  // Font 6. Large 48 pixel font, needs ~2666 bytes in FLASH, only characters 1234567890:-.apm
#define LOAD_FONT7  // Font 7. 7 segment 48 pixel font, needs ~2438 bytes in FLASH, only characters 1234567890:-.
#define LOAD_FONT8  // Font 8. Large 75 pixel font needs ~3256 bytes in FLASH, only characters 1234567890:-.
//#define LOAD_FONT8N // Font 8. Alternative to Font 8 above, slightly narrower, so 3 digits fit a 160 pixel TFT
#define LOAD_GFXFF  // FreeFonts. Include access to the 48 Adafruit_GFX free fonts FF1 to FF48 and custom fonts
// Comment out the #define below to stop the SPIFFS filing system and smooth font code being loaded - saves ~20kbytes FLASH
#define SMOOTH_FONT

// Section 4. Other options
// Define the SPI clock frequency, this affects the graphics rendering speed. Too
// fast and the TFT driver will not keep up and display corruption appears.
// With an ILI9341 display 40MHz works OK, 80MHz sometimes fails
// #define SPI_FREQUENCY   1000000
// #define SPI_FREQUENCY   5000000
// #define SPI_FREQUENCY  10000000
// #define SPI_FREQUENCY  20000000
//#define SPI_FREQUENCY  27000000  // Soren: This was the default setting
// #define SPI_FREQUENCY  40000000
// #define SPI_FREQUENCY  55000000 // STM32 SPI1 only (SPI2 maximum is 27MHz)
// #define SPI_FREQUENCY  80000000
#define SPI_FREQUENCY  60000000  // Soren: I invented this one, b/c 80MHz didn't work. This runs screen saver 20-25% faster!

// Optional reduced SPI frequency for reading TFT
#define SPI_READ_FREQUENCY  20000000

// The XPT2046 requires a lower SPI clock rate of 2.5MHz so we define that here:
#define SPI_TOUCH_FREQUENCY  2500000

// The ESP32 has 2 free SPI ports i.e. VSPI and HSPI, the VSPI is the default.
// If the VSPI port is in use and pins are not accessible (e.g. TTGO T-Beam)
// then uncomment the following line:
//#define USE_HSPI_PORT

// Comment out the following #define if "SPI Transactions" do not need to be
// supported. When commented out the code size will be smaller and sketches will
// run slightly faster, so leave it commented out unless you need it!
// Transaction support is needed to work with SD library but not needed with TFT_SdFat
// Transaction support is required if other SPI devices are connected.
// Transactions are automatically enabled by the library for an ESP32 (to use HAL mutex)
// so changing it here has no effect
// #define SUPPORT_TRANSACTIONS
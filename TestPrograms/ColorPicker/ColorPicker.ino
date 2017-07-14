/* ColorPicker.ino
 *
 * A color picker program for playing with colors and palettes of colors.
 * Can be used to define a range of color palettes that can be switched
 * between using a potentiometer.
 *
 * The user defines two pallets, a sky (or high) and a ground (or low). The
 * potentiometer selects an index from 0-255 and selects the corresponding values
 * from both palettes. It then creates a new pallet using those colors as the
 * endpoints and displays that. This allows us to play around with blending between
 * different color schemes so that all our transitions look boss as fuck.
 *
 * Author: Anders Linn
 * Date: July 2017
 */

#include "CRGBW.h"

namespace ColorPicker {

#define NUM_COLOR_PICKER_LEDS 20
#define NUM_RAW_COLOR_PICKER_LEDS crgbw2crgb( NUM_COLOR_PICKER_LEDS )
#define COLOR_PICKER_DATA_PIN 2

/* Make your changes to the original color palettes here. The first value in each
 * row is the index at which the color will appear (from 0-255). The other values are
 * RGB values, in that order as usual.
 */

DEFINE_GRADIENT_PALETTE( skymap ) {
     0,     0,    255,  255,
     128,   0x0e, 0x19, 0x58,
     255,   0,    204,  0 };

DEFINE_GRADIENT_PALETTE( groundmap ) {
     0,     255,    204,  102,
     128,   0x42, 0x32, 0x51,
     255,   153,    0,  51 };

CRGBPalette16 skyPalette = skymap;
CRGBPalette16 groundPalette = groundmap;

// pointer to rgbw string
CRGBW * leds;

// this initializes the CRGBW string
static CRGBW * colorDemoLeds() {
   static CRGBW * leds = new CRGBW[ NUM_COLOR_PICKER_LEDS ];
   return leds;
}

void colorDemoSetup() {
   leds = colorDemoLeds();

   // Here we set up the neopixel string and set it to flash red on startup
   FastLED.addLeds<NEOPIXEL, COLOR_PICKER_DATA_PIN>( leds, NUM_COLOR_DEMO_LEDS );
   for ( int i = 0; i < NUM_COLOR_PICKER_LEDS; ++i ) {
      leds[ i ] = CRGB::Red;
   }
   FastLED.setBrightness( 255 );
   FastLED.show();
   FastLED.delay( 1000 );
}

void colorDemoLoop() {
   static long drawTock = millis();
   long tick = millis();
   if ( tick > drawTock + 40 ) {
      CRGB sky = ColorFromPalette( skyPalette, phase );
      CRGB ground = ColorFromPalette( groundPalette, phase );
      CRGBPalette16 palette( sky, ground, sky );
      for ( uint8_t i = 0; i < NUM_COLOR_PICKER_LEDS; ++i ) {
         uint16_t colorIdx = ( i * 255 ) / NUM_COLOR_PICKER_LEDS;
         auto clr = ColorFromPalette( palette, colorIdx );
         leds[ i ] = gammaCorrect( clr );
      }
      drawTock = tick;
   }
   FastLED.show();
}

} // end namespace ColorPicker

void setup() {
   ColorPicker::colorPickerSetup();
}

void loop() {
   ColorPicker::colorPickerLoop();
}


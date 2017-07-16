/* ColorDemo.ino
 *
 * Playground for messing around with FastLED color palettes using a RGB neopixel
 * shield.
 */

#ifndef __FASTLED_H
#define __FASTLED_H
#include <FastLED.h>
#endif

#include "CRGBW.h"
#define NUM_COLOR_DEMO_LEDS 10
#define COLOR_DEMO_DATA_PIN 52

// pointer to rgb led shield
CRGB * leds;

static CRGB * colorDemoLeds() {
   static CRGB colorLeds[ NUM_COLOR_DEMO_LEDS ];
   return colorLeds;
}

void colorDemoSetup() {
   leds = colorDemoLeds();
   FastLED.addLeds<NEOPIXEL, COLOR_DEMO_DATA_PIN>( leds, NUM_COLOR_DEMO_LEDS );
   for ( int i = 0; i < NUM_COLOR_DEMO_LEDS; ++i ) {
      leds[ i ] = CRGB::Red;
   }
   FastLED.setBrightness( 55 );
   FastLED.setDither( true );
   FastLED.setCorrection( TypicalLEDStrip );
   FastLED.show();
   FastLED.delay( 1000 );
}

void colorDemoLoop() {
   static uint8_t location = 0;
   static uint8_t phase = 0;
   static bool phaseDirection = true;
   static uint8_t stepSize = 255 / NUM_COLOR_DEMO_LEDS;
   // TODO: set the color palette based on analog input
   static long drawTock = millis();
   static long locationTock = millis();
   long tick = millis();
   if ( tick > drawTock + 40 ) {
      CRGBPalette16 skyPalette( CRGB( 0, 255, 255 ), CRGB( 0x0E1958 ), CRGB( 0, 204, 0 ) );
      CRGBPalette16 groundPalette( CRGB( 255, 204, 102 ), CRGB( 0x423251 ), CRGB( 153, 0, 51 ) );
      CRGB sky = ColorFromPalette( skyPalette, phase );
      CRGB ground = ColorFromPalette( groundPalette, phase );
      CRGBPalette16 palette( sky, ground, sky );
      uint8_t colorIndex = 0;
      for ( uint8_t i = 0; i < NUM_COLOR_DEMO_LEDS; ++i ) {
         // ColorFromPalette wraps around in some circumstances, gotta fix that
         uint8_t idx = ( location + i ) % NUM_COLOR_DEMO_LEDS;
         auto clr = ColorFromPalette( palette, colorIndex );
         leds[ idx ] = LedUtil::gammaCorrect( clr );
         // update this to use a sine function for faster transfers
         colorIndex += stepSize;
      }
      if ( phaseDirection ) {
         if ( ++phase == 240 ) {
            phaseDirection = false;
         }
      } else {
         if ( --phase == 30 ) {
            phaseDirection = true;
         }
      }
      drawTock = tick;
   }
   if ( tick > locationTock + 180 ) {
      ++location;
      locationTock = tick;
   }
   FastLED.show();
}

void setup() {
   colorDemoSetup();
}

void loop() {
   colorDemoLoop();
}

/* CarpetLedLogic.ino
 *
 *   This is the main entry point for the carpet's LED system.
 *
 *   Author: Anders Linn
 *   Date: June 2017
 */

#ifndef __DMXSIMPLE_H
#define __DMXSIMPLE_H
#include <DmxSimple.h>
#endif

#ifndef __FASTLED_H
#define __FASTLED_H
#include <FastLED.h>
#endif

#include "CRGBW.h"
#include "MagicCarpet.h"
#include "LegacyRoutines.h"

/* Rope demo
 *
 * Basic shooting lights demo to test the full carpet rope led setup.
 */

namespace RopeDemo {

MagicCarpet * carpet;

void ropeDemoSetup() {
   carpet = theMagicCarpet();
   carpet->setup();
   for ( int i = 0; i < NUM_NEO_LEDS; ++i ) {
     carpet->ropeLeds[i] = CRGB::Red;
     carpet->ropeLeds[i].w = 0x0;
   }
   carpet->show();
   FastLED.delay( 1000 );
   for ( int i = 0; i < NUM_NEO_LEDS; ++i ) {
     carpet->ropeLeds[i] = CRGB::Black;
   }
   carpet->show();
   FastLED.delay( 1000 );
}

void ropeDemo() {
  static int loc = 0;
  static int end = NUM_NEO_LEDS;
  static int flip = 0;
  static CRGB clr( random8() % 255, random8() % 255, random8() % 255 );
  static CRGB oldclr( random8() % 255, random8() % 255, random8() % 255 );
  if ( end == 0 ) {
    end = NUM_NEO_LEDS;
    if ( flip == 0 ) {
      flip = -1;
    } else {
      flip = 0;
    }
    oldclr = clr;
    clr = CRGB( random8() % 255, random8() % 255, random8() % 255 );
    return;
  }
  for ( int i = 0; i < NUM_NEO_LEDS; ++i ) {
    if ( i == loc ) {
      carpet->ropeLeds[i] = clr;
      break;
    } else {
      carpet->ropeLeds[i] = oldclr;
    }
  }
  ++loc;
  loc %= end;
  if ( loc == 0 ) {
    --end;
  }
  carpet->show();
  FastLED.delay( 1000 );
}

} // end namespace RopeDemo

/* Alias demo
 *
 * Show off how using fractional interval changes makes moving lights look doper.
 */

namespace AliasDemo {

#define NUM_ALIAS_DEMO_LEDS 10
#define NUM_ALIAS_DEMO_SHOW_LEDS resizeCRGBW( NUM_ALIAS_DEMO_LEDS )
#define ALIAS_DEMO_DATA_PIN 52

// pointer to alias demo leds
CRGBW * leds;

static CRGBW * aliasDemoLeds() {
   static CRGBW aliasLeds[ NUM_ALIAS_DEMO_LEDS ];
   return aliasLeds;
}

static void aliasDemoShow() {
   static uint32_t numShowLeds = resizeCRGBW( NUM_ALIAS_DEMO_LEDS );
   static CRGB showLeds[ NUM_ALIAS_DEMO_SHOW_LEDS ];
   LedUtil::convertNeoArray( leds, showLeds, NUM_ALIAS_DEMO_LEDS, numShowLeds );
   FastLED.show();
}

void aliasDemoSetup() {
   leds = aliasDemoLeds();
   FastLED.addLeds<NEOPIXEL, ALIAS_DEMO_DATA_PIN>( leds, NUM_ALIAS_DEMO_LEDS );
   for ( int i = 0; i < NUM_ALIAS_DEMO_LEDS; ++i ) {
      leds[i].w = 0;
   }
   fill_solid( leds, NUM_ALIAS_DEMO_LEDS, CRGB::Red );
   aliasDemoShow();
   FastLED.delay( 1000 );
   fill_solid( leds, NUM_ALIAS_DEMO_LEDS, CRGB::Black );
   aliasDemoShow();
   FastLED.delay( 1000 );
}

void aliasDemoLoop() {
   static int idx = 0;
   static bool up = true;
   static long start = millis();
   static long tock = 0;
   long tick = ( millis() - start ) % 1000;
   uint8_t delta = 1000 - tick;
   if ( tick < tock ) {
      if ( up ) {
         if ( ++idx == NUM_NEO_LEDS ) {
            up = false;
         }
      } else if ( --idx == 0 ) {
         up = true;
      }
   }
   tock = tick;
   delta /= 4;
   for ( int i = 0; i < NUM_NEO_LEDS; ++i ) {
      if ( i == idx - 1 ) {
         leds[i] = CHSV( 150, 255, 255 - delta );
      } else if ( i == idx ) {
         leds[i] = CHSV( 150, 255, 255 );
      } else if ( i == idx + 1 ) {
         leds[i] = CHSV( 150, 255, 255 - delta );
      } else {
         leds[i] = CRGB::Black;
      }
   }
   aliasDemoShow();
}

} // end namespace AliasDemo

/* Color demo
 *
 * Playground for messing around with FastLED color palettes using a neopixel shield
 */

namespace ColorDemo {

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

} // end namespace ColorDemo

void setup() {
   LegacyRoutines::dmxSetup();
   // RopeDemo::ropeDemoSetup();
   // ColorDemo::colorDemoSetup();
   // AliasDemo::aliasDemoSetup();
}

void loop() {
   LegacyRoutines::dmxLoop();
   // RopeDemo::ropeDemo();
   // ColorDemo::colorDemoLoop();
   // AliasDemo::aliasDemoLoop();
}

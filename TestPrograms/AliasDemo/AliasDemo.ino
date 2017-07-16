/* AliasDemo.ino
 *
 * A demo program that shows off using fractional aliasing instead of integer
 * aliases. Fractional aliasing involves changing the brightness/hue of a pixel in
 * stages. For instance, to make a moving bar of color, we fade the brightness
 * in at the front and out at the end.
 *
 * This makes the light transitions look really organic and smooth.
 *
 * The code currently uses a standalone neopixel string, but could easily use the
 *  MagicCarpet class instead if we want a full scale demo.
 *
 * Author: Anders Linn
 * Date: July 2017
 */

#ifndef __FASTLED_H
#define __FASTLED_H
#include <FastLED.h>
#endif

#include "CRGBW.h"
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

void setup() {
   aliasDemoSetup();
}

void loop() {
   aliasDemoLoop();
}

/* RopeDemo.ino
 *
 * Basic shooting lights demo to test the full carpet rope led setup.
 *
 * Author: Anders Linn
 * Date: July 2017
 */

#ifndef __FASTLED_H
#define __FASTLED_H
#include <FastLED.h>
#endif

#include "CRGBW.h"
#include "MagicCarpet.h"

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

void setup() {
   ropeDemoSetup();
}

void loop() {
   ropeDemo();
}

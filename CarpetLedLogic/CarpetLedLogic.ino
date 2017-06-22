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

#define NUM_RGBW_LEDS 10
#define NUM_THEORETICAL_RGBW_LEDS ( NUM_RGBW_LEDS + ( NUM_RGBW_LEDS / 3 ) )

static MagicCarpet * carpet;

void ropeDemoSetup() {
   carpet = theMagicCarpet();
   carpet->setup();
   for ( int i = 0; i < NUM_RGBW_LEDS; ++i ) {
     carpet->ropeLeds[i] = CRGB::Red;
     carpet->ropeLeds[i].w = 0x0;
   }
   carpet->show();
   FastLED.delay( 1000 );
   for ( int i = 0; i < NUM_RGBW_LEDS; ++i ) {
     carpet->ropeLeds[i] = CRGB::Black;
   }
   carpet->show();
   FastLED.delay( 1000 );
}

void ropeDemo() {
  static int loc = 0;
  static int end = NUM_RGBW_LEDS;
  static int flip = 0;
  static CRGB clr( random8() % 255, random8() % 255, random8() % 255 );
  static CRGB oldclr( random8() % 255, random8() % 255, random8() % 255 );
  if ( end == 0 ) {
    end = NUM_RGBW_LEDS;
    if ( flip == 0 ) {
      flip = -1;
    } else {
      flip = 0;
    }
    oldclr = clr;
    clr = CRGB( random8() % 255, random8() % 255, random8() % 255 );
    return;
  }
  for ( int i = 0; i < NUM_RGBW_LEDS; ++i ) {
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
  FastLED.delay( 10 );
}

void setup() {
   // dmxSetup();
   ropeDemoSetup();
}

void loop() {
   // dmxLoop();
   ropeDemo();
}

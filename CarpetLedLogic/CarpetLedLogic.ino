/* CarpetLedLogic.ino
 *
 *   This is the main entry point for the carpet's LED system.
 *
 *   Author: Anders Linn
 *   Date: June 2017
 */


#ifndef __FASTLED_H
#define __FASTLED_H

#include <DmxSimple.h>
#include <FastLED.h>

#endif

#include "CRBGW.h"
#include "LegacyRoutines.h"

#define NUM_RGBW_LEDS 30
#define NUM_THEORETICAL_RGBW_LEDS ( NUM_RGBW_LEDS + ( NUM_RGBW_LEDS / 3 ) )
#define RGBW_DATA_PIN 6

void robeDemoSetup() {
   carpet = theMagicCarpet();
   carpet->setup();
   for ( int i = 0; i < NUM_RGBW_LEDS; ++i ) {
     carpet->ropeLeds[i] = CRGB::Red;
   }
   carpet->show();
}

void ropeDemo() {
  static int loc = 0;
  static int end = NUM_RGBW_LEDS;
  static int flip = 0;
  static CRGB clr( rand() % 255, rand() % 255, rand() % 255 );
  static CRGB oldclr( rand() % 255, rand() % 255, rand() % 255 );
  if ( end == 0 ) {
    end = NUM_RGBW_LEDS;
    if ( flip == 0 ) {
      flip = -1;
    } else {
      flip = 0;
    }
    oldclr = clr;
    clr = CRGB( rand() % 255, rand() % 255, rand() % 255);
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

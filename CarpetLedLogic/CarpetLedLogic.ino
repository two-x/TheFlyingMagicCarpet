/* CarpetLedLogic.ino
 *
 *   This is the main entry point for the carpet's LED system.
 *
 *   Author: Anders Linn
 *   Date: June 2017
 */

#include <DmxSimple.h>
#include <FastLED.h>
#include "CRBGW.h"
#include "LegacyRoutines.h"

#define NUM_DMX_LEDS 1
#define NUM_RGB_LEDS 30
#define NUM_RGBW_LEDS 30
#define NUM_THEORETICAL_RGBW_LEDS ( NUM_RGBW_LEDS + ( NUM_RGBW_LEDS / 3 ) )
#define DMX_DATA_PIN 3
#define RGB_DATA_PIN 7 // 6
#define RGBW_DATA_PIN 6 // 7

// TODO: wrap all of these in a class that represents the whole carpet
CRGB dmxleds[NUM_DMX_LEDS];
CRGB rgbleds[NUM_RGB_LEDS];

struct CRGBW rgbwleds[NUM_RGBW_LEDS];
CRGB convertedrgbwleds[NUM_THEORETICAL_RGBW_LEDS];

void robeDemoSetup() {
  // this needs to be passed the raw led array, we'll handle to conversion elsewhere
  FastLED.addLeds<NEOPIXEL, RGBW_DATA_PIN>( convertedrgbwleds, NUM_THEORETICAL_RGBW_LEDS);
  for ( int i = 0; i < NUM_RGBW_LEDS; ++i ) {
    rgbwleds[i] = CRGB::LightSteelBlue;
  }
  FastLED.show();
}

void robeDemo() {
  static int loc = 0;
  static int end = NUM_RGBW_LEDS;
  static int flip = 0;
  if ( end == 0 ) {
    end = NUM_RGBW_LEDS;
    if ( flip == 0 ) {
      flip = -1;
    } else {
      flip = 0;
    }
    return;
  }
  for ( int i = 0; i < NUM_RGBW_LEDS; ++i ) {
    if ( i == loc ) {
      rgbwleds[i] = flip ? CRGB::Gold : CRGB::LightSteelBlue;
      break;
    } else {
      rgbwleds[i] = flip ? CRGB::LightSteelBlue : CRGB::Gold;
    }
  }
  ++loc;
  loc %= end;
  if ( loc == 0 ) {
    --end;
  }
  covertCRGBWArrayToCRGBArray( rgbwleds, convertedrgbwleds, NUM_RGBW_LEDS, NUM_THEORETICAL_RGBW_LEDS );
  FastLED.show(); 
  delay(50);
}

void setup() {
   // FastLED.addLeds<DMXSIMPLE, DMX_DATA_PIN, RGB>(dmxleds, NUM_DMX_LEDS);
   // FastLED.addLeds<NEOPIXEL, RGB_DATA_PIN>(rgbleds, NUM_RGB_LEDS);

   // robeDemoSetup();
   dmxSetup();
}

void loop() {
   dmxLoop();
   // robeDemo();
}

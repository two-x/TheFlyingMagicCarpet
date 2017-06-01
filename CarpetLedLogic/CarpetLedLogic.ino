
 #include <DmxSimple.h>
#include <FastLED.h>
#include "CRBGW.h"

#define NUM_DMX_LEDS 1
#define NUM_RGB_LEDS 30
#define NUM_RGBW_LEDS 30
#define NUM_THEORETICAL_RGBW_LEDS ( NUM_RGBW_LEDS + ( NUM_RGBW_LEDS / 3 ) )
#define DMX_DATA_PIN 3
#define RGB_DATA_PIN 7 // 6
#define RGBW_DATA_PIN 6 // 7

CRGB dmxleds[NUM_DMX_LEDS];
CRGB rgbleds[NUM_RGB_LEDS];

static void convertCRGBW2CRGB( CRGBW *src, CRGB *dst ) {
  dst[0] = CRGB(src[0].r, src[0].g, src[0].b); //(red led 1, green on led 1, blue led 1)
  dst[1] = CRGB(src[1].g, src[0].w, src[1].r); //(green led 2, white on led 1, red led 2)
  dst[2] = CRGB(src[1].w, src[1].b, src[2].g); //(white led 2,blue led 2, green led 3)
  dst[3] = CRGB(src[2].b, src[2].r, src[2].w); //(blue led 3, red led 3, white led 3)
}

static void covertCRGBWArrayToCRGBArray( CRGBW *src, CRGB *dst ) {
  int s, d;
  // we want to cut out if the entire array has been converted, or if we're out of leds
  while ( s < NUM_RGBW_LEDS && d < NUM_THEORETICAL_RGBW_LEDS ) {
    convertCRGBW2CRGB( src + s, dst + d );
    s += 3;
    d += 4;
  }
}

struct CRGBW rgbwleds[NUM_RGBW_LEDS];
CRGB convertedrgbwleds[NUM_THEORETICAL_RGBW_LEDS];

void setup() {
  // FastLED.addLeds<DMXSIMPLE, DMX_DATA_PIN, RGB>(dmxleds, NUM_DMX_LEDS);
  // FastLED.addLeds<NEOPIXEL, RGB_DATA_PIN>(rgbleds, NUM_RGB_LEDS);

  // this needs to be passed the raw led array, we'll handle to conversion elsewhere
  FastLED.addLeds<NEOPIXEL, RGBW_DATA_PIN>( convertedrgbwleds, NUM_THEORETICAL_RGBW_LEDS);
  for ( int i = 0; i < NUM_RGBW_LEDS; ++i ) {
    rgbwleds[i] = CRGB::LightSteelBlue;
  }
  FastLED.show();
}

void loop() {
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
  covertCRGBWArrayToCRGBArray( rgbwleds, convertedrgbwleds );
  FastLED.show(); 
  delay(50);
}

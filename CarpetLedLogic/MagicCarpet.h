/* MagicCarpet.h
 *
 *    This is a data abstraction of the carpet.
 *
 *    TODO: total power consumption throttle
 *
 *    Author: Anders Linn
 *    Date: June 2017
 */

#ifndef __MAGIC_CARPET_H
#define __MAGIC_CARPET_H

#ifndef __DMXSIMPLE_H
#define __DMXSIMPLE_H
#include <DmxSimple.h>
#endif

#ifndef __FASTLED_H
#define __FASTLED_H
#include <FastLED.h>
#endif

#include "CRGBW.h"
#include "AudioBoard.h"
#include "LedConsts.h"

// DMX constants
#define NUM_MEGABAR_LEDS 10
#define NUM_CHINA_LEDS 8
#define MEGABAR_DATA_PIN 3
#define CHINA_DATA_PIN 3

// Neopixel constants
#define NUM_NEO_LEDS 1024
#define NUM_NEOPIXEL_STRIPS 8
// TODO: fix these numbers, we ended up with less total leds
#define NUM_NEO_SMALL_LEDS 110
#define NUM_NEO_LARGE_LEDS 146
#define NEO_DATA_PIN0 52
#define NEO_DATA_PIN1 51
#define NEO_DATA_PIN2 50
#define NEO_DATA_PIN3 49
#define NEO_DATA_PIN4 48
#define NEO_DATA_PIN5 47
#define NEO_DATA_PIN6 46
#define NEO_DATA_PIN7 45

// TODO: all of these pins need to be updated for the due

// TODO: move to sound board file
// input from sound board
#define ANALOG_LOW_PIN 3
#define ANALOG_MID_PIN 2
#define ANALOG_HIGH_PIN 1

// TODO: need to change to new controller inputs

// potentiometer input
#define ANALOG_BRIGHTNESS_PIN 0

// inputs from mode select switch
#define MODE0_PIN 8
#define MODE1_PIN 9
#define MODE2_PIN 2
#define MODE3_PIN 4

// inputs from wireless board
#define INPUT0_PIN 5
#define INPUT1_PIN 6
#define INPUT2_PIN 7

// button ids from wireless board
#define UP_BUTTON 3
#define LEFT_BUTTON 4
#define RIGHT_BUTTON 1
#define DOWN_BUTTON 7
#define CENTER_BUTTON 6

// max voltage from an analog input pin
#define MAX_VOLTAGE 1023

/* Positional Constants
 *
 * These are aliases for the index values of the carpet's led arrays. Their are two
 * different types defined: directional and temporal. Both led array start from the
 * midpoint of the front of the carpet (FRONT or TWELVE). So far these are defined
 * only for the rope light arrays, but we can do this for the dmx leds too if we
 * feel the need.
 */
#define FRONT 0x0
#define FRONT_RIGHT 0x06E
#define RIGHT 0x100
#define BACK_RIGHT 0x192
#define BACK 0x200
#define BACK_LEFT 0x26E
#define LEFT 0x300
#define FRONT_LEFT 0x392

#define TWELVE NEO_FRONT
#define ONE_THIRTY NEO_FRONT_RIGHT
#define THREE NEO_RIGHT
#define FOUR_THIRTY NEO_BACK_RIGHT
#define SIX NEO_BACK
#define SEVEN_THIRTY NEO_BACK_LEFT
#define NINE NEO_LEFT
#define TEN_THIRTY NEO_FRONT_LEFT

class MagicCarpet {
 private:

   /* FastLED doesn't support rgbw leds. We work around this by offsetting the color
    * values to accomodate the white value. See CRGBW.h for more details.
    */
   CRGB megabarShowLeds[ resizeCRGBW( NUM_MEGABAR_LEDS ) ];
   CRGB chinaShowLeds[ resizeCRGBW( NUM_CHINA_LEDS ) ];
   CRGB ropeShowLeds[ resizeCRGBW( NUM_NEO_LEDS ) ];

 public:

   // led arrays
   CRGBW megabarLeds[ NUM_MEGABAR_LEDS ];
   CRGBWUA chinaLeds[ NUM_CHINA_LEDS ];
   CRGBW ropeLeds[ NUM_NEO_LEDS ];

   void setup() {
      // TODO: all of the input setup needs to change based on the new controls
      // set inputs from wireless board
      pinMode( INPUT0_PIN, INPUT );
      pinMode( INPUT1_PIN, INPUT );
      pinMode( INPUT1_PIN, INPUT );

      // set inputs from Mode Select Switch
      pinMode( MODE0_PIN, INPUT );
      pinMode( MODE1_PIN, INPUT );
      pinMode( MODE2_PIN, INPUT );
      pinMode( MODE3_PIN, INPUT );

      // setup eliot's board
      setupAudioBoard();

      // add dmx leds
      FastLED.addLeds<DMXSIMPLE, MEGABAR_DATA_PIN>( megabarShowLeds,
                                                    resizeCRGBW( NUM_MEGABAR_LEDS ) );
      FastLED.addLeds<DMXSIMPLE, CHINA_DATA_PIN>( chinaShowLeds,
                                                  resizeCRGBWUA( NUM_CHINA_LEDS ) );

      // add eight channels of rope leds
      FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN0>( ropeShowLeds,
                                                resizeCRGBW( NUM_NEO_SMALL_LEDS ) );
      FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN1>( ropeShowLeds + FRONT_RIGHT,
                                                resizeCRGBW( NUM_NEO_LARGE_LEDS ) );
      FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN2>( ropeShowLeds + RIGHT,
                                                resizeCRGBW( NUM_NEO_LARGE_LEDS ) );
      FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN3>( ropeShowLeds + BACK_RIGHT,
                                                resizeCRGBW( NUM_NEO_SMALL_LEDS ) );
      FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN4>( ropeShowLeds + BACK,
                                                resizeCRGBW( NUM_NEO_SMALL_LEDS ) );
      FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN5>( ropeShowLeds + BACK_LEFT,
                                                resizeCRGBW( NUM_NEO_LARGE_LEDS ) );
      FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN6>( ropeShowLeds + LEFT,
                                                resizeCRGBW( NUM_NEO_LARGE_LEDS ) );
      FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN7>( ropeShowLeds + FRONT_LEFT,
                                                resizeCRGBW( NUM_NEO_SMALL_LEDS ) );

      clear(); // there might be stale values left in the leds, start from scratch

      // start with the white/uv/a leds off
      for ( int i = 0; i < NUM_MEGABAR_LEDS; ++i ) {
        megabarLeds[ i ].w = 0x0;
      }
      for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) {
        chinaLeds[ i ].w = 0x0;
        chinaLeds[ i ].u = 0x0;
        chinaLeds[ i ].a = 0x0;
      }
      for ( int i = 0; i < NUM_NEO_LEDS; ++i ) {
        ropeLeds[ i ].w = 0x0;
      }
      show();
   }

   void show() {
      /* the lights aren't always arranged the same way as they are addressed, so in
       * some cases we need to reverse whatever has been programmed in the user array
       * in order to keep the addressing consistent. This is done in-place.
       *
       * TODO: if we start running too slow we can look at ways to get around this
       *       reversal
       */
      LedUtil::reverse( ropeLeds, NUM_NEO_SMALL_LEDS );
      LedUtil::reverse( ropeLeds + RIGHT, NUM_NEO_LARGE_LEDS );
      LedUtil::reverse( ropeLeds + BACK, NUM_NEO_SMALL_LEDS );
      LedUtil::reverse( ropeLeds + LEFT, NUM_NEO_LARGE_LEDS );

      LedUtil::convertDmxArray4( megabarLeds, megabarShowLeds, NUM_MEGABAR_LEDS,
                                 resizeCRGBW( NUM_MEGABAR_LEDS ) );
      LedUtil::convertDmxArray6( chinaLeds, chinaShowLeds, NUM_CHINA_LEDS,
                                 resizeCRGBWUA( NUM_CHINA_LEDS ) );
      LedUtil::convertNeoArray( ropeLeds, ropeShowLeds, NUM_NEO_LEDS,
                                resizeCRGBW( NUM_NEO_LEDS ) );

      // make sure to reverse the values so the user has a consistent view
      LedUtil::reverse( ropeLeds, NUM_NEO_SMALL_LEDS );
      LedUtil::reverse( ropeLeds + RIGHT, NUM_NEO_LARGE_LEDS );
      LedUtil::reverse( ropeLeds + BACK, NUM_NEO_SMALL_LEDS );
      LedUtil::reverse( ropeLeds + LEFT, NUM_NEO_LARGE_LEDS );

      FastLED.show();
   }

   void clearMegabars() {
      for ( int i = 0; i < NUM_MEGABAR_LEDS; ++i ) {
        megabarLeds[ i ] = CRGB::Black;
      }
   }

   void clearChinas() {
      for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) {
        chinaLeds[ i ] = CRGB::Black;
      }
   }

   void clearRope() {
      for ( int i = 0; i < NUM_NEO_LEDS; ++i ) {
        ropeLeds[ i ] = CRGB::Black;
      }
   }

   void clearRopeWhite() {
      for ( int i = 0; i < NUM_NEO_LEDS; ++i ) {
        ropeLeds[ i ].white = 0;
      }
   }

   // clears all the lights back to full black
   void clear() {
      clearMegabars();
      clearChinas();
      clearRope();
      clearRopeWhite();
   }

   // TODO: all of this needs to change when we get the new controllers programmed
   uint8_t readMode() {
      return digitalRead( MODE2_PIN ) << 3 | digitalRead( MODE3_PIN ) << 2;
   }

   uint8_t readDigitalInput() {
      return digitalRead( INPUT0_PIN ) << 2 |
             digitalRead( INPUT1_PIN ) << 1 |
             digitalRead( INPUT2_PIN );
   }

   CRGB readAnalogInput() {
      // TODO: name these pins
      // TODO: why are we inverting the values here?
      // convert analog inputs into rgb values between 0-255
      uint8_t r = ( MAX_VOLTAGE - analogRead( ANALOG_LOW_PIN ) ) / 4;
      uint8_t g = ( MAX_VOLTAGE - analogRead( ANALOG_MID_PIN ) ) / 4;
      uint8_t b = ( MAX_VOLTAGE - analogRead( ANALOG_HIGH_PIN ) ) / 4;
      return CRGB( r, g, b );
   }

   uint8_t readBrightnessInput() {
      return ( MAX_VOLTAGE - analogRead( ANALOG_BRIGHTNESS_PIN ) ) / 4;
   }
};

// singleton representing the one-and-only Flying Magic Carpet (TM)
MagicCarpet * theMagicCarpet() {
   static MagicCarpet carpet;
   return &carpet;
}

#endif

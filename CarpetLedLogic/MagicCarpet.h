/* MagicCarpet.h
 *
 *    This is a data abstraction of the carpet.
 *
 *    TODO: we might want to add fade-in functionality here, so that instead of
 *          changing directly to the next desired color we fade from the previous
 *          one. Right now, any such fading has to be written into the light shows.
 *          If we do this, we should add a toggle to allow light shows to disable
 *          automatic fades if they want to.
 *    TODO: total power consumption throttle
 *
 *    Author: Anders Linn
 *    Date: June 2017
 */

#ifndef __MAGIC_CARPET_H
#define __MAGIC_CARPET_H

#ifndef __FASTLED_H
#define __FASTLED_H
#include <FastLED.h>
#endif

#include "CRGBW.h"
#include "LedController.h"
#include "AudioBoard.h"
#include "ArmDmx.h"

// Controller constants
#define POT_ANALOG_PIN 3
#define BUTTON_PIN 3
#define ENCODER_A_PIN 5
#define ENCODER_B_PIN 4

// DMX constants
#define NUM_MEGABAR_LEDS 10
#define NUM_CHINA_LEDS 8
#define NUM_DMX_LEDS NUM_MEGABAR_LEDS + NUM_CHINA_LEDS
#define SIZEOF_MEGABAR_LEDS ( NUM_MEGABAR_LEDS * sizeof( CRGB ) )
#define SIZEOF_CHINA_LEDS ( NUM_CHINA_LEDS * sizeof( CRGBWUA ) )
#define TOTAL_DMX_SIZE ( SIZEOF_MEGABAR_LEDS + SIZEOF_CHINA_LEDS )

// Neopixel constants
// TODO: convert the user-facing values to constexpr global vars
#define SIZEOF_SMALL_NEO 108
#define SIZEOF_LARGE_NEO 145
#define NUM_NEOPIXEL_STRIPS 8
#define NUM_NEO_SMALL_LEDS ( NUM_NEOPIXEL_STRIPS / 2 )
#define NUM_NEO_LARGE_LEDS NUM_NEO_SMALL_LEDS
#define NUM_NEO_LEDS ( ( SIZEOF_SMALL_NEO * NUM_NEO_SMALL_LEDS ) + \
                       ( SIZEOF_LARGE_NEO * NUM_NEO_LARGE_LEDS ) )
#define NUM_NEO_LEDS_PER_STRIP LedUtil::resizeCRGBW( SIZEOF_LARGE_NEO )
#define NUM_NEO_SHOW_LEDS ( NUM_NEO_LEDS_PER_STRIP * NUM_NEOPIXEL_STRIPS )
#define SIZEOF_NEO_STRIP ( NUM_NEO_LEDS_PER_STRIP * sizeof( CRGB ) )
#define SIZEOF_NEO_LEDS ( NUM_NEO_LEDS * sizeof( CRGBW ) )
#define SIZEOF_NEO_SHOW_LEDS ( NUM_NEOPIXEL_STRIPS * SIZEOF_NEO_STRIP )
#define NEO0_OFFSET 0 // small
#define NEO1_OFFSET SIZEOF_SMALL_NEO // large
#define NEO2_OFFSET ( SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO ) // large
#define NEO3_OFFSET ( SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO * 2 ) // small
#define NEO4_OFFSET ( SIZEOF_SMALL_NEO * 2 + SIZEOF_LARGE_NEO * 2 ) // small
#define NEO5_OFFSET ( SIZEOF_SMALL_NEO * 3 + SIZEOF_LARGE_NEO * 2 ) // large
#define NEO6_OFFSET ( SIZEOF_SMALL_NEO * 3 + SIZEOF_LARGE_NEO * 3 ) // large
#define NEO7_OFFSET ( SIZEOF_SMALL_NEO * 3 + SIZEOF_LARGE_NEO * 4 ) // small
// The pin order for the port bank we are using is: 25,26,27,28,14,15,29,11
#define NEO_PORT_BANK ( WS2811_PORTD )
#define NEO_PIN0 25
#define NEO_PIN1 26
#define NEO_PIN2 27
#define NEO_PIN3 28
#define NEO_PIN4 14
#define NEO_PIN5 15
#define NEO_PIN6 29
#define NEO_PIN7 11

/* Positional Constants
 *
 * These are aliases for the index values of the carpet's led arrays. Their are two
 * different types defined: directional and temporal. Both led array start from the
 * midpoint of the front of the carpet (FRONT or TWELVE). So far these are defined
 * only for the rope light arrays, but we can do this for the dmx leds too if we
 * feel the need.
 */
#define FRONT NEO0_OFFSET
#define FRONT_RIGHT NEO1_OFFSET
#define RIGHT NEO2_OFFSET
#define BACK_RIGHT NEO3_OFFSET
#define BACK NEO4_OFFSET
#define BACK_LEFT NEO5_OFFSET
#define LEFT NEO6_OFFSET
#define FRONT_LEFT NEO7_OFFSET

#define TWELVE FRONT
#define ONE_THIRTY FRONT_RIGHT
#define THREE RIGHT
#define FOUR_THIRTY BACK_RIGHT
#define SIX BACK
#define SEVEN_THIRTY BACK_LEFT
#define NINE LEFT
#define TEN_THIRTY FRONT_LEFT

class MagicCarpet {
 private:

   /* FastLED doesn't support rgbw leds. We work around this by offsetting the color
    * values to accomodate the white value. See CRGBW.h for more details.
    */
   CRGB ropeShowLeds[ NUM_NEO_SHOW_LEDS  ];

 public:

   /* DON'T CHANGE THE ORDER OF THESE ARRAYS!
    * They're declared separately to make them easy to work with, but treated as a
    * single continguous array when passed into dmx_send.
    */
   CRGB megabarLeds[ NUM_MEGABAR_LEDS ];
   CRGBWUA chinaLeds[ NUM_CHINA_LEDS ];

   // neopixel leds
   CRGBW ropeLeds[ NUM_NEO_LEDS ];

   // controls
   LedControl::Potentiometer * pot;
   LedControl::PushButton * button;
   LedControl::Encoder * encoder;

   void setup() {
      // seed random so we always get different random patterns
      randomSeed( analogRead( 0 ) );

      AudioBoard::setup();

      digitalWrite( 2, HIGH );

      // set up the controller
      pot = new LedControl::Potentiometer( POT_ANALOG_PIN );
      button = LedControl::getPushButton( BUTTON_PIN );
      encoder = LedControl::getEncoder( ENCODER_A_PIN, ENCODER_B_PIN );

      // add dmx leds
      dmx_init( TOTAL_DMX_SIZE );

      // add eight channels of rope leds
      FastLED.addLeds<NEO_PORT_BANK,NUM_NEOPIXEL_STRIPS>( ropeShowLeds,
                                                          NUM_NEO_LEDS_PER_STRIP );

      clear(); // there might be stale values left in the leds, start from scratch

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
      LedUtil::reverse( ropeLeds + NEO0_OFFSET, SIZEOF_SMALL_NEO );
      LedUtil::reverse( ropeLeds + NEO2_OFFSET, SIZEOF_LARGE_NEO );
      LedUtil::reverse( ropeLeds + NEO4_OFFSET, SIZEOF_SMALL_NEO );
      LedUtil::reverse( ropeLeds + NEO6_OFFSET, SIZEOF_LARGE_NEO );

      LedUtil::convertNeoArray( ropeLeds + NEO0_OFFSET, ropeShowLeds,
                                SIZEOF_SMALL_NEO );
      LedUtil::convertNeoArray( ropeLeds + NEO1_OFFSET,
                                ropeShowLeds + NUM_NEO_LEDS_PER_STRIP,
                                SIZEOF_LARGE_NEO );
      LedUtil::convertNeoArray( ropeLeds + NEO2_OFFSET,
                                ropeShowLeds + NUM_NEO_LEDS_PER_STRIP * 2,
                                SIZEOF_LARGE_NEO );
      LedUtil::convertNeoArray( ropeLeds + NEO3_OFFSET,
                                ropeShowLeds + NUM_NEO_LEDS_PER_STRIP * 3,
                                SIZEOF_SMALL_NEO );
      LedUtil::convertNeoArray( ropeLeds + NEO4_OFFSET,
                                ropeShowLeds + NUM_NEO_LEDS_PER_STRIP * 4,
                                SIZEOF_SMALL_NEO );
      LedUtil::convertNeoArray( ropeLeds + NEO5_OFFSET,
                                ropeShowLeds + NUM_NEO_LEDS_PER_STRIP * 5,
                                SIZEOF_LARGE_NEO );
      LedUtil::convertNeoArray( ropeLeds + NEO6_OFFSET,
                                ropeShowLeds + NUM_NEO_LEDS_PER_STRIP * 6,
                                SIZEOF_LARGE_NEO );
      LedUtil::convertNeoArray( ropeLeds + NEO7_OFFSET,
                                ropeShowLeds + NUM_NEO_LEDS_PER_STRIP * 7,
                                SIZEOF_SMALL_NEO );

      // make sure to reverse the values so the user has a consistent view
      LedUtil::reverse( ropeLeds + NEO0_OFFSET, SIZEOF_SMALL_NEO );
      LedUtil::reverse( ropeLeds + NEO2_OFFSET, SIZEOF_LARGE_NEO );
      LedUtil::reverse( ropeLeds + NEO4_OFFSET, SIZEOF_SMALL_NEO );
      LedUtil::reverse( ropeLeds + NEO6_OFFSET, SIZEOF_LARGE_NEO );

      // we don't have to pass the china light array separately. Instead, we treat
      // both arrays as a single big array, since they're contiguous in memory.
      dmx_send( ( uint8_t * ) megabarLeds );

      FastLED.show();
   }

   void clearMegabars() {
      memset( megabarLeds, 0, SIZEOF_MEGABAR_LEDS );
   }

   void clearChinas() {
      memset( chinaLeds, 0, SIZEOF_CHINA_LEDS );
   }

   void clearRope() {
      memset( ropeLeds, 0, SIZEOF_NEO_LEDS );
      memset( ropeShowLeds, 0, SIZEOF_NEO_SHOW_LEDS );
   }

   // clears all the lights back to full black
   void clear() {
      clearMegabars();
      clearChinas();
      clearRope();
   }

   void error() {
      while( true ) {
         static CRGB clr = CRGB::Black;
         static CRGBWUA chinaClr = clr;
         clr.a = 255
         fill( ropeLeds, CRGB::Red, NUM_NEO_LEDS );
         fill( dmxLeds, CRGB::Yellow, NUM_DMX_LEDS );
         fill( chinaLeds, chinaClr, NUM_CHINA_LEDS );
         FastLED.delay( 1000 );
         fill( ropeLeds, clr, NUM_NEO_LEDS );
         fill( dmxLeds, clr, NUM_DMX_LEDS );
         fill( chinaLeds, clr, NUM_CHINA_LEDS );
         FastLED.delay( 1000 );
      }
   }
};

// singleton representing the one-and-only Flying Magic Carpet (TM)
MagicCarpet * theMagicCarpet() {
   static MagicCarpet carpet;
   return &carpet;
}

#endif

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

#include "CRBGW.h"

// dmx constants
#define NUM_DMX_LEDS 18
#define NUM_CONVERTED_DMX_LEDS ( NUM_NEO_LEDS + NUM_NEO_LEDS / 3 )

// neopixel constants
#define NUM_NEO_LEDS 30 // 1024
#define NUM_CONVERTED_DMX_LEDS ( NUM_DMX_LEDS + NUM_DMX_LEDS / 3 )

// neopixel constants
#define NUM_NEO_LEDS 400 // 1024
#define NUM_NEOPIXEL_STRIPS 8
#define NUM_NEO_SMALL_LEDS 110
#define NUM_NEO_LARGE_LEDS 146
#define NUM_CONVERTED_NEO_LEDS ( NUM_NEO_LEDS + NUM_NEO_LEDS / 3 )
#define NUM_CONVERTED_NEO_SMALL_LEDS ( NUM_NEO_SMALL_LEDS + NUM_NEO_SMALL_LEDS / 3 )
#define NUM_CONVERTED_NEO_LARGE_LEDS ( NUM_NEO_LARGE_LEDS + NUM_NEO_LARGE_LEDS / 3 )

// outputs
#define DMX_DATA_PIN 3
#define NEO_DATA_PIN 6 // TODO: add more data pins!

// analog inputs
#define ANALOG_LOW_PIN 3
#define ANALOG_MID_PIN 2
#define ANALOG_HIGH_PIN 1
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

class MagicCarpet {
 private:
   /* FastLED doesn't support rgbw leds. We work around this by offsetting the color
    * values to accomodate the white value.  For three rgbw leds we end up sending
    * four rgb packets. For dmx, this is straightforward. Each of the values simply
    * shifts by one to handle the extra value:
    *
    *   red on led 1, green on led 1, blue on led 1
    *   white led 1, red on led 2, green led 2
    *   blue led 2, white led 2, red led 3
    *   green led 3, blue led 3, white led 3
    *
    * However, the protocol for the neopixel rgbw leds shuffles the values around in
    * an odd way. The ordering is as follows:
    *
    *   red on led 1, green on led 1, blue on led 1
    *   green led 2, white on led 1, red led 2
    *   white led 2, blue led 2, green led 3
    *   blue led 3, red led 3, white led 3
    *
    * credit to user joekitch on the arduino forum for figuring out neopixel order
    * https://forum.arduino.cc/index.php?topic=432470.0
    */
   CRGB convertedDmxLeds[ NUM_CONVERTED_DMX_LEDS ];
   CRGB convertedRopeLeds[ NUM_CONVERTED_NEO_LEDS ];

 public:
   // TODO: we can probably figure out a way to avoid duplicating the array if we
   //       start running low on memory (std::vararray?)
   // led arrays
   CRGBW dmxLeds[ NUM_DMX_LEDS ];
   CRGBW ropeLeds[ NUM_NEO_LEDS ];

   void setup() {
      // set inputs from wireless board
      pinMode( INPUT0_PIN, INPUT );
      pinMode( INPUT1_PIN, INPUT );
      pinMode( INPUT1_PIN, INPUT );

      // set inputs from Mode Select Switch
      pinMode( MODE0_PIN, INPUT );
      pinMode( MODE1_PIN, INPUT );
      pinMode( MODE2_PIN, INPUT );
      pinMode( MODE3_PIN, INPUT );

      // add dmx leds
      FastLED.addLeds<DMXSIMPLE, DMX_DATA_PIN>( dmxLeds,
                                                NUM_DMX_LEDS );
      // FastLED.addLeds<DMXSIMPLE, DMX_DATA_PIN>( convertedDmxLeds,
      //                                           NUM_CONVERTED_DMX_LEDS );

      // add eight channels of rope leds
      // FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN>( convertedRopeLeds,
      //                                          NUM_CONVERTED_NEO_SMALL_LEDS );
      // FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN>( convertedRopeLeds + 0x06E,
      //                                          NUM_CONVERTED_NEO_LARGE_LEDS );
      // FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN>( convertedRopeLeds + 0x100,
      //                                          NUM_CONVERTED_NEO_LARGE_LEDS );
      // FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN>( convertedRopeLeds + 0x192,
      //                                          NUM_CONVERTED_NEO_SMALL_LEDS );
      // FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN>( convertedRopeLeds + 0x200,
      //                                          NUM_CONVERTED_NEO_SMALL_LEDS );
      // FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN>( convertedRopeLeds + 0x26e,
      //                                          NUM_CONVERTED_NEO_LARGE_LEDS );
      // FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN>( convertedRopeLeds + 0x300,
      //                                          NUM_CONVERTED_NEO_LARGE_LEDS );
      // FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN>( convertedRopeLeds + 0x392,
      //                                          NUM_CONVERTED_NEO_SMALL_LEDS );

      clear(); // there might be stale values left in the leds, start from scratch

      // set to full brightness
      for ( int i = 0; i < NUM_DMX_LEDS; ++i ) {
        dmxLeds[ i ].w = 0x255;
      }
      show();
   }

   void show() {
      CRGBW::convertDmxArray( dmxLeds, convertedDmxLeds, NUM_DMX_LEDS,
                           NUM_CONVERTED_DMX_LEDS );
      /* the lights aren't always arranged the same way as they are addressed, so in
       * some cases we need to reverse whatever has been programmed in the user array
       * in order to keep the addressing consistent.
       *
       * we might be able to use std::slice and std::vararray instead but they might
       * be too slow
       */
      // TODO: use names instead of raw indices here and below
      // reverse( ropeLeds, NUM_NEO_SMALL_LEDS );
      // reverse( ropeLeds + 0x100, NUM_NEO_LARGE_LEDS );
      // reverse( ropeLeds + 0x200, NUM_NEO_SMALL_LEDS );
      // reverse( ropeLeds + 0x300, NUM_NEO_LARGE_LEDS );

      CRGBW::convertNeopixelArray( ropeLeds, convertedRopeLeds, NUM_NEO_LEDS,
                                   NUM_CONVERTED_NEO_LEDS );

      // // make sure to reverse the values so the user has a consistent view
      // reverse( ropeLeds, NUM_NEO_SMALL_LEDS );
      // reverse( ropeLeds + 0x100, NUM_NEO_LARGE_LEDS );
      // reverse( ropeLeds + 0x200, NUM_NEO_SMALL_LEDS );
      // reverse( ropeLeds + 0x300, NUM_NEO_LARGE_LEDS );

      FastLED.show();
   }

   void clearDmx() {
      for ( int i = 0; i < NUM_DMX_LEDS; ++i ) {
        dmxLeds[ i ] = CRGB::Black;
      }
   }

   void clearRope() {
      for ( int i = 0; i < NUM_NEO_LEDS; ++i ) {
        ropeLeds[ i ] = CRGB::Black;
      }
   }

   void clear() {
      clearDmx();
      clearRope();
   }

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
   static MagicCarpet * carpet = new MagicCarpet();
   return carpet;
}

#endif

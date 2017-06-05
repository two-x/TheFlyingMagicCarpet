/* MagicCarpet.h
 *
 *    This is a data abstraction of the carpet.
 *
 *    Author: Anders Linn
 *    Date: June 2017
 */

#include <DmxSimple.h>
#include <FastLED.h>
#include "CRBGW.h"

#define NUM_DMX_LEDS 10 // TODO: this number doesn't include the new floods
#define NUM_CONVERTED_DMX_LEDS ( NUM_DMX_LEDS + ( NUM_DMX_LEDS / 3 ) )
#define NUM_NEO_LEDS 1024
#define NUM_NEOPIXEL_STRIPS 8
#define NUM_NEO_SMALL_LEDS 110
#define NUM_NEO_LARGE_LEDS 146
#define NUM_CONVERTED_NEO_LEDS ( NUM_NEO_LEDS + NUM_NEO_LEDS / 3 )
#define NUM_CONVERTED_NEO_SMALL_LEDS ( NUM_NEO_SMALL_LEDS + NUM_NEO_SMALL_LEDS / 3 )
#define NUM_CONVERTED_NEO_LARGE_LEDS ( NUM_NEO_LARGE_LEDS + NUM_NEO_LARGE_LEDS / 3 )
#define DMX_DATA_PIN 3
#define NEO_DATA_PIN 6 // TODO: add more data pins!

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
      FastLED.addLeds<DMXSIMPLE, DMX_DATA_PIN>( convertedDmxLeds,
                                                NUM_CONVERTED_DMX_LEDS );
      FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN>( convertedRopeLeds,
                                               NUM_CONVERTED_NEO_SMALL_LEDS );
      FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN>( convertedRopeLeds + 0x06E,
                                               NUM_CONVERTED_NEO_LARGE_LEDS );
      FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN>( convertedRopeLeds + 0x100,
                                               NUM_CONVERTED_NEO_LARGE_LEDS );
      FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN>( convertedRopeLeds + 0x192,
                                               NUM_CONVERTED_NEO_SMALL_LEDS );
      FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN>( convertedRopeLeds + 0x200,
                                               NUM_CONVERTED_NEO_SMALL_LEDS );
      FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN>( convertedRopeLeds + 0x26e,
                                               NUM_CONVERTED_NEO_LARGE_LEDS );
      FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN>( convertedRopeLeds + 0x300,
                                               NUM_CONVERTED_NEO_LARGE_LEDS );
      FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN>( convertedRopeLeds + 0x392,
                                               NUM_CONVERTED_NEO_SMALL_LEDS );
      clear(); // there might be stale values left in the leds, start from scratch
      show();
   }

   void show() {
      // the new floods don't use 4-channel like the megabars. Could we scrap it?
      // It would make life much easier...
      convertDmxRgbwArray( dmxLeds, convertedDmxLeds, NUM_DMX_LEDS,
                           NUM_CONVERTED_DMX_LEDS );

      /* the lights aren't always arranged the same way as they are addressed, so in
       * some cases we need to reverse whatever has been programmed in the user array
       * in order to keep the addressing consistent.
       *
       * we might be able to use std::slice and std::vararray instead but they might
       * be too slow
       */
      // TODO: use names instead of raw indices here and below
      reverse( ropeLeds, NUM_NEO_SMALL_LEDS );
      reverse( ropeLeds + 0x100, NUM_NEO_LARGE_LEDS );
      reverse( ropeLeds + 0x200, NUM_NEO_SMALL_LEDS );
      reverse( ropeLeds + 0x300, NUM_NEO_LARGE_LEDS );

      convertNeopixelRgbwArray( ropeLeds, convertedRopeLeds, NUM_NEO_LEDS,
                                NUM_CONVERTED_NEO_LEDS );

      // make sure to reverse the values so the user has a consistent view
      reverse( ropeLeds, NUM_NEO_SMALL_LEDS );
      reverse( ropeLeds + 0x100, NUM_NEO_LARGE_LEDS );
      reverse( ropeLeds + 0x200, NUM_NEO_SMALL_LEDS );
      reverse( ropeLeds + 0x300, NUM_NEO_LARGE_LEDS );

      FastLED.show();
   }

   void clearDmx() {
      for ( int i = 0; i < NUM_DMX_LEDS; ++i ) {
        dmxLeds[ i ] = CRGB::Black;
        dmxLeds[ i ].w = 0x0;
      }
   }

   void clearRope() {
      for ( int i = 0; i < NUM_NEO_LEDS; ++i ) {
        ropeLeds[ i ] = CRGB::Black;
        ropeLeds[ i ].w = 0x0;
      }
   }

   void clear() {
      clearDmx();
      clearRope();
   }
};

// singleton representing the one-and-only Flying Magic Carpet (TM)
MagicCarpet carpet();


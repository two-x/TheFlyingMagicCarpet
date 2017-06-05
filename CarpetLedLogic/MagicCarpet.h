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
#define NUM_NEO_LEDS 30
#define NUM_CONVERTED_DMX_LEDS ( NUM_DMX_LEDS + ( NUM_DMX_LEDS / 3 ) )
#define NUM_CONVERTED_NEO_LEDS ( NUM_NEO_LEDS + ( NUM_NEO_LEDS / 3 ) )
#define NEOPIXEL_STRIP_SIZE 256
#define DMX_DATA_PIN 3
#define NEO_DATA_PIN 6

class MagicCarpet {
 private:

   // TODO: we can probably figure out a way to avoid duplicating the array if we
   //       start running low on memory
   // led arrays
   CRGBW dmxLeds[ NUM_DMX_LEDS ];
   CRGBW ropeLeds[ NUM_NEO_LEDS ];

   /* FastLED doesn't support rgbw leds. We work around this by offsetting the color
    * values to accomodate the white value.  For three rgbw leds we end up sending
    * four rgb packets.For dmx, this is straightforward. Each of the values simply
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
   // TODO: need to add eight of these
   CRGB convertedRopeLeds[ NUM_CONVERTED_NEO_LEDS ];

 public:

   void setup() {
      FastLED.addLeds<DMXSIMPLE, DMX_DATA_PIN>( convertedDmxLeds,
                                                NUM_CONVERTED_DMX_LEDS );
      FastLED.addLeds<NEOPIXEL, NEO_DATA_PIN>( convertedRopeLeds,
                                               NUM_CONVERTED_NEO_LEDS );
      clear(); // there might be stale values left in the leds, start from scratch
   }

   void show() {
      convertDmxRgbwArray( dmxLeds, convertedDmxLeds, NUM_DMX_LEDS,
                           NUM_CONVERTED_DMX_LEDS );
      convertNeopixelRgbwArray( ropeLeds, convertedRopeLeds, NUM_NEO_LEDS,
                                NUM_CONVERTED_NEO_LEDS );
      FastLED.show();
   }

   void clearDmx() {
      for ( int i = 0; i < NUM_DMX_LEDS; ++i ) {
        dmxLeds[ i ] = CNEO::Black;
      }
   }

   void clearRope() {
      for ( int i = 0; i < NUM_NEO_LEDS; ++i ) {
        ropeLeds[ i ] = CNEO::Black;
      }
   }

   void clear() {
      clearDmx();
      clearRope();
   }
};

// singleton representing the one-and-only Flying Magic Carpet (TM)
MagicCarpet carpet();


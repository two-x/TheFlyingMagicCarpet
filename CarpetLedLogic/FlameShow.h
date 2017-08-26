/* FlameShow.h
 *
 * It's fucking flames how cool is that. What else do you even need to know.
 *
 * TODO: but really, write a real description when you know how this works
 *
 * Author: Anders Linn
 * Date: August 2017
 */

#include "LightShow.h"

// TODO: build a better flame palette. maybe use presets? or pass in a palette?
static const CRGBPalette256 flames(
      CRGB::DarkRed,
      CRGB::Red,
      CRGB::Orange,
      CRGB::Yellow );

class FlameShow : public LightShow {
 private:

   // TODO: tune this
   static const uint8_t coolingRate = 15;
   static const uint8_t sparkingRate = 230; // maybe set this based on music?

   uint8_t currTemperature[ NUM_NEO_LEDS ] = { 0 };
   uint8_t prevTemperature[ NUM_NEO_LEDS ] = { 0 };

 public:

   FlameShow( MagicCarpet * carpetArg ) : LightShow( carpetArg ) {}

   void start() {
      CRGB clr = ColorFromPalette( flames, 0 );
      for ( int i = 0; i < NUM_NEO_LEDS; ++i ) {
         carpet->ropeLeds[ i ] = clr;
      }
   }

   void update( uint32_t time ) {
      static uint32_t timestamp = 0;

      // cool everything
      for ( int i = 0; i < NUM_NEO_LEDS; ++i ) {
         prevTemperature[ i ] = qsub8( prevTemperature[ i ], coolingRate );
      }

      // disperse heat
      for ( int i = 0; i < NUM_NEO_LEDS; ++i ) {
         uint16_t lw = i > 0 ? i - 1 : NUM_NEO_LEDS - 1;
         uint16_t hi = i < NUM_NEO_LEDS - 1 ? i + i : 0;
         currTemperature[ i ] = ( prevTemperature[ lw ] +
                                  prevTemperature[ hi ] +
                                  prevTemperature[ i ] ) / 3;
      }

      // add sparks
      // TODO: this isn't enough for us, we'll want to spread our sparks out around the car
      if ( random8() < sparkingRate ) {
         uint16_t index = random16( NUM_NEO_LEDS - 1 );
         currTemperature[ index ] = qadd8( currTemperature[ index ], random8( 160, 255 ) );
      }

      // assign color
      for ( int i = 0; i < NUM_NEO_LEDS; ++i ) {
         carpet->ropeLeds[ i ] = ColorFromPalette( flames, currTemperature[ i ] );
      }

      // store prev color for next round
      for ( int i = 0; i < NUM_NEO_LEDS; ++i ) {
         prevTemperature[ i ] = currTemperature[ i ];
      }
   }
};


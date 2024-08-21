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
#include "AudioBoard.h"

// TODO: build a better flame palette. maybe use presets? or pass in a palette?
static const CRGBPalette256 flames(
      CRGB::DarkRed,
      CRGB::Red,
      CRGB::Orange,
      CRGB::Yellow );

  // Second, this palette is like the heat colors, but blue/aqua instead of red/yellow
static const CRGBPalette256 waterflames(
      CRGB::DarkBlue,
      CRGB::Blue,
      CRGB::Aqua,
      CRGB::White );
  
class FlameShow : public LightShow {
 private:

   // TODO: tune this
   static const uint8_t baseCoolingRate = 10;
   static const uint8_t baseSparkingRate = 15; // maybe set this based on music?

   uint8_t currTemperature[ NUM_NEO_LEDS ] = { 0 };
   uint8_t prevTemperature[ NUM_NEO_LEDS ] = { 0 };

 public:

   FlameShow( MagicCarpet * carpetArg ) : LightShow( carpetArg ) {}

   void start() {
      CRGB clr = ColorFromPalette( flames, 0 );
      LedUtil::fill( carpet->ropeLeds, clr, NUM_NEO_LEDS );
   }

   void update( uint32_t time ) {
      static uint32_t timestamp = 0;
      static uint32_t rate = 10;
      static const uint8_t numModes = 2;
      static uint8_t mode = 0;
      static bool isHeld = false;

      // pick color
      if ( carpet->button->isDown() ) {
         if ( !isHeld ) {
            ++mode %= numModes;
            isHeld = true;
         }
      } else {
         isHeld = false;
      }


      uint16_t potval = scaleTo255( carpet->pot->read(), 1023, 0 );

      // uint8_t coolingRate = baseCoolingRate + potval;
      // uint8_t sparkingRate = baseSparkingRate + potval;
      uint8_t coolingRate = baseCoolingRate + 65;
      uint8_t sparkingRate = baseSparkingRate + 45;
      coolingRate = coolingRate > 255 ? 255 : coolingRate;
      sparkingRate = sparkingRate > 255 ? 255 : sparkingRate;

      if ( time - timestamp > rate ) {
         timestamp = time;
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
         for ( int i = 0; i < NUM_NEO_LEDS; ++i ) {
            if ( random8() < sparkingRate ) {
               currTemperature[ i ] = qadd8( currTemperature[ i ],
                                             random8( 160, 255 ) );
            }
         }

         // assign color
         for ( int i = 0; i < NUM_NEO_LEDS; ++i ) {
            if ( mode ) {
               carpet->ropeLeds[ i ] = ColorFromPalette( flames, currTemperature[ i ] );
            } else {
               carpet->ropeLeds[ i ] = ColorFromPalette( waterflames, currTemperature[ i ] );
            }
         }

         // store prev color for next round
         for ( int i = 0; i < NUM_NEO_LEDS; ++i ) {
            prevTemperature[ i ] = currTemperature[ i ];
         }
      } else {
         // assign color
         for ( int i = 0; i < NUM_NEO_LEDS; ++i ) {
            CRGB newclr, oldclr;
            if ( mode ) {
               newclr = ColorFromPalette( flames, currTemperature[ i ] );
               oldclr = ColorFromPalette( flames, prevTemperature[ i ] );
            } else {
               newclr = ColorFromPalette( waterflames, currTemperature[ i ] );
               oldclr = ColorFromPalette( waterflames, prevTemperature[ i ] );
            }
            int val = scaleTo255( time - timestamp, rate, 0 );
            // Serial.println( val );
            carpet->ropeLeds[ i ] = blend( newclr, oldclr, val );
         }
      }

      int dmxval = AudioBoard::getLow();
      // Serial.println( dmxval );
      CRGB dmxclr;
      if ( mode ) {
         dmxclr = ColorFromPalette( flames, dmxval );
      } else {
         dmxclr = ColorFromPalette( waterflames, dmxval );
      }
      LedUtil::gammaCorrect( dmxclr );
      LedUtil::fill( carpet->megabarLeds, dmxclr, NUM_MEGABAR_LEDS );
      LedUtil::fill( carpet->chinaLeds, dmxclr, NUM_CHINA_LEDS );

      // use this instead of the rate
      delay( potval );
   }
};


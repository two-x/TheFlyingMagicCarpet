/* NightriderShow.h
 *
 * A basic version of the Nightrider light effect, which is just a simple chase that
 * starts in the corners and bounces off the middles. Lame, but it lights up the
 * carpet and doesn't rely on the sound.
 *
 * Author: Anders Linn
 * Date: August 2017
 */

#include "LightShow.h"
#include <math.h>

/*
const CRGB topC[] {
   CRGB( 0, 255, 127), // summer day
   CRGB( 178, 118, 50), // end summer day
   CRGB( 178, 102, 9), // deep dark night
   CRGB( 0, 255, 72), // desert afternoon
   CRGB( 43, 123, 93), //under the sea
   CRGB( 236, 84, 81), // beach party
   CRGB( 248, 182, 124)
};

const CRGB bottomC[] {
 CRGB( 0, 201, 100 ),
 CRGB( 254, 181, 97 ),
 CRGB( 0, 68, 101 ),
 CRGB( 76, 255, 127 ),
 CRGB( 50, 199, 143 ),
 CRGB( 182, 35, 99 ),
 CRGB( 248, 247, 5 ),
};
*/

class NightriderShow : public LightShow {
 private:
   enum Variation { VarManualHue = 0, VarAutoHueCycle = 1 };
   static const uint8_t numVariations_ = 2;
   uint8_t variation_;

   // variation 1's auto-cycle rate is pot-driven via the shared energy
   // setting (see LightShow.h) -- adjusting it here also becomes the new
   // starting point for LighthouseShow and FlameSparkle. Variation 0's pot
   // binding (direct hue pick) is unrelated to "energy" and stays a plain
   // live read, but gets its own settle-print since it had none before.
   PotEnergyTakeover energyTakeover_;
   SettlePrinter huePrinter_;

 public:
   NightriderShow( MagicCarpet * carpetArg, uint8_t initialVariation = 0 )
      : LightShow( carpetArg ), variation_( initialVariation % numVariations_ ) {}

   uint8_t variation() {
      return variation_;
   }

   const char * variationName() {
      return variation_ == VarManualHue ? "manual hue" : "auto hue cycle";
   }

   void start() {
      for ( int i = NEO3_OFFSET; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         carpet->ropeLeds[ i ] = CRGB::Black;
      }
      energyTakeover_.reset( carpet );
   }

   void update( uint32_t time ) {
      static const CRGBPalette256 clr( CRGB::Red, CRGB::Black );
      static uint32_t timestamp = time;
      static const uint32_t rate = 300; // move the lights every 200ms
      //static const uint32_t littleRate = rate;
      //static const uint32_t bigRate = ( littleRate * SIZEOF_LARGE_NEO ) / SIZEOF_SMALL_NEO;
      static uint8_t bigPos = 0;
      static uint8_t littlePos = 0;
      static uint8_t bigPosDir = 1;
      static uint8_t littlePosDir = 1;

      uint32_t diff = time - timestamp;
      if ( true || diff > rate ) {
         timestamp = time;
         diff = 0;
         if ( bigPos == SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER - 1 ) {
            bigPosDir = 0;
         } else if ( bigPos == 0 && bigPosDir == 0 ) {
            bigPosDir = 1;
         }
         if ( littlePos == SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER - 1 ) {
            littlePosDir = 0;
         } else if ( littlePos == 0 && littlePosDir == 0 ) {
            littlePosDir = 1;
         }
         if ( bigPosDir ) {
            ++bigPos;
         } else {
            --bigPos;
         }
         if ( littlePosDir ) {
            ++littlePos;
         } else {
            --littlePos;
         }
      }

      // variation select: each encoder detent moves to the next variation
      int varDelta = carpet->encoder->readPositionDelta();
      carpet->encoder->resetPositionDelta();
      if ( varDelta != 0 ) {
         int newVariation = ( (int)variation_ + varDelta ) % (int)numVariations_;
         if ( newVariation < 0 ) newVariation += numVariations_;
         variation_ = (uint8_t)newVariation;
      }

      uint8_t val1;
      static float autoHue = 0.0f;    // 0-255 continuous hue position, used by variation 2
      static uint32_t lastAutoTime = time;
      if ( variation_ == VarManualHue ) {
         // pot directly selects the color pair, as before
         val1 = (uint8_t)( carpet->pot->readPercent() / 100.0f * 255.0f + 0.5f );
         lastAutoTime = time; // keep the clock fresh so variation 2 doesn't jump on reentry
         huePrinter_.update( val1, "hue:" );
      } else {
         // variation 2: color automatically slides through the spectrum at a
         // constant rate, as if the pot from variation 1 were being turned
         // continuously. the shared energy setting instead sets the period of
         // one full cycle: 20 min at 0% down to 1 sec at 100%, mapped
         // exponentially since a linear map would cram nearly all the usable
         // range into a sliver near one end of the pot's travel.
         static const uint32_t slowestPeriodMs = 20UL * 60UL * 1000UL; // 20 min
         static const uint32_t fastestPeriodMs = 1000UL;               // 1 sec
         float potFrac = energyTakeover_.update( carpet ) / 100.0f;    // 0..1
         // powf(), not pow() -- this was double-precision for no reason
         // (the ratio and result are both already float-precision quantities)
         uint32_t periodMs = (uint32_t)( slowestPeriodMs *
               powf( (float)fastestPeriodMs / (float)slowestPeriodMs, potFrac ) );

         uint32_t dtMs = time - lastAutoTime;
         lastAutoTime = time;
         autoHue += ( (float)dtMs / (float)periodMs ) * 255.0f;
         while ( autoHue >= 255.0f ) autoHue -= 255.0f;
         val1 = (uint8_t)autoHue;
      }
      const uint8_t val2 = ( val1 + 128 ) % 255;
      const CRGB clr1 = CHSV( val1, 255, 255 );
      const CRGB clr2 = CHSV( val2, 255, 255 );

      for ( int i = FRONT; i < FRONT_RIGHT; ++i ) {
         int i_adj = i - FRONT;
         int val = scaleTo255( abs(littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
      }
      for ( int i = FRONT_RIGHT; i < RIGHT; ++i ) {
         int i_adj = i - FRONT_RIGHT;
         int val = scaleTo255( abs( bigPos - i_adj), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
      }
      for ( int i = RIGHT; i < BACK_RIGHT; ++i ) {
         int i_adj = i - RIGHT;
         int val = scaleTo255( abs(bigPos - i_adj), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
      }
      for ( int i = BACK_RIGHT; i < BACK; ++i ) {
         int i_adj = i - BACK_RIGHT;
         int val = scaleTo255( abs( littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
      }
      for ( int i = BACK; i < BACK_LEFT; ++i ) {
         int i_adj = i - BACK;
         int val = scaleTo255( abs( littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
      }
      for ( int i = BACK_LEFT; i < LEFT; ++i ) {
         int i_adj = i - BACK_LEFT;
         int val = scaleTo255( abs( bigPos - i_adj), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
      }
      for ( int i = LEFT; i < FRONT_LEFT; ++i ) {
         int i_adj = i - LEFT;
         int val = scaleTo255( abs(bigPos - i_adj), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
      }
      for ( int i = FRONT_LEFT; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         int i_adj = i - FRONT_LEFT;
         int val = scaleTo255( abs( littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
      }
      for ( int i = 0; i < FRONT; ++i ) {
         int i_adj = i + SIZEOF_LARGE_NEO_CORNER;
         int val = scaleTo255( abs( littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
      }
      

      LedUtil::reverse( carpet->ropeLeds + FRONT, SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER );
      LedUtil::reverse( carpet->ropeLeds + BACK, SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER );
      LedUtil::reverse( carpet->ropeLeds + RIGHT, SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER );
      LedUtil::reverse( carpet->ropeLeds + LEFT, SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER );

      int lowval = (int)AudioBoard::getNormalPercent( BandLow ) * 255 / 100; // AudioBoard's getters are percent now; converted back to this show's own existing 0-255 scale
      static int lastlow = lowval;
      if (lowval > 80 && lowval > lastlow) {
         lastlow = lowval;
      } else {
         lastlow = lastlow > 15 ? lastlow - 15 : 0;
      }
      CRGB dmxclr1 = blend( clr1, clr2, lastlow );
      LedUtil::fill( carpet->megabarLeds, dmxclr1, NUM_MEGABAR_LEDS );

      int highval = (int)AudioBoard::getNormalPercent( BandHigh ) * 255 / 100; // see lowval's comment above
      static int lasthigh = highval;
      if (highval > 100 && highval > lasthigh) {
         lasthigh = highval;
      } else {
         lasthigh = lasthigh > 25 ? lasthigh - 25 : 0;
      }
      CRGBW dmxclr2 = blend( clr1, clr2, lastlow );
      dmxclr2.w = lasthigh;
      LedUtil::fill( carpet->chinaLeds, dmxclr2, NUM_CHINA_LEDS );

      /*
      static uint8_t white = 0;
      if ( carpet->encoder->button.isDown() ) {
         if ( white < 255 ) {
            ++white;
         }
      } else {
         if ( white > 0 ) {
            --white;
         }
      }
      for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         carpet->ropeLeds[ i ].w = white;
      }
      */
   }
};

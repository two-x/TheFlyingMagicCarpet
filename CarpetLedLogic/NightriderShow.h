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

class NightriderShow : public LightShow {
 public:
   NightriderShow( MagicCarpet * carpetArg ) : LightShow( carpetArg ) {}

   void start() {
      for ( int i = NEO3_OFFSET; i < NEO4_OFFSET; ++i ) {
         carpet->ropeLeds[ i ] = CRGB::Black;
      }
   }

   void update( uint32_t time ) {
      static const CRGBPalette256 clr( CRGB::Red, CRGB::Black );
      static uint32_t timestamp = time;
      static const uint32_t rate = 300; // move the lights every 200ms
      static const uint32_t littleRate = rate;
      static const uint32_t bigRate = ( littleRate * SIZEOF_LARGE_NEO ) / SIZEOF_SMALL_NEO;
      static uint8_t bigPos = 0;
      static uint8_t littlePos = 0;
      static uint8_t bigPosDir = 1;
      static uint8_t littlePosDir = 1;

      uint32_t diff = time - timestamp;
      if ( true || diff > rate ) {
         timestamp = time;
         diff = 0;
         if ( bigPos == SIZEOF_LARGE_NEO - 1 ) {
            bigPosDir = 0;
         } else if ( bigPos == 0 && bigPosDir == 0 ) {
            bigPosDir = 1;
         }
         if ( littlePos == SIZEOF_SMALL_NEO - 1 ) {
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

      uint16_t paletteIndex = scaleTo255( carpet->pot->read(), 1023, 0 );
      Serial.println( "pot" );
      Serial.println( carpet->pot->read() );
      Serial.println( paletteIndex );
      int diffIndex = scaleTo255( diff, rate, 0 );

      // Serial.println( "CRGB:0" );
      // Serial.println( getColor( paletteIndex, 0 ).r );
      // Serial.println( getColor( paletteIndex, 0 ).g );
      // Serial.println( getColor( paletteIndex, 0 ).b );
      // Serial.println( "CRGB:1" );
      // Serial.println( getColor( paletteIndex, 255 ).r );
      // Serial.println( getColor( paletteIndex, 255 ).g );
      // Serial.println( getColor( paletteIndex, 255 ).b );

      for ( int i = NEO0_OFFSET; i < NEO1_OFFSET; ++i ) {
         int val = scaleTo255( abs( i - littlePos ), SIZEOF_SMALL_NEO, 0 );
         carpet->ropeLeds[ i ] = getColor( paletteIndex, val );
         Serial.println( "CRGB" );
         Serial.println( getColor( paletteIndex, 0 ).r );
         Serial.println( getColor( paletteIndex, 0 ).g );
         Serial.println( getColor( paletteIndex, 0 ).b );
      }
      for ( int i = NEO1_OFFSET; i < NEO2_OFFSET; ++i ) {
         int val = scaleTo255( abs( i - NEO1_OFFSET - bigPos ), NEO2_OFFSET - NEO1_OFFSET, 0 );
         carpet->ropeLeds[ i ] = getColor( paletteIndex, val );
      }
      for ( int i = NEO2_OFFSET; i < NEO3_OFFSET; ++i ) {
         int val = 255 - scaleTo255( abs( i - NEO2_OFFSET - bigPos ), NEO3_OFFSET - NEO2_OFFSET, 0 );
         carpet->ropeLeds[ i ] = getColor( paletteIndex, val );
      }
      for ( int i = NEO3_OFFSET; i < NEO4_OFFSET; ++i ) {
         int val = scaleTo255( abs( i - NEO3_OFFSET - littlePos ), NEO4_OFFSET - NEO3_OFFSET, 0 );
         carpet->ropeLeds[ i ] = getColor( paletteIndex, val );
      }
      for ( int i = NEO4_OFFSET; i < NEO5_OFFSET; ++i ) {
         int val = scaleTo255( abs( i - NEO4_OFFSET - littlePos ), NEO5_OFFSET - NEO4_OFFSET, 0 );
         carpet->ropeLeds[ i ] = getColor( paletteIndex, val );
      }
      for ( int i = NEO5_OFFSET; i < NEO6_OFFSET; ++i ) {
         int val = 255 - scaleTo255( abs( i - NEO5_OFFSET - bigPos ), NEO6_OFFSET - NEO5_OFFSET, 0 );
         carpet->ropeLeds[ i ] = getColor( paletteIndex, val );
      }
      for ( int i = NEO6_OFFSET; i < NEO7_OFFSET; ++i ) {
         int val = scaleTo255( abs( i - NEO6_OFFSET - bigPos ), NEO7_OFFSET - NEO6_OFFSET, 0 );
         carpet->ropeLeds[ i ] = getColor( paletteIndex, val );
      }
      for ( int i = NEO7_OFFSET; i < NEO7_OFFSET + SIZEOF_SMALL_NEO; ++i ) {
         int val = scaleTo255( abs( i - NEO7_OFFSET - littlePos ), SIZEOF_SMALL_NEO, 0 );
         carpet->ropeLeds[ i ] = getColor( paletteIndex, val );
      }

      LedUtil::reverse( carpet->ropeLeds + NEO0_OFFSET, SIZEOF_SMALL_NEO );
      LedUtil::reverse( carpet->ropeLeds + NEO4_OFFSET, SIZEOF_SMALL_NEO );

      int dmxval = AudioBoard::getLow();
      CRGB dmxclr = getColor( paletteIndex, dmxval );
      // Serial.println( dmxval );
      LedUtil::fill( carpet->megabarLeds, dmxclr, NUM_MEGABAR_LEDS );
      LedUtil::fill( carpet->chinaLeds, dmxclr, NUM_CHINA_LEDS );

      static uint8_t white = 0;
      if ( carpet->button->isDown() ) {
         if ( white < 255 ) {
            ++white;
         }
      } else {
         if ( white > 0 ) {
            --white;
         }
      }
      for ( int i = 0; i < NUM_NEO_LEDS; ++i ) {
         carpet->ropeLeds[ i ].w = white;
      }
   }
};

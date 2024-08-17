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

class NightriderShow : public LightShow {
 public:
   NightriderShow( MagicCarpet * carpetArg ) : LightShow( carpetArg ) {}

   void start() {
      for ( int i = NEO_START; i < NEO_END; ++i ) {
         carpet->ropeLeds[ i ] = CRGB::Black;
      }
   }

   void update( uint32_t time ) {
      static uint32_t timestamp = time;
      static const uint32_t rate = 300; // move the lights at most every 300ms
      static const uint32_t endRate = rate;
      static const uint32_t sideRate = ( endRate * SIZEOF_LARGE_NEO ) / SIZEOF_SMALL_NEO;
      
      // start in the middle of the section
      static uint8_t frontPos = FRONT;
      static uint8_t backPos = BACK;
      static uint8_t rightPos = RIGHT;
      static uint8_t leftPos = LEFT;
      static uint8_t frontPosDir = 1;
      static uint8_t backPosDir = 1;
      static uint8_t rightPosDir = 1;
      static uint8_t leftPosDir = 1;

      uint32_t diff = time - timestamp;
      if ( diff > rate ) {
         timestamp = time;
         diff = 0;
         if ( frontPos == FRONT_RIGHT - 1 ) { // if we reach the corner, turn around
            frontPosDir = 0;
         } else if ( frontPos == FRONT && frontPosDir == 0 ) { // if we're back in the middle, turn around
            frontPosDir = 1;
         }
         if ( backPos == BACK_LEFT - 1 ) { // if we reach the corner, turn around
            backPosDir = 0;
         } else if ( backPos == BACK && backPosDir == 0 ) { // if we're back in the middle, turn around
            backPosDir = 1;
         }
         if ( rightPos == BACK_RIGHT - 1 ) { // if we reach the corner, turn around
            rightPosDir = 0;
         } else if ( rightPos == RIGHT && rightPosDir == 0 ) { // if we're back in the middle, turn around
            rightPosDir = 1;
         }
         if ( leftPos == FRONT_LEFT - 1 ) { // if we reach the corner, turn around
            leftPosDir = 0;
         } else if ( leftPos == LEFT && leftPosDir == 0 ) { // if we're back in the middle, turn around
            leftPosDir = 1;
         }

         frontPos = ++frontPos ? frontPosDir : --frontPos;
         backPos = ++backPos ? backPosDir : --backPos;
         rightPos = ++rightPos ? rightPosDir : --rightPos;
         leftPos = ++leftPos ? leftPosDir : --leftPos;
      }

      const uint8_t val1 = carpet->pot->read() / 4;
      const uint8_t val2 = ( val1 + 128 ) % 255;
      const CRGB clr1 = CHSV( val1, 255, 255 );
      const CRGB clr2 = CHSV( val2, 255, 255 );;
      // Serial.println( "potval" );
      // Serial.println( val1 );
      // const CRGBW clr1 = topC[val1];
      // const CRGBW clr2 = bottomC[val1];
      // Serial.println( "CRGB1" );
      // Serial.println( clr1.r );
      // Serial.println( clr1.g );
      // Serial.println( clr1.b );
      // Serial.println( clr1.w );
      // Serial.println( "CRGB2" );
      // Serial.println( clr2.r );
      // Serial.println( clr2.g );
      // Serial.println( clr2.b );
      // Serial.println( clr2.w );
      int diffIndex = scaleTo255( diff, rate, 0 );

      for ( int i = FRONT; i < FRONT_RIGHT; ++i ) {
         int val = scaleTo255( abs( i - frontPos ), FRONT_RIGHT, FRONT );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         carpet->ropeLeds[ ( NEO_END - 1 ) - i ] = blend( clr1, clr2, val ); // mirror for the other side of the section
      }
      for ( int i = RIGHT; i < BACK_RIGHT; ++i ) {
         int val = scaleTo255( abs( i - rightPos ), BACK_RIGHT, RIGHT );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         carpet->ropeLeds[ ( RIGHT - 1 ) - ( i - RIGHT ) ] = blend( clr1, clr2, val );
      }
      for ( int i = BACK; i < BACK_LEFT; ++i ) {
         int val = scaleTo255( abs( i - backPos ), BACK_LEFT, BACK );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         carpet->ropeLeds[ ( BACK - 1 ) - ( i - BACK ) ] = blend( clr1, clr2, val );
      }
      for ( int i = LEFT; i < FRONT_LEFT; ++i ) {
         int val = scaleTo255( abs( i - backPos ), FRONT_LEFT, LEFT );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         carpet->ropeLeds[ ( LEFT - 1 ) - ( i - LEFT ) ] = blend( clr1, clr2, val );
      }

      // LedUtil::reverse( carpet->ropeLeds + NEO0_OFFSET, SIZEOF_SMALL_NEO );
      // LedUtil::reverse( carpet->ropeLeds + NEO4_OFFSET, SIZEOF_SMALL_NEO );

      int dmxval = AudioBoard::getLow();
      CRGB dmxclr = blend( clr1, clr2, dmxval );
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

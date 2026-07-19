/* EqualizerShow.h
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

class EqualizerShow : public LightShow {
 public:
   EqualizerShow( MagicCarpet * carpetArg ) : LightShow( carpetArg ) {}

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

      //const uint8_t val1 = carpet->pot->read() / 4;
      //const uint8_t val2 = ( val1 + 128 ) % 255;
      //const CRGB clr1 = CHSV( val1, 255, 255 );
      //const CRGB clr2 = CHSV( val2, 255, 255 );;
      CRGB clr1 = CRGB(0,0,255);
      CRGB clr2 = CRGB(255,0,0);
      // CRGB clr3 = CRGB(0,255,0);
      // Serial.println( "potval" );
      // Serial.println( val1 );
      //const CRGBW clr1 = topC[val1];
      //const CRGBW clr2 = bottomC[val1];
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
      // int diffIndex = scaleTo255( diff, rate, 0 );

      for ( int i = FRONT; i < FRONT_RIGHT; ++i ) {
         int i_adj = i - FRONT;
        int val = scaleTo255( abs(littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Red;
      }
      for ( int i = FRONT_RIGHT; i < RIGHT; ++i ) {
         int i_adj = i - FRONT_RIGHT;
         int val = scaleTo255( abs( bigPos - i_adj), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Green;
      }
      for ( int i = RIGHT; i < BACK_RIGHT; ++i ) {
         int i_adj = i - RIGHT;
         int val = scaleTo255( abs(bigPos - i_adj), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Blue;
      }
      for ( int i = BACK_RIGHT; i < BACK; ++i ) {
         int i_adj = i - BACK_RIGHT;
        int val = scaleTo255( abs( littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Yellow;
      }
      for ( int i = BACK; i < BACK_LEFT; ++i ) {
         int i_adj = i - BACK;
        int val = scaleTo255( abs( littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Orange;
      }
      for ( int i = BACK_LEFT; i < LEFT; ++i ) {
         int i_adj = i - BACK_LEFT;
         int val = scaleTo255( abs( bigPos - i_adj), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Purple;
      }
      for ( int i = LEFT; i < FRONT_LEFT; ++i ) {
         int i_adj = i - LEFT;
         int val = scaleTo255( abs(bigPos - i_adj), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Pink;
      }
      for ( int i = FRONT_LEFT; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         int i_adj = i - FRONT_LEFT;
        int val = scaleTo255( abs( littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Grey;
      }
      for ( int i = 0; i < FRONT; ++i ) {
         int i_adj = i + SIZEOF_LARGE_NEO_CORNER;
        int val = scaleTo255( abs( littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Grey;
      }
      

      LedUtil::reverse( carpet->ropeLeds + FRONT, SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER );
       LedUtil::reverse( carpet->ropeLeds + BACK, SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER );
      LedUtil::reverse( carpet->ropeLeds + RIGHT, SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER );
      LedUtil::reverse( carpet->ropeLeds + LEFT, SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER );

      CRGB dmxclr;
      // int dmxval = AudioBoard::getLow();
      // Serial.println( dmxval );
      int lowval = AudioBoard::getLow();
      static int lastlow = lowval;
      Serial.print("lowval: ");
      Serial.println(lowval);
      Serial.flush();
      if (lowval > 80 && lowval > lastlow) {
       lastlow = lowval;
      } else {
 
       lastlow = lastlow > 15 ? lastlow - 15 : 0;
      }
      Serial.print("lastlow: ");
      Serial.println(lastlow);
      Serial.flush();
      dmxclr = blend( CRGB::Black, clr2, lastlow );
      carpet->megabarLeds[1] = dmxclr;
      carpet->megabarLeds[2] = dmxclr;
      carpet->megabarLeds[4] = dmxclr;
      carpet->megabarLeds[5] = dmxclr;
      carpet->megabarLeds[7] = dmxclr;
      carpet->megabarLeds[8] = dmxclr;
      carpet->megabarLeds[10] = dmxclr;
      carpet->megabarLeds[11] = dmxclr;


      int highval = AudioBoard::getHigh() > 150 ? AudioBoard::getHigh() : 0;
      Serial.print("highval: ");
      Serial.println(highval);
      /*static int lasthigh = highval;
      Serial.print("lasthigh: ");
      Serial.println(highval);
      Serial.flush();
      if (highval > 100 && highval > lasthigh) {
       lasthigh = highval;
      } else {
       lasthigh = lasthigh > 15 ? lasthigh - 15 : 0;
      }
      Serial.print("lasthigh: ");
      Serial.println(lasthigh);
      Serial.flush();*/
      // dmxclr = blend( dmxclr, clr1, highval );
      dmxclr = blend( CRGB::Black, clr1, highval );
      
      carpet->megabarLeds[0] = dmxclr;
      carpet->megabarLeds[3] = dmxclr;
      carpet->megabarLeds[6] = dmxclr;
      carpet->megabarLeds[9] = dmxclr;



      //CRGB dmxclr = blend( clr1, clr2, lowval);
      //dmxclr = blend( clr2, clr3, highval);
      //dmxclr = CRGB(lowval,highval,30);



      /*int midval = AudioBoard::getMid();
      int highval = AudioBoard::getHigh();
      static int maxlow = 0;
      static int minlow = 255;
      maxlow = max(maxlow, lowval);
      Serial.println("maxlow");
      Serial.println(maxlow);
      minlow = min(minlow, lowval);
      Serial.println("minlow");
      Serial.println(minlow);
      // lowval = lowval > 75 ? lowval - 75 : 0;
      // midval = midval > 75 ? midval - 75 : 0;
     //  highval = highval > 75 ? highval - 75 : 0;
      // CRGB dmxclr = (lowval > midval && lowval >= highval) ? CRGB::Red :(highval > midval) ? CRGB::Green : CRGB::Blue;
      float avg = lowval + midval + highval;
      int dmxred = (255 * ( ((float) lowval) / avg));
      int dmxblue = (255 * ( ((float) midval) / avg));
      int dmxgreen = (255 * ( ((float) highval) / avg));
      Serial.println("dmxred:");
      Serial.println(dmxred);
      Serial.println("dmxblue:");
      Serial.println(dmxblue);
      Serial.println("dmxgreen:");
      Serial.println(dmxgreen);
      //CRGB dmxclr(dmxred, 0, dmxgreen);
      Serial.println("low: ");
      Serial.println(lowval);
      Serial.println("mid");
      Serial.println(midval);
      Serial.println("high");
      Serial.println(highval);*/
      // CRGB dmxclr(lowval, midval, highval);
      //LedUtil::fill( carpet->megabarLeds, dmxclr, NUM_MEGABAR_LEDS );
      //LedUtil::fill( carpet->chinaLeds, dmxclr, NUM_CHINA_LEDS );

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
      for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         carpet->ropeLeds[ i ].w = white;
      }
   }
};
